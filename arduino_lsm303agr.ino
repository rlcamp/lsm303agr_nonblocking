/* totally non-blocking set of nested state machines for initializing and computing heading from
 the output of an LSM303AGR i2c-connected magnetometer */

#include "lsm303agr_nonblocking.h"

static struct lsm303agr_state lsm303agr_state;
static unsigned long prev;
static int state;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.printf("hello\n");

    while (lsm303agr_init(&lsm303agr_state, millis())) __WFI();

    prev = millis();
}

void loop() {
    __WFI();
    const unsigned long now = millis();

    if (0 == state) {
        int16_t values[3];
        if (lsm303agr_oneshot(values, &lsm303agr_state, now)) return;

        if (values[0] || values[1] || values[2]) {
            /* do some stupid, fragile auto calibration stuff */
            static int16_t x_min = INT16_MAX, x_max = INT16_MIN;
            static int16_t y_min = INT16_MAX, y_max = INT16_MIN;
            static int16_t z_min = INT16_MAX, z_max = INT16_MIN;

            if (values[0] < x_min) x_min = values[0];
            if (values[0] > x_max) x_max = values[0];
            if (values[1] < y_min) y_min = values[1];
            if (values[1] > y_max) y_max = values[1];
            if (values[2] < z_min) z_min = values[2];
            if (values[2] > z_max) z_max = values[2];

            const int x_calibrated = values[0] - (x_min + x_max + 1) / 2;
            const int y_calibrated = values[1] - (y_min + y_max + 1) / 2;

            /* this value will be bs until the sensor has been turned in a circle */
            const float heading = atan2f(y_calibrated, x_calibrated) * 180.0 / M_PI;

            Serial.printf("values: %d %d %d: heading %dd, strength %d\n",
                          values[0], values[1], values[2], (int)(heading),
                          (int)sqrtf(values[0] * values[0] + values[1] * values[1] + values[2] * values[2]));
        }
        else Serial.printf("no values yet\n");

        state++;
    }

    else if (1 == state) {
        /* we've made it this far without using delay(), keep at it */
        if (now - prev < 100) return;
        prev += 100;

        state = 0;
    }
}
