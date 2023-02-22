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

    prev = millis();
}

void loop() {
    __WFI();
    const unsigned long now = millis();

    if (0 == state) {
        struct lsm303agr_result result;
        if (lsm303agr_oneshot(&result, &lsm303agr_state, now)) return;

        if (result.mag[0] || result.mag[1] || result.mag[2]) {
            /* do some stupid, fragile auto calibration stuff */
            static int16_t x_min = INT16_MAX, x_max = INT16_MIN;
            static int16_t y_min = INT16_MAX, y_max = INT16_MIN;
            static int16_t z_min = INT16_MAX, z_max = INT16_MIN;

            if (result.mag[0] < x_min) x_min = result.mag[0];
            if (result.mag[0] > x_max) x_max = result.mag[0];
            if (result.mag[1] < y_min) y_min = result.mag[1];
            if (result.mag[1] > y_max) y_max = result.mag[1];
            if (result.mag[2] < z_min) z_min = result.mag[2];
            if (result.mag[2] > z_max) z_max = result.mag[2];

            const int x_calibrated = result.mag[0] - (x_min + x_max + 1) / 2;
            const int y_calibrated = result.mag[1] - (y_min + y_max + 1) / 2;

            /* this value will be bs until the sensor has been turned in a circle */
            const float heading = atan2f(y_calibrated, x_calibrated) * 180.0 / M_PI;

            Serial.printf("accel: %d %d %d\n", result.accel[0], result.accel[1], result.accel[2]);
            Serial.printf("mag: %d %d %d: heading %dd, strength %d\n",
                          result.mag[0], result.mag[1], result.mag[2], (int)(heading),
                          (int)sqrtf(result.mag[0] * result.mag[0] + result.mag[1] * result.mag[1] + result.mag[2] * result.mag[2]));
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
