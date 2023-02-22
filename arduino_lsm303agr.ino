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

static void int_cross_product(int32_t c[3], const int16_t a[3], const int16_t b[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
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

            /* TODO: validate against integer overflow */
            const int16_t x_offset = (x_min + x_max + 1) / 2;
            const int16_t y_offset = (y_min + y_max + 1) / 2;
            const int16_t z_offset = (z_min + z_max + 1) / 2;

            /* we should have a guarantee these don't overflow */
            const int16_t mag_with_offset[3] = {
              (int16_t)(result.mag[0] - x_offset),
              (int16_t)(result.mag[1] - y_offset),
              (int16_t)(result.mag[2] - z_offset)
            };

            /* left handed cross product */
            int32_t east[3];
            int_cross_product(east, result.accel, mag_with_offset);

            /* this value will be bs until the sensor has been turned in a circle */
            /* TODO: validate both of these away from magnets */
            const float heading0 = remainderf(atan2f(-mag_with_offset[1], mag_with_offset[0]) * 180.0f / (float)M_PI - 180.0f, 360.0f) + 180.0f;
            const float heading1 = remainderf(atan2f(east[0], east[1]) * 180.0f / (float)M_PI - 180.0f, 360.0f) + 180.0f;

            /* assuming +x is "forward", this gives positive angles when pitched up */
            const float pitch = atan2f(result.accel[0], result.accel[2]) * 180.0 / M_PI;

            /* assuming +x is "forward", this gives positive angles when rolled right */
            const float roll = atan2f(-result.accel[1], result.accel[2]) * 180.0 / M_PI;

            Serial.printf("pitch %dd, roll %dd\n", (int)lrint(pitch), (int)lrint(roll));

            Serial.printf("accel: % +5d % +5d % +5d\n", result.accel[0], result.accel[1], result.accel[2]);
            Serial.printf("mag raw: % +5d % +5d % +5d\n", result.mag[0], result.mag[1], result.mag[2]);
            Serial.printf("mag offsets % +5d % +5d % +5d, calibrated: % +5d % +5d % +5d\n",
                          x_offset, y_offset, z_offset, mag_with_offset[0], mag_with_offset[1], mag_with_offset[2]);
            Serial.printf("heading: magnetic only %dd, pitch/roll compensated %dd\n", (int)lrint(heading0), (int)lrint(heading1));
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
