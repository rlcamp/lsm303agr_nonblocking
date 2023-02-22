/* campbell, 2023, isc license to the extent applicable */

#include "lsm303agr_nonblocking.h"
#include <string.h>

int lsm303agr_oneshot(struct lsm303agr_result * result, struct lsm303agr_state * state, unsigned long now) {
    if (0 == state->state) {
        /* reinitialize the i2c state, in case we are here because of an abnormal condition */
        state->i2c_state = (struct i2c_state) { 0 };

        /* proceed immediately to next state */
        state->state++;
    }

    if (1 == state->state) {
        /* init i2c bus */
        int ret = i2c_init(&state->i2c_state, now);
        if (ret) {
            if (-1 == ret) state->state = 0;
            return 1;
        }
    }

    else if (state->state < 5) {
        /* table of i2c register writes versus state index */
        static const struct config_write { uint8_t val, reg, address; } writes[] = {
          { .val = 0b00100000, .reg = 0x1e, .address = 0x60 }, /* reboot magnetometer */
          { .val = 0b01000000, .reg = 0x1e, .address = 0x60 }, /* reset magnetometer */
          { .val = 0b00010000, .reg = 0x1e, .address = 0x62 }, /* configure bdu for magnetometer */
        };
        const struct config_write * write = writes + state->state - 2;

        int ret = i2c_write_one_byte(&state->i2c_state, write->val, write->reg, write->address);
        if (ret) {
            if (-1 == ret) state->state = 0;
            return 1;
        }
    }

    else if (5 == state->state) {
        /* wait ten more milliseconds */
        if (now - state->prev < 10) return 1;
    }

    else if (6 == state->state) {
        /* initiate a oneshot transaction */
        int ret = i2c_write_one_byte(&state->i2c_state, 0b10000001, 0x1e, 0x60);
        if (ret) {
            if (-1 == ret) state->state = 0;
            return 1;
        }
    }

    else if (7 == state->state) {
        /* wait ten milliseconds */
        if (now - state->prev < 10) return 1;
    }

    if (state->state < 8) {
        /* completed any state other than the final one */
        state->state++;
        state->prev = now;
        return 1;
    } else {
        /* final state */
        int ret = i2c_read_from_register(&state->i2c_state, state->buffer, 7, 0x1e, 0x67 | 0x80);
        if (ret) {
            if (-1 == ret) state->state = 0;
            return 1;
        }

        if (state->buffer[0] & 0x0F) {
            memcpy(result->mag + 0, state->buffer + 1, 2);
            memcpy(result->mag + 1, state->buffer + 3, 2);
            memcpy(result->mag + 2, state->buffer + 5, 2);
        }
        else
            memset(result->mag, 0, sizeof(int16_t[3]));

        /* note the next valid state is the beginning of a new oneshot transaction */
        state->state = 6;
        return 0;
    }
}
