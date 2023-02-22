/* campbell, 2023, isc license to the extent applicable */

#include "lsm303agr_nonblocking.h"
#include <string.h>

int lsm303agr_oneshot(int16_t values[3], struct lsm303agr_state * state, unsigned long now) {
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

    else if (2 == state->state) {
        /* reboot */
        int ret = i2c_write_one_byte(&state->i2c_state, 1U << 5, 0x1e, 0x60);
        if (ret) {
            if (-1 == ret) state->state = 0;
            return 1;
        }
    }

    else if (3 == state->state) {
        /* reset */
        int ret = i2c_write_one_byte(&state->i2c_state, 1U << 6, 0x1e, 0x60);
        if (ret) {
            if (-1 == ret) state->state = 0;
            return 1;
        }
    }

    else if (4 == state->state) {
        /* enable bdu */
        int ret = i2c_write_one_byte(&state->i2c_state, 1U << 4, 0x1e, 0x62);
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
        int ret = i2c_write_one_byte(&state->i2c_state, 1U << 7 | 1, 0x1e, 0x60);
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
        int ret = i2c_read_from_register(&state->i2c_state, state->status_and_results, 7, 0x1e, 0x67 | 0x80);
        if (ret) {
            if (-1 == ret) state->state = 0;
            return 1;
        }

        if (state->status_and_results[0] & 0x0F) {
            memcpy(values + 0, state->status_and_results + 1, 2);
            memcpy(values + 1, state->status_and_results + 3, 2);
            memcpy(values + 2, state->status_and_results + 5, 2);
        }
        else
            memset(values, 0, sizeof(int16_t[3]));

        /* note the next valid state is the beginning of a new oneshot transaction */
        state->state = 6;
        return 0;
    }
}
