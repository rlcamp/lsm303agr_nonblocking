#include "lsm303agr_nonblocking.h"
#include <string.h>

int lsm303agr_init(struct lsm303agr_state * state, unsigned long now) {
    if (0 == state->state) {
        /* init i2c bus */
        if (i2c_init(&state->i2c_state, now)) return 1;
    }

    else if (1 == state->state) {
        /* reboot */
        if (i2c_write_one_byte(&state->i2c_state, 1U << 5, 0x1e, 0x60)) return 1;
    }

    else if (2 == state->state) {
        /* reset */
        if (i2c_write_one_byte(&state->i2c_state, 1U << 6, 0x1e, 0x60)) return 1;
    }

    else if (3 == state->state) {
        /* enable bdu */
        if (i2c_write_one_byte(&state->i2c_state, 1U << 4, 0x1e, 0x62)) return 1;
    }

    if (state->state < 4) {
        /* completed any state other than the final one */
        state->state++;
        state->prev = now;
        return 1;
    } else {
        /* final state */

        /* wait ten more milliseconds */
        if (now - state->prev < 10) return 1;

        state->i2c_state.state = 0;
        state->state = 0;
        return 0;
    }
}

int lsm303agr_oneshot(int16_t values[3], struct lsm303agr_state * state, unsigned long now) {
    if (0 == state->state) {
        /* initiate a oneshot transaction */
        if (i2c_write_one_byte(&state->i2c_state, 1U << 7 | 1, 0x1e, 0x60)) return 1;
    }
    else if (1 == state->state) {
        /* wait ten milliseconds */
        if (now - state->prev < 10) return 1;
    }

    if (state->state < 2) {
        /* completed any state other than the final one */
        state->state++;
        state->prev = now;
        return 1;
    } else {
        /* final state */
        if (i2c_read_from_register(&state->i2c_state, state->status_and_results, 7, 0x1e, 0x67 | 0x80)) return 1;

        if (state->status_and_results[0] & 0x0F) {
            memcpy(values + 0, state->status_and_results + 1, 2);
            memcpy(values + 1, state->status_and_results + 3, 2);
            memcpy(values + 2, state->status_and_results + 5, 2);
        }
        else
            memset(values, 0, sizeof(int16_t[3]));

        /* reached end of final state */
        state->i2c_state.state = 0;
        state->state = 0;
        return 0;
    }
}
