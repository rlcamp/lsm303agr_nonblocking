/* campbell, 2023, isc license to the extent applicable */

#include "i2c_nonblocking.h"

#ifdef __cplusplus
extern "C" {
#endif

struct lsm303agr_state {
    unsigned int state;
    unsigned long prev;
    struct i2c_state i2c_state;

    uint8_t buffer[7];
};

struct lsm303agr_result {
//    uint16_t temperature;
//    int16_t accel[3];
    int16_t mag[3];
};

/* this should be passed a zero-initialized instance of the above struct, and should call the
 function repeatedly as needed until it returns zero, at which point the calling code may trust
 that the struct is once again zeroed and can be reused for the next call */
int lsm303agr_oneshot(struct lsm303agr_result * result, struct lsm303agr_state * state, unsigned long now);

#ifdef __cplusplus
}
#endif
