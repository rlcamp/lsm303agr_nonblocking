#include "i2c_nonblocking.h"

#ifdef __cplusplus
extern "C" {
#endif

struct lsm303agr_state {
    unsigned int state;
    unsigned long prev;
    struct i2c_state i2c_state;

    uint8_t status_and_results[7];
};

/* this should be passed a zero-initialized instance of the above struct, and should call the
 function repeatedly as needed until it returns zero, at which point the calling code may trust
 that the struct is once again zeroed and can be reused for the next call */
int lsm303agr_oneshot(int16_t values[3], struct lsm303agr_state * state, unsigned long now);

#ifdef __cplusplus
}
#endif
