/* campbell, 2023, isc license to the extent applicable */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct i2c_state {
    unsigned state;
    unsigned long prev;
};

/* each of these should be passed a zero-initialized instance of the above struct, and should call
 the function repeatedly as needed until it returns zero, at which point the calling code may trust
 that the struct is once again zeroed and can be reused for the next call */
int i2c_init(struct i2c_state * state, unsigned long now, unsigned long us_per_tick);
int i2c_write_one_byte(struct i2c_state * state, uint8_t byte, uint8_t addr, uint8_t reg);
int i2c_read_from_register(struct i2c_state * state, void * out, size_t count, uint8_t addr, uint8_t reg);

#ifdef __cplusplus
}
#endif
