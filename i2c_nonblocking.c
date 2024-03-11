/* campbell, 2023, isc license to the extent applicable */

#include "i2c_nonblocking.h"

#if __has_include(<samd51.h>)
/* as invoked by Makefile, regardless of cmsis-atmel version */
#include <samd51.h>
#else
/* as invoked by a certain ide, in case people want to use it to test modules in isolation */
#include <samd51/include/samd51.h>
#endif

#ifdef SERCOM_I2C
/* allow compiler invocation to override these macros */
#elif defined(ADAFRUIT_FEATHER_M0)
/* tested */
#define SERCOM_I2C SERCOM3
#define SERCOM_I2C_GCLK_ID_CORE SERCOM3_GCLK_ID_CORE
#define PM_APBCMASK_SERCOM_I2C PM_APBCMASK_SERCOM3
#define SCL_PORT_AND_PIN 0, 23
#define SCL_PINMUX_FUNC 2
#define SDA_PORT_AND_PIN 0, 22
#define SDA_PINMUX_FUNC 2

#elif defined(ARDUINO_QTPY_M0)
/* tested */
#define SERCOM_I2C SERCOM1
#define SERCOM_I2C_GCLK_ID_CORE SERCOM1_GCLK_ID_CORE
#define PM_APBCMASK_SERCOM_I2C PM_APBCMASK_SERCOM1
#define SCL_PORT_AND_PIN 0, 17
#define SCL_PINMUX_FUNC 2
#define SDA_PORT_AND_PIN 0, 16
#define SDA_PINMUX_FUNC 2

#elif defined(ARDUINO_TRINKET_M0)
/* TODO: NOT tested. SHOULD work with trinket silkscreen pins 0 and 2 for sda and scl */
#define SERCOM_I2C SERCOM2
#define SERCOM_I2C_GCLK_ID_CORE SERCOM2_GCLK_ID_CORE
#define PM_APBCMASK_SERCOM_I2C PM_APBCMASK_SERCOM2
#define SCL_PORT_AND_PIN 0, 9
#define SCL_PINMUX_FUNC 3
#define SDA_PORT_AND_PIN 0, 8
#define SDA_PINMUX_FUNC 3

#elif defined(ADAFRUIT_FEATHER_M4_EXPRESS)
/* tested */
#define SERCOM_I2C SERCOM2
#define SERCOM_I2C_GCLK_ID_CORE SERCOM2_GCLK_ID_CORE
#define SERCOM_I2C_APBMASK_DEST APBBMASK
#define SERCOM_I2C_APBMASK_VAL MCLK_APBBMASK_SERCOM2
#define SCL_PORT_AND_PIN 0, 13
#define SCL_PINMUX_FUNC 2
#define SDA_PORT_AND_PIN 0, 12
#define SDA_PINMUX_FUNC 2
#endif

static void pinmux(const unsigned port, const unsigned pin, const unsigned func) {
    PORT->Group[port].PMUX[pin >> 1].reg = pin % 2 ?
    ((PORT->Group[port].PMUX[pin >> 1].reg) & PORT_PMUX_PMUXE(0xF)) | PORT_PMUX_PMUXO(func) :
    ((PORT->Group[port].PMUX[pin >> 1].reg) & PORT_PMUX_PMUXO(0xF)) | PORT_PMUX_PMUXE(func);
    PORT->Group[port].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;
}

static void pin_set(const unsigned port, const unsigned pin, const unsigned value) {
    PORT->Group[port].PINCFG[pin].reg = PORT_PINCFG_DRVSTR;
    PORT->Group[port].DIRSET.reg = 1U << pin;
    if (value) PORT->Group[port].OUTSET.reg = 1U << pin;
    else PORT->Group[port].OUTCLR.reg = 1U << pin;
}

int i2c_init(struct i2c_state * state, unsigned long now, unsigned long us_per_tick) {
    if (0 == state->state) {
        /* disable the sercom prior to bitbanging */
        SERCOM_I2C->I2CM.CTRLA.bit.ENABLE = 0;
        while (SERCOM_I2C->I2CM.SYNCBUSY.bit.ENABLE);

        pin_set(SCL_PORT_AND_PIN, 1);
    }

    if (state->state <= 18) {
        /* clock out nine bits of whatever leftover state the other end may have been sending */
        if (now - state->prev < (20000 + us_per_tick / 2) / us_per_tick) return 1;
        pin_set(SCL_PORT_AND_PIN, !(state->state % 2)); /* lower pin in odd state, raise in even */
    }

    if (state->state < 19) {
        /* reached upon completing any state but the last */
        state->prev = now;
        state->state++;
        return 1;
    } else {
        /* final state */

        /* set up pinmux using board-specific macros */
        pinmux(SDA_PORT_AND_PIN, SDA_PINMUX_FUNC);
        pinmux(SCL_PORT_AND_PIN, SCL_PINMUX_FUNC);

#ifdef __SAMD51__
        MCLK->SERCOM_I2C_APBMASK_DEST.reg |= SERCOM_I2C_APBMASK_VAL;

        GCLK->PCHCTRL[SERCOM_I2C_GCLK_ID_CORE].bit.CHEN = 0;
        while (GCLK->PCHCTRL[SERCOM_I2C_GCLK_ID_CORE].bit.CHEN);

        GCLK->PCHCTRL[SERCOM_I2C_GCLK_ID_CORE].reg = (F_CPU == 48000000 ? GCLK_PCHCTRL_GEN_GCLK0 : GCLK_PCHCTRL_GEN_GCLK1) | GCLK_PCHCTRL_CHEN;
        while (!GCLK->PCHCTRL[SERCOM_I2C_GCLK_ID_CORE].bit.CHEN);
#else
        PM->APBCMASK.reg |= PM_APBCMASK_SERCOM_I2C;

        /* set up clock */
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM_I2C_GCLK_ID_CORE) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
        while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
#endif

        /* reset the sercom */
        SERCOM_I2C->I2CM.CTRLA.bit.SWRST = 1;
        while (SERCOM_I2C->I2CM.CTRLA.bit.SWRST || SERCOM_I2C->I2CM.SYNCBUSY.bit.SWRST);

        /* TODO: figure out what else is needed for i2c in standby on samd21, works fine on samd51 */
        SERCOM_I2C->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE(0x5u) | SERCOM_I2CM_CTRLA_RUNSTDBY | SERCOM_I2CM_CTRLA_LOWTOUTEN | SERCOM_I2CM_CTRLA_SEXTTOEN | SERCOM_I2CM_CTRLA_MEXTTOEN | SERCOM_I2CM_CTRLA_INACTOUT(2);

        /* assume system/peripheral clock is 48 MHz */
        const unsigned long sysclock = 48000000UL, baudrate = 100000UL;

        /* this is inexact for samd21 but not by enough to stress about it */
        SERCOM_I2C->I2CM.BAUD.bit.BAUD = sysclock / (2 * baudrate) - 1;

        /* re-enable the sercom */
        SERCOM_I2C->I2CM.CTRLA.bit.ENABLE = 1;
        while (SERCOM_I2C->I2CM.SYNCBUSY.bit.ENABLE);

        /* set the bus state to idle */
        SERCOM_I2C->I2CM.STATUS.bit.BUSSTATE = 1;
        while (SERCOM_I2C->I2CM.SYNCBUSY.bit.SYSOP);

        /* reached upon completing the last state */
        state->state = 0;
        return 0;
    }
}

static int wait_for_mb_or_error(void) {
    /* if ack has not arrived... */
    if (!SERCOM_I2C->I2CM.INTFLAG.bit.MB) {
        /* check for bus error */
        if (SERCOM_I2C->I2CM.STATUS.bit.BUSERR) {
            /* send stop command and wait for sync */
            SERCOM_I2C->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(0x3);
            while (SERCOM_I2C->I2CM.SYNCBUSY.bit.SYSOP);

            /* parent should react to to this by resetting its state */
            return -1;
        }

        /* TODO: check for other possible error states? */

        /* otherwise we are still just waiting for ack */
        else return 1;
    }
    else return 0;
}

int i2c_write_one_byte(struct i2c_state * state, uint8_t byte, uint8_t addr, uint8_t reg) {
    /* TODO: return -1 due to a number of possible error states */
    if (0 == state->state) {
        /* wait until bus state is idle or owner */
        if (SERCOM_I2C->I2CM.STATUS.bit.BUSSTATE != 1 && SERCOM_I2C->I2CM.STATUS.bit.BUSSTATE != 2) return 1;

        /* write address and wait for acknowledgment */
        SERCOM_I2C->I2CM.ADDR.bit.ADDR = addr << 1 | 0;
    }

    else if (1 == state->state) {
        int ret = wait_for_mb_or_error();
        if (ret) return ret;

        /* write register and wait for acknowledgment */
        SERCOM_I2C->I2CM.DATA.bit.DATA = reg;
    }

    else if (2 == state->state) {
        int ret = wait_for_mb_or_error();
        if (ret) return ret;

        /* write data byte and wait for acknowledgment */
        SERCOM_I2C->I2CM.DATA.bit.DATA = byte;
    }

    if (state->state < 3) {
        /* reached upon completing any state but the last */
        state->state++;
        return 1;
    } else {
        /* last state */
        int ret = wait_for_mb_or_error();
        if (ret) return ret;

        /* send stop command and wait for sync */
        SERCOM_I2C->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(0x3);
        while (SERCOM_I2C->I2CM.SYNCBUSY.bit.SYSOP);

        /* reached upon completing the last state */
        state->state = 0;
        return 0;
    }
}

int i2c_read_from_register(struct i2c_state * state, void * outv, size_t count, uint8_t addr, uint8_t reg) {
    /* TODO: return -1 due to a number of possible error states */
    unsigned char * out = (unsigned char *)outv;

    if (0 == state->state) {
        if (SERCOM_I2C->I2CM.STATUS.bit.BUSSTATE != 1 && SERCOM_I2C->I2CM.STATUS.bit.BUSSTATE != 2) return 1;
        /* write i2c device address and wait for ack */
        SERCOM_I2C->I2CM.ADDR.bit.ADDR = addr << 1 | 0;
    }

    else if (1 == state->state) {
        int ret = wait_for_mb_or_error();
        if (ret) return ret;

        /* write register address and wait for ack */
        SERCOM_I2C->I2CM.DATA.bit.DATA = reg;
    }

    else if (2 == state->state) {
        int ret = wait_for_mb_or_error();
        if (ret) return ret;

        /* write address again with LSB set, which also sends a repeated-start condition*/
        SERCOM_I2C->I2CM.ADDR.bit.ADDR = addr << 1 | 1;
    }

    else if (state->state >= 3 && state->state < 3 + count) {
        /* wait for the next byte and read it */
        if (!SERCOM_I2C->I2CM.INTFLAG.bit.SB) {
            /* if other end has unexpectedly NACKed the address... */
            if (SERCOM_I2C->I2CM.INTFLAG.bit.MB) {
                SERCOM_I2C->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(0x3);
                while (SERCOM_I2C->I2CM.SYNCBUSY.bit.SYSOP);

                /* parent should react to to this by resetting its state */
                return -1;
            }

            /* otherwise we are still just waiting for the ACK */
            return 1;
        }

        const size_t ibyte = state->state - 3;
        out[ibyte] = SERCOM_I2C->I2CM.DATA.bit.DATA;

        if (ibyte + 1 == count) {
            /* nack the byte we just received, and send a stop condition */
            SERCOM_I2C->I2CM.CTRLB.bit.ACKACT = 1;
            SERCOM_I2C->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(0x3);
        } else {
            /* ack the byte we just received, and send a read condition */
            SERCOM_I2C->I2CM.CTRLB.bit.ACKACT = 0;
            SERCOM_I2C->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(0x2);
        }
        while (SERCOM_I2C->I2CM.SYNCBUSY.bit.SYSOP);
    }

    if (state->state < 3 + count) {
        state->state++;
        return 1;
    } else {
        state->state = 0;
        return 0;
    }
}
