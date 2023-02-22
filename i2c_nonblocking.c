#include "i2c_nonblocking.h"
#include <samd.h>

#ifdef SERCOM
/* allow compiler invocation to override these macros */
#elif defined(ADAFRUIT_FEATHER_M0)
/* tested */
#define SERCOM SERCOM3
#define SERCOM_GCLK_ID_CORE SERCOM3_GCLK_ID_CORE
#define SERCOM_IRQn SERCOM3_IRQn
#define SERCOM_HANDLER SERCOM3_Handler
#define SCL_PORT_AND_PIN 0, 23
#define SCL_PINMUX_FUNC 2
#define SDA_PORT_AND_PIN 0, 22
#define SDA_PINMUX_FUNC 2

#elif defined(ARDUINO_QTPY_M0)
/* tested */
#define SERCOM SERCOM1
#define SERCOM_GCLK_ID_CORE SERCOM1_GCLK_ID_CORE
#define SERCOM_IRQn SERCOM1_IRQn
#define SERCOM_HANDLER SERCOM1_Handler
#define SCL_PORT_AND_PIN 0, 17
#define SCL_PINMUX_FUNC 2
#define SDA_PORT_AND_PIN 0, 16
#define SDA_PINMUX_FUNC 2

#elif defined(ARDUINO_TRINKET_M0)
/* TODO: NOT tested. SHOULD work with trinket silkscreen pins 0 and 2 for sda and scl */
#define SERCOM SERCOM2
#define SERCOM_GCLK_ID_CORE SERCOM2_GCLK_ID_CORE
#define SERCOM_IRQn SERCOM2_IRQn
#define SERCOM_HANDLER SERCOM2_Handler
#define SCL_PORT_AND_PIN 0, 9
#define SCL_PINMUX_FUNC 3
#define SDA_PORT_AND_PIN 0, 8
#define SDA_PINMUX_FUNC 3

#elif defined(ADAFRUIT_FEATHER_M4_EXPRESS)
/* tested */
#define SERCOM SERCOM2
#define SERCOM_GCLK_ID_CORE SERCOM2_GCLK_ID_CORE
#define SERCOM_IRQn SERCOM2_0_IRQn
#define SERCOM_HANDLER SERCOM2_0_Handler
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

int i2c_init(struct i2c_state * state, unsigned long now) {
    if (0 == state->state) {
        /* disable the sercom */
        SERCOM->I2CM.CTRLA.bit.ENABLE = 0;
        while (SERCOM->I2CM.SYNCBUSY.bit.ENABLE);

        pin_set(SCL_PORT_AND_PIN, 1);
    }
    
    if (state->state <= 18) {
        if (now - state->prev < 20) return 1;
        pin_set(SCL_PORT_AND_PIN, !(state->state % 2)); /* lower pin in odd state, raise in even */
    }

    if (state->state < 19) {
        /* reached upon completing any state but the last */
        state->prev = now;
        state->state++;
        return 1;
    } else {
        /* final state */

        /* set up pinmux using a board-specific macro for all arguments */
        pinmux(SDA_PORT_AND_PIN, SDA_PINMUX_FUNC);
        pinmux(SCL_PORT_AND_PIN, SCL_PINMUX_FUNC);

        /* set up interrupt */
        NVIC_ClearPendingIRQ(SERCOM_IRQn);
        NVIC_SetPriority(SERCOM_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
        NVIC_EnableIRQ(SERCOM_IRQn);

#ifdef __SAMD51__
        GCLK->PCHCTRL[SERCOM_GCLK_ID_CORE].bit.CHEN = 0;
        while (GCLK->PCHCTRL[SERCOM_GCLK_ID_CORE].bit.CHEN);

        GCLK->PCHCTRL[SERCOM_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
        while (!GCLK->PCHCTRL[SERCOM_GCLK_ID_CORE].bit.CHEN);
#else
        /* set up clock */
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM_GCLK_ID_CORE) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
        while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
#endif

        /* reset the sercom */
        SERCOM->I2CM.CTRLA.bit.SWRST = 1;
        while (SERCOM->I2CM.CTRLA.bit.SWRST || SERCOM->I2CM.SYNCBUSY.bit.SWRST);

        /* TODO: figure out what else is needed for i2c in standby on samd21, works fine on samd51 */
        SERCOM->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE(0x5u) | SERCOM_I2CM_CTRLA_RUNSTDBY | SERCOM_I2CM_CTRLA_LOWTOUTEN | SERCOM_I2CM_CTRLA_SEXTTOEN | SERCOM_I2CM_CTRLA_MEXTTOEN | SERCOM_I2CM_CTRLA_INACTOUT(2);

        /* assume system/peripheral clock is 48 MHz */
        const unsigned long sysclock = 48000000UL, baudrate = 100000UL;

#ifdef __SAMD51__
        SERCOM->I2CM.BAUD.bit.BAUD = sysclock / ( 2 * baudrate) - 1;
#else
        /* samd21 cargo cult rise time stuff */
        SERCOM->I2CM.BAUD.bit.BAUD = sysclock / (2 * baudrate) - 5 - (((sysclock / 1000000) * 125) / (2 * 1000));
#endif

        /* enable the sercom */
        SERCOM->I2CM.CTRLA.bit.ENABLE = 1;
        while (SERCOM->I2CM.SYNCBUSY.bit.ENABLE);

        /* set the bus state to idle */
        SERCOM->I2CM.STATUS.bit.BUSSTATE = 1;
        while (SERCOM->I2CM.SYNCBUSY.bit.SYSOP);

        /* reached upon completing the last state */
        state->state = 0;
        return 0;
    }
}

int i2c_write_one_byte(struct i2c_state * state, uint8_t byte, uint8_t addr, uint8_t reg) {
    if (0 == state->state) {
        /* wait until bus state is idle or owner */
        if (SERCOM->I2CM.STATUS.bit.BUSSTATE != 1 && SERCOM->I2CM.STATUS.bit.BUSSTATE != 2) return 1;

        /* write address and wait for acknowledgment */
        SERCOM->I2CM.ADDR.bit.ADDR = addr << 1 | 0;
    }

    else if (1 == state->state) {
        if (!SERCOM->I2CM.INTFLAG.bit.MB) return 1;
        /* write register and wait for acknowledgment */
        SERCOM->I2CM.DATA.bit.DATA = reg;
    }

    else if (2 == state->state) {
        if (!SERCOM->I2CM.INTFLAG.bit.MB) return 1;
        /* write data byte and wait for acknowledgment */
        SERCOM->I2CM.DATA.bit.DATA = byte;
    }

    if (state->state < 3) {
        /* reached upon completing any state but the last */
        state->state++;
        return 1;
    } else {
        /* last state */
        if (!SERCOM->I2CM.INTFLAG.bit.MB) return 1;
        /* send stop command and wait for sync */
        SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(0x3);
        while (SERCOM->I2CM.SYNCBUSY.bit.SYSOP);

        /* reached upon completing the last state */
        state->state = 0;
        return 0;
    }
}

int i2c_read_from_register(struct i2c_state * state, void * outv, size_t count, uint8_t addr, uint8_t reg) {
    unsigned char * out = (unsigned char *)outv;

    if (0 == state->state) {
        if (SERCOM->I2CM.STATUS.bit.BUSSTATE != 1 && SERCOM->I2CM.STATUS.bit.BUSSTATE != 2) return 1;
        /* write i2c device address and wait for ack */
        SERCOM->I2CM.ADDR.bit.ADDR = addr << 1 | 0;
    }

    else if (1 == state->state) {
        if (!SERCOM->I2CM.INTFLAG.bit.MB) return 1;
        /* write register address and wait for ack */
        SERCOM->I2CM.DATA.bit.DATA = reg;
    }

    else if (2 == state->state) {
        if (!SERCOM->I2CM.INTFLAG.bit.MB) return 1;
        /* write address again with LSB set, which also sends a repeated-start condition*/
        SERCOM->I2CM.ADDR.bit.ADDR = addr << 1 | 1;
    }

    else if (state->state >= 3 && state->state < 3 + count) {
        /* wait for the next byte and read it */
        if (!SERCOM->I2CM.INTFLAG.bit.SB) return 1;

        const size_t ibyte = state->state - 3;
        out[ibyte] = SERCOM->I2CM.DATA.bit.DATA;

        if (ibyte + 1 == count) {
            /* nack the byte we just received, and send a stop condition */
            SERCOM->I2CM.CTRLB.bit.ACKACT = 1;
            SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(0x3);
        } else {
            /* ack the byte we just received, and send a read condition */
            SERCOM->I2CM.CTRLB.bit.ACKACT = 0;
            SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(0x2);
        }
        while (SERCOM->I2CM.SYNCBUSY.bit.SYSOP);
    }

    if (state->state < 3 + count) {
        state->state++;
        return 1;
    } else {
        state->state = 0;
        return 0;
    }
}
