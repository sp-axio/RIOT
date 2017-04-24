/*
 * Copyright (C) 2014 CLENET Baptiste
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup  cpu_ms500
 * @{
 * @file
 * @brief       Low-level I2C driver implementation
 * @author      Baptiste Clenet <baptiste.clenet@xsoen.com>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @}
 */

#include <stdint.h>

#include "cpu.h"
#include "board.h"
#include "mutex.h"
#include "periph_conf.h"
#include "periph/i2c.h"

#include "sched.h"
#include "thread.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/* guard file in case no I2C device is defined */
#ifdef I2C_NUMOF

#define SAMD21_I2C_TIMEOUT  (65535)

#define BUSSTATE_UNKNOWN SERCOM_I2CM_STATUS_BUSSTATE(0)
#define BUSSTATE_IDLE SERCOM_I2CM_STATUS_BUSSTATE(1)
#define BUSSTATE_OWNER SERCOM_I2CM_STATUS_BUSSTATE(2)
#define BUSSTATE_BUSY SERCOM_I2CM_STATUS_BUSSTATE(3)

/* static function definitions */
//static void _i2c_poweron(SercomI2cm *sercom);
//static void _i2c_poweroff(SercomI2cm *sercom);

static int _start(tI2C_REG *reg, uint8_t address, uint8_t rw_flag);
static inline int _write(tI2C_REG *reg, const uint8_t *data, int length);
static inline int _read(tI2C_REG *reg, uint8_t *data, int length);
static inline void _stop(tI2C_REG *reg);
//static inline int _wait_for_response(SercomI2cm *dev, uint32_t max_timeout_counter);

/**
 * @brief Array holding one pre-initialized mutex for each I2C device
 */
static mutex_t locks[] = {
#if I2C_0_EN
    [I2C_0] = MUTEX_INIT,
#endif
#if I2C_1_EN
    [I2C_1] = MUTEX_INIT,
#endif
#if I2C_2_EN
    [I2C_2] = MUTEX_INIT
#endif
#if I2C_3_EN
    [I2C_3] = MUTEX_INIT
#endif
};

void i2c_init_pins(i2c_t dev)
{
	gpio_init_mux(i2c_config[dev].scl_pin, i2c_config[dev].mux);
	gpio_init_mux(i2c_config[dev].sda_pin, i2c_config[dev].mux);

	CLKRST->peri_rst = I2Cx_CLK_EN(dev);
	while ( !(CLKRST->peri_rst & I2Cx_CLK_EN(dev)) );
}

int i2c_init_master(i2c_t dev, i2c_speed_t speed)
{
	uint32_t scl, pres;
	tI2C_REG *ir;

    if ((unsigned int)dev >= I2C_NUMOF) {
        return -1;
    }

	switch (speed) {
        case I2C_SPEED_LOW:
			scl = 10000; // 10KHz
			pres = CLOCK_CORECLOCK / (5 * scl) - 1; // 960 - 1
			break;
        case I2C_SPEED_NORMAL:
			scl = 100000; // 100KHz
			pres = CLOCK_CORECLOCK / (5 * scl) - 1; // 96 - 1
            break;
        case I2C_SPEED_FAST:
			scl = 400000;  // 400KHz
			pres = CLOCK_CORECLOCK / (5 * scl) - 1; // 24 - 1
            break;
        case I2C_SPEED_FAST_PLUS:
			scl = 960000; // 960KHz
			pres = CLOCK_CORECLOCK / (5 * scl) - 1; // 10 - 1
			break;
        case I2C_SPEED_HIGH:
			scl = 3200000; // 3.2MHz
			pres = CLOCK_CORECLOCK / (5 * scl) - 1; // 3 - 1
			break;
        default:
            return -2;
	}

	i2c_init_pins(dev);

	ir = i2c_config[dev].reg;
	ir->pres_lo = pres & 0xff;
	ir->pres_hi = (pres & 0xff00) >> 8;

	ir->ctrl |= I2C_CTRL_EN;

    return 0;
}

int i2c_acquire(i2c_t dev)
{
	if (dev >= I2C_NUMOF) {
		return -1;
	}
	mutex_lock(&locks[dev]);
	return 0;
}

int i2c_release(i2c_t dev)
{
	if (dev >= I2C_NUMOF) {
		return -1;
	}
	mutex_unlock(&locks[dev]);
	return 0;
}

int i2c_read_byte(i2c_t dev, uint8_t address, void *data)
{
	return i2c_read_bytes(dev, address, data, 1);
}

int i2c_read_bytes(i2c_t dev, uint8_t address, void *data, int length)
{
	tI2C_REG *ireg;

	if (dev >= I2C_NUMOF) {
		return -1;
	}
	ireg = i2c_config[dev].reg;

	/* start transmission and send slave address */
	if (_start(ireg, address, I2C_FLAG_READ) < 0) {
		return 0;
	}
	/* read data to register */
	if (_read(ireg, data, length) < 0) {
		return 0;
	}
	_stop(ireg);

	/* return number of bytes sent */
	return length;
}

int i2c_read_reg(i2c_t dev, uint8_t address, uint8_t reg, void *data)
{
	return i2c_read_regs(dev, address, reg, data, 1);
}

int i2c_read_regs(i2c_t dev, uint8_t address, uint8_t reg, void *data, int length)
{
	tI2C_REG *ireg;

	if (dev >= I2C_NUMOF) {
		return -1;
	}
	ireg = i2c_config[dev].reg;

	/* start transmission and send slave address */
	if (_start(ireg, address, I2C_FLAG_WRITE) < 0) {
		return 0;
	}
	/* send register address/command and wait for complete transfer to
	 * be finished */
	if (_write(ireg, &reg, 1) < 0) {
		return 0;
	}
	return i2c_read_bytes(dev, address, data, length);
}

int i2c_write_byte(i2c_t dev, uint8_t address, uint8_t data)
{
	return i2c_write_bytes(dev, address, &data, 1);
}

int i2c_write_bytes(i2c_t dev, uint8_t address, const void *data, int length)
{
	tI2C_REG *ireg;

	if (dev >= I2C_NUMOF) {
		return -1;
	}
	ireg = i2c_config[dev].reg;

	if (_start(ireg, address, I2C_FLAG_WRITE) < 0) {
		return 0;
	}
	if (_write(ireg, data, length) < 0) {
		return 0;
	}
	_stop(ireg);

	return length;
}


int i2c_write_reg(i2c_t dev, uint8_t address, uint8_t reg, uint8_t data)
{
	return i2c_write_regs(dev, address, reg, &data, 1);
}

int i2c_write_regs(i2c_t dev, uint8_t address, uint8_t reg, const void *data, int length)
{
	tI2C_REG *ireg;

	if (dev >= I2C_NUMOF) {
		return -1;
	}
	ireg = i2c_config[dev].reg;

	/* start transmission and send slave address */
	if (_start(ireg, address, I2C_FLAG_WRITE) < 0) {
		return 0;
	}
	/* send register address and wait for complete transfer to be finished */
	if (_write(ireg, &reg, 1) < 0) {
		return 0;
	}
	/* write data to register */
	if (_write(ireg, data, length) < 0) {
		return 0;
	}
	/* finish transfer */
	_stop(ireg);

	return length;
}

void i2c_poweron(i2c_t dev)
{
}

void i2c_poweroff(i2c_t dev)
{
}

static int _start(tI2C_REG *reg, uint8_t address, uint8_t rw_flag)
{
	/* Wait for hardware module to sync */
	DEBUG("Wait for device to be ready\n");

	reg->txrx = ((address<<1)|rw_flag);
	reg->cmdstate = I2C_CMD_START | I2C_CMD_WRITE;

	while((reg->cmdstate&I2C_STATE_TIP));
	while((reg->cmdstate&I2C_STATE_RXACK));

	return 0;
}

static inline int _write(tI2C_REG* reg, const uint8_t *data, int length)
{
	DEBUG("Looping through bytes\n");

	for (int i = 0; i < length; i++) {
		reg->txrx = data[i++];
		reg->cmdstate = I2C_CMD_WRITE;
		while((reg->cmdstate&I2C_STATE_TIP));
		while((reg->cmdstate&I2C_STATE_RXACK));
	}

	return 0;
}

static inline int _read(tI2C_REG *reg, uint8_t *data, int length)
{
	DEBUG("Looping through bytes\n");

	for (int i = 0; i < length; i++) {
		reg->cmdstate = I2C_CMD_READ;
		while((reg->cmdstate&I2C_STATE_TIP));
		while((reg->cmdstate&I2C_STATE_RXACK));
		data[i] = reg->txrx;
	}
#if 1
	reg->cmdstate = I2C_CMD_READ | I2C_CMD_SENDNACK;
	while((reg->cmdstate&I2C_STATE_TIP));
#endif
	return 0;
}

static inline void _stop(tI2C_REG *reg)
{
	reg->cmdstate = I2C_CMD_STOP;
	__NOP();
	while((reg->cmdstate&(I2C_STATE_TIP|I2C_STATE_BUSBUSY)));
}

#endif /* I2C_NUMOF */
