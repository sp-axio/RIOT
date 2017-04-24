/*
 * Copyright (C) 2014-2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     driver_periph
 * @{
 *
 * @file
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Troels Hoffmeyer <troels.d.hoffmeyer@gmail.com>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"
#include "periph/gpio.h"
#include "periph_conf.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @brief   Number of external interrupt lines
 */
#define NUMOF_IRQS                  (16U)

/**
 * @brief   Mask to get PINCFG reg value from mode value
 */
#define MODE_PINCFG_MASK            (0x06)

/**
 * @brief Extract the pin's port base address from the given pin identifier
 */
static inline tGPIO_REG *_port(gpio_t pin)
{
	return (tGPIO_REG *)(pin & ~(0xf));
}

/**
 * @brief   Extract the port number form the given identifier
 */
static inline int _port_num(gpio_t pin)
{
	return ((pin >> 12) & 0x3);
}

/**
 * @brief   Extract the pin number from the last 4 bit of the pin identifier
 */
static inline int _pin_num(gpio_t pin)
{
    return (pin & 0x0f);
}

static gpio_isr_ctx_t gpio_ctx[PORT_MAX][NUMOF_IRQS];

static inline void gpio_ctx_clear(gpio_t pin)
{
	uint32_t port_num = _port_num(pin);
	uint32_t pin_num = _pin_num(pin);
	gpio_ctx[port_num][pin_num].cb = NULL;
	gpio_ctx[port_num][pin_num].arg = NULL;
}


static uint32_t pin_af_allocated[3] = { 0, 0, 0 };

/**
 * @brief   Hold one interrupt context per interrupt line
 */
void gpio_init_mux(gpio_t pin, gpio_mux_t mux)
{
	uint32_t port_num = _port_num(pin);
	uint32_t pin_num = _pin_num(pin);
	uint32_t pin_num_hi = (pin_num & 0x8) >> 3;
	uint32_t pin_num_lo = pin_num & 0x7;

#if 1 // check conflict
	if (pin_af_allocated[port_num] & (1 << pin_num)) {
		assert(0);
		return;
	}
	pin_af_allocated[port_num] |= (1 << pin_num);
#endif

	volatile uint32_t* sysconPort = (volatile uint32_t*)(&SYSCON->pa_port_mode + port_num);
	volatile uint32_t* sysconMf   = (volatile uint32_t*)(&SYSCON->pa_mfsel_lo + (port_num<<1));

	sysconPort[0] |= (1 << pin_num);
	sysconMf[pin_num_hi] |= (mux << (pin_num_lo << 2));
}

int gpio_init(gpio_t pin, gpio_mode_t mode)
{
	tGPIO_REG *reg = _port(pin);
	uint32_t port_num = _port_num(pin);
	uint32_t pin_num = _pin_num(pin);
	volatile uint32_t* sysconPort = (volatile uint32_t*)(&SYSCON->pa_port_mode + port_num);

	pin_af_allocated[port_num] &= ~(1 << pin_num);

	gpio_ctx_clear(pin);

	sysconPort[0] &= ~(1 << pin_num);

	switch (mode)
	{
		case GPIO_IN:
			reg->out_mode &= ~((1 << (pin_num + 16)) | (1 << pin_num));
			break;
		case GPIO_OUT:
			reg->out_mode |= (1 << pin_num);
			break;
		case GPIO_OD:
			reg->out_mode |= ((1 << (pin_num + 16)) | (1 << pin_num));
			break;
		default:
			return -1;
	}

	return 0;
}

int gpio_init_int(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank,
                    gpio_cb_t cb, void *arg)
{
	tGPIO_REG *reg = _port(pin);
	uint32_t port_num = _port_num(pin);
	uint32_t pin_num = _pin_num(pin);
	uint32_t tmp;

	reg->int_stsclr = (1 << pin_num);

	if (1) // edge-trigger
	{
		reg->int_type &= ~(1 << pin_num);

		tmp = reg->int_pol;
		switch (flank) {
			case GPIO_RISING:
				tmp |= (1 << pin_num);
				tmp &= ~(1 << (pin_num+16));
				break;
			case GPIO_FALLING:
				tmp &= ~(1 << pin_num);
				tmp &= ~(1 << (pin_num+16));
				break;
			case GPIO_BOTH:
				tmp |= (1<< (pin_num + 16));
				break;
			default:
				return -1;
		}
		reg->int_pol = tmp;
	}
	else // level-trigger (not supported in the RIOT)
	{
		reg->int_type |= (1 << pin_num); 

		if (1) // level sensitive mode: high
			reg->int_pol |= (1 << pin_num);
		else // level sensitive mode: low
			reg->int_pol &= ~(1 << pin_num);
	}

	if (1) // or-mode
	{
		reg->int_en |= (1 << (pin_num + 16));
	}
	else // and-mode (not supported in the RIOT)
	{
		reg->int_en &= ~(1 << (pin_num + 16));
	}

	gpio_irq_enable(pin);

	gpio_init(pin, GPIO_IN);

	gpio_ctx[port_num][pin_num].cb = cb;
	gpio_ctx[port_num][pin_num].arg = arg;

	NVIC_ClearPendingIRQ(GPIO1_IRQn + port_num);
	NVIC_EnableIRQ(GPIO1_IRQn + port_num);

    return 0;
}

void gpio_irq_enable(gpio_t pin)
{
	_port(pin)->int_en |= (1 << _pin_num(pin));
}

void gpio_irq_disable(gpio_t pin)
{
	_port(pin)->int_en &= ~(1 << _pin_num(pin));
}

int gpio_read(gpio_t pin)
{
	return (_port(pin)->data & (1 << _pin_num(pin)))?1:0;
}

void gpio_set(gpio_t pin)
{
	tGPIO_REG *reg = _port(pin);
	reg->set_reset = 0;
	reg->data_out |= (1 << _pin_num(pin));
}

void gpio_clear(gpio_t pin)
{
	tGPIO_REG *reg = _port(pin);
	reg->set_reset = 0;
	reg->data_out &= ~(1 << _pin_num(pin));
}

void gpio_toggle(gpio_t pin)
{
	tGPIO_REG *reg = _port(pin);
	uint32_t pin_num = _pin_num(pin);

	reg->set_reset = 0;
	if (reg->data_out & (1 << pin_num))
		reg->data_out &= ~(1 << pin_num);
	else
		reg->data_out |= (1 << pin_num);
}

void gpio_write(gpio_t pin, int value)
{
	tGPIO_REG *reg = _port(pin);
	reg->set_reset = 0;
	if (value)
		reg->data_out |= (1 << _pin_num(pin));
	else
		reg->data_out &= ~(1 << _pin_num(pin));
}

static inline void irq_handler(tGPIO_REG *reg)
{
	uint32_t pin, status;
	uint32_t port_num = (((uint32_t)reg >> 12) & 0x3);

	status = reg->int_stsclr;

	for (pin=0; pin<16; pin++) {
		if (status & (1<<pin)) {
			if (gpio_ctx[port_num][pin].cb)
				gpio_ctx[port_num][pin].cb(gpio_ctx[port_num][pin].arg);
			reg->int_stsclr = 1<<pin;
		}
	}

    cortexm_isr_end();
}

void isr_gpio1(void)
{
	irq_handler(GPIO1);
}

void isr_gpio2(void)
{
	irq_handler(GPIO2);
}

void isr_gpio3(void)
{
	irq_handler(GPIO3);
}
