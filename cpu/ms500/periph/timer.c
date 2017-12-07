/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *               2015 FreshTemp, LLC.
 *               2014 Freie Universität Berlin
 *
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     driver_periph
 * @{
 *
 * @file        timer.c
 * @brief       Low-level timer driver implementation
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include <stdlib.h>
#include <stdio.h>

#include "board.h"
#include "cpu.h"

#include "periph/timer.h"
#include "periph_conf.h"
#include "xtimer.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/**
 * @brief Timer state memory
 */
static timer_isr_ctx_t tmr_ctx[TIMER_NUMOF] = { {NULL, NULL}, };
static uint32_t compensate_cnt[TIMER_NUMOF] = { 0, };

#define is_capture_mode(dev,channel) ((timer_config[dev].param[channel].f.trig_mode == 1))
#define is_matchout_mode(dev,channel) ((timer_config[dev].param[channel].f.out_mode > 0))

void timer_init_pins(tim_t dev)
{
	for (int i = 0; i < timer_config[dev].channels; i++) {
		if (is_capture_mode(dev, i))
			gpio_init_mux(timer_config[dev].cap_pin[i], timer_config[dev].mux);

		if (is_matchout_mode(dev, i))
			gpio_init_mux(timer_config[dev].mat_pin[i], timer_config[dev].mux);
	}

	CLKRST->peri_rst = GPTx_CLK_EN(dev);
	while ( !(CLKRST->peri_rst & GPTx_CLK_EN(dev)) );
}

/**
 * @brief Setup the given timer
 */
int timer_init(tim_t dev, unsigned long freq, timer_cb_t cb, void *arg)
{
	tGPT_REG *reg;
	uint32_t pres;
	unsigned long desired_freq;

	if (dev >= TIMER_NUMOF)
		return -1;

	if (dev == XTIMER_DEV)
		desired_freq = XTIMER_HZ;
	else
		desired_freq = freq;

	for (pres=0; pres<16; pres++) {
		if ((CLOCK_APB >> pres) == desired_freq)
			break;
	}
	if (pres >= 16)
		return -1;

	timer_init_pins(dev);

	reg = timer_config[dev].reg;
	for (int i = 0; i < timer_config[dev].channels; i++) {
		reg->tmr_ctrl = 0 |
			((0 << 0) & TIMER_BASE_CNT_RST) |
			((i << 1) & TIMER_BASE_MST_SEL) |
			((0 << 2) & TIMER_BASE_LINK_MODE) |
			((pres << 4) & TIMER_BASE_PRESCALER);
	}
	reg->tmr_int_en = 0;

	reg->tmr1_ctrl = timer_config[dev].param[0].raw;
	reg->tmr2_ctrl = timer_config[dev].param[1].raw;

	NVIC_DisableIRQ(TIMER1_IRQn + dev);

	tmr_ctx[dev].cb = cb;
	tmr_ctx[dev].arg = arg;

	NVIC_ClearPendingIRQ(TIMER1_IRQn + dev);
	NVIC_EnableIRQ(TIMER1_IRQn + dev);

	timer_start(dev);

	return 0;
}

int timer_set(tim_t dev, int channel, unsigned int timeout)
{
	return timer_set_absolute(dev, channel, timer_read(dev) + timeout);
}

int timer_set_absolute(tim_t dev, int channel, unsigned int value)
{
	if (dev >= TIMER_NUMOF)
		return -1;

	DEBUG("Setting timer %i channel %i to %i\n", dev, channel, value);

	tGPT_REG *reg = timer_config[dev].reg;

	compensate_cnt[dev] = (compensate_cnt[dev] + reg->tmr1_cnt) & 0xFFFF;

	switch (channel) {
		case 0:
			reg->tmr1_match = ((int32_t)value - (int32_t)compensate_cnt[dev]) & 0xFFFF;
			reg->tmr_int_en |= (is_capture_mode(dev,0)?TIMER1_CAP_INTR_EN:TIMER1_MAT_INTR_EN);
			break;
		case 1:
			reg->tmr2_match = ((int32_t)value - (int32_t)compensate_cnt[dev]) & 0xFFFF;
			reg->tmr_int_en |= (is_capture_mode(dev,1)?TIMER2_CAP_INTR_EN:TIMER2_MAT_INTR_EN);
			break;
		default:
			return -1;
	}

	return 1;
}

int timer_clear(tim_t dev, int channel)
{
	if (dev >= TIMER_NUMOF)
		return -1;

	tGPT_REG *reg = timer_config[dev].reg;

	switch (channel) {
		case 0:
			reg->tmr_int_en &= (is_capture_mode(dev,0)?(~TIMER1_CAP_INTR_EN):(~TIMER1_MAT_INTR_EN));
			break;
		case 1:
			reg->tmr_int_en &= (is_capture_mode(dev,1)?(~TIMER2_CAP_INTR_EN):(~TIMER2_MAT_INTR_EN));
			break;
		default:
			return -1;
	}

	return 1;
}

unsigned int timer_read(tim_t dev)
{
	if (dev >= TIMER_NUMOF)
		return 0;

	return (compensate_cnt[dev] + timer_config[dev].reg->tmr1_cnt) & 0xFFFF;
}

void timer_stop(tim_t dev)
{
	if (dev >= TIMER_NUMOF)
		return;

	tGPT_REG *reg = timer_config[dev].reg;

	for (int i = 0; i < timer_config[dev].channels; i++)
		timer_clear(dev, i);

	switch(timer_config[dev].channels)
	{
		case 1:
			reg->tmr_en &= ~TIMER1_EN;
			break;
		case 2:
			reg->tmr_en &= ~(TIMER1_EN | TIMER2_EN);
			break;
	}
}

void timer_start(tim_t dev)
{
	if (dev >= TIMER_NUMOF)
		return;

	tGPT_REG *reg = timer_config[dev].reg;

	switch(timer_config[dev].channels)
	{
		case 1:
			reg->tmr_en |= TIMER1_EN;
			break;
		case 2:
			reg->tmr_en |= (TIMER1_EN | TIMER2_EN);
			break;
	}
}

static inline void irq_handler(tim_t dev)
{
	tGPT_REG *reg = timer_config[dev].reg;

	/* 
	 * First of all, write 0xFFFF to the match register
	 * Then
	 * 1) the counter reset to 0
	 * 2) the counter register counts up to 0xFFFF
	 */
	compensate_cnt[dev] = (compensate_cnt[dev] + reg->tmr1_match) & 0xFFFF;
	reg->tmr1_match = 0xFFFF;

	uint32_t status = reg->tmr_int_sts;
	uint32_t channel_mask[2] = {
		TIMER1_CAP_INTR_EN | TIMER1_MAT_INTR_EN,
		TIMER2_CAP_INTR_EN | TIMER2_MAT_INTR_EN };

	for (int i = 0; i < 2; i++) {
		if (status & channel_mask[i]) {
			timer_clear(dev, i);
			if (tmr_ctx[dev].cb)
				tmr_ctx[dev].cb(tmr_ctx[dev].arg, i);
		}
	}

	reg->tmr_int_sts = status; // interrupt clear

	cortexm_isr_end();
}

#ifdef TIMER_0_ISR
void TIMER_0_ISR(void)
{
	irq_handler(0);
}
#endif

#ifdef TIMER_1_ISR
void TIMER_1_ISR(void)
{
	irq_handler(1);
}
#endif

