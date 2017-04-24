/*
 * Copyright (C) 2014-2016 Freie Universität Berlin
 *               2015 Kaspar Schleiser <kaspar@schleiser.de>
 *               2015 FreshTemp, LLC.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup    
 * @{
 *
 * @file
 * @brief       Low-level SPI driver implementation
 *
 * @author
 *
 * @}
 */

#include "cpu.h"
#include "mutex.h"
#include "assert.h"
#include "periph/spi.h"

/**
 * @brief Array holding one pre-initialized mutex for each SPI device
 */
static mutex_t locks[SPI_NUMOF];

static void spi1_2_init(spi_t dev)
{
	tSPI_REG *reg = spi_config[dev].reg;
	spi_param_t param = spi_config[dev].param;

	if (param.f.is_master)
		reg->cr1 &= ~(SPI_CR1_MS);
	else
		reg->cr1 |= (SPI_CR1_MS);

	/* SPI Frame Format */
	reg->cr0 &= ~(SPI_CR0_FRF_MASK<<4);
	reg->cr0 |= SPI_CR0_FRF(param.f.frf);

	/* SPI Data Size Select */
	reg->cr0 &= ~SPI_CR0_DSS_MASK;
	reg->cr0 |= SPI_CR0_DSS(param.f.dss);

	/* SPI Clock prescale divisor */
	reg->cpsr = 0x2; // 48MHz / 2 = 24MHz

	if (param.f.is_master)
		reg->cr1 |= SPI_CR1_SOD;
	else
		reg->cr1 &= ~(SPI_CR1_SOD);

	/* SPI Enable */
	reg->cr1 |= SPI_CR1_SSE;
}

static void spi3_init(spi_t dev)
{
	tSPI3_REG *reg = spi_config[dev].reg;
	spi_param_t param = spi_config[dev].param;

	if (param.f.is_master)
		reg->ctrl &= ~(SPI3_CR_MST_SLV);
	else
		reg->ctrl |= (SPI3_CR_MST_SLV);

	/* SPI Frame Format */
	reg->ctrl &= ~(SPI3_CR_FRM_FORMAT_MASK<<2);
	reg->ctrl |= SPI3_CR_FRM_FORMAT(param.f.frf);

	/* SPI Data Size Select */
	reg->ctrl &= ~(SPI3_CR_DATA_SIZE_MASK<<8);
	reg->ctrl |= SPI3_CR_DATA_SIZE(param.f.dss);

	if (param.f.is_master)
	{
		/* SPI data output enable*/
		reg->ctrl &= ~SPI3_CR_SDO_EN;

		/* Slave-mode output disable */
		reg->ext_ctrl |= SPI3_EXT_SSEL_DIS;
	}
	else
	{
		/* SPI Clock prescale divisor */
		reg->ext_ctrl &= ~(SPI3_EXT_CLK_PRESC_MASK<<8);
		reg->ext_ctrl |= SPI3_EXT_CLK_PRESC(2);

		/* SPI data output disable */
		reg->ctrl |= SPI3_CR_SDO_EN;

		/* Slave-mode output enable */
		reg->ext_ctrl &= ~(SPI3_EXT_SSEL_DIS);
	}

	/* SPI Enable */
	reg->ctrl |= SPI3_CR_EN;
}

void spi_init(spi_t bus)
{
    /* make sure given bus is good */
    assert(bus < SPI_NUMOF);

    /* initialize the device lock */
    mutex_init(&locks[bus]);

    /* configure pins and their muxes */
	if (spi_config[bus].param.f.enable)
	{
		spi_init_pins(bus);

		if (bus == 2)
			spi3_init(bus);
		else
			spi1_2_init(bus);
	}
}

void spi_init_pins(spi_t bus)
{
    gpio_init_mux(spi_config[bus].clk_pin,  spi_config[bus].mux);
    gpio_init_mux(spi_config[bus].cs_pin,   spi_config[bus].mux);
    gpio_init_mux(spi_config[bus].miso_pin, spi_config[bus].mux);
    gpio_init_mux(spi_config[bus].mosi_pin, spi_config[bus].mux);

	CLKRST->peri_rst = SPIx_CLK_EN(bus);
	while ( !(CLKRST->peri_rst & SPIx_CLK_EN(bus)) );
}

int spi_acquire(spi_t bus, spi_cs_t cs, spi_mode_t mode, spi_clk_t clk)
{
    /* get exclusive access to the device */
    mutex_lock(&locks[bus]);

	if (bus == 2) {
		tSPI3_REG *reg = spi_config[bus].reg;

		switch (mode) {
			case SPI_MODE_0:
				reg->ctrl &= ~SPI3_CR_CLK_POL;
				reg->ctrl &= ~SPI3_CR_PHASE;
				break;
			case SPI_MODE_1:
				reg->ctrl &= ~SPI3_CR_CLK_POL;
				reg->ctrl |= SPI3_CR_PHASE;
				break;
			case SPI_MODE_2:
				reg->ctrl |= SPI3_CR_CLK_POL;
				reg->ctrl &= ~SPI3_CR_PHASE;
				break;
			case SPI_MODE_3:
				reg->ctrl |= SPI3_CR_CLK_POL;
				reg->ctrl |= SPI3_CR_PHASE;
				break;
			default:
				return SPI_NOMODE;
		}

		switch (clk) {
			case SPI_CLK_100KHZ:
				reg->ctrl &= ~(SPI3_CR_BAUDRATE_MASK<<12);
				reg->ctrl |= SPI3_CR_BAUDRATE(3);
				reg->ext_ctrl &= ~(SPI3_EXT_CLK_PRESC_MASK<<8);
				reg->ext_ctrl |= SPI3_EXT_CLK_PRESC(29); // 48MHz / (30 * 2^4) = 100KHz
				break;
			case SPI_CLK_400KHZ:
				reg->ctrl &= ~(SPI3_CR_BAUDRATE_MASK<<12);
				reg->ctrl |= SPI3_CR_BAUDRATE(1);
				reg->ext_ctrl &= ~(SPI3_EXT_CLK_PRESC_MASK<<8);
				reg->ext_ctrl |= SPI3_EXT_CLK_PRESC(29); // 48MHz / (30 * 2^2) = 400KHz
				break;
			case SPI_CLK_1MHZ:
				reg->ctrl &= ~(SPI3_CR_BAUDRATE_MASK<<12);
				reg->ctrl |= SPI3_CR_BAUDRATE(3);
				reg->ext_ctrl &= ~(SPI3_EXT_CLK_PRESC_MASK<<8);
				reg->ext_ctrl |= SPI3_EXT_CLK_PRESC(2); // 48MHz / (3 * 2^4) = 1MHz
				break;
			case SPI_CLK_5MHZ:
				reg->ctrl &= ~(SPI3_CR_BAUDRATE_MASK<<12);
				reg->ctrl |= SPI3_CR_BAUDRATE(0);
				reg->ext_ctrl &= ~(SPI3_EXT_CLK_PRESC_MASK<<8);
				reg->ext_ctrl |= SPI3_EXT_CLK_PRESC(4); // 48MHz / (5 * 2^1) = 4.8MHz
				break;
			case SPI_CLK_10MHZ:
				reg->ctrl &= ~(SPI3_CR_BAUDRATE_MASK<<12);
				reg->ctrl |= SPI3_CR_BAUDRATE(0);
				reg->ext_ctrl &= ~(SPI3_EXT_CLK_PRESC_MASK<<8);
				reg->ext_ctrl |= SPI3_EXT_CLK_PRESC(2); // 48MHz / (3 * 2^1) = 8MHz
				break;
			default:
				return SPI_NOCLK;
		}
	}
	else {
		tSPI_REG *reg = spi_config[bus].reg;

		switch (mode) {
			case SPI_MODE_0:
				reg->cr0 &= ~SPI_CR0_SPO;
				reg->cr0 &= ~SPI_CR0_SPH;
				break;
			case SPI_MODE_1:
				reg->cr0 &= ~SPI_CR0_SPO;
				reg->cr0 |= SPI_CR0_SPH;
				break;
			case SPI_MODE_2:
				reg->cr0 |= SPI_CR0_SPO;
				reg->cr0 &= ~SPI_CR0_SPH;
				break;
			case SPI_MODE_3:
				reg->cr0 |= SPI_CR0_SPO;
				reg->cr0 |= SPI_CR0_SPH;
				break;
			default:
				return SPI_NOMODE;
		}

		switch (clk) {
			case SPI_CLK_100KHZ:
				reg->cr0 &= ~(SPI_CR0_SCR_MASK<<8);
				reg->cr0 |= SPI_CR0_SCR(239); // 24MHz / 240 = 100KHz
				break;
			case SPI_CLK_400KHZ:
				reg->cr0 &= ~(SPI_CR0_SCR_MASK<<8);
				reg->cr0 |= SPI_CR0_SCR(59); // 24MHz / 60 = 400KHz
				break;
			case SPI_CLK_1MHZ:
				reg->cr0 &= ~(SPI_CR0_SCR_MASK<<8);
				reg->cr0 |= SPI_CR0_SCR(23); // 24MHz / 24 = 1MHz
				break;
			case SPI_CLK_5MHZ:
				reg->cr0 &= ~(SPI_CR0_SCR_MASK<<8);
				reg->cr0 |= SPI_CR0_SCR(4); // 24MHz / 5 = 4.8MHz
				break;
			case SPI_CLK_10MHZ:
				reg->cr0 &= ~(SPI_CR0_SCR_MASK<<8);
				reg->cr0 |= SPI_CR0_SCR(2); // 24MHz / 3 = 8MHz
				break;
			default:
				return SPI_NOCLK;
		}
	}

    return SPI_OK;
}

void spi_release(spi_t bus)
{
    /* release access to the device */
    mutex_unlock(&locks[bus]);
}

static inline uint16_t spi3_inout(tSPI3_REG* reg, uint16_t data)
{
	while (!(reg->state & SPI3_STS_TX_FIFO_EMPTY)) {}
	reg->data = data;
	while (!(reg->state & SPI3_STS_RX_FIFO_RDY)) {}
	return reg->data;
}

static inline uint16_t spi1_2_inout(tSPI_REG* reg, uint16_t data)
{
	while (!(reg->sr & SPI_PSR_TNF)) {}
	reg->dr = data;
	while (!(reg->sr & SPI_PSR_RNE)) {}
	return reg->dr;
}

void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
                        const void *out, void *in, size_t len)
{
	uint8_t *out8_buf, *in8_buf;
	uint16_t *out16_buf, *in16_buf;
	uint16_t tmp;
	size_t i;

	assert(out || in);

	if (bus == 2 && cs != SPI_CS_UNDEF) {
		gpio_clear((gpio_t)cs);
	}

	if (spi_config[bus].param.f.dss > 7)
	{
		out16_buf = (uint16_t *)out;
		in16_buf = (uint16_t *)in;

		if (bus == 2) {
			for(i = 0; i < len; i++) {
				tmp = spi3_inout(spi_config[bus].reg, out16_buf ? out16_buf[i] : 0);
				if (in16_buf)
					in16_buf[i] = tmp;
			}
		}
		else {
			for (i = 0; i < len; i++) {
				tmp = spi1_2_inout(spi_config[bus].reg, out16_buf ? out16_buf[i] : 0);
				if (in16_buf)
					in16_buf[i] = tmp;
			}
		}
	}
	else
	{
		out8_buf = (uint8_t *)out;
		in8_buf = (uint8_t *)in;

		if (bus == 2) {
			for(i = 0; i < len; i++) {
				tmp = spi3_inout(spi_config[bus].reg, out8_buf ? out8_buf[i] : 0);
				if (in8_buf)
					in8_buf[i] = tmp & 0xff;
			}
		}
		else {
			for(i = 0; i < len; i++) {
				tmp = spi1_2_inout(spi_config[bus].reg, out8_buf ? out8_buf[i] : 0);
				if (in8_buf)
					in8_buf[i] = tmp & 0xff;
			}
		}
	}

	if ((bus == 2) && (!cont) && (cs != SPI_CS_UNDEF)) {
		gpio_set((gpio_t)cs);
	}
}

int spi_init_cs(spi_t bus, spi_cs_t cs)
{
	if (bus != 2)
		return SPI_NODEV;

	if (cs == SPI_CS_UNDEF || cs == GPIO_UNDEF)
		return SPI_NOCS;

	gpio_init((gpio_t)cs, GPIO_OUT);
	gpio_set((gpio_t)cs);

	return SPI_OK;
}

uint8_t spi_transfer_byte(spi_t bus, spi_cs_t cs, bool cont, uint8_t out)
{
	if (spi_config[bus].param.f.dss > 7) {
		uint16_t in16;
		uint16_t out16 = (uint16_t)out << 8;
		spi_transfer_bytes(bus, cs, cont, &out16, &in16, 1);
		return (uint8_t)(in16 & 0xff);
	}
	else {
		uint8_t in8;
		spi_transfer_bytes(bus, cs, cont, &out, &in8, 1);
		return in8;
	}
}

uint8_t spi_transfer_reg(spi_t bus, spi_cs_t cs, uint8_t reg, uint8_t out)
{
	if (spi_config[bus].param.f.dss > 7) {
		uint16_t in16;
		uint16_t out16 = (uint16_t)reg << 8;
		spi_transfer_bytes(bus, cs, false, &out16, &in16, 1);
		return (uint8_t)(in16 & 0xff);
	}
	else {
		spi_transfer_bytes(bus, cs, true, &reg, NULL, 1);
		return spi_transfer_byte(bus, cs, false, out);
	}
}

void spi_transfer_regs(spi_t bus, spi_cs_t cs, uint8_t reg, const void *out, void *in, size_t len)
{
	spi_transfer_bytes(bus, cs, true, &reg, NULL, 1);
	spi_transfer_bytes(bus, cs, false, out, in, len);
}

