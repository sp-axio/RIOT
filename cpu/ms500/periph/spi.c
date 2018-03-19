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
#ifdef MODULE_OD
#include "od.h"
#endif

/**
 * @brief Array holding one pre-initialized mutex for each SPI device
 */
static mutex_t locks[SPI_NUMOF];

__attribute__((weak)) void spi_isr_callback(int bus) {}

void isr_spi1(void)
{
	spi_param_t param = spi_config[0].param;
	if (!param.f.interrupt) {
		core_panic(PANIC_DUMMY_HANDLER, "SPI0 HANDLER");
	}
	spi_isr_callback(0);
	cortexm_isr_end();
}

void isr_spi2(void)
{
	spi_param_t param = spi_config[1].param;
	if (!param.f.interrupt) {
		core_panic(PANIC_DUMMY_HANDLER, "SPI1 HANDLER");
	}
	spi_isr_callback(1);
	cortexm_isr_end();
}

void isr_spi3(void)
{
	spi_param_t param = spi_config[2].param;
	if (!param.f.interrupt) {
		core_panic(PANIC_DUMMY_HANDLER, "SPI2 HANDLER");
	}
	tSPI3_REG *reg = spi_config[2].reg;
	spi_isr_callback(2);
	reg->int_sts_clr |= SPI3_INT_RX_CLR;
	cortexm_isr_end();
}

static void spi1_2_init(spi_t bus)
{
	tSPI_REG *reg = spi_config[bus].reg;
	spi_param_t param = spi_config[bus].param;

	if (param.f.is_master)
	{
		reg->cr1 &= ~(SPI_CR1_MS);
	}
	else
	{
		reg->cr1 |= (SPI_CR1_MS);
	}

	/* SPI Frame Format */
	reg->cr0 &= ~(SPI_CR0_FRF_MASK<<4);
	reg->cr0 |= SPI_CR0_FRF(param.f.frf);

	/* SPI Data Size Select */
	reg->cr0 &= ~SPI_CR0_DSS_MASK;
	reg->cr0 |= SPI_CR0_DSS(7); // 8bit

	if (param.f.is_master)
	{
		reg->cr1 |= (SPI_CR1_SSE | SPI_CR1_SOD);
	}
	else
	{
		reg->cr1 &= ~(SPI_CR1_SOD);
		reg->cr1 |= SPI_CR1_SSE;

		/* Enable Interrupt */
		if (param.f.interrupt)
		{
			reg->imsc |= SPI_IMSC_RXIM;
			NVIC_ClearPendingIRQ(SPI1_IRQn + bus);
			NVIC_EnableIRQ(SPI1_IRQn + bus);
		}
	}
}

static void spi3_init(spi_t bus)
{
	tSPI3_REG *reg = spi_config[bus].reg;
	spi_param_t param = spi_config[bus].param;

	if (param.f.is_master)
	{
		reg->ctrl &= ~(SPI3_CR_MST_SLV);
	}
	else
	{
		reg->ctrl |= (SPI3_CR_MST_SLV);
	}

	/* SPI Frame Format */
	reg->ctrl &= ~(SPI3_CR_FRM_FORMAT_MASK<<2);
	reg->ctrl |= SPI3_CR_FRM_FORMAT(param.f.frf);

	/* SPI Data Size Select */
	reg->ctrl &= ~(SPI3_CR_DATA_SIZE_MASK<<8);
	reg->ctrl |= SPI3_CR_DATA_SIZE(7); // 8bit

	if (param.f.is_master)
	{
		/* SPI data output enable*/
		reg->ctrl &= ~SPI3_CR_SDO_EN;

		/* Slave-mode output disable */
		reg->ext_ctrl |= SPI3_EXT_SSEL_DIS;
	}
	else
	{
		/* SPI data output disable */
		reg->ctrl |= SPI3_CR_SDO_EN;

		/* Slave-mode output enable */
		reg->ext_ctrl &= ~(SPI3_EXT_SSEL_DIS);

		/* Enable Interrupt */
		if (param.f.interrupt)
		{
			reg->int_en |= SPI3_INT_RX_FIFO_HF_EN;
			NVIC_ClearPendingIRQ(SPI3_IRQn);
			NVIC_EnableIRQ(SPI3_IRQn);
		}
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
	gpio_init_mux(spi_config[bus].miso_pin, spi_config[bus].mux);
	gpio_init_mux(spi_config[bus].mosi_pin, spi_config[bus].mux);
	// slave must use hwcs, but let master also use hwcs in this function.
	gpio_init_mux(spi_config[bus].cs_pin,   spi_config[bus].mux);

	CLKRST->peri_rst = SPIx_CLK_EN(bus);
	while ( !(CLKRST->peri_rst & SPIx_CLK_EN(bus)) );
}

int spi_acquire(spi_t bus, spi_cs_t cs, spi_mode_t mode, spi_clk_t clk)
{
	/* get exclusive access to the device */
	mutex_lock(&locks[bus]);

	if (bus == 2)
	{
		tSPI3_REG *reg = spi_config[bus].reg;
		reg->ctrl &= ~( SPI3_CR_CLK_POL | SPI3_CR_PHASE | (SPI3_CR_BAUDRATE_MASK<<12) );
		reg->ctrl |= ( (mode << 4) | SPI3_CR_BAUDRATE(spi3_clk_config[clk].scr) );
		reg->ext_ctrl &= ~(SPI3_EXT_CLK_PRESC_MASK<<8);
		reg->ext_ctrl |= SPI3_EXT_CLK_PRESC(spi3_clk_config[clk].cpsr);
	}
	else
	{
		tSPI_REG *reg = spi_config[bus].reg;
		reg->cr0 &= ~( SPI_CR0_SPO | SPI_CR0_SPH | (SPI_CR0_SCR_MASK<<8) );
		reg->cr0 |= ( (mode << 6) | SPI_CR0_SCR(spi1_2_clk_config[clk].scr) );
		reg->cpsr = spi1_2_clk_config[clk].cpsr;
	}

	return SPI_OK;
}

void spi_release(spi_t bus)
{
	/* release access to the device */
	mutex_unlock(&locks[bus]);
}

static uint16_t spi3_inout(void *_reg, uint16_t data)
{
	tSPI3_REG *reg = (tSPI3_REG *)_reg;
	while (!(reg->state & SPI3_STS_TX_FIFO_EMPTY)) {}
	reg->data = data;
	while (!(reg->state & SPI3_STS_RX_FIFO_RDY)) {}
	return reg->data;
}

static uint16_t spi1_2_inout(void *_reg, uint16_t data)
{
	tSPI_REG *reg = (tSPI_REG *)_reg;
	while (!(reg->sr & SPI_PSR_TNF)) {}
	reg->dr = data;
	while (!(reg->sr & SPI_PSR_RNE)) {}
	return reg->dr;
}

void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
                        const void *out, void *in, size_t len)
{
	uint8_t *inbuf = (uint8_t *)in;
	uint8_t *outbuf = (uint8_t *)out;
	uint16_t tmp;
	size_t i;
	uint16_t (*spi_inout)(void *reg, uint16_t);

#ifdef MODULE_OD
	if (out) {
		printf("spi%d write\n", bus);
		od(out, len, OD_WIDTH_DEFAULT, OD_FLAGS_BYTES_HEX | OD_FLAGS_LENGTH_CHAR);
	} else {
		printf("spi%d read\n", bus);
	}
#endif

	assert(out || in);

	if (cs != SPI_CS_UNDEF)
	{
		gpio_clear((gpio_t)cs);
	}

	if (bus == 2)
		spi_inout = spi3_inout;
	else
		spi_inout = spi1_2_inout;

	for(i = 0; i < len; i++) {
		tmp = spi_inout(spi_config[bus].reg, (uint16_t)(outbuf ? outbuf[i] : 0));
		if (inbuf)
			inbuf[i] = (uint8_t)(tmp & 0xff);
	}

	if ((!cont) && (cs != SPI_CS_UNDEF))
	{
		gpio_set((gpio_t)cs);
	}
}

