/*
 * Copyright (C) 2015 Freie Universität Berlin
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
 * @brief       Low-level UART driver implementation
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Troels Hoffmeyer <troels.d.hoffmeyer@gmail.com>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"

#include "periph/uart.h"
#include "periph/gpio.h"

/**
 * @brief   Allocate memory to store the callback functions
 */
static uart_isr_ctx_t uart_ctx[UART_NUMOF];

/**
 * @brief   Get the pointer to the base register of the given UART device
 *
 * @param[in] dev       UART device identifier
 *
 * @return              base register address
 */
static inline tUART_REG *_uart(uart_t dev)
{
    return uart_config[dev].reg;
}

static int init_base(uart_t uart, uint32_t baudrate);

int uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    /* initialize basic functionality */
    int res = init_base(uart, baudrate);
    if (res != UART_OK) {
        return res;
    }

    /* register callbacks */
    uart_ctx[uart].rx_cb = rx_cb;
    uart_ctx[uart].arg = arg;

    /* configure interrupts and enable RX interrupt */
	// Enable FIFO
    _uart(uart)->ifls = (UART_FIFO_LEVEL_7_OVER_8&UART_IFLS_TXIFLSEL_MASK)|((UART_FIFO_LEVEL_1_OVER_8<<3)&UART_IFLS_RXIFLSEL_MASK);
    _uart(uart)->lcr_h |= UART_LCR_H_FEN;

    // Disable FIFO
    _uart(uart)->lcr_h &= ~(UART_LCR_H_FEN);

    _uart(uart)->imsc |= UART_IMSC_RXIM;

    NVIC_ClearPendingIRQ(UART1_IRQn + uart);
    NVIC_EnableIRQ(UART1_IRQn + uart);

    return UART_OK;
}

void uart_init_pins(uart_t uart)
{
    gpio_init_mux(uart_config[uart].rx_pin, uart_config[uart].mux);
    gpio_init_mux(uart_config[uart].tx_pin, uart_config[uart].mux);

    CLKRST->peri_rst = UARTx_CLK_EN(uart);
    while ( !(CLKRST->peri_rst & UARTx_CLK_EN(uart)) );
}

static int init_base(uart_t uart, uint32_t baudrate)
{
    tUART_REG *dev;

    if ((unsigned int)(uart) >= UART_NUMOF) {
        return UART_NODEV;
    }

    /* get the devices base register */
    dev = _uart(uart);

    /* configure pins */
    uart_init_pins(uart);

    // Disable UART, TX, RX
    dev->cr &= ~(UART_CR_UARTEN|UART_CR_TXE|UART_CR_RXE);

    // init word length
    dev->lcr_h &= ~(UART_LCR_H_WLEN_MASK);

    // 0-10bit, clear all interrupt
    dev->icr |= UART_ALL_MASK;

    // 0-10bit, clear all interrupt mask
    dev->imsc &= ~UART_ALL_MASK;

    // set word length 8
    dev->lcr_h |= (UART_WORDLENGTH_8BIT<<5)&UART_LCR_H_WLEN_MASK;

	// set baudrate
    switch (baudrate) {
    	case UART_BAUDRATE_460800:
    	case UART_BAUDRATE_230400:
    	case UART_BAUDRATE_172800:
    	case UART_BAUDRATE_115200:
    	case UART_BAUDRATE_76800:
    	case UART_BAUDRATE_57600:
    	case UART_BAUDRATE_38400:
    	case UART_BAUDRATE_19200:
    	case UART_BAUDRATE_14400:
    	case UART_BAUDRATE_9600:
    	case UART_BAUDRATE_2400:
    	case UART_BAUDRATE_1200:
    	case UART_BAUDRATE_110:
    		{
    			uint32_t ibrd = CLOCK_UART / (16*baudrate);
    			uint32_t fbrd = (((float)CLOCK_UART / (16*baudrate)) - ibrd)*64;
    			if ((fbrd * 10) % 10 > 5 )
    				fbrd++;
    			dev->ibrd = ibrd & UART_IBRD_BAUD_DIVINT_MASK;
    			dev->fbrd = fbrd & UART_FBRD_BAUD_DIVFRAC_MASK;
    		}
    		break;
    	default:
    		return UART_NOBAUD;
    }

    // Disable FIFO
    dev->lcr_h &= ~(UART_LCR_H_FEN);

    /* finally, enable the device */
    dev->cr |= UART_CR_TXE | UART_CR_RXE;
    dev->cr |= UART_CR_UARTEN;

    return UART_OK;
}

void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        while (_uart(uart)->fr & UART_FR_TXFF) {}
        _uart(uart)->dr = data[i];
    }
}

static inline void irq_handler(int dev)
{
    tUART_REG *uart = _uart(dev);

    if (uart->mis & UART_MIS_RXMIS) {
        /* interrupt flag is cleared by reading the data register */
        uart->icr |= UART_ICR_RXIC;
        uart_ctx[dev].rx_cb(uart_ctx[dev].arg, (uint8_t)(uart->dr));
    }
    else if (uart->mis & (0x1f << 6)) {	/* Errors */
        /* clear error flag */
        uart->icr = (0x1f << 6);
    }
    cortexm_isr_end();
}

#ifdef UART_0_ISR
void UART_0_ISR(void)
{
    irq_handler(0);
}
#endif

#ifdef UART_1_ISR
void UART_1_ISR(void)
{
    irq_handler(1);
}
#endif

#ifdef UART_2_ISR
void UART_2_ISR(void)
{
    irq_handler(2);
}
#endif

