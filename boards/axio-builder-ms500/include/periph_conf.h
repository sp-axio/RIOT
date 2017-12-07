/*
 * Copyright (C)  2016 Freie Universität Berlin
 *                2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_arduino-zero
 * @{
 *
 * @file
 * @brief       Configuration of CPU peripherals for Arduino Zero board
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Peter Kietzmann <peter.kietzmann@haw-hamburg.de>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include <stdint.h>

#include "cpu.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Clock system configuration
 */
#define CLOCK_CORECLOCK     (64000000U)
#define CLOCK_APB           (32000000U)
#define CLOCK_UART          (48000000U)
#define CLOCK_SPI           (32000000U)

/**
 * @name Timer peripheral configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .reg      = TIMER1,
        .channels = 2,
        .mat_pin  = { GPIO_PIN(PC, 12), GPIO_PIN(PC, 14) },
        .cap_pin  = { GPIO_PIN(PC, 13), GPIO_PIN(PC, 15) },
        .mux      = GPIO_MUX_4,
        .param    = {
                { .f = { .tmr_mode = 0, // periodic, one-shot, free-running
                    .trig_mode = 0, // not-used, capture, timer clock, timer enable
                    .trig_pol  = 0, // low, high
                    .trig_sel  = 0, // external, internal
                    .match_out = 0,
                    .out_mode  = 0 // do nothing, high, low, toggle
                } },
                { .f = { .tmr_mode = 0, // periodic, one-shot, free-running
                    .trig_mode = 0, // not-used, capture, timer clock, timer enable
                    .trig_pol  = 0, // low, high
                    .trig_sel  = 0, // external, internal
                    .match_out = 0,
                    .out_mode  = 0 // do nothing, high, low, toggle
                } }
            },
    },
    {
        .reg      = TIMER2,
        .channels = 2,
        .mat_pin  = { GPIO_PIN(PB, 2), GPIO_PIN(PB, 0) },
        .cap_pin  = { GPIO_PIN(PB, 3), GPIO_PIN(PB, 1) },
        .mux      = GPIO_MUX_4,
        .param    = {
                { .f = { .tmr_mode = 0, // periodic, one-shot, free-running
                    .trig_mode = 0, // not-used, capture, timer clock, timer enable
                    .trig_pol  = 0, // low, high
                    .trig_sel  = 0, // external, internal
                    .match_out = 0,
                    .out_mode  = 0 // do nothing, high, low, toggle
                } },
                { .f = { .tmr_mode = 0, // periodic, one-shot, free-running
                    .trig_mode = 0, // not-used, capture, timer clock, timer enable
                    .trig_pol  = 0, // low, high
                    .trig_sel  = 0, // external, internal
                    .match_out = 0,
                    .out_mode  = 0 // do nothing, high, low, toggle
                } }
        },
    },
};

#define TIMER_0_ISR         isr_timer1
#define TIMER_1_ISR         isr_timer2

#define TIMER_NUMOF         (sizeof(timer_config)/sizeof(timer_config[0]))
/** @} */

/**
 * @name UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .reg    = UART1,
        .rx_pin = GPIO_PIN(PA,9),
        .tx_pin = GPIO_PIN(PA,8),
        .mux    = GPIO_MUX_2,
    },
    {
        .reg    = UART2,
        .rx_pin = GPIO_PIN(PB,5),
        .tx_pin = GPIO_PIN(PB,4),
        .mux    = GPIO_MUX_2,
    },
    {
        .reg    = UART3,
        .rx_pin = GPIO_PIN(PB,7),
        .tx_pin = GPIO_PIN(PB,6),
        .mux    = GPIO_MUX_2,
    }
};

/* interrupt function name mapping */
#define UART_0_ISR          isr_uart1
#define UART_1_ISR          isr_uart2
#define UART_2_ISR          isr_uart3

#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
/** @} */

/**
 * @brief   Pre-calculated clock divider values for SPI1 and SPI2 based on a 32MHz
 *
 * SPI1, SPI2: CLOCK_SPI / cspr * (1+scr)
 *             where cspr: 2, 4, ..., 254 (even number)
 *                   scr:  0, 1, ..., 255
 */
static const spi_clk_conf_t spi1_2_clk_config[] = {
    { .cpsr = 32, .scr =  9 },  /* divided by 32 * 10 : 100khz */
    { .cpsr = 16, .scr =  4 },  /* divided by 16 *  5 : 400khz */
    { .cpsr =  8, .scr =  3 },  /* divided by  8 *  4 :   1MHz */
    { .cpsr =  4, .scr =  1 },  /* divided by  4 *  2 :   4MHz */
    { .cpsr =  2, .scr =  1 }   /* divided by  2 *  2 :   8MHz */
};

/**
 * @brief   Pre-calculated clock divider values for SPI3 based on a 32MHz
 *
 * SPI3      : CLOCK_SPI / ((cspr+1) * 2 ^ (scr+1))
 *             where cspr: 0, 1, ..., 63
 *                   scr:  0, 1, ..., 9
 */
static const spi_clk_conf_t spi3_clk_config[] = {
    { .cpsr =  9, .scr =  4 },  /* divided by 10 * 32 : 100KHz */
    { .cpsr =  9, .scr =  2 },  /* divided by 10 *  8 : 400KHz */
    { .cpsr =  7, .scr =  1 },  /* divided by  8 *  4 :   1MHz */
    { .cpsr =  3, .scr =  0 },  /* divided by  4 *  2 :   4MHz */
    { .cpsr =  1, .scr =  0 }   /* divided by  2 *  2 :   8MHz */
};

/**
 * @name SPI configuration
 * @{
 */
static const spi_conf_t spi_config[] = {
    {
        .reg       = SPI1,
        .clk_pin   = GPIO_PIN(PB, 0), // PB0, PB11
        .miso_pin  = GPIO_PIN(PB, 2), // PB2, PB13
        .mosi_pin  = GPIO_PIN(PB, 3), // PB3, PB14
        .mux       = GPIO_MUX_1,
        .param     = { .f = { .enable = 1, .is_master = 1, .interrupt = 0, .frf = 0 } }
    },
    {
        .reg       = SPI2,
        .clk_pin   = GPIO_PIN(PA, 12), // PA12, PC0
        .miso_pin  = GPIO_PIN(PA, 14), // PA14, PC2
        .mosi_pin  = GPIO_PIN(PA, 15), // PA15, PC3
        .mux       = GPIO_MUX_1,
        .param     = { .f = { .enable = 0, .is_master = 1, .interrupt = 0, .frf = 0 } }
    },
    {
        .reg       = SPI3,
        .clk_pin   = GPIO_PIN(PB, 4), // PB4, PC8
        .miso_pin  = GPIO_PIN(PB, 6), // PB6, PC10
        .mosi_pin  = GPIO_PIN(PB, 7), // PB7, PC11
        .mux       = GPIO_MUX_1,
        .param     = { .f = { .enable = 0, .is_master = 1, .interrupt = 0, .frf = 0} }
    }
};

#define SPI_NUMOF           (sizeof(spi_config) / sizeof(spi_config[0]))
/** @} */

/**
 * @name I2C configuration
 * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        .reg		= I2C1,
        .scl_pin	= GPIO_PIN(PB, 2), // PB2, PB8
        .sda_pin	= GPIO_PIN(PB, 3), // PB3, PB9
        .mux        = GPIO_MUX_3,
    },
    {
        .reg		= I2C2,
        .scl_pin	= GPIO_PIN(PA, 4), // PA4, PB6
        .sda_pin	= GPIO_PIN(PA, 5), // PA5, PB7
        .mux        = GPIO_MUX_3,
    },
    {
        .reg		= I2C3,
        .scl_pin	= GPIO_PIN(PA, 2),
        .sda_pin	= GPIO_PIN(PA, 3),
        .mux        = GPIO_MUX_3,
    },
    {
        .reg		= I2C4,
        .scl_pin	= GPIO_PIN(PB, 0),
        .sda_pin	= GPIO_PIN(PB, 1),
        .mux        = GPIO_MUX_3,
    },
};

#define I2C_NUMOF          (sizeof(i2c_config) / sizeof(i2c_config[0]))
#define I2C_0_EN           1
#define I2C_1_EN           1
#define I2C_2_EN           0
#define I2C_3_EN           1
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
