/*
 * Copyright (C) 2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_arduino-zero Arduino Zero
 * @ingroup     boards
 * @brief       Support for the Arduino Zero board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the Arduino Zero
 *              board
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "periph_conf.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   xtimer configuration
 * @{
 */
#define XTIMER_DEV          (0)
#define XTIMER_CHAN         (0)
#define XTIMER_HZ           (1500000lu)
#define XTIMER_WIDTH        (16)
#define XTIMER_SHIFT        (0)
#define XTIMER_OVERHEAD     (30)
#define XTIMER_BACKOFF      (60)
#define XTIMER_ISR_BACKOFF  (100)
/** @} */

/**
 * @brief Arduino pinmap
 */
#define PIN_D0  GPIO_PIN(PB,  5)
#define PIN_D1  GPIO_PIN(PB,  4)
#define PIN_D2  GPIO_PIN(PA, 13)
#define PIN_D3  GPIO_PIN(PA, 12)
#define PIN_D4  GPIO_PIN(PB,  7)
#define PIN_D5  GPIO_PIN(PB,  6)
#define PIN_D6  GPIO_PIN(PA,  9)
#define PIN_D7  GPIO_PIN(PA,  8)
#define PIN_D8  GPIO_PIN(PA, 15)
#define PIN_D9  GPIO_PIN(PA, 14)
#define PIN_D10 GPIO_PIN(PB,  1)
#define PIN_D11 GPIO_PIN(PB,  3)
#define PIN_D12 GPIO_PIN(PB,  2)
#define PIN_D13 GPIO_PIN(PB,  0)
#define PIN_A0  GPIO_PIN(PA,  0)
#define PIN_A1  GPIO_PIN(PA,  1)
#define PIN_A2  GPIO_PIN(PA,  3)
#define PIN_A3  GPIO_PIN(PA, 10)
#define PIN_A4  GPIO_PIN(PA,  5)
#define PIN_A5  GPIO_PIN(PA,  4)
/** @} */

/**
 * @brief Use the UART1, 115200 Baudrate for STDIO on this board
 */
#define UART_STDIO_DEV      UART_DEV(1)
#define UART_STDIO_BAUDRATE (UART_BAUDRATE_115200)
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
