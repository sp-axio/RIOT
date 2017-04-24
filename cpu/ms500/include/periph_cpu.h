/*
 * Copyright (C) 2015-2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_ms500
 * @{
 *
 * @file
 * @brief           CPU specific definitions for internal peripheral handling
 *
 * @author          Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef CPU_PERIPH_H
#define CPU_PERIPH_H

#include <limits.h>

#include "vendor/ewbm_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Length of the CPU_ID in octets
 */
#define CPUID_LEN           (16U)

/**
 * @brief   Available ports on the MS500
 */
enum {
    PA = 0,                 /**< port A */
    PB = 1,                 /**< port B */
    PC = 2,                 /**< port C */
    PORT_MAX
};

/**
 * @brief   Override GPIO type
 */
#define HAVE_GPIO_T
typedef uint32_t gpio_t;
/** @} */

/**
 * @brief   Macro for accessing GPIO pins
 */
#define GPIO_PIN(x, y)      (((uint32_t)GPIO1)|(x<<12)|y)

/**
 * @brief   Available MUX values for configuring a pin's alternate function
 */
typedef enum {
    GPIO_MUX_0 = 0x0,       /**< select peripheral function A */
    GPIO_MUX_1 = 0x1,       /**< select peripheral function B */
    GPIO_MUX_2 = 0x2,       /**< select peripheral function C */
    GPIO_MUX_3 = 0x3,       /**< select peripheral function D */
    GPIO_MUX_4 = 0x4,       /**< select peripheral function E */
    GPIO_MUX_5 = 0x5,       /**< select peripheral function F */
} gpio_mux_t;

/**
 * @brief   UART device configuration
 */
typedef struct {
    tUART_REG *reg;         /**< pointer to the used UART device */
    gpio_t rx_pin;          /**< pin used for RX */
    gpio_t tx_pin;          /**< pin used for TX */
    gpio_mux_t mux;         /**< alternative function for pins */
} uart_conf_t;

/**
 * @brief   SPI device configuration
 */
typedef union {
    uint16_t raw;
    struct spi_field {
        uint8_t enable     : 1;
        uint8_t is_master  : 1;
        uint8_t interrupt  : 1;
		uint8_t rsvd1      : 5;
        uint8_t frf        : 2; // frame format (0: Motorola, 1: TI, 2: NSM)
        uint8_t dss        : 4; // data size select 3(4bit) ~ 15(16bit)
		uint8_t rsvd2      : 2;
    } f;
} spi_param_t;

typedef struct {
    void *reg;              /**< pointer to the used SPI device */
    gpio_t clk_pin;         /**< used CLK pin */
    gpio_t cs_pin;          /**< used CS pin */
    gpio_t miso_pin;        /**< used MISO pin */
    gpio_t mosi_pin;        /**< used MOSI pin */
    gpio_mux_t mux;         /**< alternate function for pin (mux) */
    spi_param_t param;
} spi_conf_t;

/**
 * @brief   I2C device configuration
 */
typedef struct {
    void *reg;              /**< pointer to the used I2C device */
    gpio_t scl_pin;         /**< used SCL pin */
    gpio_t sda_pin;         /**< used SDA pin */
    gpio_mux_t mux;         /**< alternate function for pin (mux) */
} i2c_conf_t;

/**
 * @brief   Timer configuration data
 */
typedef union {
	uint16_t raw;
	struct timer_field {
		uint8_t tmr_mode   : 2;
		uint8_t trig_mode  : 2;
		uint8_t trig_pol   : 1;
		uint8_t trig_sel   : 1;
		uint8_t match_out  : 1;
		uint8_t rsvd1      : 1;
		uint8_t out_mode   : 2;
		uint8_t rsvd2      : 6;
	} f;
} timer_param_t;

typedef struct {
    tGPT_REG *reg;  /**< timer device */
	uint8_t channels;
	gpio_t mat_pin[2];
	gpio_t cap_pin[2];
	gpio_mux_t mux;
	timer_param_t param[2];
} timer_conf_t;

/**
 * @brief   Set up alternate function (PMUX setting) for a PORT pin
 *
 * @param[in] pin   Pin to set the multiplexing for
 * @param[in] mux   Mux value
 */
void gpio_init_mux(gpio_t pin, gpio_mux_t mux);


#ifdef __cplusplus
}
#endif

#endif /* CPU_PERIPH_H */
/** @} */
