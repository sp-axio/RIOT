/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_ms500
 * @{
 *
 * @file        vectors.c
 * @brief       Startup code and interrupt vector definition
 *
 * @author      Security Platform Inc.
 *
 * @}
 */

#include <stdint.h>

#include "cpu_conf.h"
#include "vectors_cortexm.h"

/* get the start of the ISR stack as defined in the linkerscript */
extern uint32_t _estack;

/* define a local dummy handler as it needs to be in the same compilation unit
 * as the alias definition */
void dummy_handler(void) {
    dummy_handler_default();
}

/* Cortex-M common interrupt vectors */
WEAK_DEFAULT void isr_svc(void);
WEAK_DEFAULT void isr_pendsv(void);
WEAK_DEFAULT void isr_systick(void);
/* ARMCM0 specific interrupt vector */
WEAK_DEFAULT void isr_systeminit(void);
WEAK_DEFAULT void isr_rtc(void);
WEAK_DEFAULT void isr_dma(void);
WEAK_DEFAULT void isr_dmaerror(void);
WEAK_DEFAULT void isr_sdio(void);
WEAK_DEFAULT void isr_sdmmc(void);
WEAK_DEFAULT void isr_trng(void);
WEAK_DEFAULT void isr_aria(void);
WEAK_DEFAULT void isr_spacc(void);
WEAK_DEFAULT void isr_securekey(void);
WEAK_DEFAULT void isr_pka(void);
WEAK_DEFAULT void isr_clockreset(void);
WEAK_DEFAULT void isr_wdt(void);
WEAK_DEFAULT void isr_gpio1(void);
WEAK_DEFAULT void isr_gpio2(void);
WEAK_DEFAULT void isr_gpio3(void);
WEAK_DEFAULT void isr_i2c1(void);
WEAK_DEFAULT void isr_i2c2(void);
WEAK_DEFAULT void isr_i2c3(void);
WEAK_DEFAULT void isr_i2c4(void);
WEAK_DEFAULT void isr_spi1(void);
WEAK_DEFAULT void isr_spi2(void);
WEAK_DEFAULT void isr_spi3(void);
WEAK_DEFAULT void isr_uart1(void);
WEAK_DEFAULT void isr_uart2(void);
WEAK_DEFAULT void isr_uart3(void);
WEAK_DEFAULT void isr_timer1(void);
WEAK_DEFAULT void isr_timer2(void);

/* interrupt vector table */
ISR_VECTORS const void *interrupt_vector[] = {
    /* Exception stack pointer */
    (void*) &_estack,               /* pointer to the top of the stack */
    /* Cortex-M0+ handlers */
    (void*) reset_handler_default,  /* entry point of the program */
    (void*) nmi_default,            /* non maskable interrupt handler */
    (void*) hard_fault_default,     /* hard fault exception */
    (void*) (0UL),                  /* reserved */
    (void*) (0UL),                  /* reserved */
    (void*) (0UL),                  /* reserved */
    (void*) (0UL),                  /* reserved */
    (void*) (0UL),                  /* reserved */
    (void*) (0UL),                  /* reserved */
    (void*) (0UL),                  /* reserved */
    (void*) isr_svc,                /* system call interrupt, in RIOT used for
                                     * switching into thread context on boot */
    (void*) (0UL),                  /* reserved */
    (void*) (0UL),                  /* reserved */
    (void*) isr_pendsv,             /* pendSV interrupt, in RIOT the actual
                                     * context switching is happening here */
    (void*) isr_systick,            /* SysTick interrupt, not used in RIOT */
    /* ARMCM0 Specific Interrupts */
    (void*) isr_systeminit,         /* 00 : System Init ( Clock Status, WAKEUP pin), plus event which can be used as NMI */
    (void*) isr_rtc,                /* 01 : RTC ( alarm or wakeup )   */
    (void*) isr_dma,                /* 02 : DMA IRQ */
    (void*) isr_dmaerror,           /* 03 : DMA Error IRQ */
    (void*) isr_sdio,               /* 04 : SDIO global interrupt */
    (void*) isr_sdmmc,              /* 05 : SDMMC global interrupt */
    (void*) isr_trng,               /* 06 : Security Module (TRNG) */
    (void*) isr_aria,               /* 07 : ARIA interrupt */
    (void*) isr_spacc,              /* 08 : Security Engine (SPAcc) */
    (void*) isr_securekey,          /* 09 : SecureKey Policy interrupt */
    (void*) isr_pka,                /* 10 : PKA global interrupt */
    (void*) isr_clockreset,         /* 11 : Clock reset manager interrupt */
    (void*) isr_wdt,                /* 12 : WDT interrupt */
    (void*) isr_gpio1,              /* 13 : GPIO1 global interrupt */
    (void*) isr_gpio2,              /* 14 : GPIO2 global interrupt */
    (void*) isr_gpio3,              /* 15 : GPIO3 global interrupt */
    (void*) isr_i2c1,               /* 16 : I2C1 global interrupt */
    (void*) isr_i2c2,               /* 17 : I2C2 global interrupt */
    (void*) isr_i2c3,               /* 18 : I2C3 global interrupt */
    (void*) isr_i2c4,               /* 19 : I2C4 global interrupt */
    (void*) isr_spi1,               /* 20 : SPI1 global interrupt */
    (void*) isr_spi2,               /* 21 : SPI2 global interrupt */
    (void*) isr_spi3,               /* 22 : SPI3 global interrupt */
    (void*) isr_uart1,              /* 23 : UART1 global interrupt */
    (void*) isr_uart2,              /* 24 : UART2 global interrupt */
    (void*) isr_uart3,              /* 25 : UART3 global interrupt */
    (void*) isr_timer1,             /* 26 : Timer1 global interrupt */
    (void*) isr_timer2              /* 27 : Timer2 global interrupt */
};
