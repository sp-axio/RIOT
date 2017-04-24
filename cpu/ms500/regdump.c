#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <ctype.h>
#include "vendor/ewbm_device.h"

void __attribute__((weak)) hexdump(uint32_t *addr, int len, int canonical)
{
	printf("Need shell_commands module\n");
}

int cmd_regdump(int argc, char **argv)
{
	struct _reg {
		char name[8];
		void *addr;
		int len;
	} *r;
	const struct _reg reg[] = {
		{"syscon",   SYSCON,         sizeof(tSYSCON_REG)},
		{"clkrst",   CLKRST,         sizeof(tCLKRST_REG)},
		{"uart1",    UART1,          sizeof(tUART_REG)},
		{"uart2",    UART2,          sizeof(tUART_REG)}, {"uart3",    UART3,          sizeof(tUART_REG)},
		{"dma",      DMA,            sizeof(tDMAC_REG)},
		{"i2c1",     I2C1,           sizeof(tI2C_REG)},
		{"i2c2",     I2C2,           sizeof(tI2C_REG)},
		{"i2c3",     I2C3,           sizeof(tI2C_REG)},
		{"i2c4",     I2C4,           sizeof(tI2C_REG)},
		{"spi1",     SPI1,           sizeof(tSPI_REG)},
		{"spi2",     SPI2,           sizeof(tSPI_REG)},
		{"spi3",     SPI3,           sizeof(tSPI3_REG)},
		{"bspi",     BSPI,           sizeof(tBSPI_REG)},
		{"sdiom",    SDIO_M,         sizeof(tSDIO_M_REG)},
		{"sdios",    SDIO_S,         sizeof(tSDIO_S_REG)},
		{"gpio1",    GPIO1,          sizeof(tGPIO_REG)},
		{"gpio2",    GPIO2,          sizeof(tGPIO_REG)},
		{"gpio3",    GPIO3,          sizeof(tGPIO_REG)},
		{"rtc",      RTC,            sizeof(tRTC_REG)},
		{"gpt1",     TIMER1,         sizeof(tGPT_REG)},
		{"gpt2",     TIMER2,         sizeof(tGPT_REG)},
		{"wdt",      WDT,            sizeof(tWDT_REG)},
		{"nvm",      NVM,            sizeof(tNVM_REG)},
		{"pmu",      PMU,            sizeof(tPMU_REG)},
		{"icache",   ICACHE,         sizeof(tICACHE_REG)},
		{"trng",     TRNG,           sizeof(tTRNG_REG)},
		{"spacc",    SPACC,          sizeof(tSPACC_REG)},
		{"aria", (void *)ARIA_BASE_ADDR, sizeof(t_ARIAREG)},
		{"scb",      SCB,            sizeof(SCB_Type)},
		{"systick",  SysTick,        sizeof(SysTick_Type)},
		{"nvic",     NVIC,           sizeof(NVIC_Type)},
		{"",         0,              0}
	};
	int len = 0;

	if (argc == 2 || argc == 3) {
		if (argc == 3)
			len = strtoul(argv[2], 0, 16);

		for (r=(struct _reg *)reg; r->addr; r++) {
			if (!strcmp(r->name, argv[1])) {
				if (len == 0)
					len = r->len;
				hexdump(r->addr, len, 0);
				return 0;
			}
		}
	}

	if (len == 0)
		printf("Usage: %s register_name {len(hex)}\n", argv[0]);

	return 1;
}

