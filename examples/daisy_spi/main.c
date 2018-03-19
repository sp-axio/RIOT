/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Daisy chain SPI
 *
 * @author      Jongho Lee
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "shell.h"
#include "periph/spi.h"

typedef struct {
	spi_t dev;
	spi_mode_t mode;
	spi_clk_t clk;
	spi_cs_t cs;
} spiconf_t;

static spiconf_t spi[3];
static spiconf_t *master = NULL;
#define SPI_MASTER 2

#define spi_get(s) (spi_acquire(s->dev, s->cs, s->mode, s->clk) == SPI_OK)
#define spi_put(s) spi_release(s->dev)
#define spi_io(s, outb, inb, size) spi_transfer_bytes(s->dev, s->cs, false, outb, inb, size)

static void daisy_spi_init(void)
{
	// in this application, don't use gpio control...
	//gpio_t cs_pin[3] = { GPIO_PIN(PB, 1), GPIO_PIN(PC, 1), GPIO_PIN(PC, 9) };
	gpio_t cs_pin[3] = { SPI_HWCS(-1), SPI_HWCS(-1), SPI_HWCS(-1) };
	for (int i = 0; i < SPI_NUMOF; i++)
	{
		spi[i].dev  = SPI_DEV(i);
		spi[i].mode = SPI_MODE_0;
		spi[i].clk  = SPI_CLK_1MHZ;
		spi[i].cs   = cs_pin[i];
	}

	master = &spi[SPI_MASTER];
}

static void daisy_spi_write(spiconf_t *spi, uint8_t *buffer, size_t size)
{
	if (!spi_get(spi)) {
		printf("error: unable to acquire the SPI%d bus", spi->dev);
		return;
	}
	spi_io(spi, buffer, NULL, size);
	spi_put(spi);
}

static void daisy_spi_read(spiconf_t *spi, uint8_t *buffer, uint8_t size)
{
	if (!spi_get(spi)) {
		printf("error: unable to acquire the SPI%d bus", spi->dev);
		return;
	}
	spi_io(spi, NULL, buffer, size);
	spi_put(spi);
}

void spi_isr_callback(int bus)
{
	uint8_t buffer[4];
	memset(buffer, 0xff, 4);

	daisy_spi_read(&spi[bus], buffer, 4);

	printf("spi%d recv: %02x %02x %02x %02x\n",bus,
			buffer[0], buffer[1], buffer[2], buffer[3]);
}

static int sendh(int argc, char **argv)
{
	if (argc != 2) {
		return 1;
	}

	int i, j;
	char tmp[3];
	uint8_t buf[128];

	tmp[2] = 0;
	for (i=0,j=0; i<strlen(argv[1]) && j<sizeof(buf); i+=2,j+=1)
	{
		tmp[0] = argv[1][i];
		tmp[1] = argv[1][i+1];
		buf[j] = strtoul(tmp, NULL, 16);
	}

	daisy_spi_write(master, buf, j);

	return 0;
}

static int senda(int argc, char **argv)
{
	if (argc != 2) {
		return 1;
	}

	daisy_spi_write(master, (uint8_t *)argv[1], strlen(argv[1]));

	return 0;
}

static const shell_command_t shell_commands[] = {
    { "sendh", "send hexstring", sendh },
    { "senda", "send ascii", senda },
    { NULL, NULL, NULL }
};

int main(void)
{
    puts("Hello World!");

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

	daisy_spi_init();

    /* start the shell */
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
