#include <stdio.h>

#ifdef MODULE_OD
#include "od.h"
#endif

typedef struct {
	int16_t id;
	int16_t foo;
	uint32_t bar;
	uint32_t baz;
} spi_packet_t;

void do_job(void *body, uint16_t length)
{
#ifdef MODULE_OD
	od(body, length, OD_WIDTH_DEFAULT, OD_FLAGS_BYTES_HEX | OD_FLAGS_LENGTH_CHAR);
#endif
	spi_packet_t *pkt = (spi_packet_t *)body;
	printf("pkt id: %u\n", pkt->id);
}

