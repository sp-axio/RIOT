#include <stdio.h>

#ifdef MODULE_OD
#include "od.h"
#endif

typedef struct {
	int16_t id;
	int16_t foo;
	uint32_t bar;
	uint32_t baz;
} ctrl_packet_t;

typedef struct {
	int16_t id;
	int16_t foo;
	uint32_t bar;
	uint32_t baz;
} resp_packet_t;

void send_response(uint8_t *buffer, size_t size);

void do_job(void *body, uint16_t length)
{
#ifdef MODULE_OD
	od(body, length, OD_WIDTH_DEFAULT, OD_FLAGS_BYTES_HEX | OD_FLAGS_LENGTH_CHAR);
#endif
	ctrl_packet_t *pkt = (ctrl_packet_t *)body;

#if 1
	printf("pkt id: %u\n", pkt->id);
#else
	/*
		do stuff
	 */
#endif

#if 1
	uint8_t buffer[4];
	uint32_t *p = (uint32_t *)buffer;
	*p = 0xdead;
	send_response(buffer, 4);
#else
	/*
		send response
	 */
#endif
}

void do_resp(void *body, uint16_t length)
{
#ifdef MODULE_OD
	od(body, length, OD_WIDTH_DEFAULT, OD_FLAGS_BYTES_HEX | OD_FLAGS_LENGTH_CHAR);
#endif
	resp_packet_t *pkt = (resp_packet_t *)body;

#if 1
	printf("pkt comes from : %u\n", pkt->id);
#else
	/*
		do stuff
	 */
#endif
}

