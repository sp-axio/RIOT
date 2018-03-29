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
	resp_packet_t resp;
	// don't touch resp.id
	resp.foo = 0x1234;
	resp.bar = 0xdeadbeef;
	resp.baz = 0x05050a0a;
	send_response((uint8_t *)&resp, sizeof(resp));
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

