#include <stdio.h>
#include <malloc.h>

#ifdef MODULE_OD
#include "od.h"
#endif

typedef struct {
	int16_t id; // don't touch this
	int16_t foo;
	uint32_t bar;
	uint32_t baz;
} ctrl_packet_t; // size should be less than RECV.body

typedef struct {
	int16_t id; // don't touch this
	int16_t foo;
	uint32_t bar;
	uint32_t baz;
} resp_packet_t; // size should be less than RECV.body

extern int tprintf(const char *fmt,...);

void send_response(uint8_t *buffer, size_t size);

void do_job(void *pkt)
{
	ctrl_packet_t *ctl = (ctrl_packet_t *)pkt;
	resp_packet_t *resp;
#if 1
	tprintf("pkt id: %u", ctl->id);
#ifdef MODULE_OD
	od(ctl, sizeof(ctrl_packet_t), OD_WIDTH_DEFAULT, OD_FLAGS_BYTES_HEX | OD_FLAGS_LENGTH_CHAR);
#endif
	// don't touch resp.id
	resp = malloc(sizeof(resp_packet_t));
	if (!resp) {
		tprintf("Not enough memory\n");
		return;
	}
	resp->foo = 0x1234;
	resp->bar = 0xdeadbeef;
	resp->baz = 0x05050a0a;
	tprintf("send response");
	send_response((uint8_t *)resp, sizeof(resp_packet_t));
	free(resp);
#else
	/*
	   do stuff
	 */
#endif
}

void do_resp(void *pkt)
{
	resp_packet_t *resp = (resp_packet_t *)pkt;
#if 1
	tprintf("pkt comes from : %u, %x, %08lx, %08lx", resp->id, resp->foo, resp->bar, resp->baz);
#ifdef MODULE_OD
	od(resp, sizeof(resp_packet_t), OD_WIDTH_DEFAULT, OD_FLAGS_BYTES_HEX | OD_FLAGS_LENGTH_CHAR);
#endif
#else
	/*
	   do stuff
	 */
#endif
}

