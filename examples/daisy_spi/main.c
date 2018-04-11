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
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#ifdef MODULE_OD
#include "od.h"
#endif
#include "byteorder.h"
#include "checksum/crc16_ccitt.h"
#include "xtimer.h"
#include "shell.h"
#include "periph/spi.h"

#define DBG_BIT_SPI        (1<<0)
#define DBG_BIT_SPI_THREAD (1<<1)
#define DBG_BIT_PREPROCESS (1<<2)
#define DBG_BIT_PKT_THREAD (1<<3)

//#define DBG_FLAG (DBG_BIT_PKT_THREAD | DBG_BIT_PREPROCESS | DBG_BIT_SPI_THREAD | DBG_BIT_SPI)
//#define DBG_FLAG (DBG_BIT_PKT_THREAD | DBG_BIT_PREPROCESS | DBG_BIT_SPI_THREAD)
#define DBG_FLAG (DBG_BIT_PKT_THREAD | DBG_BIT_PREPROCESS)

#if DBG_FLAG == 0
#define dprintf(f,fmt...)
#else
#define dprintf(f,fmt...) do { if(DBG_FLAG&f) printf(fmt); } while(0);
#endif

static const char protocol_magic1[] = "Axio";
static const char protocol_magic2[] = "Enum";
static const char protocol_magic3[] = "Resp";

static int16_t axio_id = -1;

enum {
	RECV_ERROR = 0,
	RECV_INVALID_HEADER,
	RECV_INVALID_LENGTH,
	RECV_INVALID_ALIGN,
	RECV_COMPLETE,
	RECV_IN_PROGRESS,
	RECV_TIMEOUT,
	RECV_SPI_MSG_MAX
};

enum {
	RECV_CTRL_PKT = 0,
	RECV_RESP_PKT,
	RECV_PKT_MSG_MAX
};

typedef struct recv_t {
	// don't touch these elements
	uint8_t magic[4];
	uint16_t length;
	uint16_t csum;
	uint8_t body[4096];
	// These elements must be at the end
	int state;
	uint32_t *pos;
	uint16_t remainder;
	xtimer_t timer;
	msg_t msg;
	kernel_pid_t pid;
	uint32_t msg_cnt[RECV_SPI_MSG_MAX];
	uint32_t crc_error;
} recv_t;

static recv_t RECV = { .state = 0, };

static kernel_pid_t pkt_proc_pid = 0;

static void recv_init(void)
{
	xtimer_remove(&RECV.timer);
	memset(&RECV, 0, (uint32_t)&RECV.pid - (uint32_t)&RECV);
}

typedef struct {
	spi_t dev;
	spi_mode_t mode;
	spi_clk_t clk;
	spi_cs_t cs;
} spiconf_t;

static spiconf_t spi[3];
static spiconf_t *master = NULL;
// see board/axio-builder-ms500/include/periph_conf.h for slave setting
#define SPI_MASTER 2

#define spi_get(s) spi_acquire((s)->dev, (s)->cs, (s)->mode, (s)->clk)
#define spi_put(s) spi_release((s)->dev)
#define spi_io(s, outb, inb, size) spi_transfer_bytes((s)->dev, (s)->cs, false, outb, inb, size)

static void daisy_spi_init(void)
{
	//gpio_t cs_pin[3] = { GPIO_PIN(PB, 1), GPIO_PIN(PC, 1), GPIO_PIN(PC, 9) };
	for (int i = 0; i < SPI_NUMOF; i++)
	{
		spi[i].dev  = SPI_DEV(i);
		spi[i].mode = SPI_MODE_0;
		spi[i].clk  = SPI_CLK_5MHZ;
		spi[i].cs   = SPI_HWCS(-1); //in this application, don't use gpio control...
	}

	master = &spi[SPI_MASTER];
}

static void daisy_spi_raw_write(uint8_t *buffer, size_t size)
{
#ifdef MODULE_OD
	if (DBG_FLAG&DBG_BIT_SPI)
	{
		od(buffer, size, OD_WIDTH_DEFAULT, OD_FLAGS_BYTES_HEX | OD_FLAGS_LENGTH_CHAR);
	}
#endif
	assert(!(size%4));

	int n = size/4;
	uint32_t *ptr = (uint32_t *)buffer;
	uint32_t recv;
	uint32_t prev;

	spi_get(master);

	for (int i = 0; i < n; i++) {
		recv = 0;
		spi_io(master, &ptr[i], &recv, 4);
		if (i>2) {
			if (prev + 1 != recv) {
				printf("maybe packet lost! idx:%d, recv: %lx, %08lx\n", i, recv, ptr[i]);
			}
		}
		prev = recv;
		xtimer_spin(xtimer_ticks_from_usec(300));
	}

	spi_put(master);
}

static void daisy_spi_write(void)
{
	daisy_spi_raw_write((uint8_t *)(&RECV), RECV.length + 8 /* magic 4, length 2, csum 2 */);
}

void spi_isr_callback(int bus)
{
#define send_message(t) do { msg_t msg; msg.type = t; msg_send(&msg, RECV.pid); } while(0);
	static uint32_t recv_counter = 0;

	dprintf(DBG_BIT_SPI, "spi%d callback\n", bus);

	uint8_t buffer[4] = { 0xff, 0xff, 0xff, 0xff };

	spiconf_t *slave = &spi[bus];

	spi_get(slave);
	spi_io(slave, (void *)&recv_counter, (void *)&buffer, sizeof(buffer));
	spi_put(slave);

	dprintf(DBG_BIT_SPI, "state: %d, buffer: %02x%02x%02x%02x\n",
			RECV.state, buffer[0], buffer[1], buffer[2], buffer[3]);

	if (RECV.state != 0)
	{
		recv_counter++;
	}

	switch(RECV.state)
	{
		case 0: // check magic with network-order stream
			if (!memcmp(&buffer, protocol_magic1, 4) ||
					!memcmp(&buffer, protocol_magic2, 4) ||
					!memcmp(&buffer, protocol_magic3, 4)) {
				recv_counter = 1;
				RECV.msg.type = RECV_TIMEOUT;
				xtimer_set_msg(&RECV.timer, 5000000, &RECV.msg, RECV.pid);
				memcpy(&RECV.magic, buffer, 4);
				RECV.state++;
			}
			else {
				send_message(RECV_INVALID_HEADER);
			}
			break;
		case 1:
			{
				uint16_t *len = (uint16_t *)buffer;
				uint16_t *crc16 = (uint16_t *)(buffer+2);
				if (*len > sizeof(RECV.body)) {
					send_message(RECV_INVALID_LENGTH);
					break;
				}
				if (*len % 4) {
					send_message(RECV_INVALID_ALIGN);
					break;
				}
				RECV.remainder = RECV.length = *len; // excluding magic, length and csum
				RECV.csum = *crc16;
				RECV.pos = (uint32_t *)RECV.body;
				RECV.state++;
			}
			break;
		case 2:
			if (RECV.remainder > 0) {
				dprintf(DBG_BIT_SPI, "pos: %p, remaining: %u\n", RECV.pos, RECV.remainder);
				memcpy(RECV.pos, buffer, 4);
				RECV.pos++;
				RECV.remainder -= 4;
				if (RECV.remainder == 0) {
					RECV.state++;
					xtimer_remove(&RECV.timer);
					dprintf(DBG_BIT_SPI, "send complete message\n");
					send_message(RECV_COMPLETE);
				}
			}
			else {
				send_message(RECV_ERROR);
			}
			break;
		default:
			send_message(RECV_IN_PROGRESS);
	}
}

static char _spi_stack[1024];
static char _pkt_stack[1024];

static msg_t spi_msg[8];
static msg_t pkt_msg[8];

static void send_to_pkt_process(uint16_t mtype)
{
	msg_t msg;
	void *pkt = malloc(RECV.length);
	if (!pkt) {
		dprintf(DBG_BIT_PREPROCESS, "Not enough memory\n");
	}
	memcpy(pkt, RECV.body, RECV.length);
	msg.type = mtype;
	msg.content.ptr = pkt;
	if (msg_send(&msg, pkt_proc_pid) <= 0) {
		dprintf(DBG_BIT_PREPROCESS, "Can't send message to pkt process\n");
		free(pkt);
	}
}

static void preprocess_pkt(void)
{
	int16_t *id = (int16_t *)RECV.body;

	if (!memcmp(&RECV.magic, protocol_magic2, 4)) {
		dprintf(DBG_BIT_PREPROCESS, "enumeration packet has been arrived\n");
		if (*id > 0)
		{
			if (axio_id == 0) {
				dprintf(DBG_BIT_PREPROCESS, "enumeration packet has come back. last id is %d\n", *id);
				return;
			}
			else if (axio_id < 0) {
				axio_id = *id;
				dprintf(DBG_BIT_PREPROCESS, "set my ID = %d\n", axio_id);
				(*id)++; // increase ID for next Axio
				RECV.csum = crc16_ccitt_calc(RECV.body, RECV.length);
			}
			else {
				/* unreachable code normally */
				dprintf(DBG_BIT_PREPROCESS, "my ID (%d) is already set, passing this packet to next\n", axio_id);
				*id = axio_id + 1;
				RECV.csum = crc16_ccitt_calc(RECV.body, RECV.length);
			}
			goto send_to_next;
		}
		dprintf(DBG_BIT_PREPROCESS, "Invalid ID is present, consume this packet\n");
		return;
	}
	else if (!memcmp(&RECV.magic, protocol_magic3, 4)) {
		if (axio_id > 0) {
			dprintf(DBG_BIT_PREPROCESS, "Resp packet has been arrived, passing this packet to next\n");
			goto send_to_next;
		}
		dprintf(DBG_BIT_PREPROCESS, "Resp packet has been arrived, consume this packet, Length: %u\n", RECV.length);
		send_to_pkt_process(RECV_RESP_PKT);
		return;
	}
	else if (!memcmp(&RECV.magic, protocol_magic1, 4)) {
		if (axio_id < 0) {
			dprintf(DBG_BIT_PREPROCESS, "my ID was not set, consume this packet\n");
			return;
		}
		else if (*id != axio_id) {
			dprintf(DBG_BIT_PREPROCESS, "Packet ID is %u, passing packet to next axio\n", *id);
			goto send_to_next;
		}
		dprintf(DBG_BIT_PREPROCESS, "Control packet (%u) has been arrived, Length: %u\n", *id, RECV.length);
		send_to_pkt_process(RECV_CTRL_PKT);
		return;
	}

	dprintf(DBG_BIT_PREPROCESS, "Unknown packet has been arrived, drop this packet\n");
	return;

send_to_next:
	daisy_spi_write();
}

static void process_spi(msg_t *msg)
{
	dprintf(DBG_BIT_SPI_THREAD, "spi thread debugging: mtype: %d\n", msg->type);
	if (msg->type < RECV_SPI_MSG_MAX)
	{
		RECV.msg_cnt[msg->type]++;
	}
	switch (msg->type)
	{
		case RECV_ERROR:
			dprintf(DBG_BIT_SPI_THREAD, "recv error\n");
			recv_init();
			break;
		case RECV_INVALID_HEADER:
			dprintf(DBG_BIT_SPI_THREAD, "invalid header\n");
			recv_init();
			break;
		case RECV_INVALID_LENGTH:
			dprintf(DBG_BIT_SPI_THREAD, "too long packet, aborting\n");
			recv_init();
			break;
		case RECV_INVALID_ALIGN:
			dprintf(DBG_BIT_SPI_THREAD, "length must be 4byte-aligned, aborting\n");
			recv_init();
			break;
		case RECV_COMPLETE:
			{
				dprintf(DBG_BIT_SPI_THREAD, "packet processing has started\n");
#ifdef MODULE_OD
				if (DBG_FLAG&DBG_BIT_SPI_THREAD)
				{ // dump
					od(RECV.body, RECV.length, OD_WIDTH_DEFAULT, OD_FLAGS_BYTES_HEX | OD_FLAGS_LENGTH_CHAR);
				}
#endif
				uint16_t crc16 = crc16_ccitt_calc(RECV.body, RECV.length);
				if (RECV.csum == crc16)
				{ // check csum
					preprocess_pkt();
				}
				else
				{
					RECV.crc_error++;
					printf("crc is invalid (%x) expected (%x) length (%u)\n", RECV.csum, crc16, RECV.length);
				}
				// finish
				recv_init();
			}
			break;
		case RECV_IN_PROGRESS:
			dprintf(DBG_BIT_SPI_THREAD, "the previous packet is processing now\n");
			break;
		case RECV_TIMEOUT:
			printf("timeout, goto initial state\n");
			recv_init();
			break;
		default:
			printf("spi thread: unknown message(%d)\n", msg->type);
	}
}

static void process_pkt(msg_t *msg)
{
	void do_resp(void *pkt);
	void do_job(void *pkt);
	void *pkt = msg->content.ptr;
	dprintf(DBG_BIT_PKT_THREAD, "pkt thread debugging: mtype: %d\n", msg->type);
	switch (msg->type)
	{
		case RECV_RESP_PKT:
			if (pkt) {
				do_resp(pkt);
				free(pkt);
			}
			break;
		case RECV_CTRL_PKT:
			if (pkt) {
				do_job(pkt);
				free(pkt);
			}
			break;
		default:
			dprintf(DBG_BIT_PKT_THREAD, "pkt thread debugging: mtype: %d\n", msg->type);
	}
}

static void *process_spi_recv(void *arg)
{
	msg_init_queue(spi_msg, 8);
	msg_t spi_msg;

	while(1)
	{
		msg_receive(&spi_msg);
		process_spi(&spi_msg);
	}

	return NULL;
}

static void *process_pkt_recv(void *arg)
{
	msg_init_queue(pkt_msg, 8);
	msg_t pkt_msg;

	while(1)
	{
		msg_receive(&pkt_msg);
		process_pkt(&pkt_msg);
	}

	return NULL;
}

static int getid(int argc, char **argv)
{
	if (argc != 1) {
		return 1;
	}
	printf("my ID: %d\n", axio_id);
	return 0;
}

void send_response(uint8_t *buffer, size_t size)
{
	memcpy(&RECV.magic, protocol_magic3, 4);
	int16_t *id = (int16_t *)&RECV.body[0];
	*id = axio_id; // set my ID;
	memcpy(&RECV.body[2], buffer+2, size-2);
	RECV.length = size;
	if (RECV.length % 4) {
		int padlen = 4 - (RECV.length % 4);
		if (RECV.length + padlen > sizeof(RECV.body)) {
			printf("too long packet!\n");
			return;
		}
		RECV.length += padlen;
	}
	RECV.csum = crc16_ccitt_calc(RECV.body, RECV.length);
	daisy_spi_write();
}

static int send_data_packet(int argc, char **argv)
{
	if (argc != 3) {
		return 1;
	}

	int16_t _id = strtol(argv[1], 0, 10);
	if (_id < 0 || _id > 100) {
		printf("Out of range.\n");
		return 1;
	}

	int16_t *id = (int16_t *)&RECV.body[0];
	*id = _id;
	uint8_t *pos = &RECV.body[2];

	int i, j;
	char tmp[3];
	tmp[2] = 0;
	for (i=0,j=0; i<strlen(argv[2]) && j<sizeof(RECV.body); i+=2,j+=1)
	{
		tmp[0] = argv[2][i];
		tmp[1] = argv[2][i+1];
		pos[j] = strtoul(tmp, NULL, 16);
		if (j > sizeof(RECV.body)-2) {
			printf("too long packet!\n");
			return 1;
		}
	}

	memcpy(&RECV.magic, protocol_magic1, 4);
	RECV.length = j + sizeof(int16_t);

	if (RECV.length % 4) {
		int padlen = 4 - (RECV.length % 4);
		if (RECV.length + padlen > sizeof(RECV.body)) {
			printf("too long packet!\n");
			return 1;
		}
		RECV.length += padlen;
		for (i=0; i<padlen; i++)
		{
			pos[j++] = 0;
		}
	}

	RECV.csum = crc16_ccitt_calc(RECV.body, RECV.length);

	daisy_spi_write();

	return 0;
}

static int send_large_packet(int argc, char **argv)
{
	if (argc < 2 || argc > 3) {
		return 1;
	}

	int16_t _id = strtol(argv[1], 0, 10);
	if (_id < 0 || _id > 100) {
		printf("Out of range.\n");
		return 1;
	}

	int16_t *id = (int16_t *)&RECV.body[0];
	*id = _id;

	uint16_t len = 4096;
	if (argc == 3) {
		len = strtoul(argv[2], 0, 10);
		if (len > 4096) {
			printf("Out of range.\n");
			return 1;
		}
		if (len % 4) {
			printf("Length must be 4byte aligned\n");
			return 1;
		}
	}

	uint16_t *pos = (uint16_t *)&RECV.body[2];
	for (unsigned i=0; i<(len/2) - 1 /* exclude id */; i++) {
		pos[i] = i;
	}

	memcpy(&RECV.magic, protocol_magic1, 4);
	RECV.length = len;
	RECV.csum = crc16_ccitt_calc(RECV.body, RECV.length);

	daisy_spi_write();

	return 0;
}

static int send_enum_packet(int argc, char **argv)
{
	if (argc != 1) {
		return 1;
	}

	axio_id = 0; // set master ID is zero.

	int16_t *id = (int16_t *)&RECV.body;
	id[0] = 1; /* set next id = 1 */
	id[1] = 0; /* fill 4 byte */

	memcpy(&RECV.magic, protocol_magic2, 4);
	RECV.length = 4;
	RECV.csum = crc16_ccitt_calc(RECV.body, RECV.length);

	daisy_spi_write();

	return 0;
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

	daisy_spi_raw_write(buf, j);

	return 0;
}

static int senda(int argc, char **argv)
{
	if (argc != 2) {
		return 1;
	}

	daisy_spi_raw_write((uint8_t *)argv[1], strlen(argv[1]));

	return 0;
}

static int crc16ccitt(int argc, char **argv)
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

	printf("crc: %04x\n", crc16_ccitt_calc(buf, j));

	return 0;
}

static int dbginfo(int argc, char **argv)
{
	printf("RECV Info\n");
	printf("state         %d\n",     RECV.state);
	printf("pos           %p\n",     RECV.pos);
	printf("remainder     %u\n",     RECV.remainder);
	printf("magic         0x%02x%02x%02x%02x\n", RECV.magic[0], RECV.magic[1], RECV.magic[2], RECV.magic[3]);
	printf("length        %u\n",     RECV.length);
	printf("csum          0x%04x\n", RECV.csum);
	printf("body addr     %p\n",     RECV.body);
	printf("crc error     %lu\n",    RECV.crc_error);
	printf("\nMessage Counter\n");
	for (int i = 0; i < RECV_SPI_MSG_MAX; i++)
	{
		printf("[%d] %lu\n", i, RECV.msg_cnt[i]);
	}
	return 0;
}

static const shell_command_t shell_commands[] = {
	/*
	 * Command example
	 *
	 * send 2 deadbeef
	 * sendh 456e756d0400a47801000000
	 * sendh 4178696f1000b497020012345678abcdef01234567890abc
	 */
	{ "getid", "get my ID", getid },
	{ "send", "send data packet", send_data_packet },
	{ "send2", "send large data", send_large_packet },
	{ "enum", "send enum packet", send_enum_packet },
	{ "sendh", "(test) send hex", sendh },
	{ "senda", "(test) send ascii", senda },
	{ "crc16", "(test) calc crc", crc16ccitt },
	{ "dbg",   "(test) debug info", dbginfo },
	{ NULL, NULL, NULL }
};

int main(void)
{
    puts("Hello World!");

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

	RECV.pid = thread_create(_spi_stack, sizeof(_spi_stack),
			THREAD_PRIORITY_MAIN - 2, THREAD_CREATE_WOUT_YIELD | THREAD_CREATE_STACKTEST,
			process_spi_recv, NULL, "spi");

	if (RECV.pid <= 0) {
		printf("thread create failed\n");
		return 1;
	}

	pkt_proc_pid = thread_create(_pkt_stack, sizeof(_pkt_stack),
			THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_WOUT_YIELD | THREAD_CREATE_STACKTEST,
			process_pkt_recv, NULL, "pkt");

	if (pkt_proc_pid <= 0) {
		printf("thread create failed\n");
		return 1;
	}

	daisy_spi_init();

    /* start the shell */
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
