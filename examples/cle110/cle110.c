#include <string.h>
#include <timex.h>
#include <xtimer.h>
#include <unistd.h>

#include "cle110.h"

static uart_t _uart;
static int error_count = 0;

#define RX_BUFSIZE 256
static ringbuffer_t rx_buf;
char rx_mem[RX_BUFSIZE];
void rx_cb_uart1(void *arg, uint8_t c)
{
    ringbuffer_t *buf = (ringbuffer_t *)arg;
//	printf("rx_cb...[%c]\n", c);
    ringbuffer_add_one(buf, (char) c);
}

void cle110_init(uart_t uart)
{
    _uart = uart;
	memset(rx_mem, 0x00, RX_BUFSIZE);
    ringbuffer_init(&rx_buf, rx_mem, RX_BUFSIZE);
    uart_init(uart, 9600, rx_cb_uart1, &rx_buf);
}


int jiffies = 0;
unsigned long millis(void)
{
    return (unsigned long)(xtimer_now_usec()/US_PER_MS);
}


int cle110_write(uint8_t c)
{
//    printf("write '%c'\r\n", c);
    uart_write(_uart, &c, 1);
//    printf("return\r\n");
    return 1;
}

int cle110_read(void)
{
    return (int) ringbuffer_get_one(&rx_buf);
}

int cle110_send(const uint8_t *data, int len, int timeout)
{
    int write_bytes = 0;
    int write_error = 0;
    unsigned long start_millis;

    if (data == NULL) {
	return 0;
    }
    start_millis = millis();
//    printf("start milli = %ld, timeout %d\r\n", start_millis, timeout);
    while(write_bytes < len) {
	if (cle110_write(data[write_bytes]) == 1) {
	    write_bytes ++;
	    write_error = 0;
	} else {
	    if (write_error) {
		if (timeout == 0 || (millis() - start_millis) > timeout) {
		    break;
		}
	    } else {
		write_error = 1;
		start_millis = millis();
	    }
	}
    }
    return write_bytes;
}

int cle110_append(char *p, int len, int pos)
{
    char c = cle110_read();
    int i;

	if (c > 127)  // skip non-ascii char
		return pos;

//	printf("\'%c\'\n", c);
    if (pos >= len) {
		for (i=0; i<len; ++i) {
			p[i] = p[i+1];
		}
		p[len-1] = c;
		return len;
	} 
    p[pos] = c;
    return pos + 1;
}

int cle110_find(char *a, int l, int timeout)
{
    unsigned long start;
    unsigned long now;
    char *p = malloc(l);
    int n = 0;
    int found = 0;
	memset(p, 0x00, l);

	if(timeout == 0)
		timeout = 15000;
    start = millis();
//	printf("start:%ld\n", start);

//	printf("string:\"%s\" len:%d resp:%s\n", a, l, p);

    while(memcmp(a, p, l) != 0) {
		n = cle110_append(p, l, n);
		now = millis();
//		printf("curtime :%ld\n", now);
		if (timeout && now - start > timeout){
//			printf("resp timeout:\"%s\", spand time :%ld\n", p, now - start);
			goto out;
		}
	}
//	printf("resp:\"%s\"\n", p);
    found = 1;
out:
    free(p);
    return found;
}

int cle110_ask(char *q, char *a, int timeout)
{
    int q_len = strlen(q);
    printf("send \"%s\", %d\r\n", q, q_len);

    cle110_send((uint8_t *)q, q_len, timeout);
    if (a != NULL) {
		int found = cle110_find((char *)a, strlen(a), timeout);
		if (!found) {
//			printf("not found\r\n");
			return 0;
	}
//	printf("found\r\n");
    }
    return 1;
}

int cle110_receive(uint8_t *buf, int len, int timeout)
{
    int read_bytes = 0;
    int ret;
    unsigned long end_millis;

    while (read_bytes < len) {
    end_millis = millis() + timeout;
    do {
	ret = cle110_read();
	if (ret >= 0) {
	break;
	}
    } while (millis() < end_millis);

    if (ret < 0) {
	return read_bytes;
    }
    buf[read_bytes] = (char)ret;
    read_bytes++;
//	printf("read...:[%c]\n", ret);
    }

    return read_bytes;
}

void cle110_clear(void)
{
    char r;
    while(cle110_receive((uint8_t*)&r, 1, 10) == 1) {
    ;
    }
}

int cle110_sendcommand(char *cmd, char *ack, int timeout)
{
//    printf("call clear()\r\n");
    cle110_clear();
//    printf("after clear()\r\n");
    if (!cle110_ask(cmd, ack, timeout)) {
//	printf("after ask()\r\n");
	error_count++;
	return 0;
    }
    error_count = 0;
    return 1;
}

void cle110_reset(void)
{
    cle110_sendcommand("at\r", "[OK]", 0);
    msleep(10);
    cle110_sendcommand("at+reset\r", "[OK]", 0);
    msleep(1000);
}

void cle110_prompt(void)
{
    cle110_sendcommand("at\r", "[OK]", 0);
}

static int cle110_associated = 0;
int cle110_leave(void)
{
    if (cle110_sendcommand("at+wclose\r", "[OK]", 0)) {
    cle110_associated = 0;
    return 1;
    }
    return 0;
}

#define MAX_CMD_LEN 256
int cle110_connect(char *addr)
{
    char cmd[MAX_CMD_LEN];
	snprintf(cmd, sizeof(cmd), "AT+CONNECT=%s\r", addr);
    if (cle110_sendcommand(cmd, "+CONNECTED", 0)) {
	return 1;
    }
    return 0;
}

int cle110_reconnect(void)
{
    if (cle110_sendcommand("AT+RECONNECT\r","+CONNECTED", 0)) {
	return 1;
    }
    return 0;
}

int cle110_autoconnect_on(void)
{
    if (cle110_sendcommand("AT+AUTOCONNECT=ON\r","+OK", 0)) {
	return 1;
    }
    return 0;
}

int cle110_autoconnect_off(void)
{
    if (cle110_sendcommand("AT+AUTOCONNECT=OFF\r","+OK", 0)) {
	return 1;
    }
    return 0;
}


int cle110_disconnect(void)
{
    if (cle110_sendcommand("AT+DISCONNECT\r","+DISCONNECTED", 0)) {
	return 1;
    }
    return 0;
}


/* vim: ai: sts=4 sw=4
 */
