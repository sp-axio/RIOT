#include <jsn270.h>
#include <string.h>
#include <timex.h>
#include <xtimer.h>
#include <unistd.h>

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

void jsn270_init(uart_t uart)
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


int jsn270_write(uint8_t c)
{
//    printf("write '%c'\r\n", c);
    uart_write(_uart, &c, 1);
//    printf("return\r\n");
    return 1;
}

int jsn270_read(void)
{
    return (int) ringbuffer_get_one(&rx_buf);
}

int jsn270_send(const uint8_t *data, int len, int timeout)
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
	if (jsn270_write(data[write_bytes]) == 1) {
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

int jsn270_append(char *p, int len, int pos)
{
    char c = jsn270_read();
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

int jsn270_find(char *a, int l, int timeout)
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
	printf("start:%ld\n", start);

//	printf("string:\"%s\" len:%d resp:%s\n", a, l, p);

    while(memcmp(a, p, l) != 0) {
		n = jsn270_append(p, l, n);
		now = millis();
//		printf("curtime :%ld\n", now);
		if (timeout && now - start > timeout){
			printf("resp timeout:\"%s\", cur:%ld spand time :%ld\n", p, now, now - start);
			goto out;
		}
	}
//	printf("resp:\"%s\"\n", p);
    found = 1;
out:
    free(p);
    return found;
}

int jsn270_ask(char *q, char *a, int timeout)
{
    int q_len = strlen(q);
    printf("send \"%s\", %d\r\n", q, q_len);

    jsn270_send((uint8_t *)q, q_len, timeout);
    if (a != NULL) {
		int found = jsn270_find((char *)a, strlen(a), timeout);
		if (!found) {
//			printf("not found\r\n");
			return 0;
	}
//	printf("found\r\n");
    }
    return 1;
}

int jsn270_receive(uint8_t *buf, int len, int timeout)
{
    int read_bytes = 0;
    int ret;
    unsigned long end_millis;

    while (read_bytes < len) {
    end_millis = millis() + timeout;
    do {
	ret = jsn270_read();
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

void jsn270_clear(void)
{
    char r;
    while(jsn270_receive((uint8_t*)&r, 1, 10) == 1) {
    ;
    }
}

int jsn270_sendcommand(char *cmd, char *ack, int timeout)
{
//    printf("call clear()\r\n");
    jsn270_clear();
//    printf("after clear()\r\n");
    if (!jsn270_ask(cmd, ack, timeout)) {
//	printf("after ask()\r\n");
	error_count++;
	return 0;
    }
    error_count = 0;
    return 1;
}

void jsn270_reset(void)
{
    jsn270_sendcommand("at\r", "[OK]", 0);
    msleep(10);
    jsn270_sendcommand("at+reset\r", "[OK]", 0);
    msleep(1000);
}

void jsn270_prompt(void)
{
    jsn270_sendcommand("at\r", "[OK]", 0);
}

static int jsn270_associated = 0;
int jsn270_leave(void)
{
    if (jsn270_sendcommand("at+wclose\r", "[OK]", 0)) {
    jsn270_associated = 0;
    return 1;
    }
    return 0;
}

#define MAX_CMD_LEN 256
int jsn270_staticIP(const char *ip, const char *mask, const char *gateway)
{
    char cmd[MAX_CMD_LEN];

    jsn270_sendcommand("at+ndhcp=0\r", "[OK]", 0); 
    msleep(10);
    jsn270_sendcommand("at+nset=", NULL, 0);
    snprintf(cmd, MAX_CMD_LEN, "%s,%s,%s\r", ip, mask, gateway);
    jsn270_sendcommand(cmd, "[OK]", 0);
    return 1;
}

int jsn270_dynamicIP(void)
{
    if (jsn270_sendcommand("at+ndhcp=1\r", "[OK]", 0)) { 
	return 1;
    }
    return 0;
}

// mqtt client
int jsn270_mqtt_set(const char *host, uint16_t port, const char *id, const char *pw, const char *sub_topic, const char *pub_topic)
{
    char cmd[MAX_CMD_LEN];

    // broker ip
    if (host != NULL) {
	memset(cmd, 0, MAX_CMD_LEN);
	snprintf(cmd, sizeof(cmd), "at+mqtt_set=0 %s\r", host);
	jsn270_sendcommand(cmd,"[OK]", 0);    
	msleep(10);
    }
    else {
	return 0;
    }

    // port
    if (port != 0) {
	memset(cmd, 0, MAX_CMD_LEN);
	snprintf(cmd, sizeof(cmd), "at+mqtt_set=1 %d\r", port);
	jsn270_sendcommand(cmd,"[OK]", 0);    
	msleep(10);
    }
    else {
	return 0;
    }

    // id
    if (id != NULL) {
	memset(cmd, 0, MAX_CMD_LEN);
	snprintf(cmd, sizeof(cmd), "at+mqtt_set=4 %s\r", id);
	jsn270_sendcommand(cmd,"[OK]", 0);    
	msleep(10);
    }
    else {
	return 0;
    }

    // pw
    if (pw != NULL) {
	memset(cmd, 0, MAX_CMD_LEN);
	snprintf(cmd, sizeof(cmd), "at+mqtt_set=5 %s\r", pw);
	jsn270_sendcommand(cmd,"[OK]", 0);    
	msleep(10);
    }
    else {
	return 0;
    }

    // subscribe topic
    if (sub_topic != NULL) {
	memset(cmd, 0, MAX_CMD_LEN);
	snprintf(cmd, sizeof(cmd), "at+mqtt_set=7 %s\r", sub_topic);
	jsn270_sendcommand(cmd,"[OK]", 0);    
	msleep(10);
    }
    else {
	return 0;
    }

    // publish topic
    if (pub_topic != NULL) {
	memset(cmd, 0, MAX_CMD_LEN);
	snprintf(cmd, sizeof(cmd), "at+mqtt_set=6 %s\r", pub_topic);
	jsn270_sendcommand(cmd,"[OK]", 0);    
	msleep(10);
    }
    else {
	return 0;
    }

    return 1;
}

// mqtt subscribe
int jsn270_mqtt_sub(void)
{
    char cmd[MAX_CMD_LEN];

    snprintf(cmd, sizeof(cmd), "at+mqtt_sub\r");
    jsn270_sendcommand(cmd,"[OK]", 0);    
    msleep(10);

    return 1;
}

// mqtt publish
int jsn270_mqtt_pub(const char *message)
{
    char cmd[MAX_CMD_LEN];

    if (message != NULL) {
	snprintf(cmd, sizeof(cmd), "at+mqtt_pub=3 %s\r", message);
	jsn270_sendcommand(cmd,"[OK]", 0);    
	msleep(10);
    }
    else {
	return 0;
    }

    return 1;
}

// client
int jsn270_client(const char *host, uint16_t port, const char *protocol)
{
    char cmd[MAX_CMD_LEN];

    if (!strcmp(protocol, "TCP")) {
	snprintf(cmd, sizeof(cmd), "at+nauto=0,1,%s,%d\r", host, port);
    }
    else if (!strcmp(protocol, "UDP")) {
	snprintf(cmd, sizeof(cmd), "at+nauto=0,0,%s,%d\r", host, port);
    }

    jsn270_sendcommand(cmd,"[OK]", 0);    
    msleep(10);

    if (jsn270_sendcommand("at+socket\r","CONNECT ", 0)) {
	jsn270_clear();
	return 1;
    }
    return 0;
}

// server
int jsn270_server(uint16_t localport, const char *protocol)
{
    char cmd[MAX_CMD_LEN];
    
    if (!strcmp(protocol, "TCP")) {
	snprintf(cmd, sizeof(cmd), "at+nauto=1,1,0.0.0.0,%d\r", localport);
    }
    else if (!strcmp(protocol, "UDP")) {
	snprintf(cmd, sizeof(cmd), "at+nauto=1,0,0.0.0.0,%d\r", localport);
    }

    jsn270_sendcommand(cmd,"[OK]", 0);
    msleep(10);
    
    if (jsn270_sendcommand("at+socket\r","[OK]", 0)) {
	jsn270_clear();
	return 1;
    }
    return 0;
}

int jsn270_connected(void)
{
    int found = jsn270_find("CONNECTED 0", 11, 0);
    if (found) {
	return 1;
    }
    return 0;
}

int jsn270_disconnect(void)
{
    if (jsn270_sendcommand("at+nclose\r","[OK]", 0)) {
	return 1;
    }
    return 0;
}

int jsn270_join(const char *ssid, const char *phrase, const char *auth)
{
    char cmd[MAX_CMD_LEN];

    // ssid
    snprintf(cmd, MAX_CMD_LEN, "at+wauto=0,%s\r", ssid);
    jsn270_sendcommand(cmd,"[OK]", 0);

    //key
    if (!strcmp(auth, "WPA2") || !strcmp(auth, "WPA")) {
	snprintf(cmd, MAX_CMD_LEN, "at+wwpa=%s\r", phrase);
	jsn270_sendcommand(cmd,"[OK]", 0);
    }
    else if (!strcmp(auth, "WEP")) {
	snprintf(cmd, MAX_CMD_LEN, "at+wwep=%s\r", phrase);
	jsn270_sendcommand(cmd,"[OK]", 0);
    }

    if (jsn270_sendcommand("at+wstart\r", "[IP ACQUIRED]", 0)) {
	jsn270_associated = 0;
	return 1;
    }

    return 0;
}


/* vim: ai: sts=4 sw=4
 */
