#pragma once

#include <net/sock/udp.h>
#include <net/sock/tcp.h>
#include <ringbuffer.h>
#include <periph/uart.h>

#define msleep(time) usleep(time * 1000)
int jsn270_write(uint8_t c);
void jsn270_init(uart_t uart);
void jsn270_clear(void);

void jsn270_reset(void);
int jsn270_dynamicIP(void);
int jsn270_join(const char *ssid, const char *phrase, const char *auth);
int jsn270_receive(uint8_t *buf, int len, int timeout);
int jsn270_sendcommand(char *cmd, char *ack, int timeout);


int jsn270_mqtt_set(const char *host, uint16_t port, const char *id, const char *pw, const char *sub_topic, const char *pub_topic);
int jsn270_mqtt_sub(void);
int jsn270_mqtt_pub(const char *message);

