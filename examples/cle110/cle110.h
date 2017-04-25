//#pragma once

#ifndef CLE110_H 
#define CLE110_H

#include <net/sock/udp.h>
#include <net/sock/tcp.h>
#include <ringbuffer.h>
#include <periph/uart.h>

#define msleep(time) usleep(time * 1000)
void cle110_init(uart_t uart);
int cle110_write(uint8_t c);
void cle110_clear(void);

void cle110_reset(void);
int cle110_receive(uint8_t *buf, int len, int timeout);
int cle110_sendcommand(char *cmd, char *ack, int timeout);

int cle110_connect(char *addr);
int cle110_reconnect(void);
int cle110_autoconnect_on(void);
int cle110_autoconnect_off(void);
int cle110_disconnect(void);
#endif
