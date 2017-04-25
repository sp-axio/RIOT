/*
 * Copyright 2017 Security Platform Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @ingroup     
 * @{
 *
 * @file
 * @brief       Application using cle110 to send receive data over ble 
 *
 * @author      KM Kim <kkim@securityplatform.co.kr>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "cle110.h"
#include "thread.h"
#include "shell.h"
#include "shell_commands.h"
#include <periph/gpio.h>


static int ifconfig_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return 0;
}


static int reset_cmd(int argc, char **argv)
{
    cle110_reset();
    return 0;
}

static const shell_command_t shell_commands[] = {
    { "ifconfig", "Shows assigned IPv4 addresses", ifconfig_cmd },
    { "reset", "reset cle110", reset_cmd },
    { NULL, NULL, NULL }
};
char line_buf[SHELL_DEFAULT_BUFSIZE];

int main(void)
{
	char c;

    cle110_init(UART_DEV(0));

    (void) puts("Welcome to RIOT!");

	cle110_sendcommand("AT\r", NULL, 0); 
	cle110_sendcommand("AT\r", NULL, 0); msleep(1000);
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

	cle110_sendcommand("ATZ\r", NULL, 0); 
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

	cle110_sendcommand("AT&F\r", NULL, 0); msleep(5000);
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

	cle110_sendcommand("AT+ROLE?\r", NULL, 0); msleep(2000);  
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

	cle110_sendcommand("AT+SERVER=P\r", NULL, 0); msleep(2000); 
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

	cle110_sendcommand("AT+MANUF=CINA2\r", NULL, 0); msleep(2000); 
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

	cle110_sendcommand("AT+TXPWR?\r", NULL, 0); msleep(2000); 
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

	cle110_sendcommand("AT+ADVDATA=ENCR\r", NULL, 0); msleep(2000);
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

	cle110_sendcommand("AT+ADVDATA?\r", NULL, 0); msleep(2000); 
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

	cle110_sendcommand("AT+ADVINTERVAL=100\r", NULL, 0); msleep(2000); 
	while(cle110_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	} printf("\n");

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
/* vim: ai: sts=4 sw=4
 */
