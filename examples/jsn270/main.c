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
 * @brief       Application using jsn270 to send receive data over wifi
 *
 * @author      KM Kim <kkim@securityplatform.co.kr>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "thread.h"
#include "shell.h"
#include "shell_commands.h"
#include <periph/gpio.h>

#include "jsn270.h"

#define SSID      "909f3377067c_24"		// your wifi network SSID
#define KEY       "SP@BravoSK"		// your wifi network password
#define AUTH       "WPA2" 		// your wifi network security (NONE, WEP, WPA, WPA2)


#define HOST_IP        "192.168.122.17"
#define REMOTE_PORT    1883
#define ID       "admin"			// id should not be null
#define PW       "admin"			// pw should not be null
#define SUB_TOPIC	"/jsn270"
#define PUB_TOPIC	"/jsn270"

#define MESSAGE "JSN270"		// no space allowed


static int ifconfig_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return 0;
}

static int wlan_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (argc < 4) {
	(void)puts("usage: wlan ssid psk auth");
    }
    jsn270_join(argv[1], argv[2], argv[3]);
    return 0;
}

static int reset_cmd(int argc, char **argv)
{
    jsn270_reset();
    return 0;
}

static const shell_command_t shell_commands[] = {
    { "ifconfig", "Shows assigned IPv4 addresses", ifconfig_cmd },
    { "wlan", "Shows or sets wlan ssid configurations", wlan_cmd },
    { "reset", "reset jsn270", reset_cmd },
    { NULL, NULL, NULL }
};
char line_buf[SHELL_DEFAULT_BUFSIZE];

int main(void)
{
	char c;

    gpio_init(D3, GPIO_IN);
    gpio_init(D2, GPIO_IN);

    jsn270_init(UART_DEV(0));

    (void) puts("Welcome to RIOT!");
	msleep(5000);
	jsn270_sendcommand("at+ver\r", NULL, 0);
	msleep(10);
	while(jsn270_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	}
	msleep(1000);

	jsn270_dynamicIP();

	if (jsn270_join(SSID, KEY, AUTH)) {
		printf("WiFi connect to " SSID);
	}
	msleep(1000);

	jsn270_sendcommand("at+wstat\r", NULL, 0);
	msleep(5);
	while(jsn270_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	}
	msleep(1000);        

	jsn270_sendcommand("at+nstat\r", NULL, 0);
	msleep(5);
	while(jsn270_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	}
	msleep(1000);

	jsn270_mqtt_set(HOST_IP, REMOTE_PORT, ID, PW, SUB_TOPIC, PUB_TOPIC);
	msleep(1000);

	// subscribe topic
	jsn270_mqtt_sub();
	msleep(1000);

	while(jsn270_receive((uint8_t *)&c, 1, 1000) > 0) {
		printf("%c", (char)c);
	}

	// publish message
	jsn270_mqtt_pub(MESSAGE);

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
/* vim: ai: sts=4 sw=4
 */
