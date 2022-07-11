/*
 * simcom7600.h
 *
 *  Created on: 10 Jun 2022
 *      Author: nguyenphuonglinh
 */

#ifndef SIMCOM7600_SIMCOM7600_H_
#define SIMCOM7600_SIMCOM7600_H_
#include "simcom7070.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "../common.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "../string_user/location_parser.h"
#include "../string_user/string_parse.h"
#include "gps_nmea/GPS.h"
#include "7070_config.h"
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_TASK_STACK_SIZE    (2048)
#define BUF_SIZE (2048)

typedef struct simcom_t
{
	uart_port_t uart_num;
	int tx_io_num;
	int rx_io_num;
	int baud_rate;
	bool AT_buff_avai;
	uint8_t AT_buff[BUF_SIZE];
	void (*mqtt_CB)();
}simcom;
typedef struct client_t
{
	char user_name[20];
	char password[20];
	char client_id[50];
	char url[50];
	uint32_t time_alive;
	bool cleans;
	bool retain;
}client;
typedef struct LBS_t
{
	float lat;
	float lon;
	uint16_t acc;
	bool fix_status;
}LBS;
typedef struct _Bat
{
	int charge_status;
	int level;
	uint16_t voltage;
}Bat;
typedef enum
{
	AT_OK,
	AT_ERROR,
	AT_TIMEOUT,
}AT_res;


bool powerOn(gpio_num_t powerKey);
bool powerOff();
void powerOff_(gpio_num_t powerKey);
bool echoATSwtich(bool enable);
bool getBatteryInfor(Bat *Battery);
bool setFunction(int fun);
bool networkType(network net_type, int retry);
bool isRegistered(int retry);
bool networkInfor(int retry, Network_Signal* network);
bool checkNetworkActiveStatus(int * status);
bool activeNetwork();
bool deactiveNetwork();
bool mqttConnect(client clientMqtt);
bool mqttDisconnect(client clientMqtt);
bool mqttPublish(client clientMqtt, char* data, char* topic, int qos, bool retain);
bool mqttSubcribe(client clientMqtt, char* topic, int qos, void (*mqttSubcribeCB)(char * data));
bool sendSMS(char *phone, char *text);
bool httpGet(char * url, uint32_t* len);
bool httpReadRespond(uint8_t* data, int len_expect, uint16_t *len_real);
bool getLBS(LBS *LBS_infor);
#endif /* SIMCOM7600_SIMCOM7600_H_ */
