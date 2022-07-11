/*
 * simcom7600.c
 *
 *  Created on: 10 Jun 2022
 *      Author: nguyenphuonglinh
 */


#include "simcom7070.h"

void uart_simcom(void *arg);
void _sendAT(char *AT_command);
AT_res ___readSerial(uint32_t timeout, char* expect);
bool _readSerial(uint32_t timeout);

simcom simcom_7600 = {};
void init_simcom(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baud_rate)
{
	simcom_7600.uart_num = uart_num;
	simcom_7600.tx_io_num = tx_io_num;
	simcom_7600.rx_io_num = rx_io_num;
	simcom_7600.baud_rate = baud_rate;
	uart_config_t uart_config =
	{
		.baud_rate = baud_rate,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	int intr_alloc_flags = 0;
	ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_io_num, rx_io_num, ECHO_TEST_RTS, ECHO_TEST_CTS));
	xTaskCreate(uart_simcom, "uart_echo_task1", 4096, NULL, 10, NULL);
}
void uart_simcom(void *arg)
{
	uint8_t data[BUF_SIZE];
	while (1) {
		int len = uart_read_bytes(simcom_7600.uart_num, data, (BUF_SIZE - 1), 100 / portTICK_PERIOD_MS);
		// Write data back to the UART
		if (len) {
			data[len] = '\0';
			ESP_LOGI(TAG, "Rec: %s", (char*) data);
			if(strstr((char*)data, "+CMQTTRXSTART"))
			{
				simcom_7600.mqtt_CB(data);
			}
			else
			{
				memcpy(simcom_7600.AT_buff, data, len);
				simcom_7600.AT_buff_avai = true;
			}
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}
void _sendAT(char *AT_command)
{
	ESP_LOGI(TAG, "Send: %s", AT_command);
	simcom_7600.AT_buff_avai = false;
	memset(simcom_7600.AT_buff, 0, BUF_SIZE);
	uart_write_bytes(simcom_7600.uart_num, (const char *) AT_command, strlen((char *)AT_command));
	uart_write_bytes(simcom_7600.uart_num, (const char *)"\r\n", strlen("\r\n"));
	vTaskDelay(100/portTICK_PERIOD_MS);
}
AT_res ___readSerial(uint32_t timeout, char* expect)
{
	uint64_t timeOld = esp_timer_get_time() / 1000;
	while (!(esp_timer_get_time() / 1000 > timeOld + timeout))
	{
		vTaskDelay(10/portTICK_PERIOD_MS);
		if(simcom_7600.AT_buff_avai)
		{
			if(strstr((char*)simcom_7600.AT_buff, "ERROR")) return AT_ERROR;
			else if(strstr((char*)simcom_7600.AT_buff, expect)) return AT_OK;
		}
	}
	return AT_TIMEOUT;
}
bool _readSerial(uint32_t timeout)
{
	uint64_t timeOld = esp_timer_get_time() / 1000;
	while (!simcom_7600.AT_buff_avai && !(esp_timer_get_time() / 1000 > timeOld + timeout))
	{
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
	if(simcom_7600.AT_buff_avai == false) return false;
	else return true;
}
bool isInit(int retry)
{
	AT_res res;
	while(retry--)
	{
		_sendAT("AT");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool powerOn(gpio_num_t powerKey)
{
	int retry = 3;
	AT_res res;
	gpio_set_level(powerKey, 1);
	vTaskDelay(200/portTICK_PERIOD_MS);
	gpio_set_level(powerKey, 0);
	vTaskDelay(2000/portTICK_PERIOD_MS);
	gpio_set_level(powerKey, 1);
	vTaskDelay(4000/portTICK_PERIOD_MS);
	res = isInit(retry);
	if (res) return true;
	else return false;
}
bool powerOff()
{
	int retry = 2;
	AT_res res;
	while(retry--)
	{
		_sendAT("AT+CPOWD=1");
		res = ___readSerial(3000, "NORMAL POWER DOWN");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
void powerOff_(gpio_num_t powerKey)
{
	gpio_set_level(powerKey, 1);
	vTaskDelay(200/portTICK_PERIOD_MS);
	gpio_set_level(powerKey, 0);
	vTaskDelay(2000/portTICK_PERIOD_MS);
	gpio_set_level(powerKey, 1);
}
bool echoATSwtich(bool enable)
{
	int retry = 3;
	AT_res res;
	char buff_send[5];
	sprintf(buff_send, "ATE%d", enable);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool setFunction(int fun)
{
	int retry = 2;
	AT_res res;
	char buff_send[10];
	sprintf(buff_send, "AT+CFUN=%d", fun);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(10000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool networkType(network net_type, int retry)
{
	AT_res res;
	char buff_send[20];
	if (net_type == GSM) sprintf(buff_send, "AT+CNMP=%d", 13);
	else if (net_type == NB) sprintf(buff_send, "AT+CNMP=%d", 38);
	else if (net_type == BOTH) sprintf(buff_send, "AT+CNMP=%d", 2);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool isRegistered(int retry)
{
	while(retry--)
	{
		vTaskDelay(1000/portTICK_PERIOD_MS);
		_sendAT("AT+CREG?");
		if(_readSerial(1000) == false) continue;
		if(strstr((char*)simcom_7600.AT_buff, "0,1") || strstr((char*)simcom_7600.AT_buff, "0,5") || strstr((char*)simcom_7600.AT_buff, "1,1") || strstr((char*)simcom_7600.AT_buff, "1,5")) return true;
		else continue;
	}
	return false;
}
bool networkInfor(int retry, Network_Signal* network)
{
	AT_res res;
	while (retry--)
	{
		_sendAT("AT+CPSI?");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK)
		{
			CPSI_Decode((char*)simcom_7600.AT_buff, network);
			return true;
		}
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool checkNetworkActiveStatus(int * status)
{
	int retry = 2;
	AT_res res;
	while (retry--)
	{
		_sendAT("AT+CNACT?");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK)
		{
			if(strstr((char*)simcom_7600.AT_buff, "0,1")) *status = 1;
			else status = 0;
		}
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool activeNetwork()
{
	int retry = 2;
	bool _res;
	AT_res res;
	int active_status;
	_res = checkNetworkActiveStatus(&active_status);
	if (!_res) return false;
	if (active_status == 1) return true;
	while (retry--)
	{
		_sendAT("AT+CNACT=0,1");
		res = ___readSerial(1000, "+APP PDP: 0,ACTIVE");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool deactiveNetwork()
{
	int retry = 2;
	bool _res;
	AT_res res;
	int active_status;
	_res = checkNetworkActiveStatus(&active_status);
	if (!_res) return false;
	if (active_status == 1) return true;
	while (retry--)
	{
		_sendAT("AT+CNACT=0,0");
		res = ___readSerial(1000, "+APP PDP: 0,DEACTIVE");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool mqttConnect(client clientMqtt)
{
	int retry = 3;
	bool res = false;
	char buff_send[200];
	sprintf(buff_send, "AT+SMCONF=\"URL\",%s", clientMqtt.url);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if(res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	sprintf(buff_send, "AT+SMCONF=\"USERNAME\",\"%s\"", clientMqtt.user_name);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if(res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	sprintf(buff_send, "AT+SMCONF=\"PASSWORD\",\"\%s""", clientMqtt.password);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if(res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	sprintf(buff_send, "AT+SMCONF=\"CLIENTID\",\"%s\"", clientMqtt.client_id);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if(res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	sprintf(buff_send, "AT+SMCONF=\"KEEPTIME\",%d", clientMqtt.time_alive);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if(res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	sprintf(buff_send, "AT+SMCONF=\"CLEANSS\",%d", clientMqtt.cleans);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if(res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	retry = 2;
	while(retry--)
	{
		_sendAT("AT+SMCONN");
		res = ___readSerial(10000, "OK");
		if(res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool mqttDisconnect(client clientMqtt)
{
	int retry = 2;
	AT_res res;
	while(retry--)
	{
		_sendAT("AT+SMDISC");
		res = ___readSerial(2000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool mqttPublish(client clientMqtt, char* data, char* topic, int qos, bool retain)
{
	AT_res res;
	int retry = 2;
	char buff_send[200];
	while (retry--)
	{
		sprintf(buff_send, "AT+SMPUB=\"%s\",%d,%d,%d", topic, strlen(data)+2, qos, retain);
		_sendAT(buff_send);
		res = ___readSerial(1000, ">");
		if (res == AT_ERROR) return false;
		else if (res == AT_OK)
		{
			_sendAT(data);
			res = ___readSerial(2000, "OK");
			if (res == AT_OK) return true;
			else if (res == AT_ERROR) return false;
		}
	}
	return false;
}
bool mqttSubcribe(client clientMqtt, char* topic, int qos, void (*mqttSubcribeCB)(char * data))
{
	AT_res res;
	int retry = 2;
	char buff_send[50];
	sprintf(buff_send, "AT+SMSUB=\"%s\",%d", topic, qos);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(2000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool sendSMS(char *phone, char *text)
{
	AT_res res;
	char buff_send[100];
	int retry = 3;
	while (retry--)
	{
		_sendAT("AT+CMGF=1");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	retry = 3;
	sprintf(buff_send, "AT+CMGS=\"%s\"", phone);
	while (retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, ">");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	retry = 3;
	while (retry--)
	{
		char data[] = { 0x1A };
		_sendAT(text);
		uart_write_bytes(simcom_7600.uart_num, data, sizeof(data));
		res = ___readSerial(10000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool httpGet(char * url, uint32_t* len)
{
	AT_res res;
	int retry = 2;
	char buff_send[100];
	sprintf(buff_send, "AT+HTTPPARA=\"URL\",\"%s\"", url);
	while(retry--)
	{
		_sendAT("AT+HTTPINIT");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	retry = 2;
	while (retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	retry = 2;
	while(retry--)
	{
		_sendAT("AT+HTTPACTION=0");
		res = ___readSerial(5000, "+HTTPACTION: 0,200");
		if (res == AT_OK)
		{
			sscanf((char*)simcom_7600.AT_buff, "\r\n+HTTPACTION: 0,200,%d\r\n\r\nOK\r\n", len);
			printf("content_lengt: %d\r\n", *len);
			sscanf((char*)simcom_7600.AT_buff, "\r\nOK\r\n\r\n+HTTPACTION: 0,200,%d\r\n\r\nOK\r\n", len);
			return true;
		}
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool httpReadRespond(uint8_t* data, int len_expect, uint16_t *len_real)
{
	AT_res res;
	int retry = 2;
	char buff_send[30];
	sprintf(buff_send, "AT+HTTPREAD=%d", len_expect);
	while (retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK)
		{
			int read_len;
			sscanf((char*)simcom_7600.AT_buff, "\r\nOK\r\n\r\n+HTTPREAD: DATA,%d", &read_len);
			*len_real = read_len;
			uint8_t buff_temp[read_len+strlen("\r\n+HTTPREAD:0\r\n")];
			int index_CRLF = 0;
			for(int i = 0; i < BUF_SIZE; i++)
			{
				if(*(simcom_7600.AT_buff+i-1) == '\n' && *(simcom_7600.AT_buff+i-2) == '\r')
				{
					index_CRLF++;
				}
				if(index_CRLF == 4)
				{
					memcpy(buff_temp, simcom_7600.AT_buff+i, read_len+strlen("\r\n+HTTPREAD:0\r\n"));
					break;
				}
			}
			memcpy(data, buff_temp, read_len);
			return true;
		}
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool getLBS(LBS *LBS_infor)
{
	AT_res res;
	int retry = 2;
	char buf_temp[50];
	while (retry--)
	{
		_sendAT("AT+CLBS=1");
		res = ___readSerial(10000, "+CLBS:");
		if (res == AT_OK)
		{
			if (strstr((char*)simcom_7600.AT_buff, "+CLBS: 0"))
			{
				getSubStrig((char*)simcom_7600.AT_buff, "+CLBS: 0,", "\r\n", buf_temp);
				sscanf(buf_temp, "%f,%f,%"SCNd16"", &LBS_infor->lat, &LBS_infor->lon, &LBS_infor->acc);
				LBS_infor->fix_status = 1;
				return true;
			}
			else return false;
		}
	}
	return false;
}
bool getBatteryInfor(Bat *Battery)
{
	int retry = 2;
	AT_res res;
	while (retry--)
	{
		_sendAT("AT+CBC");
		res = ___readSerial(10000, "OK");
		if (res == AT_OK)
		{
			if (strstr((char*)simcom_7600.AT_buff, "+CBC: "))
			{
				sscanf((char*)simcom_7600.AT_buff, "+CBC: %d,%d,%"SCNd16"", &Battery->charge_status, &Battery->level, &Battery->voltage);
				return true;
			}
			else return false;
		}
	}
	return false;
}
