/*
 * FOTA.c
 *
 *  Created on: Sep 16, 2021
 *      Author: HAOHV6
 */
#include "../MyLib/FOTA.h"

#include "../MyLib/ESP32_GPIO.h"
#include "../MyLib/SPIFFS_user.h"

#define FIRMWARE_VERSION	0.3
//server test
//#define UPDATE_JSON_URL		"http://202.191.56.104:5551/uploads/fota_profile.txt"

#ifdef TEST
	#define UPDATE_JSON_URL		"http://202.191.56.104:5551/uploads/fota_profile_test.txt"
#else
	#define UPDATE_JSON_URL		"http://171.244.133.231:8080/Thingworx/Things/2958380837/Services/OTA"
#endif

char rcv_buffer[200];
static EventGroupHandle_t s_wifi_event_group;
TaskHandle_t xHandle_SC = NULL;
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const char *TAG_wifi = "smartconfig";

#define MAIN_TASK_PRIO     			1
#define UART_RX_TASK_PRIO      		2
#define CHECKTIMEOUT_TASK_PRIO     	3

extern bool Flag_Fota;
extern bool Flag_Fota_success;
extern bool Flag_wifi_init;
extern bool Flag_Fota_fail;
extern RTC_DATA_ATTR char DeviceID_TW_Str[50];
extern RTC_DATA_ATTR Device_Param VTAG_DeviceParameter;
extern Network_Signal VTAG_NetworkSignal;

extern TaskHandle_t main_task_handle;
extern TaskHandle_t uart_rx_task_handle;
extern TaskHandle_t check_timeout_task_handle;
extern TaskHandle_t gps_scan_task_handle;
extern TaskHandle_t gps_processing_task_handle;
extern TaskHandle_t mqtt_submessage_processing_handle;
TaskHandle_t time_out_handle;
extern RTC_DATA_ATTR CFG VTAG_Configure;
extern char *base_path;
extern void GetDeviceTimestamp(void);
extern void main_task(void *arg);
extern void uart_rx_task(void *arg);
extern void check_timeout_task(void *arg);
extern void gps_scan_task(void *arg);
extern void gps_processing_task(void *arg);
extern void mqtt_submessage_processing_task(void *arg);
extern void  ResetAllParameters();
extern void Create_Tasks();
extern bool Flag_Fota_led;
static void mqtt_app_start_ack(void);
extern void fota_ack(void *arg);
extern CFG VTAG_Configure_temp;
extern bool Flag_wifi_got_led;
extern void JSON_Analyze(char* my_json_string, CFG* config);
static const char *TAG = "MQTT_EXAMPLE";
extern void ESP32_Clock_Config(int Freq_Max, int Freq_Min, bool LighSleep_Select);
bool Flag_cancel_timeout = false;
bool step_get = false;
/*------------------------------------log_error_if_nonzero-------------------------------------------------*/
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
/*------------------------------------MQTT hanlder-------------------------------------------------*/
static void mqtt_event_handler_ack(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED\r\n");
        GetDeviceTimestamp();
		memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
		sprintf(MQTT_ID_Topic, "messages/%s/control", DeviceID_TW_Str);
        msg_id = esp_mqtt_client_subscribe(client, MQTT_ID_Topic, 1);
        ESP_LOGI(TAG, "subcribe to topic: %s\r\n", MQTT_ID_Topic);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
		GetDeviceTimestamp();
		memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
		sprintf(MQTT_ID_Topic, "messages/%s/gps", DeviceID_TW_Str);
		MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "GET", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network);
		msg_id = esp_mqtt_client_publish(client, MQTT_ID_Topic, Mqtt_TX_Str, 200, 1, 0);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
    	ESP_LOGI(TAG, "sent publish successful\r\n");
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA\r\n");
        ESP_LOGI(TAG, "Data sub: %s\r\n", event->data);
        //Flag_Fota = false;
        JSON_Analyze(event->data, &VTAG_Configure_temp);
        if(step_get == true)
        {
        	xTaskCreate(&check_update_task, "check_update_task",1024 * 8, NULL, 5, NULL);
        	step_get = false;
        }
        else
        {
        	step_get = true;
			GetDeviceTimestamp();
			memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
			sprintf(MQTT_ID_Topic, "messages/%s/devconf", DeviceID_TW_Str);
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOS", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network);
			msg_id = esp_mqtt_client_publish(client, MQTT_ID_Topic, Mqtt_TX_Str, 200, 1, 0);
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
    	ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED\r\n");
		GetDeviceTimestamp();
		memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
		sprintf(MQTT_ID_Topic, "messages/%s/control", DeviceID_TW_Str);
		msg_id = esp_mqtt_client_subscribe(client, MQTT_ID_Topic, 1);
		ESP_LOGI(TAG, "subcribe to topic: %s\r\n", MQTT_ID_Topic);
		break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
		GetDeviceTimestamp();
		memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
		sprintf(MQTT_ID_Topic, "messages/%s/devconf", DeviceID_TW_Str);
		MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOSS", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Version_next, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network);
		msg_id = esp_mqtt_client_publish(client, MQTT_ID_Topic, Mqtt_TX_Str, 200, 1, 0);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
    	ESP_LOGI(TAG, "sent publish successful\r\n");
		break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        Flag_Fota = false;
        gpio_set_level(LED_1, 1);
		vTaskDelay(2000/portTICK_PERIOD_MS);
		esp_restart();
        break;

    case MQTT_EVENT_ERROR:
    	ESP_LOGI(TAG, "MQTT_EVENT_DATA\r\n");
		ESP_LOGI(TAG, "Data sub: %s\r\n", event->data);
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void check_timeout(void *arg)
{
	uint16_t start_timeout = 120;
	while(1)
	{
		start_timeout--;
		//ESP_LOGE(TAG, "Time remaining: %d", start_timeout);
		vTaskDelay(1000/portTICK_PERIOD_MS);
		if(start_timeout == 0 && Flag_cancel_timeout == false)
		{
			esp_restart();
		}
	}
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
#ifdef SERVER_TEST
        .uri = "mqtt://203.113.138.18:4445",
#else
		.uri = "mqtt://171.244.133.251:1883",
#endif
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static void mqtt_app_start_ack(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
#ifdef SERVER_TEST
        .uri = "mqtt://203.113.138.18:4445",
#else
		.uri = "mqtt://171.244.133.251:1883",
#endif
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler_ack, NULL);
    esp_mqtt_client_start(client);
}

void smartconfig_task(void *parm)
{
	printf("smartconfig_example_task\r\n");
	EventBits_t uxBits;
	ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
	smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
	while (1)
	{
		uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
		if (uxBits & CONNECTED_BIT)
		{
			ESP_LOGI(TAG_wifi, "WiFi Connected to ap");
		}
		if (uxBits & ESPTOUCH_DONE_BIT)
		{
			Flag_wifi_got_led = true;
			ESP_LOGI(TAG_wifi, "smartconfig over");
			esp_smartconfig_stop();
			GetDeviceTimestamp();
			memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
			sprintf(MQTT_ID_Topic, "messages/%s/devconf", DeviceID_TW_Str);
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOS", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, 0, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network);
			mqtt_app_start_ack();
			//xTaskCreate(&advanced_ota_example_task, "advanced_ota_example_task",1024 * 8, NULL, 5, NULL);
//			xTaskCreate(&check_update_task, "check_update_task",1024 * 8, NULL, 5, NULL);
			vTaskDelete(NULL);
		}
	}
}
void init_OTA_component()
{
	Flag_wifi_init = false;
	esp_netif_init();
	esp_event_loop_create_default();
	esp_wifi_set_ps(WIFI_PS_NONE);
//	esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
//	assert(sta_netif);
	esp_netif_create_default_wifi_sta();
	Flag_wifi_init = true;
}

static void event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		xTaskCreate(smartconfig_task, "smartconfig_example_task", 4096,	NULL, 3, &xHandle_SC);
	}
	else if (event_base == WIFI_EVENT	&& event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
		Flag_Fota_fail = true;
		ESP_LOGI(TAG_wifi, "WiFi disconnect\r\n");
//		esp_wifi_connect();
		(esp_wifi_scan_stop());
		(esp_wifi_stop());
		(esp_wifi_deinit());
		ESP32_Clock_Config(20, 20, false);
		xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
		while(Flag_Fota == true)
		{
			vTaskDelay(500/portTICK_PERIOD_MS);
		}
		esp_restart();
	}
	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
		xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
	}
	else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE)
	{
		ESP_LOGI(TAG_wifi, "Scan done");
	}
	else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL)
	{
		ESP_LOGI(TAG_wifi, "Found channel");
	}
	else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
	{
		ESP_LOGI(TAG_wifi, "Got SSID and password");
		Flag_cancel_timeout = true;
		smartconfig_event_got_ssid_pswd_t *evt	=	(smartconfig_event_got_ssid_pswd_t*) event_data;
		wifi_config_t wifi_config;
		uint8_t ssid[33] = { 0 };
		uint8_t password[65] = { 0 };
		uint8_t rvd_data[33] = { 0 };

		bzero(&wifi_config, sizeof(wifi_config_t));
		memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
		memcpy(wifi_config.sta.password, evt->password,	sizeof(wifi_config.sta.password));
		wifi_config.sta.bssid_set = evt->bssid_set;
		if (wifi_config.sta.bssid_set == true)
		{
			memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
		}

		memcpy(ssid, evt->ssid, sizeof(evt->ssid));
		memcpy(password, evt->password, sizeof(evt->password));
		ESP_LOGI(TAG_wifi, "SSID:%s", ssid);
		ESP_LOGI(TAG_wifi, "PASSWORD:%s", password);
		if (evt->type == SC_TYPE_ESPTOUCH_V2)
		{
			ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
			ESP_LOGI(TAG_wifi, "RVD_DATA:");
			for (int i = 0; i < 33; i++)
			{
				printf("%02x ", rvd_data[i]);
			}
			printf("\n");
		}

		(esp_wifi_disconnect());
		(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
		esp_wifi_connect();
	}
	else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE)
	{
		xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
	}
}
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{

	switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            break;
        case HTTP_EVENT_ON_CONNECTED:
            break;
        case HTTP_EVENT_HEADER_SENT:
            break;
        case HTTP_EVENT_ON_HEADER:
            break;
        case HTTP_EVENT_ON_DATA:
//            if (!esp_http_client_is_chunked_response(evt->client)) {
//            	ESP_LOGI(TAG_wifi, "evt->data: %s\r\n",(char*)evt->data);
//				strncpy(rcv_buffer, (char*)evt->data, evt->data_len);
//				ESP_LOGI(TAG_wifi, "wait.....111111\r\n");
//            }
            strncpy(rcv_buffer, (char*)evt->data, evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            break;
        case HTTP_EVENT_DISCONNECTED:
            break;
    }
    return ESP_OK;
}

void check_update_task(void *pvParameter)
{
	char URL[200] = {0};
	sprintf(URL, "http://171.244.133.231:8080/Thingworx/Things/%s/Services/OTA", DeviceID_TW_Str);
	printf("URL: %s", URL);
	printf("Looking for a new firmware...\n");
	while(1) {

			// configure the esp_http_client
			esp_http_client_config_t config = {
					.method = HTTP_METHOD_POST,
					.url = URL,
					.event_handler = _http_event_handler,
			};
			esp_http_client_handle_t client = esp_http_client_init(&config);
			esp_http_client_set_post_field(client, "{}", strlen("{}"));
			esp_http_client_set_header(client, "appKey", "5403be97-566e-4f98-b6e2-ef20573432df");
			esp_http_client_set_header(client, "Accept", "application/json");
			esp_http_client_set_header(client, "Content-Type", "application/json");
			// downloading the json file
			esp_err_t err = esp_http_client_perform(client);
			if(err == ESP_OK)
			{

				// parse the json file
				ESP_LOGI(TAG_wifi, "rcv_buffer:%s", rcv_buffer);
				cJSON *json = cJSON_Parse(rcv_buffer);
				if(json == NULL) printf("downloaded file is not a valid json, aborting...\n");
				else {
					cJSON *version = cJSON_GetObjectItemCaseSensitive(json, "app");
					for(int i = 0; i < strlen(version->valuestring); i++)
					{
						if(version->valuestring[i] == '.' && version->valuestring[i+1] == 'b' && version->valuestring[i+2] == 'i' && version->valuestring[i+3] == 'n')
						{
							version->valuestring[i] = 0;
							break;
						}
					}
					strcpy(VTAG_Version_next, version->valuestring);
					cJSON *file = cJSON_GetObjectItemCaseSensitive(json, "host");

					// check the version
					if((!cJSON_IsNumber(version)) && 0) printf("unable to read new version, aborting...\n");
					else
					{
						double new_version = version->valuedouble;
						if(new_version >= FIRMWARE_VERSION || 1)
						{
							printf("current firmware version (%.1f) is lower than the available one (%.1f), upgrading...\n", FIRMWARE_VERSION, new_version);
							if(cJSON_IsString(file) && (file->valuestring != NULL))
							{
								printf("downloading and installing new firmware (%s)...\n", file->valuestring);

								esp_http_client_config_t ota_client_config = {
									.url = file->valuestring,
									.cert_pem = NULL,
									.skip_cert_common_name_check = true
								};
								esp_err_t ret = esp_https_ota(&ota_client_config);
								if (ret == ESP_OK)
								{
									ResetAllParameters();
									Flag_Fota_success = true;
									//send DOSS to server via cellular
									//Create_Tasks();
									//send DOSS to server via wifi
									//writetofile(base_path, "vtag_ver.txt", VTAG_Version_next);
									mqtt_app_start();

									while(Flag_Fota == true)
									{
										vTaskDelay(500 / portTICK_PERIOD_MS);
									}
//									Flag_Fota_led = false;
//									gpio_set_level(LED_1, 1);
//									vTaskDelay(2000/portTICK_PERIOD_MS);
//									gpio_set_level(LED_1, 0);
//									printf("OTA OK, restarting...\n");
//									esp_restart();
								}
								else
								{
									printf("OTA failed...\n");
								}
							}
							else printf("unable to read the new file name, aborting...\n");
						}
						else printf("current firmware version (%.1f) is greater or equal to the available one (%.1f), nothing to do...\n", FIRMWARE_VERSION, new_version);
					}
				}
			}
			else printf("unable to download the json file, aborting...\n");

			ResetAllParameters();
			Flag_Fota_fail = true;
//			send DOFA to server via cellular
//			xTaskCreatePinnedToCore(mqtt_submessage_processing_task, "mqtt submessage processing", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &mqtt_submessage_processing_handle, tskNO_AFFINITY);
//			xTaskCreatePinnedToCore(fota_ack, "fota_ack", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, NULL, tskNO_AFFINITY);
			//send DOFA to server via wifi
//			GetDeviceTimestamp();
//			memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
//			sprintf(MQTT_ID_Topic, "messages/%s/devconf", DeviceID_TW_Str);
//			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOFA", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network);
//
//			mqtt_app_start();
			while(Flag_Fota == true)
			{
				vTaskDelay(500 / portTICK_PERIOD_MS);
			}
			// cleanup
//			esp_http_client_cleanup(client);

	        //vTaskDelay(30000 / portTICK_PERIOD_MS);
			esp_restart();
			vTaskDelete(NULL);
	    }
}


void ini_wifi()
{
	xTaskCreatePinnedToCore(check_timeout, "check_timeout", 4096, NULL, MAIN_TASK_PRIO, &time_out_handle, tskNO_AFFINITY);
	//ESP_ERROR_CHECK(esp_netif_init());
	s_wifi_event_group = xEventGroupCreate();
	xTaskCreatePinnedToCore(fota_ack, "fota_ack", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, NULL, tskNO_AFFINITY);

	xTaskCreatePinnedToCore(check_timeout_task, "check_timeout", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &check_timeout_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(mqtt_submessage_processing_task, "mqtt submessage processing", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &mqtt_submessage_processing_handle, tskNO_AFFINITY);
	//ESP_ERROR_CHECK(esp_event_loop_create_default());

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	(esp_wifi_init(&cfg));

	(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
	(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
	(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

	(esp_wifi_set_mode(WIFI_MODE_STA));
	//ESP_ERROR_CHECK(esp_wifi_start());
}
