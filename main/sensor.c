/*
 * sensor.c
 *
 *  Created on: Sep 3, 2021
 *      Author: ASUS
 */
#include "../MyLib/sensor.h"
#include "math.h"
#include "soc/rtc.h"
#include "../MyLib/ESP32_GPIO.h"
#include "../MyLib/led_indicator.h"
#include "../MyLib/Sim7070G_General_Control.h"

SemaphoreHandle_t print_mux = NULL;
 xQueueHandle gpio_evt_queue = NULL;
TaskHandle_t xClear_ISR_Handle;
TaskHandle_t xHandle_main_running;


volatile uint16_t count_ISR = 0;
extern RTC_DATA_ATTR bool Flag_motion_detected;
extern uint16_t acc_counter;
extern RTC_DATA_ATTR uint64_t acc_capture;
extern bool Flag_send_DAST;
extern bool Flag_send_DASP;
extern bool flag_start_motion;
extern bool Flag_sos;
extern bool Flag_update_cfg;
extern uint8_t ACK_Succeed[2];
extern bool Flag_Unpair_Task;
extern bool Flag_mainthread_run;
extern bool Flag_button_cycle_start;
extern TaskHandle_t check_motion_handle;
extern bool Flag_Fota;
extern bool Flag_button_do_nothing;
extern int Ext1_Wakeup_Pin;
extern bool flag_end_motion;
extern void RTC_IRAM_ATTR wake_stub(void);
extern RTC_DATA_ATTR uint64_t t_stop;
extern bool Flag_checkmotin_end;
extern bool Flag_motion_acc_wake_check;
extern RTC_DATA_ATTR uint8_t Backup_Array_Counter;
extern RTC_DATA_ATTR char Location_Backup_Array[BU_Arr_Max_Num][500];
extern RTC_DATA_ATTR uint8_t Retry_count;
extern RTC_DATA_ATTR uint64_t t_send_backup;
extern RTC_DATA_ATTR uint64_t t_send_voltage;
extern const char *TAG;
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
static esp_err_t i2c_WriteByte(i2c_port_t i2c_num, uint8_t slaveAdd, uint8_t Register, uint8_t Data)
{
	int ret = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slaveAdd << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Register, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	//printf("err write: %d\n", ret);
	i2c_cmd_link_delete(cmd);
	return ret;
	vTaskDelay(1	 / portTICK_PERIOD_MS);
}
static esp_err_t i2c_ReadByte(i2c_port_t i2c_num, uint8_t slaveAdd, uint8_t Register, uint8_t* Data)
{
	int ret = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slaveAdd << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Register, ACK_CHECK_EN);
	//i2c_master_stop(cmd);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slaveAdd << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, Data, NACK_VAL);
	i2c_master_stop(cmd);

	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	//printf("err read: %d\n", ret);
	//printf("read: %02x\n", *Data);
	i2c_cmd_link_delete(cmd);
	return ret;
	vTaskDelay(1 / portTICK_PERIOD_MS);
}

 void clear_interrupt_source(void)
{
	uint8_t read = 0;
#ifdef MC3416
	for (int i = 0; i < 2; i++)
	{
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_STAT_2, &read);
		//printf("ISR: %02x\r\n", read);
		//vTaskDelay(1 / portTICK_RATE_MS);
	}
#endif
#ifdef MC3413
	for (int i = 0; i < 2; i++)
	{
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SR, &read);
		//printf("ISR: %02x\r\n", read);
		//vTaskDelay(1 / portTICK_RATE_MS);
	}
#endif
}
void acc_power_down(void)
{
	uint8_t read;
	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x10);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGW(TAG_SENSOR, "Sensor power down\r\n");
}
 void acc_power_up(void)
{
    uint8_t read;
    i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x11);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGW(TAG_SENSOR, "Sensor power up\r\n");
}
static void clear_ISR_task(void *arg)
{
	uint32_t io_num;
	    for(;;)
	    {
			if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
			{
				count_ISR++;
				clear_interrupt_source();
				ESP_LOGW(TAG_SENSOR, "%d \r\n", count_ISR++);
				vTaskDelay(50/portTICK_RATE_MS);
			}
	    }
}

static void IRAM_ATTR gpio_sensor_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
static void check_motion(void* arg)
{
	#define ACC_OFF_TIME            60   //sec
	#define ACC_WAIT_TIME           60   //sec
    uint32_t time_tik = xTaskGetTickCount();
    uint32_t sec_count = 0;
    bool start = false;
    bool first_time_dectect = true;
    bool first_time_off = true;
    bool first_time_wait = true;
    bool start_wait = false;
    uint32_t current_dectect = 0;
    uint32_t current_off = 0;
    uint32_t current_wait = 0;
    bool end = true;
    while(1)
    {
    	if(xTaskGetTickCount() - time_tik > 1000/portTICK_PERIOD_MS)
    	{
    		Flag_checkmotin_end = false;
    		sec_count++;
    		time_tik = xTaskGetTickCount();

			if(first_time_dectect == true)
			{
				first_time_dectect = false;
				current_dectect = sec_count;
			}
			if(sec_count - current_dectect > 3)
			{
				Flag_checkmotin_end = true;
				if((count_ISR > 75 || (count_ISR > 20 && Flag_motion_detected == true)))
				{
					if(Flag_motion_acc_wake_check == false) Flag_motion_acc_wake_check = true;
					start_wait = false;
					first_time_wait = true;
					end = false;
					count_ISR = 0;
					//start = true;
					first_time_dectect = true;
					if(Flag_motion_detected == false && Flag_send_DASP == false)
					{
						Flag_send_DAST = true;
						flag_start_motion = true;
						Flag_motion_detected = true;
						flag_end_motion = false;
						LED_StartMove();
					}
					//Flag_motion_detected = true; // Flag_motion_detected in main.c
					//acc_capture = (uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get(), esp_clk_slowclk_cal_get())/1000000);
					acc_capture = (uint64_t)round(rtc_time_get());
					ESP_LOGW(TAG_SENSOR, "Motion detected\r\n");

				}
				else
				{
					clear_interrupt_source();
					clear_interrupt_source();
					count_ISR = 0;
					first_time_dectect = true;
					while( Flag_button_cycle_start == true);
					vTaskDelay(15 / portTICK_RATE_MS);
					// Addded: Ext1_Wakeup_Pin != CHARGE condition
					while(Flag_button_cycle_start == true);
					if((Ext1_Wakeup_Pin != CHARGE && Flag_Fota == false && Flag_mainthread_run == false && Flag_Unpair_Task == false && Flag_update_cfg == false && Flag_motion_detected == false && Flag_send_DASP == false && esp_sleep_get_wakeup_cause()!= ESP_SLEEP_WAKEUP_UNDEFINED && Flag_send_DAST == false && Flag_sos == false))
					{
						if(ACK_Succeed[0] == 0)
						{
							ESP_sleep(0);
							// if using this method, t_slept maybe is negative number because esp_clk_slowclk_cal_get() gives different value
							//t_stop = (uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get(), esp_clk_slowclk_cal_get())/1000000);
							t_stop = rtc_time_get();
							ESP_LOGW(TAG_SENSOR, "No motion Entering deep sleep\n");
							esp_set_deep_sleep_wake_stub(&wake_stub);
							esp_deep_sleep_start();
						}
						else if(ACK_Succeed[0] == 3 && ACK_Succeed[1] == 1)
						{
							ESP_sleep(0);
							// if using this method, t_slept maybe is negative number because esp_clk_slowclk_cal_get() gives different value
							//t_stop = (uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get(), esp_clk_slowclk_cal_get())/1000000);
							t_stop = rtc_time_get();
							ESP_LOGW(TAG_SENSOR, "No motion Entering deep sleep\n");
							esp_set_deep_sleep_wake_stub(&wake_stub);
							esp_deep_sleep_start();
						}
					}
				}
			}
        }
    	vTaskDelay(1);
    }
}
void gpio_init(void)
{
	print_mux = xSemaphoreCreateMutex();
	gpio_config_t io_conf;

	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
	io_conf.pin_bit_mask =  ((1ULL<<2));
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	if(esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED)
	{
		xTaskCreate(check_motion, "check_motion", 1024*8, NULL, 5, &check_motion_handle);
	}
	xTaskCreate(clear_ISR_task, "clear_ISR_task", 1024*2, NULL, 5, NULL);
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(2, gpio_sensor_isr_handler, (void*) 2);
	clear_interrupt_source();
}
void acc_config(void)
{
	uint8_t read = 0;

#ifdef MC3416
	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x10); // 0x00
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGI(TAG_SENSOR,"MODE: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_CTRL, 0x0f); // 0x1f
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_CTRL, &read);
	ESP_LOGI(TAG_SENSOR,"INTR_CTRL: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MOTION_CTRL, 0x3f); //0x3f
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MOTION_CTRL, &read);
	ESP_LOGI(TAG_SENSOR, "MOTION_CTRL: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, RANGE, 0x09); //0x09
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, RANGE, &read);
	ESP_LOGI(TAG_SENSOR,"RANGE: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SR, 0x04); // 0x09
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SR, &read);
	ESP_LOGI(TAG_SENSOR,"SR: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XGAIN, 0x0f);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XGAIN, &read);
	ESP_LOGI(TAG_SENSOR,"XGAIN: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YGAIN, 0x0f);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YGAIN, &read);
	ESP_LOGI(TAG_SENSOR,"YGAIN: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZGAIN, 0x0f);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZGAIN, &read);
	ESP_LOGI(TAG_SENSOR,"ZGAIN: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_LSB, 0x02);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_LSB, &read);
	ESP_LOGI(TAG_SENSOR,"TF_THRESH_LSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_MSB, 0x00);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_MSB, &read);
	ESP_LOGI(TAG_SENSOR,"TF_THRESH_MSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_LSB, 0x02);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_LSB, &read);
	ESP_LOGI(TAG_SENSOR,"AM_THRESH_LSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_MSB, 0x00);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_MSB, &read);
	ESP_LOGI(TAG_SENSOR,"AM_THRESH_MSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_LSB, 0x02);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_LSB, &read);
	ESP_LOGI(TAG_SENSOR,"SHK_THRESH_LSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_MSB, 0x00);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_MSB, &read);
	ESP_LOGI(TAG_SENSOR,"SHK_THRESH_MSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x11);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGI(TAG_SENSOR,"MODE: %02x", read);
	clear_interrupt_source();
#endif
#ifdef MC3413
	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x80);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	printf("MODE: %02x\r\n", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTEN, 0x3F);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTEN, &read);
	printf("INTR_CTRL: %02x\r\n", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SRTFR, 0x70);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SRTFR, &read);
	printf("MOTION_CTRL: %02x\r\n", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TAPEN, 0xFF);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TAPEN, &read);
	printf("RANGE: %02x\r\n", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, OUTCFG, 0x00);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, OUTCFG, &read);
	printf("SR: %02x\r\n", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TTTRX, 0x05);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TTTRX, &read);
	printf("TF_THRESH_LSB: %02x\r\n", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TTTRY, 0x05);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TTTRY, &read);
	printf("TF_THRESH_MSB: %02x\r\n", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TTTRZ, 0x05);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TTTRZ, &read);
	printf("AM_THRESH_LSB: %02x\r\n", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x81);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	printf("MODE: %02x\r\n", read);
	clear_interrupt_source();

#endif
}



