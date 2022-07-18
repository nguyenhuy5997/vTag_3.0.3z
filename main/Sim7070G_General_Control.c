/*
 * Sim7070G_General_Control.c
 *
 *  Created on: Aug 25, 2021
 *      Author: QuangDan
 */
#include "../MyLib/Sim7070G_General_Control.h"
#include "../MyLib/ESP32_GPIO.h"
#include "../MyLib/sensor.h"
#include "../MyLib/Common.h"
#include "../MyLib/AT_Function.h"
extern const char *TAG;

extern uint16_t RTOS_TICK_PERIOD_MS;
extern uint16_t Reboot7070_Delay_Counter;
extern int TrackingRuntime;
extern uint32_t VTAG_Tracking_Period;
extern int wakeup_time_sec;
extern CFG VTAG_Configure;

extern RTC_DATA_ATTR uint64_t t_actived;
extern RTC_DATA_ATTR uint64_t t_stop;
extern RTC_DATA_ATTR bool Flag_period_wake;
extern RTC_DATA_ATTR bool Flag_acc_wake;
extern uint64_t time_slept;
extern uint64_t t_acc;
extern uint64_t acc_counter;
extern RTC_DATA_ATTR uint64_t acc_capture;
extern bool flag_end_motion;
extern bool flag_start_motion;
extern RTC_DATA_ATTR bool Flag_motion_detected;
extern bool Flag_sos;
extern bool Flag_send_DAST;
extern bool Flag_send_DASP;
extern bool Flag_button_do_nothing;
extern bool Flag_mainthread_run;
extern bool Flag_button_cycle_start;
extern bool Flag_Unpair_Task;
extern bool Flag_Cycle_Completed;
extern bool Flag_backup_data;
extern VTAG_MessageType VTAG_MessType_G;
extern bool Flag_Wait_Exit;
extern bool Flag_Fota;
extern bool Flag_FullBattery;
extern void WaitandExitLoop(bool *Flag);
extern void BackUp_UnsentMessage(VTAG_MessageType Mess_Type);
extern void RTC_IRAM_ATTR wake_stub(void);
extern void GetDeviceTimestamp();
uint16_t tp = 0;
extern RTC_DATA_ATTR uint8_t Backup_Array_Counter;
extern RTC_DATA_ATTR char Location_Backup_Array[16][500];
extern bool Flag_wakeup_led;
extern RTC_DATA_ATTR char Device_PairStatus[5];
extern RTC_DATA_ATTR uint8_t Retry_count;
extern RTC_DATA_ATTR uint64_t Gettimeofday_capture;
extern RTC_DATA_ATTR Device_Param VTAG_DeviceParameter;
extern RTC_DATA_ATTR uint64_t t_stop_calib;
extern RTC_DATA_ATTR uint64_t t_slept_calib;
extern RTC_DATA_ATTR bool Flag_reboot_7070;
extern RTC_DATA_ATTR uint64_t t_total_passed_vol;
extern RTC_DATA_ATTR uint64_t t_total_passed_BU;

#define RESET_NET  1
#define HARD_RESET 0
#define SOFT_RESET 0
#define MIN(a,b,c,d)  ((a < b && a < c && a < d) ? a: ((b < c && b < d) ? b : (c < d ? c : d)))
bool Is_7070_Sleep()
{
	gpio_set_level(UART_SW, 0);
	// Check AT response
	ATC_SendATCommand("AT\r\n", "OK", 1000, 4, ATResponse_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);
	if(AT_RX_event == EVEN_OK)
	{
		return false;
	}
	else
	{
		return true;
	}
}
void Hard_reset7070G(void)
{
	REBOOT:
		ESP_LOGW(TAG, "shut down 7070\r\n");
		gpio_hold_dis(VCC_7070_EN);
		gpio_set_level(VCC_7070_EN, 0);
		vTaskDelay(300/RTOS_TICK_PERIOD_MS);
		gpio_set_level(VCC_7070_EN, 1);
		gpio_hold_en(VCC_7070_EN);
		vTaskDelay(500 / RTOS_TICK_PERIOD_MS);

		ESP_LOGW(TAG, "Turn on 7070\r\n");
		gpio_set_level(UART_SW, 0);
		TurnOn7070G();
		vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);
		Flag_Wait_Exit = false;
		ATC_SendATCommand("AT\r\n", "OK", 1000, 4, ATResponse_Callback);
		WaitandExitLoop(&Flag_Wait_Exit);
		if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
		{
			goto REBOOT;
		}
}
void SoftReboot7070G(void)
{
	ESP_LOGW(TAG, "Turn off 7070\r\n");
	TurnOn7070G();
	vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
REBOOT:
	ESP_LOGW(TAG, "Turn on 7070\r\n");
	gpio_set_level(UART_SW, 0);
	TurnOn7070G();
	vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);
	Flag_Wait_Exit = false;
	ATC_SendATCommand("AT\r\n", "OK", 1000, 4, ATResponse_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);
	if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
	{
		goto REBOOT;
	}
}
void Reboot7070G(void)
{
#if RESET_NET
	Flag_Wait_Exit = false;
	ATC_SendATCommand("AT+CFUN=0\r\n", "OK", 3000, 3, ATResponse_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);
	vTaskDelay(3000/ RTOS_TICK_PERIOD_MS);
	Flag_Wait_Exit = false;
	ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 10000, 0, ATResponse_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);
#elif HARD_RESET
	REBOOT:
			gpio_hold_dis(VCC_7070_EN);
			gpio_set_level(VCC_7070_EN, 0);
			vTaskDelay(300/RTOS_TICK_PERIOD_MS);
			gpio_set_level(VCC_7070_EN, 1);
			gpio_hold_en(VCC_7070_EN);
			vTaskDelay(500 / RTOS_TICK_PERIOD_MS);

			ESP_LOGW(TAG, "Turn on 7070\r\n");
			gpio_set_level(UART_SW, 0);
			TurnOn7070G();
			vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);
			Flag_Wait_Exit = false;
			ATC_SendATCommand("AT\r\n", "OK", 1000, 4, ATResponse_Callback);
			WaitandExitLoop(&Flag_Wait_Exit);
			if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
			{
				goto REBOOT;
			}
#elif SOFT_RESET
			ESP_LOGW(TAG, "Turn off 7070\r\n");
			TurnOn7070G();
			vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
	REBOOT:
			ESP_LOGW(TAG, "Turn on 7070\r\n");
			gpio_set_level(UART_SW, 0);
			TurnOn7070G();
			vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);
			Flag_Wait_Exit = false;
			ATC_SendATCommand("AT\r\n", "OK", 1000, 4, ATResponse_Callback);
			WaitandExitLoop(&Flag_Wait_Exit);
			if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
			{
				goto REBOOT;
			}
#endif
}
void TurnOn7070G(void)
{
	gpio_set_level(PowerKey, 1);
	while(1)
	{
		if(Reboot7070_Delay_Counter  < 40) { Reboot7070_Delay_Counter++;}
		else { Reboot7070_Delay_Counter = 0; break;}
		vTaskDelay(50 / RTOS_TICK_PERIOD_MS);
	}
	gpio_set_level(PowerKey, 0);
}
#define MUL_FACT 1

void ESP_sleep(bool Turn_off_7070)
{
	GetDeviceTimestamp();
	Gettimeofday_capture =  VTAG_DeviceParameter.Device_Timestamp;
	if(strstr(Device_PairStatus, "U") || strlen(Device_PairStatus) == 0)
	{
		ESP_LOGW(TAG, "Shut down device\r\n");
		gpio_hold_dis(ESP32_PowerLatch);
		gpio_set_level(ESP32_PowerLatch, 0);	// For power latch
		vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
	}

	Flag_mainthread_run = true;
	if(Turn_off_7070 == true)
	{
		ESP_LOGW(TAG, "Turn off 7070G\r\n");
		TurnOn7070G();
	}
	while(Flag_button_cycle_start == true);
	vTaskDelay(15 / RTOS_TICK_PERIOD_MS);
	if(Flag_sos == true || Flag_Unpair_Task == true || Flag_Fota == true || Flag_send_DAST == true || Flag_FullBattery == true)
	{
		Flag_Cycle_Completed = true;
		return;
	}
	ESP_LOGW(TAG, "Tracking runtime: %d s\r\n", TrackingRuntime);
	//wakeup_time_sec = VTAG_Configure.Period*60 - TrackingRuntime;
	if(Flag_motion_detected)
	{
		t_total_passed_vol = t_total_passed_BU = 0;
		t_actived = t_actived + time_slept + TrackingRuntime;
		if((t_actived > VTAG_Configure.Period * 60 * MUL_FACT|| flag_start_motion == true) && Flag_button_do_nothing == false)
		{
			flag_start_motion = false;
			time_slept = 0;
			t_actived = 0;
			//ESP_LOGW(TAG, "time_slept %d sec\r\n",(int) time_slept);
			t_actived = t_actived + TrackingRuntime;
		}
		acc_counter = (uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get() - acc_capture, esp_clk_slowclk_cal_get())/1000000);
		ESP_LOGW(TAG, "acc_counter: %d s\r\n",(int)acc_counter);
		ESP_LOGW(TAG, "t_actived: %d sec\r\n",(int) t_actived);
		if(VTAG_Configure.Period * 60 * MUL_FACT >= t_actived)
		{
			tp =  VTAG_Configure.Period * 60 * MUL_FACT - t_actived;
			t_stop =  (uint64_t)round(rtc_time_get());
			ESP_LOGW(TAG, "t_stop: " "%" PRIu64 " s\r\n",t_stop);
		}
		else
		{
			t_stop =  (uint64_t)round(rtc_time_get());
			tp = 0;
		}

		ESP_LOGW(TAG, "tp: %d sec\r\n", tp);
		if(180 > acc_counter)
		{
			t_acc = 180 - acc_counter;
		}
		else t_acc = 0;
		ESP_LOGW(TAG, "t_acc: %d sec\r\n",(int) t_acc);
		if((t_acc + 35) < tp &&  Flag_motion_detected == true)
		{
			Flag_acc_wake = true;
			 Flag_period_wake = false;
			 ESP_LOGW(TAG, "Enter to deep sleep mode, wake by timer in %d sec\r\n",(int) t_acc);
			esp_sleep_enable_timer_wakeup(t_acc * 1000000);
			acc_power_down();
		}
		if((t_acc + 35) >= tp && Flag_motion_detected == true)
		{
			ESP_LOGW(TAG, "Enter to deep sleep mode, wake by timer in %d sec\r\n", tp);
			Flag_period_wake = true;
			Flag_acc_wake = false;
			esp_sleep_enable_timer_wakeup(tp * 1000000);
			acc_power_down();
		}
		if(Flag_button_do_nothing == true && Flag_motion_detected == true)
		{
			if((t_acc + 35) < tp &&  Flag_motion_detected == true) Flag_acc_wake = true;
			if((t_acc + 35) >= tp && Flag_motion_detected == true) Flag_period_wake = true;
		}
	}
	if((flag_end_motion == true || esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED || (Flag_button_do_nothing == true && Flag_motion_detected == false)))
	{
		esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
		ESP_LOGW(TAG, "Enter to deep sleep mode wake by ACC\r\n");
		acc_power_up();
		esp_sleep_enable_ext0_wakeup(ACC_INT, 0);
	}
	if((Flag_motion_detected == false))
	{
		uint64_t t_sleep_vol = 0;
		uint64_t t_sleep_BU = 0;
		t_total_passed_vol = t_total_passed_vol  + time_slept + TrackingRuntime;
		ESP_LOGW(TAG, "t_total_passed: " "%" PRIu64 " s\r\n",t_total_passed_vol);
		if(gpio_get_level(CHARGE) == 0)
		{
			if(10 * 60 > t_total_passed_vol) t_sleep_vol = (10 * 60 - t_total_passed_vol);
			else t_sleep_vol = 0;
			esp_sleep_enable_timer_wakeup(t_sleep_vol * 1000000);
			ESP_LOGW(TAG, "Enter to deep sleep mode wake by timer in %"PRIu64": \r\n", t_sleep_vol);
		}
		else
		{
			if(12 * 3600 > t_total_passed_vol) t_sleep_vol = (12 * 3600 - t_total_passed_vol);
			else t_sleep_vol = 0;
			esp_sleep_enable_timer_wakeup(t_sleep_vol * 1000000);
			ESP_LOGW(TAG, "Enter to deep sleep mode wake by timer in  %"PRIu64": \r\n", t_sleep_vol);
		}
		ESP_LOGW(TAG, "Enter to deep sleep mode wake by ACC\r\n");
		acc_power_up();
		esp_sleep_enable_ext0_wakeup(ACC_INT, 0);
		//if backup array has DASP, wake up after 2minute to send backup array
		for(int i = 0; i < Backup_Array_Counter; i++)
		{
			if(Flag_motion_detected == false && (strstr(Location_Backup_Array[i], "DBF") || strstr(Location_Backup_Array[i], "DASP")))
			{
				t_total_passed_BU = t_total_passed_BU  + time_slept + TrackingRuntime;
				ESP_LOGW(TAG, "t_total_passed_BU: " "%" PRIu64 " s\r\n",t_total_passed_BU);
				if(Retry_count < MAX_RETRY)
				{
					esp_sleep_enable_timer_wakeup(5 * 1000000);
					ESP_LOGW(TAG, "Enter to deep sleep mode, wake by timer in 5 sec\r\n");
				}
				else
				{
					if( 3600 > t_total_passed_BU) t_sleep_BU = (3600 - t_total_passed_BU);
					else t_sleep_vol = 0;
					esp_sleep_enable_timer_wakeup(t_sleep_BU * 1000000);
					ESP_LOGW(TAG, "Enter to deep sleep mode, wake by timer in %"PRIu64"\r\n", t_sleep_BU);
				}
			}
		}
	}
	if(gpio_get_level(CHARGE) == 0)
	{
		esp_sleep_enable_ext1_wakeup((1ULL << CHARGE) | (1ULL << BUTTON), ESP_EXT1_WAKEUP_ANY_HIGH);
		ESP_LOGW(TAG, "Enter to deep sleep mode, wake by CHARGE, BUTTON\r\n");
	}
	else
	{
		esp_sleep_enable_ext1_wakeup((1ULL << BUTTON), ESP_EXT1_WAKEUP_ANY_HIGH);
		ESP_LOGW(TAG, "Enter to deep sleep mode, wake by BUTTON\r\n");
	}
	if(Flag_reboot_7070 == true)
	{
		ESP_LOGW(TAG, "Shut down 7070\r\n");
		gpio_hold_dis(VCC_7070_EN);
		gpio_set_level(VCC_7070_EN, 0);
	}
	t_stop = (uint64_t)round(rtc_time_get());
	t_stop_calib =  (uint64_t)round(rtc_time_get());
	t_slept_calib = 0;
	esp_task_wdt_reset();  		  //Comment this line to trigger a MWDT timeout
	esp_set_deep_sleep_wake_stub(&wake_stub);
	esp_deep_sleep_start();
}



