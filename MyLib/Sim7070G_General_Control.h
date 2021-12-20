/*
 * Sim7070G_General_Control.h
 *
 *  Created on: Aug 25, 2021
 *      Author: QuangDan
 */

#ifndef MYLIB_SIM7070G_GENERAL_CONTROL_H_
#define MYLIB_SIM7070G_GENERAL_CONTROL_H_

#include "Common.h"

//--------------------------------------------------------------------------------------------------------------// Funtion for controlling 7070G
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<26) | (1ULL<<19))
#define Sim7070G_PWRKEY    		19
#define ESP32_PowerLatch    	26

void TurnOn7070G(void);
void TurnOff7070G(void);

#endif /* MYLIB_SIM7070G_GENERAL_CONTROL_H_ */
