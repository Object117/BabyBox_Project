/*
 * device_ultraSonic.h
 *
 *  Created on: 2018. 12. 12.
 *      Author: leejh
 */
#include "main.h"
#include "stateMachine.h"
#include"usart.h"
#ifndef DEVICE_ULTRASONIC_H_
#define DEVICE_ULTRASONIC_H_

#define TRIG_SET		1
#define TRIG_RELEASE	0
#define SENDCOUNT	2	// 10us

extern uint32_t IC2_Val_1;
extern uint32_t IC2_Val_2;
extern uint32_t PrevDistance;
extern uint32_t Distance;
extern int SetSonicTrigger;

char DisplayBuff[20];
char DisplayBuff2[20];
char DisplayBuff3[50];

void CountByTick(int turnOffOn);
void ultraSonic_trigger(void);
void ultraSonic_measureCaptureVal(TIM_HandleTypeDef* htim);
void ultraSonic_triggerNextStep(USER_ACTION* nextStep, BABY_ACTION isThereBaby, int detectionLevel, int ignoreLevel);
void ultraSonic_triggerOff(void);
#endif /* DEVICE_ULTRASONIC_H_ */
