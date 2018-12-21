/*
 * device_ultraSonic.c
 *
 *  Created on: 2018. 12. 12.
 *      Author: leejh
 */
#include "device_ultraSonic.h"

int SetSonicTrigger = 0;
uint8_t measering = 0;
uint32_t PrevDistance = 0;
uint32_t Distance = 0;
uint32_t IC2_Val_1;
uint32_t IC2_Val_2;
int DistRdyCount = 0;

float Cycle, Width, DutyCycle, Frequency;

#define SELECT_TIME	1000
void CountByTick(int turnOffOn) {
	static uint32_t prevTick = 0;

	if((HAL_GetTick() - prevTick) > SELECT_TIME) {
		prevTick = HAL_GetTick();
		if(turnOffOn == TRIG_SET) {
			printf("TRIG_SET__________________\n");
			SetSonicTrigger = TRIG_SET;
		}
		else {		//turnOffOn == TRIG_RELEASE
			printf("TRIG_RELASE____________\n");
			SetSonicTrigger = TRIG_RELEASE;
		}
	}
	else {

	}
}

void ultraSonic_trigger(void) {
	static int count = 0;

	if(SetSonicTrigger == TRIG_SET) {
		if(count > SENDCOUNT - 1) {
			HAL_GPIO_WritePin(UltraSonic_TRIG_GPIO_Port, UltraSonic_TRIG_Pin, GPIO_PIN_RESET);
			SetSonicTrigger = TRIG_RELEASE;
			count = 0;

		}
		else {
			HAL_GPIO_WritePin(UltraSonic_TRIG_GPIO_Port, UltraSonic_TRIG_Pin, GPIO_PIN_SET);
			count++;
		}
	}
}

void ultraSonic_measureCaptureVal(TIM_HandleTypeDef* htim) {
	if(htim->Instance == TIM3) {

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			IC2_Val_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			TIM1->CNT = 0;
		}

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			IC2_Val_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			//DutyCycle = (float)(IC2_Val_2 * 100) / (float)IC2_Val_1;
			//Frequency = 1000000 / (float)IC2_Val_1;
		}
	}
	else {
		DutyCycle = 0;
		Frequency = 0;
	}
}

void ultraSonic_triggerNextStep(USER_ACTION* nextStep, BABY_ACTION isThereBaby, int detectionLevel, int ignoreLevel) {
	int diffDistance = 0;

	CountByTick(TRIG_SET);

	Distance = (int)(IC2_Val_2 - IC2_Val_1);

	if(IC2_Val_1 > IC2_Val_2) {
		Distance = 0;
	}
	else {
//		sprintf(DisplayBuff2, "%d", PrevDistance);
//		printf("PrevDistance - %s\n", DisplayBuff2);

//		sprintf(DisplayBuff, "%d", Distance);
//		printf("Distance - %s\n", DisplayBuff);

		if(PrevDistance != Distance) {
			if((isThereBaby == BABY_IN) && (PrevDistance > Distance)){
				diffDistance = (int)(PrevDistance - Distance);
			}
			else if((isThereBaby == BABY_NONE) && (Distance > PrevDistance)){		// isThereBaby == BABY_NONE
				diffDistance = (int)(Distance - PrevDistance);
			}
			else {
				diffDistance = 0;
			}
			sprintf(DisplayBuff3, "%d", diffDistance);
			printf("_______________________Diff : %s\n", DisplayBuff3);
		}

		if(DistRdyCount > 5) {

			if(diffDistance > ignoreLevel) {
				Distance = 0;
			}
			else {
				if(diffDistance > detectionLevel) {
					sprintf(DisplayBuff, "%d", diffDistance);
					printf("temp : %s\n", DisplayBuff);
					  changeingState = nextStep;
					  DistRdyCount = 0;
				}
			}
		}
		else {
			DistRdyCount++;
		}
		PrevDistance = Distance;
	}
	HAL_Delay(100);				// Need?
}

void ultraSonic_triggerOff(void) {
	CountByTick(TRIG_RELEASE);
}
