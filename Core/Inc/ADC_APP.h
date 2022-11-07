/*
 * ADC_APP.h
 *
 *  Created on: Sep 8, 2020
 *      Author: USER
 */

#ifndef INC_ADC_APP_H_
#define INC_ADC_APP_H_



#endif /* INC_ADC_APP_H_ */


/* Exported types ------------------------------------------------------------*/
#define ADC_Buffer_Ch 2
typedef struct
{
	uint16_t BufferValue[ADC_Buffer_Ch];
	uint32_t BufferValue_Sum[ADC_Buffer_Ch];
	uint16_t BufferValue_Avg[ADC_Buffer_Ch];
	uint16_t Vol[ADC_Buffer_Ch];
	uint16_t Vdda_mv;
	float BatV;
	float BatVTemp;
	float BatVLightload;
	float BatVHeavyload;
	_Bool BatStatus;
} ADC1_t;

/* Private defines -----------------------------------------------------------*/
#define RedLED GPIO_PIN_13

/* Exported functions prototypes ---------------------------------------------*/
void ADC1_Conv(ADC_HandleTypeDef *hadc);
