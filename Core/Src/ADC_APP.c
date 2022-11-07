/*
 * ADC_APP.c
 *
 *  Created on: Sep 8, 2020
 *      Author: USER
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
ADC1_t ADC_1;

void ADC1_Conv(ADC_HandleTypeDef *hadc)
{
	//ADC start, LED status = 0
	HAL_GPIO_WritePin(GPIOD, RedLED , GPIO_PIN_RESET);
	unsigned char Count_in, Count_Sum;
	unsigned char Count_Sum_value = 30;//adc value Sum count
	uint16_t BufferValuelist[Count_Sum_value];
    for(Count_Sum=0; Count_Sum < Count_Sum_value; Count_Sum++)//take the adc value sum by count 60
    {
    	for(Count_in=0; Count_in < ADC_Buffer_Ch; Count_in++)//Convert ADC IN by scan channel
    	{
    		/*##-1- Start the conversion process #######################################*/
    		HAL_ADC_Start(hadc);
    		/*##-2- Wait for the end of conversion #####################################*/
    		/*  Before starting a new conversion, you need to check the current state of
    			the peripheral; if it?�s busy you need to wait for the end of current
    			conversion before starting a new one.
    			For simplicity reasons, this example is just waiting till the end of the
    	 	 	conversion, but application may perform other tasks while conversion
    			operation is ongoing. */
    		HAL_ADC_PollForConversion(hadc, 50);
    
    			/* Check if the continous conversion of regular channel is finished */
    		while(!HAL_IS_BIT_SET(HAL_ADC_GetState(hadc), HAL_ADC_STATE_REG_EOC)){};
    		/*##-3- Get the converted value of regular channel  ######################*/
    		ADC_1.BufferValue[Count_in] = HAL_ADC_GetValue(hadc);
            ADC_1.BufferValue_Sum[Count_in] += ADC_1.BufferValue[Count_in];
            ADC_1.BufferValue[Count_in]=0;//clear adc bufferValue
    	}
    	HAL_ADC_Stop(hadc);
    }//End adc value sum by count 60

    for(Count_in=0; Count_in < ADC_Buffer_Ch; Count_in++)//Calculate the ADC value average Convert to the Temp value
    {
        ADC_1.BufferValue_Avg[Count_in] = ADC_1.BufferValue_Sum[Count_in]/Count_Sum_value ;

        //ADC voltage digital valve, max:4095
        ADC_1.Vol[Count_in]=ADC_1.BufferValue_Avg[Count_in];

        ADC_1.BufferValue_Sum[Count_in]=0;//clear Sum ValueV
        ADC_1.BufferValue_Avg[Count_in]=0;//clear avg Value

    }
    //Calculate Vdda_mv, Vrefint(Internal voltage reference) --------------------
    //get Internal voltage reference calibration, value:1656, 1.213V
    uint16_t Vref_cal ;
    Vref_cal =*((uint16_t*)VREFINT_CAL_ADDR);
    //Calculate real reference voltage by ADC1 :Vdda_mv
    ADC_1.Vdda_mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(ADC_1.Vol[1], LL_ADC_RESOLUTION_12B);
    //Calculate Battery voltage(For TeKcell SB-AA11)--------------------
//    if(ADC_1.Vol[0] > 1350)
//    	ADC_1.BatV=3.3-((ADC_1.Vol[0]-1349)*(0.1/5))+0.14;
//    else
//    	ADC_1.BatV=3.65;
//    if(ADC_1.BatV >= 3.0)
//    	ADC_1.BatStatus = 0;
//    else
//    	ADC_1.BatStatus = 1;
//    ADC_1.BatV =(float)ADC_1.Vdda_mv/1000*ADC_1.Vol[0]/4095*3.06+0.1;
//    if(ADC_1.BatV > 3.3)
//    	ADC_1.BatV = 3.65;
//    if(ADC_1.BatV >= 3.0)
    ADC_1.BatVTemp =(float)ADC_1.Vdda_mv/1000*ADC_1.Vol[0]/4095*3.06+0.1;
    if(ADC_1.BatVTemp > 3.24)
    	ADC_1.BatVTemp = 3.65;
//    if(ADC_1.BatVTemp >= 2.5)
//        ADC_1.BatStatus = 0;
//       else
//        ADC_1.BatStatus = 1;
    //------------------------------------------------------------------
    //ADC end cycle, LED status =1
    HAL_GPIO_WritePin(GPIOD, RedLED , GPIO_PIN_SET);
}
