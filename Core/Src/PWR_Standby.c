/*
 * PWR_Standby.c V1.1
 *
 *  Created on: June 04, 2021
 *      Author: Wayne Wu
 *
 *20210604----V1.1
 *Add alarm standby function, improve wake-up pin cause RTC reset issue.
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private variables ---------------------------------------------------------*/
PWRST_t PWRST;
uint8_t IT_source,Wtmr_flag;
uint32_t rtctimers;

void EnterStandbyPWR_Mode(RTC_HandleTypeDef *hrtc)
{
	//-------Wake up Pin setting---------------------

	// Disable all used wakeup sources: Pin2(PC13)
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
	// Clear the related wakeup pin flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	// Re-enable all used wakeup sources: Pin2(PC13)
	HAL_PWREx_EnableInternalWakeUpLine();
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);



	//-------RTC Wake up setting---------------------
//	IT_source =__HAL_RTC_WAKEUPTIMER_GET_IT_SOURCE(hrtc, RTC_FLAG_WUTF);
//	Wtmr_flag =__HAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTF);
	if ((__HAL_RTC_WAKEUPTIMER_GET_IT_SOURCE (hrtc, RTC_FLAG_WUTF) != 0U	&& __HAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTF) != 0U)
			|| __HAL_RTC_WAKEUPTIMER_GET_IT_SOURCE (hrtc, RTC_FLAG_WUTF) == 0U)
	{
		//Set StnadbyTime
//		PWRST.StandbyTime_Sec = 86400;//24hr
		PWRST.StandbyTime_Sec = 21600;//6hr
//		PWRST.StandbyTime_Sec = 5;
		//set RTC WakeUpTimer
//		StandbyTime_Sec > 65536
//		if (HAL_RTCEx_SetWakeUpTimer_IT(hrtc, PWRST.StandbyTime_Sec, RTC_WAKEUPCLOCK_CK_SPRE_17BITS) != HAL_OK)
		//StandbyTime_Sec < 65536
		if (HAL_RTCEx_SetWakeUpTimer_IT(hrtc, PWRST.StandbyTime_Sec, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
		{
			Error_Handler();
		}
	}
//	rtctimers =HAL_RTCEx_GetWakeUpTimer(&hrtc);

	//-------GPIO state setting of standby mode ---------------------
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_11);//LoRa chip reset pin
	HAL_PWREx_EnablePullUpPullDownConfig();

	/* Request to enter STANDBY mode  */
	HAL_PWR_EnterSTANDBYMode();
}

void EnterAlarmStnadby_Mode(RTC_HandleTypeDef *hrtc)	//detected alarm, improve wake-up frequency. disable wake-up pin.
{
	//-------Wake up Pin setting---------------------

	// Disable all used wakeup sources: Pin2(PC13)
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
	// Clear the related wakeup pin flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	// Re-enable all used wakeup sources: Pin2(PC13)
	HAL_PWREx_EnableInternalWakeUpLine();
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);


	/* Request to enter STANDBY mode  */
	HAL_PWR_EnterSTANDBYMode();
}

void RUN_PWR_Mode_Init(RTC_HandleTypeDef *hrtc)
{
	//-------Wake up Pin setting---------------------
	  __HAL_RCC_PWR_CLK_ENABLE();//PWR APB1 clock enable
//	  IT_source=__HAL_PWR_GET_FLAG(PWR_FLAG_SB);
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB)!=RESET)
	{		// Clear the related wakeup pin flag
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		// Disable all used wakeup sources: Pin2(PC13)
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
	}

	//-------RTC Wake up setting---------------------
//	IT_source =__HAL_RTC_WAKEUPTIMER_GET_IT_SOURCE(hrtc, RTC_FLAG_WUTF);
//	Wtmr_flag =__HAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTF);
//	rtctimers =HAL_RTCEx_GetWakeUpTimer(&hrtc);
	if ((__HAL_RTC_WAKEUPTIMER_GET_IT_SOURCE (hrtc, RTC_FLAG_WUTF) != 0U	&& __HAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTF) != 0U)
			|| __HAL_RTC_WAKEUPTIMER_GET_IT_SOURCE (hrtc, RTC_FLAG_WUTF) == 0U)
	{
		//Disable RTC WakeUpTimer
		HAL_RTCEx_DeactivateWakeUpTimer(hrtc);
		//Clear RTC flag
		__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	}
	//-------GPIO state setting of standby mode ---------------------
	HAL_PWREx_DisablePullUpPullDownConfig();//disable function
}
