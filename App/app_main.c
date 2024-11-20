#include "app_main.h"
void app_main(void)
{
	initSystem(&g_ctx);

	while(1)
	{
		processState(&g_ctx);
	}
}
// Callback function for rising edge interrupt on GPIO EXTI line
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SystemClock_Config();
	HAL_ResumeTick();
	if(GPIO_Pin == GPIO_PIN_15)
	{
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) // Rising edge
		{
			if (buttonState == 1)
			{
				holdTime = 0;
			}
			wupFlag = 1;
			buttonState = 0;
		}
		else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) // Falling edge
		{
			buttonState = 1;
			holdTime = HAL_GetTick(); // Record button hold time
			wupFlag = 1;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_9)
	{
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin) == GPIO_PIN_RESET) // Falling edge
		{
			HAL_TIM_Base_Start_IT(&htim21);
		}
	}
}

// Callback function for RTC Alarm A event
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	SystemClock_Config();
	HAL_ResumeTick();
	wupFlag = 1;
	rtcFlag = 1;
}

// Callback function for TIM21 period elapsed interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim21)
	{
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET)
		{
			floodFlag = 1; // Set flood flag
		}
		HAL_TIM_Base_Stop_IT(&htim21);
	}
}
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	SystemClock_Config();
	HAL_ResumeTick();
	wupFlag = 1;
	alert_flag = 1;
}

void initSystem(SystemContext *ctx)
{
	ctx->currentState = STATE_INIT;

	RTC_ExitInitMode(&hrtc);
	strcpy(message, "EFloodGuardLP(v3.5)\r\n");
	console(message);

	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
	{
		floodFlag = 0;
		HAL_Delay(100);
		openValve();
		ctx->currentState = STATE_NORMAL;
	}
	else
	{
		floodFlag = 1;
		HAL_Delay(100);
		closeValve();
		ctx->currentState = STATE_FLOOD;
	}
	alert();
}

void processState(SystemContext *ctx)
{
	uint32_t now = HAL_GetTick();
	switch(ctx->currentState)
	{
	case STATE_NORMAL:

		if((floodFlag && Low_battery != 2) || (valve_open == 0 && Low_battery == 0))
		{
			if((now - holdTime >= 1000) && buttonState)
			{
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
				{
					strcpy(message, "Reset\r\n");
					console(message);
					holdTime = 0;
					resetFloodEvent();
				}
				else
				{
					removeFloodAlert();
				}
			}
		}
		if (floodFlag)
		{
			ctx->currentState = STATE_FLOOD;
		}
		else if(rtcFlag)
		{
			ctx->currentState = STATE_NORMAL ;
			strcpy(message, "RTC Event\r\n");
			console(message);
			rtcFlag = 0;
		}
		else if(wupFlag)
		{
			ctx->currentState = STATE_SLEEP;
		}
		break;

	case STATE_FLOOD:
		if(valve_open == 1)
		{
			HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 10239, 10239);
			strcpy(message, "Closing Valve\r\n");
			console(message);
			closeValve();
			strcpy(message, "Valve closed\r\n");
			console(message);
			alert_flag = 1;
		}
		if(alert_flag == 1)
		{
			strcpy(message, "Flood\r\n");
			console(message);
			alert();
			alert_flag = 0;
		}
		if(!buttonState)
		{
			ctx->currentState = STATE_SLEEP;
		}
		else
		{
			ctx->currentState = STATE_NORMAL;
		}
		break;

	case STATE_SLEEP:
		monitorBattery();
		sprintf(message, "Low battery = %d, Flood Flag = %d\r\n", Low_battery, floodFlag);
		console(message);
		wupFlag = 0;
		strcpy(message, "Entering Sleep\r\n");
		console(message);
		HAL_SuspendTick();
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		strcpy(message, "After Sleep\r\n");
		console(message);
		ctx->currentState = STATE_NORMAL;
		break;

	default:
		ctx->currentState = STATE_NORMAL;
		break;
	}
}
// Function to open the valve
void openValve(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // Activate valve
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Start PWM signal for valve control
	HAL_Delay(50);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, valveOpenuS); // Set PWM duty cycle for valve closing
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Stop PWM signal
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // Deactivate valve
	valve_open = 1;
}

// Function to close the valve
void closeValve(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // Activate valve
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Start PWM signal for valve control
	HAL_Delay(50);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, valveCloseuS); // Set PWM duty cycle for valve closing
	HAL_Delay(500);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Stop PWM signal
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // Deactivate valve
	valve_open = 0;
}

// Function to reset flood event
void resetFloodEvent(void)
{
	// Check if the button is pressed and the valve is open
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
	{
		if(valve_open == 0)
		{
			openValve(); // Open the valve
		}
		strcpy(message, "Valve open\r\n");
		console(message);
		floodFlag = 0; // Clear the flood flag
		HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
	}
}

// Function to measure battery voltage
uint16_t measureBattery(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); 	// Enable battery voltage measurement
	HAL_ADC_Start(&hadc); 									// Start ADC conversion
	HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY); 		// Wait for ADC conversion to complete
	uint16_t analogbatt = HAL_ADC_GetValue(&hadc); 			// Read ADC value
	HAL_Delay(5);
	HAL_ADC_Stop(&hadc); 									// Stop ADC conversion
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 	// Disable battery voltage measurement

	// Check battery voltage threshold
	if(analogbatt < 3200 && analogbatt >= 3100)
	{
		Low_battery = 1; 									// Set low battery flag if voltage is below threshold
	}
	else if(analogbatt < 2800 && analogbatt > 0)
	{
		Low_battery = 2; 									// Set low battery flag flag if voltage is below critical threshold
	}
	else
	{
		Low_battery = 0;
	}
	return analogbatt; 										// Return battery voltage reading
}

// Function to monitor battery voltage
void monitorBattery(void)
{
	uint16_t vBatt = measureBattery(); 						// Measure battery voltage
	if(Low_battery == 1)
	{
		batteryAlarm();
		RTC_AlarmConfig(second[0]);
		strcpy(message, "Battery low\r\n");
		console(message);
	}
	else if(Low_battery == 2)
	{
		batteryAlarm();
		if(item>=6)
		{
			item = 0;
		}
		RTC_AlarmConfig(second[item++]);
		if (valve_open == 1)
		{
			floodFlag = 1;
			closeValve();									// Close Valve if Critically low Battery
		}
		strcpy(message, "Battery critically low\r\n");
		console(message);
	}
	sprintf(message, "Battery Voltage: %d\r\n", vBatt); 	// Format battery voltage message
	console(message); 										// Send battery voltage message via UART
}

// Function to control status LED
void statusled(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); 	// Activate status LED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); 	// Activate status LED
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); 	// Deactivate status LED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); 	// Deactivate status LED
}

// Function to activate battery led
void batteryled(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); 	// Activate battery LED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(200); 										// Delay for LED indication
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); 	// Deactivate battery LED
}

// Function to activate battery Alarm
void batteryAlarm(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); 	// Activate battery LED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); 	// Activate battery LED
	HAL_Delay(500); 										// Delay for LED indication
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 	// Deactivate battery LED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); 	// Deactivate battery LED
}

void RTC_AlarmConfig(uint8_t seconds)
{
	RTC_TimeTypeDef sTime;
	RTC_AlarmTypeDef sAlarm;             					// Declare RTC Alarm structure


	memset(&sAlarm, 0, sizeof(sAlarm));
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	uint8_t curr_sec = sTime.Seconds + seconds;

	sAlarm.AlarmTime.Hours = 0x0;
	sAlarm.AlarmTime.Minutes = 0x0;
	sAlarm.AlarmTime.Seconds = curr_sec;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
			|RTC_ALARMMASK_MINUTES;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_A;

	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD);

	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}

}

// Function to activate buzzer and warning LED
void alert(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); 		// Activate buzzer
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); 		// Activate warning LED
	HAL_Delay(1000);	 										// Delay for alert indication
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); 		// Deactivate buzzer
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 		// Deactivate warning LED
}
void removeFloodAlert(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); 		// Activate buzzer
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); 		// Activate warning LED
	HAL_Delay(200);	 											// Delay for alert indication
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); 		// Deactivate buzzer
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 		// Deactivate warning LED
	HAL_Delay(500);												// Delay for alert indication
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); 		// Activate buzzer
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); 		// Activate warning LED
	HAL_Delay(200);	 											// Delay for alert indication
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); 		// Deactivate buzzer
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 		// Deactivate warning LED
}
// Function to transmit messages via UART
void console(char *log)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t *)log, strlen(log), HAL_MAX_DELAY); // Transmit message via UART
	HAL_Delay(10);
	memset(log, '\0', strlen(log)); 										// Clear message buffer
}
