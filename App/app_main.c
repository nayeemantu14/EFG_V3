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
	strcpy(message, "EFloodGuardLP(v3.7)\r\n");
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
		RTC_AlarmConfig(second[1]);
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
		strcpy(message, "Entering Sleep.\r\n");
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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   // power to actuator
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    /* current PWM = whatever was last written; get it from register     */
    uint16_t start = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    rampValvePWM(start, valveOpenuS);

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    valve_open = 1;
}

// Function to close the valve
void closeValve(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   // power to actuator
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    uint16_t start = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    rampValvePWM(start, valveCloseuS);

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    valve_open = 0;
}

static void rampValvePWM(uint16_t from, uint16_t to)
{
	/* ---- Tunables ------------------------------------------------------- */
	const uint16_t STEP_US   = 10;   // 10 µs per step  (≈0.5 deg for most hobby servos)
	const uint8_t  STEP_MS   = 10;   // 10 ms between steps (≈100 deg/s over 900-1800 µs range)
	const uint16_t MIN_US    = 500;  // absolute safety limits
	const uint16_t MAX_US    = 2500;
	/* -------------------------------------------------------------------- */

	/* Safety clamp                                                     */
	if (to   < MIN_US) to   = MIN_US;
	if (to   > MAX_US) to   = MAX_US;
	if (from < MIN_US) from = MIN_US;
	if (from > MAX_US) from = MAX_US;

	/* Decide direction + do the walk                                    */
	if (from < to)                               /* increasing (e.g., open→close) */
	{
		for (uint16_t p = from; p <= to; p += STEP_US)
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, p);
			HAL_Delay(STEP_MS);
		}
	}
	else if (from > to)                          /* decreasing (e.g., close→open) */
	{
		for (uint16_t p = from; p >= to; p -= STEP_US)
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, p);
			HAL_Delay(STEP_MS);
			if (p < STEP_US) break;              /* protect against unsigned wrap */
		}
	}
	/* small settle time so the motor can finish the move                */
	HAL_Delay(100);
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
	// 1. Enable battery voltage measurement (once)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // Power the sensing circuit

	uint32_t sum = 0;              // Accumulator for multiple ADC readings
	const int NUM_SAMPLES = 5;     // Number of ADC samples to average

	// 2. Read the ADC multiple times
	for (int i = 0; i < NUM_SAMPLES; i++)
	{
		// Start ADC conversion
		HAL_ADC_Start(&hadc);
		// Wait for ADC conversion to complete
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		// Add the reading to sum
		sum += HAL_ADC_GetValue(&hadc);
		// Stop ADC
		HAL_ADC_Stop(&hadc);

		// Brief delay between samples (adjust if needed)
		HAL_Delay(5);
	}

	// 3. Disable battery voltage measurement
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

	// 4. Calculate the average reading
	uint16_t analogbatt = (uint16_t)(sum / NUM_SAMPLES);

	// 5. Apply hysteresis logic (thresholds defined in app_main.h)
	switch (Low_battery)
	{
	case 0: // Currently NORMAL
		if (analogbatt < THRESH_NORMAL_DOWN)
		{
			// If below 3360, decide if Low or Critical
			if (analogbatt < THRESH_LOW_DOWN)
			{
				Low_battery = 2; // Critical
			}
			else
			{
				Low_battery = 1; // Low
			}
		}
		// else remain Normal
		break;

	case 1: // Currently LOW
		if (analogbatt < THRESH_LOW_DOWN)
		{
			Low_battery = 2; // Drop to Critical
		}
		else if (analogbatt > THRESH_NORMAL_UP)
		{
			Low_battery = 0; // Go back to Normal
		}
		// else remain Low
		break;

	case 2: // Currently CRITICAL
		// Must rise above THRESH_LOW_UP to go back to LOW
		if (analogbatt >= THRESH_LOW_UP)
		{
			Low_battery = 1; // From Critical up to Low
		}
		// No direct jump to Normal from Critical
		break;

	default:
		// Fallback in case Low_battery has an invalid value
		Low_battery = 0;
		break;
	}

	// 6. Return the averaged ADC reading (for logging/debugging)
	return analogbatt;
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
