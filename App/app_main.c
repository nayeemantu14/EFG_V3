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
	strcpy(message, "EFloodGuardLP(v3.8)\r\n");
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
    static volatile bool isOpening = false;
    HAL_StatusTypeDef status;

    /* Re-entrancy protection */
    if (isOpening) {
        return;
    }
    isOpening = true;

    /* Validate PWM parameters */
    if (valveOpenuS < VALVE_MIN_PWM || valveOpenuS > VALVE_MAX_PWM) {
        Error_Handler();
        isOpening = false;
        return;
    }

    /* Safety check: Ensure timer is stopped before starting */
    HAL_TIM_PWM_Stop(VALVE_PWM_TIMER, VALVE_PWM_CHANNEL);

    /* Actuator power control */
    HAL_GPIO_WritePin(VALVE_GPIO_PORT, VALVE_GPIO_PIN, GPIO_PIN_SET);

    /* Start PWM with error handling */
    status = HAL_TIM_PWM_Start(VALVE_PWM_TIMER, VALVE_PWM_CHANNEL);
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(VALVE_GPIO_PORT, VALVE_GPIO_PIN, GPIO_PIN_RESET);
        isOpening = false;
        Error_Handler();
        return;
    }

    /* Gradual PWM ramp-up */
    uint16_t currentPWM = __HAL_TIM_GET_COMPARE(VALVE_PWM_TIMER, VALVE_PWM_CHANNEL);
    rampValvePWM(currentPWM, valveOpenuS);

    /* Post-operation stabilization */
    HAL_Delay(VALVE_PWM_HOLD_DELAY_MS);

    /* Graceful shutdown sequence */
    status = HAL_TIM_PWM_Stop(VALVE_PWM_TIMER, VALVE_PWM_CHANNEL);
    if (status != HAL_OK) {
        // Consider adding retry logic here
    }

    /* Final deactivation */
    HAL_Delay(VALVE_DEACTIVATE_DELAY_MS);
    HAL_GPIO_WritePin(VALVE_GPIO_PORT, VALVE_GPIO_PIN, GPIO_PIN_RESET);

    /* State update */
    valve_open = 1;
    isOpening = false;
}

// Function to close the valve
void closeValve(void)
{
    static volatile bool isClosing = false;
    HAL_StatusTypeDef status;

    /* Re-entrancy protection */
    if (isClosing) {
        return;
    }
    isClosing = true;

    /* Validate PWM parameters */
    if (valveCloseuS < VALVE_MIN_PWM || valveCloseuS > VALVE_MAX_PWM) {
        Error_Handler();
        isClosing = false;
        return;
    }

    /* Safety check: Ensure timer is stopped before starting */
    HAL_TIM_PWM_Stop(VALVE_PWM_TIMER, VALVE_PWM_CHANNEL);

    /* Actuator power control */
    HAL_GPIO_WritePin(VALVE_GPIO_PORT, VALVE_GPIO_PIN, GPIO_PIN_SET);

    /* Start PWM with error handling */
    status = HAL_TIM_PWM_Start(VALVE_PWM_TIMER, VALVE_PWM_CHANNEL);
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(VALVE_GPIO_PORT, VALVE_GPIO_PIN, GPIO_PIN_RESET);
        isClosing = false;
        Error_Handler();
        return;
    }

    /* Gradual PWM ramp-down */
    uint16_t currentPWM = __HAL_TIM_GET_COMPARE(VALVE_PWM_TIMER, VALVE_PWM_CHANNEL);
    rampValvePWM(currentPWM, valveCloseuS);

    /* Post-operation stabilization */
    HAL_Delay(VALVE_PWM_HOLD_DELAY_MS);

    /* Graceful shutdown sequence */
    status = HAL_TIM_PWM_Stop(VALVE_PWM_TIMER, VALVE_PWM_CHANNEL);
    if (status != HAL_OK) {
        // Consider adding retry logic here
    }

    /* Final deactivation */
    HAL_Delay(VALVE_DEACTIVATE_DELAY_MS);
    HAL_GPIO_WritePin(VALVE_GPIO_PORT, VALVE_GPIO_PIN, GPIO_PIN_RESET);

    /* State update */
    valve_open = 0;
    isClosing = false;
}

static void rampValvePWM(uint16_t from, uint16_t to)
{
    /* Parameter validation */
    if (from < VALVE_MIN_PWM) from = VALVE_MIN_PWM;
    if (to < VALVE_MIN_PWM) to = VALVE_MIN_PWM;
    if (from > VALVE_MAX_PWM) from = VALVE_MAX_PWM;
    if (to > VALVE_MAX_PWM) to = VALVE_MAX_PWM;

    const uint16_t STEP_US = 10;
    const uint8_t STEP_MS = 10;

    /* Direction detection */
    int16_t direction = (to > from) ? 1 : -1;

    /* Smooth transition */
    while (from != to) {
        from += (direction * STEP_US);

        /* Boundary checks */
        if (direction > 0 && from > to) from = to;
        if (direction < 0 && from < to) from = to;

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, from);
        HAL_Delay(STEP_MS);
    }

    /* Final stabilization */
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
