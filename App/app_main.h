/*
 * app_main.h
 *
 *  Created on: Oct 18, 2024
 *      Author: antun
 */

#ifndef APP_MAIN_H_
#define APP_MAIN_H_

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define UID_BASE_ADDRESS 0x1FF80050
#define valveOpenuS 	 900
#define valveCloseuS	 1800

#define THRESH_NORMAL_DOWN 3380   // normal to low
#define THRESH_LOW_DOWN    3250   // low to critical
#define THRESH_LOW_UP      3270  // critical up to low
#define THRESH_NORMAL_UP   3400   // low up to normal

// Add to app_main.h near other defines
#define VALVE_ACTIVATE_DELAY_MS    50
#define VALVE_PWM_HOLD_DELAY_MS    500
#define VALVE_DEACTIVATE_DELAY_MS  50
#define VALVE_PWM_TIMER           (&htim2)
#define VALVE_PWM_CHANNEL         TIM_CHANNEL_1
#define VALVE_GPIO_PORT           GPIOA
#define VALVE_GPIO_PIN            GPIO_PIN_7
#define VALVE_MIN_PWM             500   // Absolute minimum PWM pulse width
#define VALVE_MAX_PWM             2500  // Absolute maximum PWM pulse width

// External peripheral handlers declaration
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef hlpuart1;
extern RTC_HandleTypeDef hrtc;
extern LPTIM_HandleTypeDef hlptim1;
extern TIM_HandleTypeDef htim21;

// Global variable declaration
char message[40];

volatile uint8_t wupFlag = 1;
volatile uint8_t rtcFlag = 0;
volatile uint8_t alert_flag = 0;
volatile uint8_t Low_battery;

volatile  uint8_t valve_open;
volatile  uint8_t floodFlag = 0;
volatile  uint8_t buttonState = 0;
volatile  uint32_t holdTime = 0;

volatile uint8_t second[6] = {0x0, 0x10, 0x20, 0x30, 0x40, 0x50};
volatile uint8_t item = 0;

volatile static bool isOpening = false;
volatile static bool isClosing = false;

// State definitions
typedef enum {
	STATE_INIT,
	STATE_NORMAL,
	STATE_FLOOD,
	STATE_SLEEP
} SystemState;

// System context
typedef struct {
	SystemState currentState;
} SystemContext;

// Global context
static SystemContext g_ctx;

// Function prototypes
void SystemClock_Config();
void openValve(void);
void closeValve(void);
void alert(void);
static void rampValvePWM(uint16_t from, uint16_t to);
void removeFloodAlert(void);
void resetFloodEvent(void);
uint16_t measureBattery(void);
void monitorBattery(void);
void statusled(void);
void RTC_AlarmConfig(uint8_t seconds);
void batteryled(void);
void batteryAlarm(void);
void console(char *log);

// State machine functions
void initSystem(SystemContext *ctx);
void processState(SystemContext *ctx);

#endif /* APP_MAIN_H_ */
