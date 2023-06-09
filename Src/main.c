/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "W25X40.h"
#include "string.h"
#include "stdlib.h"

#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_motion_indicator.h"
#include "ds3231_for_stm32_hal.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define is_interrupt 0 /*is_interrupt = 1 => get data by interrupt, = 0 => get data by polling */
//#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup
#define DS3231_ADDRESS 0xD0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define AT25DF041B_WIP_MASK 0b00000001
#define AT25DF041B_WEL_MASK 0b00000010

extern uint32_t mem_used_addr;
extern uint32_t num_entries_addr;
extern uint32_t mem_1_addr;
extern uint32_t mem_2_addr;
extern uint32_t mem_3_addr;
extern uint32_t mem_4_addr;
extern uint32_t mem_5_addr;
extern uint32_t mem_6_addr;
extern uint32_t mem_7_addr;
extern uint32_t mem_8_addr;
extern uint32_t mem_9_addr;
extern uint32_t mem_10_addr;
extern uint32_t data_storage;
extern uint8_t num_entries;
extern uint8_t string_to_array[256];
extern uint8_t string_length;

extern uint32_t entry1; //addresses holding actual text data
extern uint32_t entry2;
extern uint32_t entry3;
extern uint32_t entry4;
extern uint32_t entry5;
extern uint32_t entry6;
extern uint32_t entry7;
extern uint32_t entry8;
extern uint32_t entry9;
extern uint32_t entry10;

extern volatile int RTC_Alarm_Flag;
extern volatile int BTN_CNTR_FLG;
extern volatile int BTN_R_FLG;
extern volatile int BTN_L_FLG;
extern volatile uint8_t counter;
extern volatile int set_Val;

uint8_t set_seconds=0;
uint8_t set_minutes=0;
uint8_t set_hours=0;

uint8_t rx_counter = 0;
uint8_t UART1_rxBuffer[256] = { 1 };
uint8_t UART3_rxBuffer[256] = { 1 };
uint8_t UART1_rxCMD[1] = { 10 };
uint8_t UART3_rxCMD[1] = { 10 };
uint8_t UART1_Data[256] = { 0 };
uint8_t UART3_Data[256] = { 0 };
bool deleteFlag = 0;
bool readFlag = 0;

uint8_t STATE = 0;
bool firstTime = true;

// TOF DISTANCE VARS
uint16_t subAlphas[7] = {0};
uint16_t alphas[2] = {0};
uint8_t alphaCounter = 0;

uint8_t aIndex = 0;
uint16_t aSum = 0;
uint8_t aWindowSize = 7;
uint16_t aAverage = 0;

uint8_t bIndex = 0;
uint16_t bSum = 0;
uint8_t bWindowSize = 2;
uint16_t bAverage = 0;

int16_t alphaDiff = 0;
int16_t hourDelta = 19;		// Trial and error
//int16_t minuteDelta = 17; 	// found us these vals
int16_t minuteDelta = 19;
bool ampmDelta = 100;
int16_t hourInput = 1;
int16_t minuteInput = 0;
bool ampmInput = 0;

// TOF SWIPE VARS
int16_t leftSum = 0;
int16_t rightSum = 0;
int16_t leftAvg = 0;
int16_t rightAvg = 0;

int16_t lrDiff = 0;
uint16_t lrThreshold = 1000;

bool swipeTrigger = true;
uint16_t swipeCounter = 0;
uint16_t swipeCooldown = 20;

// STORE VARS
uint8_t EOFCounter = 0;
extern uint8_t fullFileBuffer[22][256];

// ADC VARS
float analog_sol =0;
float analog_bat=0;
uint16_t raw_analog;

// ALARM VARS
int16_t alarmCounter = 0;
bool ampmFlag = false;
//bool alrm_ampmFlag = false;

// TOGGLE BUTTON/TOF VARS
bool tofBtnFlag = 1; // 0 for buttons, 1 for TOF
uint16_t tofHold = 3;
uint16_t tofHoldCounter = 0;
uint16_t tofHoldVal = 0;
bool tofHoldFlag = false;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile int IntCount;
VL53L5CX_Configuration Dev;
VL53L5CX_ResultsData Results;
uint8_t isAlive, isReady, status, loop;
VL53L5CX_Motion_Configuration motion_config; /* Motion configuration*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void VL53L5_Init(void);
void VL53L5_Motion(void);
void updateLCD(void);
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period,
		uint16_t pulse);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 100);
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/// IMPORTANT NOTE!!!
	/// Generating code from .ioc will remove RTC headers in main.h file.
	/// You have to manually add it after each .ioc rebuild in order for build to complete
	/// OR can fix by actually adding in RTC stuff into .ioc?

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	HAL_StatusTypeDef ret;
	uint8_t buf[12] = { 0 };
	uint8_t alarm_time[12] = { 0 };

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */


	//RTC STUFF//
//	alarm_time[0] = 0x12;	//sec
//	alarm_time[1] = 0x48;	//min
//	alarm_time[2] = 0x10;	//hour
	DS3231_Init(&hi2c1);
	W25X40_ReadData(&hspi1, mem_used_addr, alarm_time, 3);
	__disable_irq();
	//DS3231_SetFullTime(10, 48, 4);

	DS3231_SetHour(12);
	DS3231_SetMinute(0);
	DS3231_SetSecond(0);

	DS3231_SetInterruptMode(DS3231_ALARM_INTERRUPT);
	DS3231_ClearAlarm1Flag();
	DS3231_EnableAlarm1(DS3231_ENABLED);
	DS3231_SetAlarm1Mode(DS3231_A1_MATCH_S_M_H);
	HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, DS3231_ALARM1_ADDR, 1, alarm_time,
			3, 100);	//set alarm with alarm_time
	__enable_irq();
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, DS3231_TIME_CAL_ADDR, 1, buf, 3,
			100);



	//TOF STUFF//
	VL53L5_Init();	  	//Initialize TOF sensor

	//LCD STUFF//
	lcd_init();
	lcd_clear();
	lcd_put_cur(0, 0);

	/// UART & SPI Section ///

	W25X40_Set_Status(&hspi1);
	W25X40_Manu_ID_Check(&hspi1);

	W25X40_Available_Entries(&hspi1);
	W25X40_Available_MEM(&hspi1);

	HAL_UART_Receive_IT(&huart1, UART1_rxCMD, 1);
	HAL_UART_Receive_IT(&huart3, UART3_rxCMD, 1);

	HAL_TIM_Base_Start_IT(&htim4);

	// ADC Section

	HAL_GPIO_WritePin(READ_BAT_GPIO_Port, READ_BAT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(READ_SOL_GPIO_Port, READ_SOL_Pin, GPIO_PIN_RESET);

	/// END UART & SPI Section ///

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */



		__disable_irq();
		//VL53L5_Motion();
		__enable_irq();

		switch(STATE) {		// STATE MACHINE: Everything above this in the while loop will eventually end up here.

		case 0:			// IDLE - Displays time

			if(firstTime && STATE == 0)
			{
				HAL_UART_Receive_IT(&huart1, UART3_rxCMD, 1); // DELETE LATER
				HAL_UART_Receive_IT(&huart3, UART3_rxCMD, 1); // DELETE LATER
				lcd_init();
				lcd_clear();
				firstTime = 0;
			}

			if(BTN_CNTR_FLG)
			{
				STATE=1;
				firstTime = 1;
				counter = 0;
				BTN_CNTR_FLG=0;
			}
			if(BTN_R_FLG)		// ALARM SET
			{
				STATE=2;
				firstTime = 1;
				counter = 0;
				BTN_R_FLG=0;
			}
			if(BTN_L_FLG)		// ADC
			{
				STATE=5;
				firstTime = 1;
				counter = 0;
				BTN_L_FLG=0;
			}

			if(RTC_Alarm_Flag)
			{
				updateLCD();
				RTC_Alarm_Flag=0;
			}


			break;
		case 1:			// SET TIME - User can input a clock time to set on the RTC
			if (firstTime && STATE == 1) {
				lcd_init();
				lcd_clear();
				char ln1[16] = {0};
				char ln2[16] = {0};

				sprintf(ln1, "CLK 00:00:00 AM");	// 5, 8, 11 for cursor positions
				sprintf(ln2, "SET");

				lcd_put_cur(0, 0);
				lcd_send_string(ln1);
				lcd_put_cur(1, 0);
				lcd_send_string(ln2);	// Can't decide whether or not to save all 3 values to the end or set them individually

				firstTime = 0;
			}
			if(RTC_Alarm_Flag || BTN_CNTR_FLG || BTN_R_FLG || BTN_L_FLG)
			{
				BTN_R_FLG = 0;
				BTN_L_FLG = 0;

				char ln1[16] = { 0 };	// Maybe circle back and find a way to not initialize these every time?
				char ln2[16] = { 0 };

				if(counter == 0 )
				{
					if (tofBtnFlag) {
						VL53L5_Motion();
						set_Val = hourInput;
						sprintf(ln1, "%02d", set_Val);

						if (tofHoldVal != set_Val) {
							tofHoldVal = set_Val;
							tofHoldCounter = 0;
						}
						else
							tofHoldCounter++;
					}
					else {
						set_Val = (set_Val % 12);
						if (set_Val < 0)
							set_Val = 11;
						sprintf(ln1, "%02d", set_Val + 1);
					}
					sprintf(ln2, " ^");

					lcd_put_cur(0, 4);
					lcd_send_string(ln1);
					lcd_put_cur(1, 4);
					lcd_send_string(ln2);

					if(BTN_CNTR_FLG || tofHoldFlag)
					{
						if (tofBtnFlag)
							set_hours = set_Val;
						else
							set_hours = set_Val + 1;
						set_Val = 0;
						BTN_CNTR_FLG=0;
						tofHoldFlag = 0;
						tofHoldCounter = 0;
						counter++;
					}
				}
				if(counter == 1 )
				{
					if (tofBtnFlag) {	// TOF
						VL53L5_Motion();
						set_Val = minuteInput;

						if (tofHoldVal != set_Val) {
							tofHoldVal = set_Val;
							tofHoldCounter = 0;
						}
						else
							tofHoldCounter++;
					}
					else {				// BOOTUN
						set_Val = set_Val % 60;
						if (set_Val < 0)
							set_Val = 59;
					}

					sprintf(ln1, "%02d", set_Val);
					sprintf(ln2, "    ^");

					lcd_put_cur(0, 7);
					lcd_send_string(ln1);
					lcd_put_cur(1, 4);
					lcd_send_string(ln2);

					if(BTN_CNTR_FLG || tofHoldFlag)
					{
						set_minutes = set_Val;
						set_Val = 0;
						BTN_CNTR_FLG=0;
						tofHoldFlag = 0;
						tofHoldCounter = 0;
						counter++;
					}
				}
				if(counter == 2 )
				{
					if (tofBtnFlag) {	// TOF
						VL53L5_Motion();
						set_Val = minuteInput;

						if (tofHoldVal != set_Val) {
							tofHoldVal = set_Val;
							tofHoldCounter = 0;
						}
						else
							tofHoldCounter++;
					}
					else {				// BOOTUN
						set_Val = set_Val % 60;
						if (set_Val < 0)
							set_Val = 59;
					}
					sprintf(ln1, "%02d", set_Val);
					sprintf(ln2, "       ^");

					lcd_put_cur(0, 10);
					lcd_send_string(ln1);
					lcd_put_cur(1, 4);
					lcd_send_string(ln2);

					if(BTN_CNTR_FLG || tofHoldFlag)
					{
						set_seconds = set_Val;
						set_Val = 0;
						BTN_CNTR_FLG=0;
						tofHoldFlag = 0;
						tofHoldCounter = 0;
						counter++;
					}
				}
				if(counter == 3 )
				{
					if (tofBtnFlag) {	// TOF
						VL53L5_Motion();
						ampmFlag = ampmInput;
						set_Val = ampmInput;

						if (tofHoldVal != set_Val) {
							tofHoldVal = set_Val;
							tofHoldCounter = 0;
						}
						else
							tofHoldCounter++;
					}
					else {				// BOOTUN
						set_Val = set_Val % 2;
						ampmFlag = set_Val;
					}

					if (ampmFlag)
						sprintf(ln1, "PM", set_Val);
					else
						sprintf(ln1, "AM", set_Val);
					//sprintf(ln1, "%02d", set_Val);

					sprintf(ln2, "          ^");

					lcd_put_cur(0, 13);
					lcd_send_string(ln1);
					lcd_put_cur(1, 4);
					lcd_send_string(ln2);

					if(BTN_CNTR_FLG || tofHoldFlag)
					{
						//set_seconds = set_Val;
						set_Val = 0;
						BTN_CNTR_FLG=0;
						tofHoldFlag = 0;
						tofHoldCounter = 0;
						counter++;
					}
				}
				if(counter == 4)	//TODO: Add one more for AM/PM
				{
					if (ampmFlag)
						set_hours += 12;

					//DS3231_SetFullTime(set_hours, set_minutes, set_seconds);
					DS3231_SetHour(set_hours);
					DS3231_SetMinute(set_minutes);
					DS3231_SetSecond(set_seconds);

					BTN_CNTR_FLG=0;
					STATE = 0;
					firstTime = 1;
				}

				RTC_Alarm_Flag=0;
			}

			break;
		case 2:			// SET ALARM - User can input an RTC alarm time
			if (firstTime && STATE == 2) {
				lcd_init();
				lcd_clear();
				char ln1[16] = {0};
				char ln2[16] = {0};

				sprintf(ln1, "ALM 00:00:00 AM");	// 5, 8, 11 for cursor positions
				sprintf(ln2, "SET");

				lcd_put_cur(0, 0);
				lcd_send_string(ln1);
				lcd_put_cur(1, 0);
				lcd_send_string(ln2);	// Can't decide whether or not to save all 3 values to the end or set them individually

				firstTime = 0;
			}
			if(RTC_Alarm_Flag || BTN_CNTR_FLG || BTN_R_FLG || BTN_L_FLG)
			{
				BTN_R_FLG = 0;
				BTN_L_FLG = 0;

				char ln1[16] = { 0 };	// Maybe circle back and find a way to not initialize these every time?
				char ln2[16] = { 0 };

				if(counter == 0 )
				{
					if (tofBtnFlag) {
						VL53L5_Motion();
						set_Val = hourInput;
						sprintf(ln1, "%02d", set_Val);

						if (tofHoldVal != set_Val) {
							tofHoldVal = set_Val;
							tofHoldCounter = 0;
						}
						else
							tofHoldCounter++;
					}
					else {
						set_Val = (set_Val % 12);
						if (set_Val < 0)
							set_Val = 11;
						sprintf(ln1, "%02d", set_Val + 1);
					}
					sprintf(ln2, " ^");

					lcd_put_cur(0, 4);
					lcd_send_string(ln1);
					lcd_put_cur(1, 4);
					lcd_send_string(ln2);

					if(BTN_CNTR_FLG || tofHoldFlag)
					{
						if (tofBtnFlag)
							set_hours = set_Val;
						else
							set_hours = set_Val + 1;
						set_Val = 0;
						BTN_CNTR_FLG=0;
						tofHoldFlag = 0;
						tofHoldCounter = 0;
						counter++;
					}
				}
				if(counter == 1 )
				{
					if (tofBtnFlag) {	// TOF
						VL53L5_Motion();
						set_Val = minuteInput;

						if (tofHoldVal != set_Val) {
							tofHoldVal = set_Val;
							tofHoldCounter = 0;
						}
						else
							tofHoldCounter++;
					}
					else {				// BOOTUN
						set_Val = set_Val % 60;
						if (set_Val < 0)
							set_Val = 59;
					}

					sprintf(ln1, "%02d", set_Val);
					sprintf(ln2, "    ^");

					lcd_put_cur(0, 7);
					lcd_send_string(ln1);
					lcd_put_cur(1, 4);
					lcd_send_string(ln2);

					if(BTN_CNTR_FLG || tofHoldFlag)
					{
						set_minutes = set_Val;
						set_Val = 0;
						BTN_CNTR_FLG=0;
						tofHoldFlag = 0;
						tofHoldCounter = 0;
						counter++;
					}
				}
				if(counter == 2 )
				{
					if (tofBtnFlag) {	// TOF
						VL53L5_Motion();
						set_Val = minuteInput;

						if (tofHoldVal != set_Val) {
							tofHoldVal = set_Val;
							tofHoldCounter = 0;
						}
						else
							tofHoldCounter++;
					}
					else {				// BOOTUN
						set_Val = set_Val % 60;
						if (set_Val < 0)
							set_Val = 59;
					}
					sprintf(ln1, "%02d", set_Val);
					sprintf(ln2, "       ^");

					lcd_put_cur(0, 10);
					lcd_send_string(ln1);
					lcd_put_cur(1, 4);
					lcd_send_string(ln2);

					if(BTN_CNTR_FLG || tofHoldFlag)
					{
						set_seconds = set_Val;
						set_Val = 0;
						BTN_CNTR_FLG=0;
						tofHoldFlag = 0;
						tofHoldCounter = 0;
						counter++;
					}
				}
				if(counter == 3 )
				{
					if (tofBtnFlag) {	// TOF
						VL53L5_Motion();
						ampmFlag = ampmInput;
						set_Val = ampmInput;

						if (tofHoldVal != set_Val) {
							tofHoldVal = set_Val;
							tofHoldCounter = 0;
						}
						else
							tofHoldCounter++;
					}
					else {				// BOOTUN
						set_Val = set_Val % 2;
						ampmFlag = set_Val;
					}

					if (ampmFlag)
						sprintf(ln1, "PM", set_Val);
					else
						sprintf(ln1, "AM", set_Val);
					//sprintf(ln1, "%02d", set_Val);

					sprintf(ln2, "          ^");

					lcd_put_cur(0, 13);
					lcd_send_string(ln1);
					lcd_put_cur(1, 4);
					lcd_send_string(ln2);

					if(BTN_CNTR_FLG || tofHoldFlag)
					{
						//set_seconds = set_Val;
						set_Val = 0;
						BTN_CNTR_FLG=0;
						tofHoldFlag = 0;
						tofHoldCounter = 0;
						counter++;
					}
				}
				if(counter == 4) //TODO: Add one more for AM/PM
				{
					if (ampmFlag)
						set_hours += 12;

					alarm_time[0] = ((set_seconds/10) * 16) + (set_seconds % 10);
					alarm_time[1] = ((set_minutes/10) * 16) + (set_minutes % 10);
					alarm_time[2] = ((set_hours/10) * 16) + (set_hours % 10);

					HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, DS3231_ALARM1_ADDR, 1, alarm_time,
							3, 100); //set alarm with alarm_time

					W25X40_SectorErase(&hspi1, mem_used_addr);
					W25X40_SectorErase(&hspi1, mem_used_addr);
					W25X40_PageProgram(&hspi1, mem_used_addr, alarm_time, 3);

					BTN_CNTR_FLG=0;
					STATE = 0;
					firstTime = 1;
				}

				RTC_Alarm_Flag=0;
			}

			break;
		case 3:			// ALARM - Alarm sounds until user presses dismiss/snooze button
			if (firstTime && STATE == 3) {
				HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

				char ln1[16] = {0};
				char ln2[16] = {0};
				HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, DS3231_ALARM1_ADDR, 1,
						alarm_time, 3, 100);
				lcd_init();
				lcd_clear();
				lcd_put_cur(0, 4);
				uint8_t temp1 = alarm_time[0];
				uint8_t temp2 = alarm_time[1];
				uint8_t temp3 = alarm_time[2];
				sprintf(ln1, "%x:%x:%x", temp3, temp2, temp1);
				sprintf(ln2, "WAKE UP ASSFACE");

				lcd_put_cur(0, 0);
				lcd_send_string(ln1);
				lcd_put_cur(1, 0);
				lcd_send_string(ln2);	// Can't decide whether or not to save all 3 values to the end or set them individually

				firstTime = 0;
			}

			for (int i = 1; i < 250; i++) {
				int buzzerFreq = ((double)(1 / (double)i)) * 4000000;	// Replace constant here by actually acquiring this value
				setPWM(htim2, TIM_CHANNEL_1, buzzerFreq, (buzzerFreq / 2.0));

				HAL_Delay(20);

				//for (int j = 0; j < 16; j++) {
					VL53L5_Motion();
				//}

				if (BTN_CNTR_FLG || !swipeTrigger) {
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
					BTN_CNTR_FLG = 0;
					STATE = 0;
					firstTime = 1;
					break;
				}
			}

			break;
		case 4:			// SNOOZE - Displays time, countdown until next alarm
			if (firstTime && STATE == 4) {


				firstTime = 0;
			}



			break;
		case 5:			// VOLTAGE READOUTS - Reads out voltage from battery/solar power/ etc.
			if (firstTime && STATE == 5) {
				lcd_init();
				lcd_clear();
				firstTime = 0;
			}
			if(RTC_Alarm_Flag)

			{
				HAL_GPIO_WritePin(READ_BAT_GPIO_Port, READ_BAT_Pin, GPIO_PIN_SET);//set gate pin high
				HAL_GPIO_WritePin(READ_SOL_GPIO_Port, READ_SOL_Pin, GPIO_PIN_SET);//set gate pin high

				//start conversion
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 1);
				raw_analog = HAL_ADC_GetValue(&hadc1);//get value from SOL ANALOG
				analog_sol = 3.3*(((float)(raw_analog)*2)/4096.0)+0.6;

				HAL_ADC_Start(&hadc2);
				HAL_ADC_PollForConversion(&hadc2, 1);
				raw_analog = HAL_ADC_GetValue(&hadc2);//get value from register BAT ANALOG
				analog_bat = 3.3*(((float)(raw_analog)*2)/4096.0)+0.6;

				lcd_clear();
				lcd_put_cur(0, 0);
				char analog[8] = {0};
				sprintf(analog, "SOL VOLT: %f",analog_sol);
				lcd_send_string(analog);
				lcd_put_cur(1, 0);
				sprintf(analog, "BAT VOLT: %f",analog_bat);
				lcd_send_string(analog);
				RTC_Alarm_Flag =0;
			}

			if (BTN_L_FLG) {
				BTN_L_FLG = 0;
				STATE = 0;
				firstTime = 1;
			}

			break;
		default:		// You should never hit here, but keep this for debugging for now
			printf("DEFAULT STATE HIT!!!\r\n");
			break;
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00702991;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 8000000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 4000000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 7999;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 10000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 57600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 57600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
	huart3.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(READ_BAT_GPIO_Port, READ_BAT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, READ_SOL_Pin|CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(PWR_EN_C_GPIO_Port, PWR_EN_C_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, I2C_RST_C_Pin|LPn_C_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : READ_BAT_Pin */
	GPIO_InitStruct.Pin = READ_BAT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(READ_BAT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : READ_SOL_Pin CS_Pin */
	GPIO_InitStruct.Pin = READ_SOL_Pin|CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : INT_C_Pin */
	GPIO_InitStruct.Pin = INT_C_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(INT_C_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PWR_EN_C_Pin I2C_RST_C_Pin LPn_C_Pin */
	GPIO_InitStruct.Pin = PWR_EN_C_Pin|I2C_RST_C_Pin|LPn_C_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BTN_DOWN_Pin */
	GPIO_InitStruct.Pin = BTN_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BTN_DOWN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_R_Pin BTN_CNTR_Pin BTN_L_Pin RTC_INT_Pin */
	GPIO_InitStruct.Pin = BTN_R_Pin|BTN_CNTR_Pin|BTN_L_Pin|RTC_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BTN_UP_Pin */
	GPIO_InitStruct.Pin = BTN_UP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BTN_UP_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t alarm_time[12] = { 0 };
	uint8_t time[32] = { 0 };
	uint8_t prompt[128] = {0};

	if ((UART1_rxCMD[0] == '0') || (UART3_rxCMD[0] == '0'))
	{
		if(rx_counter == 0)
		{
			sprintf(prompt, "\n\rEnter Alrm Hour\n\r");
			HAL_UART_Transmit(&huart3, prompt, sizeof(prompt), 100);//send time over buffer to app
			HAL_UART_Receive_IT(&huart3, UART3_rxBuffer, 2);
			rx_counter++;
		}
		else if(rx_counter == 1)
		{
			sprintf(prompt, "\n\rEnter Alrm Min\n\r");
			set_hours = (UART3_rxBuffer[0]-48)*10 + (UART3_rxBuffer[1]-48);
			HAL_UART_Transmit(&huart3, prompt, sizeof(prompt), 100);//send time over buffer to app
			HAL_UART_Receive_IT(&huart3, UART3_rxBuffer, 2);
			rx_counter++;
		}
		else if(rx_counter == 2)
		{
			sprintf(prompt, "\n\rEnter Alrm Sec\n\r");
			set_minutes = (UART3_rxBuffer[0]-48)*10 + (UART3_rxBuffer[1]-48);
			HAL_UART_Transmit(&huart3, prompt, sizeof(prompt), 100);//send time over buffer to app
			HAL_UART_Receive_IT(&huart3, UART3_rxBuffer, 2);
			rx_counter++;
		}
		else if(rx_counter == 3)
		{
			//send new data to RTC
			set_seconds = (UART3_rxBuffer[0]-48)*10 + (UART3_rxBuffer[1]-48);

			alarm_time[0] = ((set_seconds/10) * 16) + (set_seconds % 10);
			alarm_time[1] = ((set_minutes/10) * 16) + (set_minutes % 10);
			alarm_time[2] = ((set_hours/10) * 16) + (set_hours % 10);

			HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, DS3231_ALARM1_ADDR, 1, alarm_time,
					3, 100); //set alarm with alarm_time

			W25X40_SectorErase(&hspi1, mem_used_addr);
			W25X40_SectorErase(&hspi1, mem_used_addr);
			W25X40_PageProgram(&hspi1, mem_used_addr, alarm_time, 3);

			rx_counter = 0 ;
			HAL_UART_Receive_IT(&huart1, UART1_rxCMD, 1);
			HAL_UART_Receive_IT(&huart3, UART3_rxCMD, 1);

		}

	}

	else if ((UART1_rxCMD[0] == '1') || (UART3_rxCMD[0] == '1')) //directory read
	{
		printf("\r\nEntered DIR\r\n");

		HAL_UART_Receive_IT(&huart1, UART1_rxCMD, 1);
		HAL_UART_Receive_IT(&huart3, UART3_rxCMD, 1);

		uint8_t alarm_time[12] = { 0 };
		uint8_t time[32] = { 0 };

		HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, DS3231_ALARM1_ADDR, 1,
				alarm_time, 3, 100);

		uint8_t temp1 = alarm_time[0];
		uint8_t temp2 = alarm_time[1];
		uint8_t temp3 = alarm_time[2];

		sprintf(time, "\n\rALM: %x:%02x:%02x\n\r", temp3, temp2, temp1);
		if(UART3_rxCMD[0] == '1')
		{
			HAL_UART_Transmit(&huart3, time, sizeof(time), 100);//send time over buffer to app
			HAL_UART_Transmit(&huart1, time, sizeof(time), 100);//send time over buffer to console
		}
		if(UART1_rxCMD[0] == '1')
		{
			HAL_UART_Transmit(&huart1, time, sizeof(time), 100);//send time over buffer to console
		}
		UART1_rxCMD[0] = 10;
		UART3_rxCMD[0] = 10;

	}
	else
	{
		HAL_UART_Receive_IT(&huart1, UART1_rxCMD, 1);
		HAL_UART_Receive_IT(&huart3, UART3_rxCMD, 1);
	}


}

void updateLCD(void)
{
	uint8_t buf[12] = { 0 };
	uint8_t alarm_time[12] = { 0 };

	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, DS3231_TIME_CAL_ADDR, 1,
			buf, 3, 100);
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, DS3231_ALARM1_ADDR, 1,
			alarm_time, 3, 100);

	lcd_clear();
	lcd_put_cur(0, 0);
	uint8_t temp1 = buf[0];
	uint8_t temp2 = buf[1];
	uint8_t temp3 = buf[2];
	char time[16] = { 0 };

	if (temp3 > 0x12) {
		temp3 -= 0x12;
		ampmFlag = true;
	}
	else
		ampmFlag = false;

	sprintf(time, "TIM: %x:%02x:%02x ", temp3, temp2, temp1);	// TODO: Put in AM/PM functionality
	lcd_send_string(time);

	lcd_put_cur(0, 14);
	if (ampmFlag)
		sprintf(time, "PM");
	else
		sprintf(time, "AM");
	lcd_send_string(time);

	temp1 = alarm_time[0];
	temp2 = alarm_time[1];
	temp3 = alarm_time[2];

	if (temp3 > 0x12) {
		temp3 -= 0x12;
		ampmFlag = true;
	}
	else
		ampmFlag = false;

	lcd_put_cur(1, 0);
	sprintf(time, "ALM: %x:%02x:%02x", temp3, temp2, temp1);
	lcd_send_string(time);

	lcd_put_cur(1, 14);
	if (ampmFlag)
		sprintf(time, "PM");
	else
		sprintf(time, "AM");
	lcd_send_string(time);
}

void VL53L5_Motion(void) {
	status = vl53l5cx_start_ranging(&Dev);

	loop = 0;
//	while (loop < 15) {
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */

		status = vl53l5cx_check_data_ready(&Dev, &isReady);

//		if (isReady) {
		for (int i = 0; i < 16; i++) {
			vl53l5cx_get_ranging_data(&Dev, &Results);
			//Results.distance_mm

			// TOF DISTANCE

			uint16_t distanceSum = (Results.distance_mm[5]
														+ Results.distance_mm[6] + Results.distance_mm[9]
																									   + Results.distance_mm[10]);
			uint16_t distanceAvg = distanceSum / 4;

			aSum -= subAlphas[aIndex];
			subAlphas[aIndex] = distanceAvg;
			aSum += distanceAvg;
			aIndex = (++aIndex) % aWindowSize;

			aAverage = aSum / aWindowSize;

			bSum -= alphas[bIndex];
			alphas[bIndex] = aAverage;
			bSum += aAverage;
			bIndex = (++bIndex) % bWindowSize;

			//alphaDiff = alphas[0] - alphas[1];
			if (aAverage < 100) {
				hourInput = 1;
				minuteInput = 0;
				ampmInput = 0;
			}
			else if (aAverage > 300) {
				hourInput = 12;
				minuteInput = 55;
				ampmInput = 1;
			}
			else {
				hourInput = ((aAverage - 100) / hourDelta) + 2;
				minuteInput = (((aAverage - 100) / minuteDelta) * 5) + 5;
				ampmInput = ((aAverage - 100) / ampmDelta);
			}

			if (tofHoldCounter >= tofHold) {
				tofHoldCounter = 0;
				tofHoldFlag = true;
			}

			// TOF SWIPE

			leftSum = (Results.motion_indicator.motion[motion_config.map_id[12]] + Results.motion_indicator.motion[motion_config.map_id[13]] + Results.motion_indicator.motion[motion_config.map_id[8]] + Results.motion_indicator.motion[motion_config.map_id[9]]);
			leftSum += (Results.motion_indicator.motion[motion_config.map_id[4]] + Results.motion_indicator.motion[motion_config.map_id[5]] + Results.motion_indicator.motion[motion_config.map_id[0]] + Results.motion_indicator.motion[motion_config.map_id[1]]);

			rightSum = (Results.motion_indicator.motion[motion_config.map_id[14]] + Results.motion_indicator.motion[motion_config.map_id[15]] + Results.motion_indicator.motion[motion_config.map_id[10]] + Results.motion_indicator.motion[motion_config.map_id[11]]);
			rightSum += (Results.motion_indicator.motion[motion_config.map_id[6]] + Results.motion_indicator.motion[motion_config.map_id[7]] + Results.motion_indicator.motion[motion_config.map_id[2]] + Results.motion_indicator.motion[motion_config.map_id[3]]);

			leftAvg = leftSum / 8;
			rightAvg = rightSum / 8;

			lrDiff = leftSum - rightSum;



			// SERIAL PRINT

//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[12]]);
//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[13]]);
//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[14]]);
//			printf("%3lu\t\t",
//					Results.motion_indicator.motion[motion_config.map_id[15]]);

			printf("%3d ", distanceAvg);

			//			printf("%3d ", Results.distance_mm[12]);
			//			printf("%3d ", Results.distance_mm[13]);
			//			printf("%3d ", Results.distance_mm[14]);
			//			printf("%3d\r\n ", Results.distance_mm[15]);
			printf("\r\n");

//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[8]]);
//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[9]]);
//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[10]]);
//			printf("%3lu\t\t",
//					Results.motion_indicator.motion[motion_config.map_id[11]]);

			printf("%3d ", aAverage);

			//			printf("%3d ", Results.distance_mm[8]);
			//			printf("%3d ", Results.distance_mm[9]);
			//			printf("%3d ", Results.distance_mm[10]);
			//			printf("%3d\r\n ", Results.distance_mm[11]);
			printf("\r\n");

//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[4]]);
//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[5]]);
//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[6]]);
//			printf("%3lu\t\t",
//					Results.motion_indicator.motion[motion_config.map_id[7]]);

			printf("%d    %d", hourInput, minuteInput);

			//			printf("%3d ", Results.distance_mm[4]);
			//			printf("%3d ", Results.distance_mm[5]);
			//			printf("%3d ", Results.distance_mm[6]);
			//			printf("%3d\r\n ", Results.distance_mm[7]);
			printf("\r\n");

//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[0]]);
//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[1]]);
//			printf("%3lu ",
//					Results.motion_indicator.motion[motion_config.map_id[2]]);
//			printf("%3lu\t\t",
//					Results.motion_indicator.motion[motion_config.map_id[3]]);

			//			uint8_t incDec = abs(alphaDiff) / delta;
			//
			//			if (incDec > 0) {
			//				if (alphaDiff < 0) {
			//					printf("DECREMENT - %d", incDec);
			//				}
			//				if (alphaDiff > 0) {
			//					printf("INCREMENT - %d", incDec);
			//				}
			//			}

			if (swipeTrigger) {
				if (abs(lrDiff) > lrThreshold) {
					if (lrDiff > 0) {
						// leftSwipe();
						swipeTrigger = false;
						printf("LEFT SWIPE");
					}
					if (lrDiff < 0) {
						// rightSwipe();
						swipeTrigger = false;
						printf("RIGHT SWIPE");
					}
				}
			}
			else {
				// COOLDOWN SESSION
				// sipeTrigger = true;
				swipeCounter++;
				if (swipeCounter >= swipeCooldown) {
					swipeCounter = 0;
					swipeTrigger = true;
				}
				printf("COOLDOWN");
			}

			//			printf("%3d ", Results.distance_mm[0]);
			//			printf("%3d ", Results.distance_mm[1]);
			//			printf("%3d ", Results.distance_mm[2]);
			//			printf("%3d\r\n ", Results.distance_mm[3]);
			printf("\r\n");



//			printf("\n\r\n\r\n\r");
			loop++;
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		WaitMs(&(Dev.platform), 5);
	//}
}

void VL53L5_Init(void) {
	//VL53L5CX_Configuration Dev;
	//VL53L5CX_ResultsData  Results;
	//uint8_t isAlive,isReady,status,loop;
	//VL53L5CX_Motion_Configuration 	motion_config;	/* Motion configuration*/

	Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
	//HAL_GPIO_WritePin(I2C_RST_C_GPIO_Port, I2C_RST_C_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(PWR_EN_C_GPIO_Port, PWR_EN_C_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LPn_C_GPIO_Port, LPn_C_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	//HAL_GPIO_WritePin(PWR_EN_C_GPIO_Port, PWR_EN_C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LPn_C_GPIO_Port, LPn_C_Pin, GPIO_PIN_SET);
	status = vl53l5cx_is_alive(&Dev, &isAlive);
	if (!isAlive) {
		printf("VL53L5CXV0 not detected at requested address (0x %x)\n\t",
				Dev.platform.address);
		//return 255;
	}

	printf("Sensor initializing, please wait few seconds\n\r");
	status = vl53l5cx_init(&Dev);
	if (status == 0) {
		printf("VL51L5CX Initialization Complete\n\r");
	}
	status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4); // must be called before updating ranging frequency
	if (status == 0) {
		printf("VL51L5CX Resolution Set\n\r");
	}
	status = vl53l5cx_set_ranging_frequency_hz(&Dev, 15); // Set 15Hz ranging frequency
	if (status == 0) {
		printf("VL51L5CX Ranging Freq Set\n\r");
	}
	status = vl53l5cx_set_ranging_mode(&Dev, VL53L5CX_RANGING_MODE_CONTINUOUS); // Set mode continuous
	if (status == 0) {
		printf("VL51L5CX Ranging Mode Set\n\r");
	}
	status = vl53l5cx_motion_indicator_init(&Dev, &motion_config,
			VL53L5CX_RESOLUTION_4X4);
	if (status) {
		printf("Motion indicator init failed with status : %u\n", status);
		//return status;
	}
	if (status == 0) {
		printf("VL51L5CX Motion Indicator Initialization Set\n\r");
	}
	status = vl53l5cx_motion_indicator_set_distance_motion(&Dev, &motion_config,
			500, 1500);
	if (status) {
		printf("Motion indicator set distance motion failed with status : %u\n",
				status);
		//return status;
	}
	if (status == 0) {
		printf("VL51L5CX Motion Indicator Set Distance Set\n\r");
	}

}

// credit where credit is due -> https://simonmartin.ch/resources/stm32/dl/STM32%20Tutorial%2002%20-%20PWM%20Generation%20using%20HAL%20(and%20FreeRTOS).pdf
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period,
		uint16_t pulse) {
	HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
	TIM_OC_InitTypeDef sConfigOC;
	timer.Init.Period = period; // set the period duration
	HAL_TIM_PWM_Init(&timer); // reinitialise with new period value
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
	case GPIO_PIN_6:
		// BTN_UP CODE HERE

		printf("BTN_UP\r\n");
		break;
	case GPIO_PIN_15:
		// BTN_L CODE HERE
		printf("BTN_L\r\n");
		break;
	case GPIO_PIN_14:
		// BTN_CNTR CODE HERE
		printf("BTN_CNTR\r\n");
		break;
	case GPIO_PIN_13:
		// BTN_R CODE HERE
		printf("BTN_R\r\n");
		break;
	case GPIO_PIN_12:
		// BTN_DWN CODE HERE
		printf("BTN_DWN\r\n");
		break;
		//	default:
		//		printf("ENTERED BUTTON DEFAULT CASE.\r\n");
		//		break;
	}

	if(GPIO_Pin == GPIO_PIN_9)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8); // Toggle The Output (LED) Pin
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
