/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @author         : Morshu8800
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024-2025 STMicroelectronics.
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
#include "adc.h"
#include "crc.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ST7565.h"
#include "sht3x.h"
#include "bmp280.h"
#include "AT24Cxx.h"
#include "DS3231.h"
#include "icons.h"
#include "RCC.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
BMP280_HandleTypedef bmp280;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_ADDR 0
#define ALARM1HR_ADDR 8
#define ALARM1MI_ADDR 16
#define ALARM2HR_ADDR 24
#define ALARM2MI_ADDR 32
#define ALARM1FLAG_ADDR 40
#define ALARM2FLAG_ADDR 48
#define INVERSION_ADDR 56
#define BRIGHTNESS_ADDR 64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t SystemCoreClock = 16000000UL;
uint32_t hr = 0, mi = 0, sc = 0, scnew = 0, mt = 0, dy = 0, wd = 0, dynew = 0,
		yr = 0;
uint32_t dig1 = 0, dig2 = 0, dig3 = 0, dig4 = 0;
const uint16_t BRIGHTNESS[21] = { 0x0000, 0x0019, 0x0033, 0x0066, 0x0099,
		0x00CC, 0x00FF, 0x0132, 0x0166, 0x0199, 0x01CC, 0x01FF, 0x0232, 0x0265,
		0x0298, 0x02CC, 0x02FF, 0x0332, 0x0365, 0x0398, 0x03FF };
int_fast8_t subbright = 2, defbright = 10, bright = 10;
int32_t pressure = 0, temper = 0, humi = 0;
const uint_fast8_t MEMKEY = 0xD0;
uint_fast8_t KEYCHECK = 0x00;
float temperature = 0.00F, humidity = 0.00F, prs = 0.00F, tmp = 0.00F, hum =
		0.00F, YEAR = 0.00F, year = 0.00F, del = 0.00F, vol = 0.000F, volref =
		3.325F;
char DIG1[4], DIG2[4], DIG3[4], DIG4[4], TEMPERATURE[8], HUMIDITY[8],
		PRESSURE[8], PROC = '%', DY[8], MT[8], YE[8] /*, T[8]*/;
char *DOW[8] = { "ПН", "ВТ", "СР", "ЧТ", "ПТ", "СБ", "ВС" };
bool clear_flag = false, clear_temp = false;
bool mainsel = false, mainmenu = true, sleep = false, alarm_set = false,
		time_date = false, other_settings = false, about_prog = false;
uint32_t timelock = 0, settimelockdef = 60000, subtime = 0, setlocktime = 0,
		buttontime = 0, speakertime = 0, close_men_time = 0/*, alarm_wait = 0,
		 lowbat_mes_time = 0, verylowbat_mes_time = 0, mes_time = 0*/;
int_fast8_t menusel = 0;
//int32_t t = 0;
bool alarm1_set = false, alarm2_set = false, alarm1_settings = false,
		alarm2_settings = false, alarm1_hr_set = false, alarm1_mi_set = false,
		alarm2_hr_set = false, alarm2_mi_set = false, isAlarm1 = false,
		isAlarm2 = false;
int_fast8_t alarm_sel = 0, alarm1_sel = 0, alarm2_sel = 0;
int_fast8_t al1_hr = 0, al1_mi = 0, al2_hr = 0, al2_mi = 0;
volatile bool ALARM_TRG = false;

bool read_alarms = true,
		alarm_s_flag = false/*, alarm1_pass = false, alarm2_pass = false*/;
uint32_t al_hr_trg = 0, al_mi_trg = 0, alarm_interval = 0;
bool time_config = false, date_config = false, time_hr_set = false,
		time_mi_set = false, date_dy_set = false, date_mo_set = false,
		date_yr_set = false, date_dow_set = false, bring_date = true;
int_fast8_t time_hr = 0, time_mi = 0, date_dy = 1, date_mo = 1, date_dow = 1,
		time_date_sel = 0, time_sel = 0, date_sel = 0;
int_fast16_t date_yr = 2000;
uint_fast16_t bat = 0;
bool invert_disp = false, brightset = false, invertset = false,
		otherload = true, mes_flag = false, first_mes_flag = true, charge_flag =
		false;
sht3x_handle_t handle = { .i2c_handle = &hi2c1, .device_address =
SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW };
RTC_TimeTypeDef sTime;
RTC_AlarmTypeDef sAlarm;
RTC_DateTypeDef sDate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CMSIS_Clock_Config(void);
static void EEPROM_INIT(void);
static void PERPERAL_INIT(void);
static void PERPERAL_SLEEP_INIT(void);
void time_disp(void);
static void dat_disp(void);
static void temp_humi_measure(void);
static void pressure_measure(void);
void change_main_sel(void);
void PREPARE_TO_SLEEP(void);
void disp_menu(void);
static void change_menu_sel(void);
void alarm_settings(void);
void alarm_check(void);
static void alarm1_think(void);
static void alarm2_think(void);
static void alarm_sound(void);
void time_date_set(void);
void about(void);
static void battery_meashure(void);
void status_bar(void);
void other(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PERPERAL_INIT(void) {
	HAL_GPIO_WritePin(LCD_VDD_GPIO_Port, LCD_VDD_Pin, 1);
	HAL_GPIO_WritePin(METEO_VDD_GPIO_Port, METEO_VDD_Pin, 1);
	HAL_GPIO_WritePin(MEMORY_VDD_GPIO_Port, MEMORY_VDD_Pin, 1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	TIM1->CCR1 = BRIGHTNESS[bright];
	//TIM3->CCR1 = 50;
	//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_Delay(100);
	DS3231_Init(&hi2c1);
	HAL_Delay(200);
	ST7565_Init();
	ST7565_Clear();
	ST7565_Update();
	ST7565_Print(0, 0, "STMeteo версия 1.0", &Font_7x9, 1, PIX_ON);
	ST7565_Print(0, 11, "(c)Morshu8800,", &Font_7x9, 1, PIX_ON);
	ST7565_Print(0, 21, "2024-2025", &Font_7x9, 1, PIX_ON);
	ST7565_Update();
	HAL_GPIO_WritePin(LCD_VDD_GPIO_Port, LCD_VDD_Pin, 1);
	HAL_GPIO_WritePin(METEO_VDD_GPIO_Port, METEO_VDD_Pin, 1);
	HAL_GPIO_WritePin(MEMORY_VDD_GPIO_Port, MEMORY_VDD_Pin, 1);
	sht3x_handle_t handle = { .i2c_handle = &hi2c1, .device_address =
	SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW };
	if (!sht3x_init(&handle)) {
		while (1) {
			ST7565_Clear();
			ST7565_Print(0, 0, "Ошибка датчика SHT", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 11, "проверьте подкл.", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, "или обратитесь к", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, "специалисту (0x02)", &Font_7x9, 1, PIX_ON);
			ST7565_Update();
			TIM3->CCR1 = 0;
			HAL_Delay(10000);
			NVIC_SystemReset();
		}
	}
	HAL_GPIO_WritePin(LCD_VDD_GPIO_Port, LCD_VDD_Pin, 1);
	HAL_GPIO_WritePin(METEO_VDD_GPIO_Port, METEO_VDD_Pin, 1);
	HAL_GPIO_WritePin(MEMORY_VDD_GPIO_Port, MEMORY_VDD_Pin, 1);
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c2;
	if (!bmp280_init(&bmp280, &bmp280.params)) {
		while (1) {
			ST7565_Clear();
			ST7565_Print(0, 0, "Ошибка датчика BMP", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 11, "проверьте подкл.", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, "или обратитесь к", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, "специалисту (0x01)", &Font_7x9, 1, PIX_ON);
			ST7565_Update();
			TIM3->CCR1 = 0;
			HAL_Delay(10000);
			NVIC_SystemReset();
		}
	}
	EEPROM_INIT();
}
void EEPROM_INIT(void) {
	clear_flag = 1;
	if (!AT24xx_Connect_test()) {
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		TIM1->CCR1 = BRIGHTNESS[bright];
		while (1) {
			if (clear_flag) {
				ST7565_Print(0, 43, "Ошибка подкл. ПЗУ", &Font_6x8, 1, PIX_ON);
				ST7565_Print(0, 54, "(0x00)", &Font_6x8, 1, PIX_ON);
				ST7565_Update();
				clear_flag = 0;
			}
			TIM3->CCR1 = 0;
			HAL_Delay(5000);
			NVIC_SystemReset();
		}
	} else {
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		TIM1->CCR1 = BRIGHTNESS[bright];
		if (clear_flag) {
			ST7565_Print(0, 54, "Подкл. ПЗУ ОК!", &Font_6x8, 1, PIX_ON);
			ST7565_Update();
			clear_flag = 0;
		}
	}
	HAL_Delay(1000);
	TIM3->CCR1 = 0;
	HAL_Delay(1000);
	AT24Cxx_read_data(KEY_ADDR, (uint8_t*) &KEYCHECK, sizeof(KEYCHECK));
	ST7565_Clear();
	ST7565_Update();
	if (KEYCHECK != MEMKEY) {
		ST7565_Print(0, 0, "Потерян ключ ПЗУ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 11, "Выполняется сброс!", &Font_6x8, 1, PIX_ON);
		ST7565_Update();
		HAL_Delay(5000);
		//while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
		AT24Cxx_erase_chip();
		if (!AT24Cxx_write_data(KEY_ADDR, (uint8_t*) &MEMKEY, sizeof(MEMKEY))) {
			while (1) {
				ST7565_Clear();
				ST7565_Print(0, 0, "Ошибка сброса!", &Font_6x8, 1, PIX_ON);
				ST7565_Print(0, 11, "Скорее всего ПЗУ", &Font_6x8, 1, PIX_ON);
				ST7565_Print(0, 21, "мертво, замените", &Font_6x8, 1, PIX_ON);
				ST7565_Print(0, 32, "ПЗУ и нажмите люб.", &Font_6x8, 1, PIX_ON);
				ST7565_Print(0, 43, "кнопку для сброса", &Font_6x8, 1, PIX_ON);
				ST7565_Update();
				while (HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 1
						|| HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 1
						|| HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 1
						|| HAL_GPIO_ReadPin(alarm_disable_GPIO_Port,
						alarm_disable_Pin) == 1)
					;
				NVIC_SystemReset();
			}
		} else {
			if (AT24Cxx_read_data(KEY_ADDR, (uint8_t*) &KEYCHECK,
					sizeof(KEYCHECK))) {
				AT24Cxx_write_data(ALARM1HR_ADDR, (uint8_t*) &al1_hr,
						sizeof(al1_hr));
				AT24Cxx_write_data(ALARM1MI_ADDR, (uint8_t*) &al1_mi,
						sizeof(al1_mi));
				AT24Cxx_write_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
						sizeof(alarm1_set));
				AT24Cxx_write_data(ALARM2HR_ADDR, (uint8_t*) &al2_hr,
						sizeof(al2_hr));
				AT24Cxx_write_data(ALARM2MI_ADDR, (uint8_t*) &al2_mi,
						sizeof(al2_mi));
				AT24Cxx_write_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
						sizeof(alarm2_set));
				AT24Cxx_write_data(BRIGHTNESS_ADDR, (uint8_t*) &defbright,
						sizeof(defbright));
				AT24Cxx_write_data(INVERSION_ADDR, (uint8_t*) &invert_disp,
						sizeof(invert_disp));
				ST7565_Clear();
				ST7565_Print(0, 0, "Сброс ОК!", &Font_6x8, 1, PIX_ON);
				ST7565_Update();
				HAL_Delay(2000);
			} else {
				while (1) {
					ST7565_Clear();
					ST7565_Print(0, 0, "Ошибка сброса!", &Font_6x8, 1, PIX_ON);
					ST7565_Print(0, 11, "ПЗУ мертво или", &Font_6x8, 1, PIX_ON);
					ST7565_Print(0, 21, "не подкл, замените", &Font_6x8, 1,
							PIX_ON);
					ST7565_Print(0, 32, "ПЗУ и нажмите люб.", &Font_6x8, 1,
							PIX_ON);
					ST7565_Print(0, 43, "кнопку для сброса", &Font_6x8, 1,
							PIX_ON);
					ST7565_Update();
					while (HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 1
							|| HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 1
							|| HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 1
							|| HAL_GPIO_ReadPin(alarm_disable_GPIO_Port,
							alarm_disable_Pin) == 1)
						;
					NVIC_SystemReset();
				}
			}

		}
	}
	HAL_ADCEx_Calibration_Start(&hadc1);
}

void PERPERAL_SLEEP_INIT(void) {
	HAL_GPIO_WritePin(LCD_VDD_GPIO_Port, LCD_VDD_Pin, 1);
	HAL_GPIO_WritePin(METEO_VDD_GPIO_Port, METEO_VDD_Pin, 1);
	HAL_GPIO_WritePin(MEMORY_VDD_GPIO_Port, MEMORY_VDD_Pin, 1);
	HAL_Delay(100);
	DS3231_Init(&hi2c1);
	HAL_Delay(200);
	//DS3231_SetInterruptMode(DS3231_ALARM_INTERRUPT);
	ST7565_Init();
	ST7565_Clear();
	ST7565_Update();
	HAL_GPIO_WritePin(LCD_VDD_GPIO_Port, LCD_VDD_Pin, 1);
	HAL_GPIO_WritePin(METEO_VDD_GPIO_Port, METEO_VDD_Pin, 1);
	HAL_GPIO_WritePin(MEMORY_VDD_GPIO_Port, MEMORY_VDD_Pin, 1);
	HAL_Delay(100);
	sht3x_handle_t handle = { .i2c_handle = &hi2c1, .device_address =
	SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW };
	if (!sht3x_init(&handle)) {
		while (1) {
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			TIM1->CCR1 = BRIGHTNESS[bright];
			ST7565_Clear();
			ST7565_Print(0, 0, "Ошибка датчика SHT", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 11, "проверьте подкл.", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, "или обратитесь к", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, "специалисту (0x02)", &Font_7x9, 1, PIX_ON);
			ST7565_Update();
			HAL_Delay(10000);
			NVIC_SystemReset();
		}
	}
	HAL_GPIO_WritePin(LCD_VDD_GPIO_Port, LCD_VDD_Pin, 1);
	HAL_GPIO_WritePin(METEO_VDD_GPIO_Port, METEO_VDD_Pin, 1);
	HAL_GPIO_WritePin(MEMORY_VDD_GPIO_Port, MEMORY_VDD_Pin, 1);
	HAL_Delay(100);
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c2;
	if (!bmp280_init(&bmp280, &bmp280.params)) {
		while (1) {
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			TIM1->CCR1 = BRIGHTNESS[bright];
			ST7565_Clear();
			ST7565_Print(0, 0, "Ошибка датчика BMP", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 11, "проверьте подкл.", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, "или обратитесь к", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, "специалисту (0x01)", &Font_7x9, 1, PIX_ON);
			ST7565_Update();
			HAL_Delay(10000);
			NVIC_SystemReset();
		}
	}
	EEPROM_INIT();
}

//display time and other info
void time_disp(void) {
	sc = DS3231_GetSecond();
	if (sc != scnew) {
		scnew = sc;
		hr = DS3231_GetHour();
		mi = DS3231_GetMinute();
		dig1 = hr * 0.1;
		dig2 = hr % 10;
		dig3 = mi * 0.1;
		dig4 = mi % 10;
		sprintf(DIG1, "%ld", dig1);
		sprintf(DIG2, "%ld", dig2);
		sprintf(DIG3, "%ld", dig3);
		sprintf(DIG4, "%ld", dig4);
		ST7565_Print(0, 12, DIG1, &Font_16x26, 1, PIX_ON);
		ST7565_Print(17, 12, DIG2, &Font_16x26, 1, PIX_ON);
		ST7565_Print(34, 10, ":", &Font_16x26, 1, PIX_ON);
		ST7565_Print(50, 12, DIG3, &Font_16x26, 1, PIX_ON);
		ST7565_Print(67, 12, DIG4, &Font_16x26, 1, PIX_ON);
		ST7565_Update();
	}
	temp_humi_measure();
	pressure_measure();
	dat_disp();

}

//date_display
void dat_disp(void) {
	dy = DS3231_GetDate();
	if (dy != dynew) {
		dynew = dy;
		mt = DS3231_GetMonth();
		wd = DS3231_GetDayOfWeek();
		YEAR = DS3231_GetYear();
		YEAR = YEAR * 0.01;
		del = modff(YEAR, &year);
		del = del * 100;
		del = roundf(del);
		yr = (uint32_t) del;
		sprintf(DY, "%ld", dy);
		sprintf(MT, "%ld", mt);
		sprintf(YE, "%ld", yr);
		if (dy > 9 && mt > 9 && yr > 9) {
			ST7565_Print(0, 38, DY, &Font_7x9, 1, PIX_ON);
			ST7565_Print(15, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(23, 38, MT, &Font_7x9, 1, PIX_ON);
			ST7565_Print(38, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 38, YE, &Font_7x9, 1, PIX_ON);
		} else if (dy > 9 && mt > 9 && yr < 10) {
			ST7565_Print(0, 38, DY, &Font_7x9, 1, PIX_ON);
			ST7565_Print(15, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(23, 38, MT, &Font_7x9, 1, PIX_ON);
			ST7565_Print(38, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(53, 38, YE, &Font_7x9, 1, PIX_ON);
		} else if (dy > 9 && mt < 10 && yr > 9) {
			ST7565_Print(0, 38, DY, &Font_7x9, 1, PIX_ON);
			ST7565_Print(15, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(23, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(30, 38, MT, &Font_7x9, 1, PIX_ON);
			ST7565_Print(38, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 38, YE, &Font_7x9, 1, PIX_ON);
		} else if (dy > 9 && mt < 10 && yr < 10) {
			ST7565_Print(0, 38, DY, &Font_7x9, 1, PIX_ON);
			ST7565_Print(15, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(23, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(30, 38, MT, &Font_7x9, 1, PIX_ON);
			ST7565_Print(38, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(53, 38, YE, &Font_7x9, 1, PIX_ON);
		} else if (dy < 10 && mt > 9 && yr > 9) {
			ST7565_Print(0, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(7, 38, DY, &Font_7x9, 1, PIX_ON);
			ST7565_Print(15, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(23, 38, MT, &Font_7x9, 1, PIX_ON);
			ST7565_Print(38, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 38, YE, &Font_7x9, 1, PIX_ON);
		} else if (dy < 10 && mt > 9 && yr < 10) {
			ST7565_Print(0, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(7, 38, DY, &Font_7x9, 1, PIX_ON);
			ST7565_Print(15, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(23, 38, MT, &Font_7x9, 1, PIX_ON);
			ST7565_Print(38, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(53, 38, YE, &Font_7x9, 1, PIX_ON);
		} else if (dy < 10 && mt < 10 && yr > 9) {
			ST7565_Print(0, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(7, 38, DY, &Font_7x9, 1, PIX_ON);
			ST7565_Print(15, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(23, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(30, 38, MT, &Font_7x9, 1, PIX_ON);
			ST7565_Print(38, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 38, YE, &Font_7x9, 1, PIX_ON);
		} else if (dy < 10 && mt < 10 && yr < 10) {
			ST7565_Print(0, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(7, 38, DY, &Font_7x9, 1, PIX_ON);
			ST7565_Print(15, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(23, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(30, 38, MT, &Font_7x9, 1, PIX_ON);
			ST7565_Print(38, 38, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 38, "0", &Font_7x9, 1, PIX_ON);
			ST7565_Print(53, 38, YE, &Font_7x9, 1, PIX_ON);
		}
		switch (wd) {
		case 1:
			ST7565_Print(61, 38, ",", &Font_7x9, 1, PIX_ON);
			ST7565_Print(69, 38, DOW[0], &Font_7x9, 1, PIX_ON);
			break;
		case 2:
			ST7565_Print(61, 38, ",", &Font_7x9, 1, PIX_ON);
			ST7565_Print(69, 38, DOW[1], &Font_7x9, 1, PIX_ON);
			break;
		case 3:
			ST7565_Print(61, 38, ",", &Font_7x9, 1, PIX_ON);
			ST7565_Print(69, 38, DOW[2], &Font_7x9, 1, PIX_ON);
			break;
		case 4:
			ST7565_Print(61, 38, ",", &Font_7x9, 1, PIX_ON);
			ST7565_Print(69, 38, DOW[3], &Font_7x9, 1, PIX_ON);
			break;
		case 5:
			ST7565_Print(61, 38, ",", &Font_7x9, 1, PIX_ON);
			ST7565_Print(69, 38, DOW[4], &Font_7x9, 1, PIX_ON);
			break;
		case 6:
			ST7565_Print(61, 38, ",", &Font_7x9, 1, PIX_ON);
			ST7565_Print(69, 38, DOW[5], &Font_7x9, 1, PIX_ON);
			break;
		case 7:
			ST7565_Print(61, 38, ",", &Font_7x9, 1, PIX_ON);
			ST7565_Print(69, 38, DOW[6], &Font_7x9, 1, PIX_ON);
			break;
		}
	}
}

//temperature_humidity
void temp_humi_measure(void) {
	sht3x_read_temperature_and_humidity(&handle, &temperature, &humidity);
	humi = (int32_t) humidity;
	temper = (int32_t) temperature;
	sprintf(TEMPERATURE, "%ld", temper);
	sprintf(HUMIDITY, "%ld", humi);
	//sprintf(T, "%ld", t);
	//temperature
	if (temper >= 0 && temper > 9) {
		ST7565_DrawBitmap(90, 12, termometer_7x9, 7, 9, PIX_ON);
		ST7565_Print(98, 12, TEMPERATURE, &Font_7x9, 1, PIX_ON);
		ST7565_DrawPixel(113, 13, PIX_OFF);
		ST7565_DrawRectangleFilled(115, 12, 4, 9, PIX_OFF);
		ST7565_DrawRectangleFilled(112, 15, 3, 6, PIX_OFF);
		ST7565_DrawBitmap(112, 12, celsus_7x9, 7, 9, PIX_ON);
		ST7565_Print(119, 12, " ", &Font_7x9, 1, PIX_ON);
	} else if (temper >= 0 && temper < 10) {
		ST7565_DrawBitmap(90, 12, termometer_7x9, 7, 9, PIX_ON);
		ST7565_Print(98, 12, TEMPERATURE, &Font_7x9, 1, PIX_ON);
		ST7565_DrawPixel(106, 13, PIX_OFF);
		ST7565_DrawRectangleFilled(108, 12, 4, 9, PIX_OFF);
		ST7565_DrawRectangleFilled(105, 15, 3, 6, PIX_OFF);
		ST7565_DrawBitmap(105, 12, celsus_7x9, 7, 9, PIX_ON);
		ST7565_Print(112, 12, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(119, 12, " ", &Font_7x9, 1, PIX_ON);
	} else if (temper < 0 && temper > -10) {
		ST7565_DrawBitmap(90, 12, termometer_7x9, 7, 9, PIX_ON);
		ST7565_Print(98, 12, TEMPERATURE, &Font_7x9, 1, PIX_ON);
		ST7565_DrawPixel(113, 13, PIX_OFF);
		ST7565_DrawRectangleFilled(115, 12, 4, 9, PIX_OFF);
		ST7565_DrawRectangleFilled(112, 15, 3, 6, PIX_OFF);
		ST7565_DrawBitmap(112, 12, celsus_7x9, 7, 9, PIX_ON);
		ST7565_Print(119, 12, " ", &Font_7x9, 1, PIX_ON);
	} else if (temper < 0 && temper < -9 && temper >= -40) {
		ST7565_DrawBitmap(90, 12, termometer_7x9, 7, 9, PIX_ON);
		ST7565_Print(98, 12, TEMPERATURE, &Font_7x9, 1, PIX_ON);
		ST7565_DrawBitmap(119, 12, celsus_7x9, 7, 9, PIX_ON);
	} else if (temper >= 0 && temper > 99) {
		ST7565_DrawBitmap(90, 12, termometer_7x9, 7, 9, PIX_ON);
		ST7565_Print(98, 12, "ВЫС", &Font_7x9, 1, PIX_ON);
		ST7565_Print(119, 12, " ", &Font_7x9, 1, PIX_ON);
	} else if (temper < 0 && temper < -40) {
		ST7565_DrawBitmap(90, 12, termometer_7x9, 7, 9, PIX_ON);
		ST7565_Print(98, 12, "НИЗ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(119, 12, " ", &Font_7x9, 1, PIX_ON);
	}

	//humidity
	if (humi >= 10 && humi < 90) {
		ST7565_DrawBitmap(90, 22, blob_7x9, 7, 9, PIX_ON);
		ST7565_Print(98, 22, HUMIDITY, &Font_7x9, 1, PIX_ON);
		ST7565_Print(112, 22, &PROC, &Font_7x9, 1, PIX_ON);
	}
	if (humi >= 90) {
		ST7565_DrawBitmap(90, 22, blob_7x9, 7, 9, PIX_ON);
		ST7565_Print(98, 22, "ВЫС", &Font_7x9, 1, PIX_ON);
	} else if (humi < 10) {
		ST7565_DrawBitmap(90, 22, blob_7x9, 7, 9, PIX_ON);
		ST7565_Print(98, 22, "НИЗ", &Font_7x9, 1, PIX_ON);
	}

	//smiles
	if (temper < 19 && humi < 20) {
		ST7565_DrawPixel(93, 4, PIX_OFF);
		ST7565_DrawPixel(98, 4, PIX_OFF);
		ST7565_DrawLine(94, 5, 97, 5, PIX_OFF);
		ST7565_DrawBitmap(90, 0, sad_12x8, 12, 8, PIX_ON);
	} else if (temper < 19 && humi >= 20 && humi <= 85) {
		ST7565_DrawPixel(93, 4, PIX_OFF);
		ST7565_DrawPixel(98, 4, PIX_OFF);
		ST7565_DrawLine(94, 5, 97, 5, PIX_OFF);
		ST7565_DrawBitmap(90, 0, sad_12x8, 12, 8, PIX_ON);
	} else if (temper < 19 && humi > 85) {
		ST7565_DrawPixel(93, 4, PIX_OFF);
		ST7565_DrawPixel(98, 4, PIX_OFF);
		ST7565_DrawLine(94, 5, 97, 5, PIX_OFF);
		ST7565_DrawBitmap(90, 0, sad_12x8, 12, 8, PIX_ON);
	} else if (temper >= 19 && temper <= 27 && humi < 20) {
		ST7565_DrawPixel(93, 4, PIX_OFF);
		ST7565_DrawPixel(98, 4, PIX_OFF);
		ST7565_DrawLine(94, 5, 97, 5, PIX_OFF);
		ST7565_DrawBitmap(90, 0, sad_12x8, 12, 8, PIX_ON);
	} else if (temper >= 19 && temper <= 27 && humi >= 20 && humi <= 85) {
		ST7565_DrawPixel(93, 5, PIX_OFF);
		ST7565_DrawPixel(98, 5, PIX_OFF);
		ST7565_DrawLine(94, 4, 97, 4, PIX_OFF);
		ST7565_DrawBitmap(90, 0, smile_12x8, 12, 8, PIX_ON);
	} else if (temper >= 19 && temper <= 27 && humi > 85) {
		ST7565_DrawPixel(93, 4, PIX_OFF);
		ST7565_DrawPixel(98, 4, PIX_OFF);
		ST7565_DrawLine(94, 5, 97, 5, PIX_OFF);
		ST7565_DrawBitmap(90, 0, sad_12x8, 12, 8, PIX_ON);
	} else if (temper > 27 && humi < 20) {
		ST7565_DrawPixel(93, 4, PIX_OFF);
		ST7565_DrawPixel(98, 4, PIX_OFF);
		ST7565_DrawLine(94, 5, 97, 5, PIX_OFF);
		ST7565_DrawBitmap(90, 0, sad_12x8, 12, 8, PIX_ON);
	} else if (temper > 27 && humi >= 20 && humi <= 85) {
		ST7565_DrawPixel(93, 4, PIX_OFF);
		ST7565_DrawPixel(98, 4, PIX_OFF);
		ST7565_DrawLine(94, 5, 97, 5, PIX_OFF);
		ST7565_DrawBitmap(90, 0, sad_12x8, 12, 8, PIX_ON);
	} else if (temper > 27 && humi > 85) {
		ST7565_DrawPixel(93, 4, PIX_OFF);
		ST7565_DrawPixel(98, 4, PIX_OFF);
		ST7565_DrawLine(94, 5, 97, 5, PIX_OFF);
		ST7565_DrawBitmap(90, 0, sad_12x8, 12, 8, PIX_ON);
	}

}

//pressure
void pressure_measure(void) {
	bmp280_read_float(&bmp280, &tmp, &prs, &hum);
	pressure = (uint32_t) prs;
	pressure = pressure / 133.3224;
	sprintf(PRESSURE, "%ld", pressure);
	ST7565_DrawBitmap(89, 32, cloud_7x9, 7, 9, PIX_ON);
	ST7565_Print(98, 34, PRESSURE, &Font_6x8, 1, PIX_ON);
	ST7565_Print(116, 34, "ММ", &Font_6x8, 1, PIX_ON);
}

//main menu cursor
void change_main_sel(void) {
	ST7565_Print(0, 54, "Меню", &Font_7x9, 1, PIX_ON);
	ST7565_Print(64, 54, "Будильник", &Font_7x9, 1, PIX_ON);
	if (!mainsel) {
		ST7565_Print(29, 54, "<", &Font_7x9, 1, PIX_ON);
		ST7565_Print(56, 54, " ", &Font_7x9, 1, PIX_ON);
	} else {
		ST7565_Print(56, 54, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(29, 54, " ", &Font_7x9, 1, PIX_ON);
	}
	if (!mainsel && HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0
			&& HAL_GetTick() - buttontime > 200) {
		buttontime = HAL_GetTick();
		mainmenu = false;
		ST7565_Clear();
		ST7565_ClearBuffer();
		ST7565_Update();
		close_men_time = HAL_GetTick();
	} else if (mainsel && HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0
			&& HAL_GetTick() - buttontime > 200) {
		buttontime = HAL_GetTick();
		mainmenu = false;
		ST7565_Clear();
		ST7565_ClearBuffer();
		ST7565_Update();
		alarm_set = true;
		close_men_time = HAL_GetTick();
		TIM1->CCR1 = BRIGHTNESS[bright];
		ST7565_Update();
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0) {
		mainsel = !mainsel;
		timelock = HAL_GetTick();
		buttontime = HAL_GetTick();
		TIM1->CCR1 = BRIGHTNESS[bright];
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0) {
		mainsel = !mainsel;
		timelock = HAL_GetTick();
		buttontime = HAL_GetTick();
		TIM1->CCR1 = BRIGHTNESS[bright];
		ST7565_Update();
	}
}

//YOU'RE GOING TO SLEEP!!
void PREPARE_TO_SLEEP(void) {
	HAL_ADCEx_DisableVoltageRegulator(&hadc1);
	HAL_ADC_DeInit(&hadc1);
	HAL_PWREx_EnableFlashPowerDown(PWR_FLASHPD_STOP);

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pins : RES_Pin DC_Pin CS_Pin */
	GPIO_InitStruct.Pin = RES_Pin | DC_Pin | CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_VDD_Pin */
	GPIO_InitStruct.Pin = LCD_VDD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	// GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LCD_VDD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : METEO_VDD_Pin MEMORY_VDD_Pin */
	GPIO_InitStruct.Pin = METEO_VDD_Pin | MEMORY_VDD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//menu
void disp_menu(void) {
	ST7565_Print(8, 11, "Будильник", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 21, "Дата и время", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 32, "Другие настройки", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 43, "Об устройстве", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 54, "Выход из меню", &Font_7x9, 1, PIX_ON);
	ST7565_Update();
}

//menu cursor
void change_menu_sel(void) {
	if (menusel < 0)
		menusel = 4;
	else if (menusel > 4)
		menusel = 0;
	switch (menusel) {
	case 0:
		ST7565_Print(0, 11, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 1:
		ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 2:
		ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 3:
		ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 4:
		ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, ">", &Font_7x9, 1, PIX_ON);
		break;
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0) {
		menusel--;
		buttontime = HAL_GetTick();
		ST7565_Update();
		close_men_time = HAL_GetTick();

	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0) {
		menusel++;
		buttontime = HAL_GetTick();
		ST7565_Update();
		close_men_time = HAL_GetTick();
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
		buttontime = HAL_GetTick();
		switch (menusel) {
		case 0:
			ST7565_Clear();
			ST7565_ClearBuffer();
			ST7565_Update();
			menusel = 0;
			alarm_set = true;
			read_alarms = true;
			break;
		case 1:
			ST7565_Clear();
			ST7565_ClearBuffer();
			ST7565_Update();
			menusel = 0;
			time_date = true;
			bring_date = true;
			break;
		case 2:
			ST7565_Clear();
			ST7565_ClearBuffer();
			ST7565_Update();
			menusel = 0;
			otherload = true;
			other_settings = true;
			break;
		case 3:
			ST7565_Clear();
			ST7565_ClearBuffer();
			ST7565_Update();
			menusel = 0;
			about_prog = true;
			break;
		case 4:
			mainmenu = true;
			dynew = 0;
			ST7565_Clear();
			ST7565_ClearBuffer();
			ST7565_Update();
			menusel = 0;
			mainsel = false;
			timelock = HAL_GetTick();
			break;
		}
	}
}

//alarm settings... Very bad code... and so crutcher...
void alarm_settings(void) {
	static char AL1_HR[5], AL2_HR[5], AL1_MI[5], AL2_MI[5];
	//static uint_fast8_t date_for_alarm1 = 0, date_for_alarm2 = 0;
	if (read_alarms) {
		if (!AT24Cxx_read_data(ALARM1HR_ADDR, (uint8_t*) &al1_hr,
				sizeof(al1_hr))) {
		}
		AT24Cxx_read_data(ALARM1MI_ADDR, (uint8_t*) &al1_mi, sizeof(al1_mi));
		AT24Cxx_read_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
				sizeof(alarm1_set));
		AT24Cxx_read_data(ALARM2HR_ADDR, (uint8_t*) &al2_hr, sizeof(al2_hr));
		AT24Cxx_read_data(ALARM2MI_ADDR, (uint8_t*) &al2_mi, sizeof(al2_mi));
		AT24Cxx_read_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
				sizeof(alarm2_set));
		read_alarms = false;
	}
	if (alarm_sel < 0)
		alarm_sel = 4;
	else if (alarm_sel > 4)
		alarm_sel = 0;
	if (alarm1_sel > 3)
		alarm1_sel = 3;
	else if (alarm1_sel < 0)
		alarm1_sel = 0;
	if (alarm2_sel > 3)
		alarm2_sel = 3;
	else if (alarm2_sel < 0)
		alarm2_sel = 0;
	if (alarm1_set)
		ST7565_Print(44, 11, "ВКЛ ", &Font_7x9, 1, PIX_ON);
	else if (!alarm1_set)
		ST7565_Print(44, 11, "ВЫКЛ", &Font_7x9, 1, PIX_ON);
	if (alarm2_set)
		ST7565_Print(44, 21, "ВКЛ ", &Font_7x9, 1, PIX_ON);
	else if (!alarm2_set)
		ST7565_Print(44, 21, "ВЫКЛ", &Font_7x9, 1, PIX_ON);
	if (al1_hr > 23)
		al1_hr = 0;
	else if (al1_hr < 0)
		al1_hr = 23;
	if (al1_mi > 59)
		al1_mi = 0;
	else if (al1_mi < 0)
		al1_mi = 59;
	if (al2_hr > 23)
		al2_hr = 0;
	else if (al2_hr < 0)
		al2_hr = 23;
	if (al2_mi > 59)
		al2_mi = 0;
	else if (al2_mi < 0)
		al2_mi = 59;
	ST7565_Print(8, 11, "Буд1:", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 21, "Буд2:", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 32, "Выкл.будильники", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 43, "Сохр.изменения", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 54, "Выход", &Font_7x9, 1, PIX_ON);
	sprintf(AL1_HR, "%d", al1_hr);
	sprintf(AL2_HR, "%d", al2_hr);
	sprintf(AL1_MI, "%d", al1_mi);
	sprintf(AL2_MI, "%d", al2_mi);
	if (al1_hr > 9 && al1_mi > 9) {
		ST7565_Print(81, 11, AL1_HR, &Font_7x9, 1, PIX_ON);
		ST7565_Print(96, 11, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(104, 11, AL1_MI, &Font_7x9, 1, PIX_ON);
	} else if (al1_hr > 9 && al1_mi < 10) {
		ST7565_Print(81, 11, AL1_HR, &Font_7x9, 1, PIX_ON);
		ST7565_Print(96, 11, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(104, 11, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(111, 11, AL1_MI, &Font_7x9, 1, PIX_ON);
	} else if (al1_hr < 10 && al1_mi > 9) {
		ST7565_Print(81, 11, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(88, 11, AL1_HR, &Font_7x9, 1, PIX_ON);
		ST7565_Print(96, 11, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(104, 11, AL1_MI, &Font_7x9, 1, PIX_ON);
	} else if (al1_hr < 10 && al1_mi < 10) {
		ST7565_Print(81, 11, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(88, 11, AL1_HR, &Font_7x9, 1, PIX_ON);
		ST7565_Print(96, 11, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(104, 11, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(111, 11, AL1_MI, &Font_7x9, 1, PIX_ON);
	}

	if (al2_hr > 9 && al2_mi > 9) {
		ST7565_Print(81, 21, AL2_HR, &Font_7x9, 1, PIX_ON);
		ST7565_Print(96, 21, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(104, 21, AL2_MI, &Font_7x9, 1, PIX_ON);
	} else if (al2_hr > 9 && al2_mi < 10) {
		ST7565_Print(81, 21, AL2_HR, &Font_7x9, 1, PIX_ON);
		ST7565_Print(96, 21, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(104, 21, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(111, 21, AL2_MI, &Font_7x9, 1, PIX_ON);
	} else if (al2_hr < 10 && al2_mi > 9) {
		ST7565_Print(81, 21, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(88, 21, AL2_HR, &Font_7x9, 1, PIX_ON);
		ST7565_Print(96, 21, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(104, 21, AL2_MI, &Font_7x9, 1, PIX_ON);
	} else if (al2_hr < 10 && al2_mi < 10) {
		ST7565_Print(81, 21, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(88, 21, AL2_HR, &Font_7x9, 1, PIX_ON);
		ST7565_Print(96, 21, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(104, 21, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(111, 21, AL2_MI, &Font_7x9, 1, PIX_ON);
	}
	ST7565_Update();
	switch (alarm_sel) {
	case 0:
		if (!alarm1_settings) {
			ST7565_Print(0, 11, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		} else if (alarm1_settings) {
			ST7565_Print(0, 11, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		}
		break;
	case 1:
		if (!alarm2_settings) {
			ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		} else if (alarm2_settings) {
			ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		}
		break;
	case 2:
		ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 3:
		ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 4:
		ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, ">", &Font_7x9, 1, PIX_ON);
		break;
	}
	switch (alarm1_sel) {
	case 0:
		ST7565_Print(73, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(119, 11, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 1:
		ST7565_Print(73, 11, "<", &Font_7x9, 1, PIX_ON);
		ST7565_Print(119, 11, " ", &Font_7x9, 1, PIX_ON);
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			alarm1_set = !alarm1_set;
		}
		break;
	case 2:
		if (!alarm1_hr_set) {
			ST7565_Print(73, 11, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(119, 11, " ", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(73, 11, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(119, 11, " ", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			alarm1_hr_set = !alarm1_hr_set;
		}
		break;
	case 3:
		if (!alarm1_mi_set) {
			ST7565_Print(73, 11, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(119, 11, "<", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(73, 11, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(119, 11, "?", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			alarm1_mi_set = !alarm1_mi_set;
		}
		break;
	}

	switch (alarm2_sel) {
	case 0:
		ST7565_Print(73, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(119, 21, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 1:
		ST7565_Print(73, 21, "<", &Font_7x9, 1, PIX_ON);
		ST7565_Print(119, 21, " ", &Font_7x9, 1, PIX_ON);
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			alarm2_set = !alarm2_set;
		}
		break;
	case 2:
		if (!alarm2_hr_set) {
			ST7565_Print(73, 21, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(119, 21, " ", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(73, 21, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(119, 21, " ", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			alarm2_hr_set = !alarm2_hr_set;
		}
		break;
	case 3:
		if (!alarm2_mi_set) {
			ST7565_Print(73, 21, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(119, 21, "<", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(73, 21, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(119, 21, "?", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			alarm2_mi_set = !alarm2_mi_set;
		}
		break;
	}

	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !alarm1_settings
			&& !alarm2_settings && !alarm1_hr_set && !alarm2_hr_set
			&& !alarm1_mi_set && !alarm2_mi_set) {
		alarm_sel--;
		buttontime = HAL_GetTick();
		ST7565_Update();

	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0
			&& !alarm1_settings && !alarm2_settings && !alarm1_hr_set
			&& !alarm2_hr_set && !alarm1_mi_set && !alarm2_mi_set) {
		alarm_sel++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0
			&& alarm1_settings && !alarm2_settings && !alarm1_hr_set
			&& !alarm2_hr_set && !alarm1_mi_set && !alarm2_mi_set) {
		alarm1_sel++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && alarm1_settings
			&& !alarm2_settings && !alarm1_hr_set && !alarm2_hr_set
			&& !alarm1_mi_set && !alarm2_mi_set) {
		alarm1_sel--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !alarm1_settings
			&& alarm2_settings && !alarm1_hr_set && !alarm2_hr_set
			&& !alarm1_mi_set && !alarm2_mi_set) {
		alarm2_sel--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0
			&& !alarm1_settings && alarm2_settings && !alarm1_hr_set
			&& !alarm2_hr_set && !alarm1_mi_set && !alarm2_mi_set) {
		alarm2_sel++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0
			&& alarm1_settings && !alarm2_settings && alarm1_hr_set
			&& !alarm2_hr_set && !alarm1_mi_set && !alarm2_mi_set) {
		al1_hr--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && alarm1_settings
			&& !alarm2_settings && alarm1_hr_set && !alarm2_hr_set
			&& !alarm1_mi_set && !alarm2_mi_set) {
		al1_hr++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0
			&& alarm1_settings && !alarm2_settings && !alarm1_hr_set
			&& !alarm2_hr_set && alarm1_mi_set && !alarm2_mi_set) {
		al1_mi--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && alarm1_settings
			&& !alarm2_settings && !alarm1_hr_set && !alarm2_hr_set
			&& alarm1_mi_set && !alarm2_mi_set) {
		al1_mi++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0
			&& !alarm1_settings && alarm2_settings && !alarm1_hr_set
			&& alarm2_hr_set && !alarm1_mi_set && !alarm2_mi_set) {
		al2_hr--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !alarm1_settings
			&& alarm2_settings && !alarm1_hr_set && alarm2_hr_set
			&& !alarm1_mi_set && !alarm2_mi_set) {
		al2_hr++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0
			&& !alarm1_settings && alarm2_settings && !alarm1_hr_set
			&& !alarm2_hr_set && !alarm1_mi_set && alarm2_mi_set) {
		al2_mi--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !alarm1_settings
			&& alarm2_settings && !alarm1_hr_set && !alarm2_hr_set
			&& !alarm1_mi_set && alarm2_mi_set) {
		al2_mi++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
		buttontime = HAL_GetTick();
		switch (alarm_sel) {
		case 0:
			if (alarm1_sel == 0)
				alarm1_settings = !alarm1_settings;
			break;
		case 1:
			if (alarm2_sel == 0)
				alarm2_settings = !alarm2_settings;
			break;
		case 2:
			alarm1_set = false;
			alarm2_set = false;
			break;
		case 3:
			if (al1_hr == al2_hr && al1_mi == al2_mi) {
				ST7565_Clear();
				ST7565_Print(1, 21, "Изм-ния не сохр-ны", &Font_7x9, 1, PIX_ON);
				ST7565_Print(1, 32, "Время срабатывания", &Font_7x9, 1, PIX_ON);
				ST7565_Print(3, 32, "будильников совп.", &Font_7x9, 1, PIX_ON);
				ST7565_Update();
				HAL_Delay(3000);
				ST7565_Clear();
				break;
			}
			if (alarm1_set && !alarm2_set) {
				if (DS3231_GetSecond() == 59)
					HAL_Delay(1500);
				if (!AT24Cxx_write_data(ALARM1HR_ADDR, (uint8_t*) &al1_hr,
						sizeof(al1_hr))) {
					Error_Handler();
				}
				AT24Cxx_write_data(ALARM1MI_ADDR, (uint8_t*) &al1_mi,
						sizeof(al1_mi));
				AT24Cxx_write_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
						sizeof(alarm1_set));
				if (!AT24Cxx_write_data(ALARM2HR_ADDR, (uint8_t*) &al2_hr,
						sizeof(al2_hr))) {
					Error_Handler();
				}
				AT24Cxx_write_data(ALARM2MI_ADDR, (uint8_t*) &al2_mi,
						sizeof(al2_mi));
				AT24Cxx_write_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
						sizeof(alarm2_set));
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
				sTime.Hours = 0;
				sTime.Minutes = 0;
				sTime.Seconds = 0;
				sTime.SubSeconds = 0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
				sDate.Month = RTC_MONTH_JANUARY;
				sDate.Date = 1;
				sDate.Year = 0;

				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sAlarm.AlarmTime.Hours = 0;
				sAlarm.AlarmTime.Minutes = 0;
				sAlarm.AlarmTime.Seconds = 10;
				sAlarm.AlarmTime.SubSeconds = 0;
				sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
				sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY
						| RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
				sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
				sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
				sAlarm.AlarmDateWeekDay = 1;
				sAlarm.Alarm = RTC_ALARM_A;
				if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN)
						!= HAL_OK) {
					Error_Handler();
				}
			} else if (alarm1_set && alarm2_set) {
				if (DS3231_GetSecond() == 59)
					HAL_Delay(1500);
				if (!AT24Cxx_write_data(ALARM1HR_ADDR, (uint8_t*) &al1_hr,
						sizeof(al1_hr))) {
					Error_Handler();
				}
				AT24Cxx_write_data(ALARM1MI_ADDR, (uint8_t*) &al1_mi,
						sizeof(al1_mi));
				AT24Cxx_write_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
						sizeof(alarm1_set));
				if (!AT24Cxx_write_data(ALARM2HR_ADDR, (uint8_t*) &al2_hr,
						sizeof(al2_hr))) {
					Error_Handler();
				}
				AT24Cxx_write_data(ALARM2MI_ADDR, (uint8_t*) &al2_mi,
						sizeof(al2_mi));
				AT24Cxx_write_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
						sizeof(alarm2_set));
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
				sTime.Hours = 0;
				sTime.Minutes = 0;
				sTime.Seconds = 0;
				sTime.SubSeconds = 0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
				sDate.Month = RTC_MONTH_JANUARY;
				sDate.Date = 1;
				sDate.Year = 0;

				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sAlarm.AlarmTime.Hours = 0;
				sAlarm.AlarmTime.Minutes = 0;
				sAlarm.AlarmTime.Seconds = 10;
				sAlarm.AlarmTime.SubSeconds = 0;
				sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
				sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY
						| RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
				sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
				sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
				sAlarm.AlarmDateWeekDay = 1;
				sAlarm.Alarm = RTC_ALARM_A;
				if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN)
						!= HAL_OK) {
					Error_Handler();
				}
			} else if (!alarm1_set && alarm2_set) {
				if (DS3231_GetSecond() == 59)
					HAL_Delay(1500);
				if (!AT24Cxx_write_data(ALARM1HR_ADDR, (uint8_t*) &al1_hr,
						sizeof(al1_hr))) {
					Error_Handler();
				}
				AT24Cxx_write_data(ALARM1MI_ADDR, (uint8_t*) &al1_mi,
						sizeof(al1_mi));
				AT24Cxx_write_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
						sizeof(alarm1_set));
				if (!AT24Cxx_write_data(ALARM2HR_ADDR, (uint8_t*) &al2_hr,
						sizeof(al2_hr))) {
					Error_Handler();
				}
				AT24Cxx_write_data(ALARM2MI_ADDR, (uint8_t*) &al2_mi,
						sizeof(al2_mi));
				AT24Cxx_write_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
						sizeof(alarm2_set));
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
				sTime.Hours = 0;
				sTime.Minutes = 0;
				sTime.Seconds = 0;
				sTime.SubSeconds = 0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
				sDate.Month = RTC_MONTH_JANUARY;
				sDate.Date = 1;
				sDate.Year = 0;

				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sAlarm.AlarmTime.Hours = 0;
				sAlarm.AlarmTime.Minutes = 0;
				sAlarm.AlarmTime.Seconds = 10;
				sAlarm.AlarmTime.SubSeconds = 0;
				sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
				sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY
						| RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
				sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
				sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
				sAlarm.AlarmDateWeekDay = 1;
				sAlarm.Alarm = RTC_ALARM_A;
				if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN)
						!= HAL_OK) {
					Error_Handler();
				}
			} else if (!alarm1_set && !alarm2_set) {
				if (!AT24Cxx_write_data(ALARM1HR_ADDR, (uint8_t*) &al1_hr,
						sizeof(al1_hr))) {
					Error_Handler();
				}
				AT24Cxx_write_data(ALARM1MI_ADDR, (uint8_t*) &al1_mi,
						sizeof(al1_mi));
				AT24Cxx_write_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
						sizeof(alarm1_set));
				if (!AT24Cxx_write_data(ALARM2HR_ADDR, (uint8_t*) &al2_hr,
						sizeof(al2_hr))) {
					Error_Handler();
				}
				AT24Cxx_write_data(ALARM2MI_ADDR, (uint8_t*) &al2_mi,
						sizeof(al2_mi));
				AT24Cxx_write_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
						sizeof(alarm2_set));
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
				sTime.Hours = 0;
				sTime.Minutes = 0;
				sTime.Seconds = 0;
				sTime.SubSeconds = 0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
				sDate.Month = RTC_MONTH_JANUARY;
				sDate.Date = 1;
				sDate.Year = 0;

				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
			}
			ST7565_Clear();
			ST7565_Print(3, 32, "Изменения сохр-ны", &Font_7x9, 1, PIX_ON);
			ST7565_Update();
			HAL_Delay(2000);
			ST7565_Clear();
			break;
		case 4:
			menusel = 0;
			alarm_sel = 0;
			ST7565_Clear();
			ST7565_ClearBuffer();
			ST7565_Update();
			close_men_time = HAL_GetTick();
			alarm_set = false;
			break;

		}
	}

}
//checking alarms
void alarm_check(void) {
	al_hr_trg = DS3231_GetHour();
	al_mi_trg = DS3231_GetMinute();
	if (al_hr_trg == al1_hr && al_mi_trg == al1_mi && alarm1_set == true) {
		timelock = HAL_GetTick();
		sleep = false;
		ALARM_TRG = false;
		isAlarm1 = true;
		clear_temp = true;
		alarm1_think();
	} else {
		ALARM_TRG = false;
	}
	if (al_hr_trg == al2_hr && al_mi_trg == al2_mi && alarm2_set == true) {
		timelock = HAL_GetTick();
		sleep = false;
		ALARM_TRG = false;
		isAlarm2 = true;
		clear_temp = true;
		alarm2_think();
	} else {
		ALARM_TRG = false;
	}
}
//alarm 1 actions
void alarm1_think(void) {
	if (clear_temp) {
		ST7565_ClearBuffer();
		ST7565_Clear();
		clear_temp = false;
	}
	while (isAlarm1 && al_mi_trg == al1_mi) {
		al_hr_trg = DS3231_GetHour();
		al_mi_trg = DS3231_GetMinute();
		if (al_hr_trg == al2_hr && al_mi_trg == al2_mi && alarm2_set == true) {
			timelock = HAL_GetTick();
			sleep = false;
			ALARM_TRG = false;
			isAlarm2 = true;
		} else {
			ALARM_TRG = false;
		}
		ST7565_Print(31, 11, "Будильник1", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 21, "Выключить?", &Font_7x9, 1, PIX_ON);
		ST7565_Print(101, 21, "<", &Font_7x9, 1, PIX_ON);
		ST7565_Update();
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(alarm_disable_GPIO_Port,
				alarm_disable_Pin) == 0) {
			isAlarm1 = false;
			if (!alarm2_set) {
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
			}
			alarm1_set = false;
			ALARM_TRG = false;
			if (!AT24Cxx_write_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
					sizeof(alarm1_set))) {
				Error_Handler();
			}
			sTime.Hours = 0;
			sTime.Minutes = 0;
			sTime.Seconds = 0;
			sTime.SubSeconds = 0;
			sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			sTime.StoreOperation = RTC_STOREOPERATION_RESET;
			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
			sDate.WeekDay = RTC_WEEKDAY_MONDAY;
			sDate.Month = RTC_MONTH_JANUARY;
			sDate.Date = 1;
			sDate.Year = 0;

			if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
			timelock = HAL_GetTick();
			ST7565_ClearBuffer();
			ST7565_Clear();
		} else if (HAL_GPIO_ReadPin(ok_GPIO_Port,
		ok_Pin) == 0 && HAL_GetTick() - buttontime > 200) {
			isAlarm1 = false;
			if (!alarm2_set) {
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
			}
			alarm1_set = false;
			ALARM_TRG = false;
			if (!AT24Cxx_write_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
					sizeof(alarm1_set))) {
				Error_Handler();
			}
			sTime.Hours = 0;
			sTime.Minutes = 0;
			sTime.Seconds = 0;
			sTime.SubSeconds = 0;
			sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			sTime.StoreOperation = RTC_STOREOPERATION_RESET;
			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
			sDate.WeekDay = RTC_WEEKDAY_MONDAY;
			sDate.Month = RTC_MONTH_JANUARY;
			sDate.Date = 1;
			sDate.Year = 0;

			if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
			timelock = HAL_GetTick();
			ST7565_ClearBuffer();
			ST7565_Clear();
		}
		alarm_sound();
		status_bar();
	}
	isAlarm1 = false;
	if (!isAlarm1) {
		TIM3->CCR1 = 0;
		alarm_s_flag = false;
		dynew = 0;
		if (!alarm2_set) {
			HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
		}
		alarm1_set = false;
		ALARM_TRG = false;
		if (!AT24Cxx_write_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
				sizeof(alarm1_set))) {
			Error_Handler();
		}
		sTime.Hours = 0;
		sTime.Minutes = 0;
		sTime.Seconds = 0;
		sTime.SubSeconds = 0;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Month = RTC_MONTH_JANUARY;
		sDate.Date = 1;
		sDate.Year = 0;

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
		timelock = HAL_GetTick();
		ST7565_ClearBuffer();
		ST7565_Clear();
	}
}

//alarm 2 actions
void alarm2_think(void) {
	if (clear_temp) {
		ST7565_ClearBuffer();
		ST7565_Clear();
		clear_temp = false;
	}
	while (isAlarm2 && al_mi_trg == al2_mi) {
		al_hr_trg = DS3231_GetHour();
		al_mi_trg = DS3231_GetMinute();
		if (al_hr_trg == al1_hr && al_mi_trg == al1_mi && alarm1_set == true) {
			timelock = HAL_GetTick();
			sleep = false;
			ALARM_TRG = false;
			isAlarm1 = true;
		} else {
			ALARM_TRG = false;
		}
		ST7565_Print(31, 11, "Будильник2", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 21, "Выключить?", &Font_7x9, 1, PIX_ON);
		ST7565_Print(101, 21, "<", &Font_7x9, 1, PIX_ON);
		ST7565_Update();
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(alarm_disable_GPIO_Port,
				alarm_disable_Pin) == 0) {
			isAlarm2 = false;
			if (!alarm1_set) {
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
			}
			alarm2_set = false;
			ALARM_TRG = false;
			if (!AT24Cxx_write_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
					sizeof(alarm2_set))) {
				Error_Handler();
			}
			sTime.Hours = 0;
			sTime.Minutes = 0;
			sTime.Seconds = 0;
			sTime.SubSeconds = 0;
			sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			sTime.StoreOperation = RTC_STOREOPERATION_RESET;
			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
			sDate.WeekDay = RTC_WEEKDAY_MONDAY;
			sDate.Month = RTC_MONTH_JANUARY;
			sDate.Date = 1;
			sDate.Year = 0;

			if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
			timelock = HAL_GetTick();
			ST7565_ClearBuffer();
			ST7565_Clear();
		} else if (HAL_GPIO_ReadPin(ok_GPIO_Port,
		ok_Pin) == 0 && HAL_GetTick() - buttontime > 200) {
			isAlarm2 = false;
			if (!alarm1_set) {
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
			}
			alarm2_set = false;
			ALARM_TRG = false;
			if (!AT24Cxx_write_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
					sizeof(alarm2_set))) {
				Error_Handler();
			}
			sTime.Hours = 0;
			sTime.Minutes = 0;
			sTime.Seconds = 0;
			sTime.SubSeconds = 0;
			sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			sTime.StoreOperation = RTC_STOREOPERATION_RESET;
			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
			sDate.WeekDay = RTC_WEEKDAY_MONDAY;
			sDate.Month = RTC_MONTH_JANUARY;
			sDate.Date = 1;
			sDate.Year = 0;

			if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
			timelock = HAL_GetTick();
			ST7565_ClearBuffer();
			ST7565_Clear();
		}
		alarm_sound();
		status_bar();
	}
	isAlarm2 = false;
	if (!isAlarm2) {
		TIM3->CCR1 = 0;
		alarm_s_flag = false;
		dynew = 0;
		if (!alarm1_set) {
			HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
		}
		alarm2_set = false;
		ALARM_TRG = false;
		if (!AT24Cxx_write_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
				sizeof(alarm2_set))) {
			Error_Handler();
		}
		sTime.Hours = 0;
		sTime.Minutes = 0;
		sTime.Seconds = 0;
		sTime.SubSeconds = 0;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Month = RTC_MONTH_JANUARY;
		sDate.Date = 1;
		sDate.Year = 0;

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
		timelock = HAL_GetTick();
		ST7565_ClearBuffer();
		ST7565_Clear();
	}
	/*	alarm_wait = HAL_GetTick();
	 while (isAlarm2) {
	 ST7565_Print(31, 11, "Будильник2", &Font_7x9, 1, PIX_ON);
	 ST7565_Print(31, 32, "Выключить?", &Font_7x9, 1, PIX_ON);
	 ST7565_Print(11, 54, "Да", &Font_7x9, 1, PIX_ON);
	 ST7565_Print(81, 54, "Нет", &Font_7x9, 1, PIX_ON);
	 if (!alarm_off_sel2) {
	 ST7565_Print(46, 54, "<", &Font_7x9, 1, PIX_ON);
	 ST7565_Print(73, 54, " ", &Font_7x9, 1, PIX_ON);
	 if (HAL_GetTick() - buttontime > 150
	 && HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
	 buttontime = HAL_GetTick();
	 alarm2_pass = false;
	 isAlarm2=false;
	 alarm2_set=false;
	 HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);
	 if (!AT24Cxx_write_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
	 sizeof(alarm2_set))) {
	 }
	 }

	 } else {
	 if (HAL_GetTick() - buttontime > 150
	 && HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
	 buttontime = HAL_GetTick();
	 alarm2_pass = true;
	 isAlarm2=false;
	 }
	 ST7565_Print(73, 54, ">", &Font_7x9, 1, PIX_ON);
	 ST7565_Print(46, 54, " ", &Font_7x9, 1, PIX_ON);
	 }
	 ST7565_Update();
	 if (HAL_GetTick() - buttontime > 150
	 && HAL_GPIO_ReadPin(alarm_disable_GPIO_Port, alarm_disable_Pin)
	 == 0) {
	 buttontime = HAL_GetTick();
	 alarm2_pass = true;
	 }
	 }*/

}

//alarm_signal
void alarm_sound(void) {
	if (HAL_GetTick() - alarm_interval >= 500) {
		alarm_s_flag = !alarm_s_flag;
		alarm_interval = HAL_GetTick();
	}
	if (alarm_s_flag) {
		TIM3->CCR1 = 230;
	} else if (!alarm_s_flag) {
		TIM3->CCR1 = 0;
	}
}

//time and date settings
void time_date_set(void) {
	static char TIME_HR[5], TIME_MI[5], DATE_DY[5], DATE_MO[5], DATE_YR[8];
	if (bring_date) {
		time_hr = DS3231_GetHour();
		time_mi = DS3231_GetMinute();
		date_dy = DS3231_GetDate();
		date_mo = DS3231_GetMonth();
		date_yr = DS3231_GetYear();
		date_dow = DS3231_GetDayOfWeek();
		bring_date = false;
	}
	if (time_date_sel < 0)
		time_date_sel = 3;
	else if (time_date_sel > 3)
		time_date_sel = 0;
	if (date_mo < 1)
		date_mo = 12;
	else if (date_mo > 12)
		date_mo = 1;
	if (date_yr > 2099)
		date_yr = 2000;
	else if (date_yr < 2000) {
		date_yr = 2099;
	}
	if (time_hr > 23)
		time_hr = 0;
	else if (time_hr < 0)
		time_hr = 23;
	if (time_mi > 59)
		time_mi = 0;
	else if (time_mi < 0)
		time_mi = 59;
	if (date_dow > 7)
		date_dow = 1;
	else if (date_dow < 1)
		date_dow = 7;

	if (date_yr % 4) {
		switch (date_mo) {
		case 1:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 2:
			if (date_dy > 28)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 28;
			break;
		case 3:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 4:
			if (date_dy > 30)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 30;
			break;
		case 5:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 6:
			if (date_dy > 30)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 30;
			break;
		case 7:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 8:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 9:
			if (date_dy > 30)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 30;
			break;
		case 10:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 11:
			if (date_dy > 30)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 30;
			break;
		case 12:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		}
	} else {
		switch (date_mo) {
		case 1:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 2:
			if (date_dy > 29)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 29;
			break;
		case 3:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 4:
			if (date_dy > 30)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 30;
			break;
		case 5:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 6:
			if (date_dy > 30)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 30;
			break;
		case 7:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 8:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 9:
			if (date_dy > 30)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 30;
			break;
		case 10:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		case 11:
			if (date_dy > 30)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 30;
			break;
		case 12:
			if (date_dy > 31)
				date_dy = 1;
			else if (date_dy < 1)
				date_dy = 31;
			break;
		}
	}
	if (time_sel > 2)
		time_sel = 2;
	else if (time_sel < 0)
		time_sel = 0;
	if (date_sel > 4)
		date_sel = 4;
	else if (date_sel < 0)
		date_sel = 0;

	sprintf(TIME_HR, "%d", time_hr);
	sprintf(TIME_MI, "%d", time_mi);
	sprintf(DATE_DY, "%d", date_dy);
	sprintf(DATE_MO, "%d", date_mo);
	sprintf(DATE_YR, "%d", date_yr);
	if (time_hr < 10 && time_mi < 10) {
		ST7565_Print(8, 11, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(15, 11, TIME_HR, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(23, 11, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 11, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(38, 11, TIME_MI, &Font_7x9, 1, PIX_ON);
	} else if (time_hr < 10 && time_mi > 9) {
		ST7565_Print(8, 11, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(15, 11, TIME_HR, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(23, 11, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 11, TIME_MI, &Font_7x9, 1, PIX_ON);
	} else if (time_hr > 9 && time_mi < 10) {
		ST7565_Print(8, 11, TIME_HR, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(23, 11, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 11, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(38, 11, TIME_MI, &Font_7x9, 1, PIX_ON);
	} else if (time_hr > 9 && time_mi > 9) {
		ST7565_Print(8, 11, TIME_HR, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(23, 11, ":", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 11, TIME_MI, &Font_7x9, 1, PIX_ON);
	}
	if (date_dy < 10 && date_mo < 10) {
		ST7565_Print(8, 21, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(15, 21, DATE_DY, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 21, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(38, 21, DATE_MO, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(54, 21, DATE_YR, &Font_7x9, 1, PIX_ON);
	} else if (date_dy < 10 && date_mo > 9) {
		ST7565_Print(8, 21, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(15, 21, DATE_DY, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 21, DATE_MO, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(54, 21, DATE_YR, &Font_7x9, 1, PIX_ON);
	} else if (date_dy > 9 && date_mo < 10) {
		ST7565_Print(8, 21, DATE_DY, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 21, "0", &Font_7x9, 1, PIX_ON);
		ST7565_Print(38, 21, DATE_MO, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(54, 21, DATE_YR, &Font_7x9, 1, PIX_ON);
	} else if (date_dy > 9 && date_mo > 9) {
		ST7565_Print(8, 21, DATE_DY, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(31, 21, DATE_MO, &Font_7x9, 1, PIX_ON);
		//ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(54, 21, DATE_YR, &Font_7x9, 1, PIX_ON);
	}
	switch (date_dow) {
	case 1:
//		ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		ST7565_Print(90, 21, DOW[0], &Font_7x9, 1, PIX_ON);
		break;
	case 2:
//		ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		ST7565_Print(90, 21, DOW[1], &Font_7x9, 1, PIX_ON);
		break;
	case 3:
//		ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		ST7565_Print(90, 21, DOW[2], &Font_7x9, 1, PIX_ON);
		break;
	case 4:
//		ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		ST7565_Print(90, 21, DOW[3], &Font_7x9, 1, PIX_ON);
		break;
	case 5:
//		ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		ST7565_Print(90, 21, DOW[4], &Font_7x9, 1, PIX_ON);
		break;
	case 6:
//		ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		ST7565_Print(90, 21, DOW[5], &Font_7x9, 1, PIX_ON);
		break;
	case 7:
//		ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		ST7565_Print(90, 21, DOW[6], &Font_7x9, 1, PIX_ON);
		break;
	}
	ST7565_Print(8, 32, "Сохр.изменения", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 43, "Выход", &Font_7x9, 1, PIX_ON);
	switch (time_date_sel) {
	case 0:
		if (!time_config) {
			ST7565_Print(0, 11, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(0, 11, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		}
		break;
	case 1:
		if (!date_config) {
			ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		}
		break;
	case 2:
		ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 3:
		ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, ">", &Font_7x9, 1, PIX_ON);
		break;
	}
	if (!time_config) {
		ST7565_Print(23, 11, ":", &Font_7x9, 1, PIX_ON);
	}
	if (!date_config) {
		ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
	}
	switch (time_sel) {
	case 0:
		ST7565_Print(23, 11, ":", &Font_7x9, 1, PIX_ON);
		break;
	case 1:
		if (time_hr_set)
			ST7565_Print(23, 11, "?", &Font_7x9, 1, PIX_ON);
		else {
			ST7565_Print(23, 11, "<", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			time_hr_set = !time_hr_set;
			ST7565_Update();

		}
		break;
	case 2:
		if (time_mi_set)
			ST7565_Print(23, 11, "?", &Font_7x9, 1, PIX_ON);
		else {
			ST7565_Print(23, 11, ">", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			time_mi_set = !time_mi_set;
			ST7565_Update();
		}
		break;
	}
	switch (date_sel) {
	case 0:
		ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
		ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		break;
	case 1:
		if (date_dy_set) {
			ST7565_Print(23, 21, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(23, 21, "<", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			date_dy_set = !date_dy_set;
			ST7565_Update();
		}
		break;
	case 2:
		if (date_mo_set) {
			ST7565_Print(23, 21, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(23, 21, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			date_mo_set = !date_mo_set;
			ST7565_Update();
		}
		break;
	case 3:
		if (date_yr_set) {
			ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 21, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 21, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(82, 21, ",", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			date_yr_set = !date_yr_set;
			ST7565_Update();
		}
		break;
	case 4:
		if (date_dow_set) {
			ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(82, 21, "?", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(23, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(46, 21, "/", &Font_7x9, 1, PIX_ON);
			ST7565_Print(82, 21, ">", &Font_7x9, 1, PIX_ON);
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
			buttontime = HAL_GetTick();
			date_dow_set = !date_dow_set;
			ST7565_Update();
		}
		break;
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !time_config
			&& !date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		time_date_sel--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && !time_config
			&& !date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		time_date_sel++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && time_config
			&& !date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		time_sel++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && time_config
			&& !date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		time_sel--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		date_sel++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		date_sel--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && time_config
			&& !date_config && time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		time_hr--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && time_config
			&& !date_config && time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		time_hr++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && time_config
			&& !date_config && !time_hr_set && time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		time_mi--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && time_config
			&& !date_config && !time_hr_set && time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		time_mi++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		date_dy--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && date_dy_set
			&& !date_mo_set && !date_yr_set && !date_dow_set) {
		date_dy++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& date_mo_set && !date_yr_set && !date_dow_set) {
		date_mo--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& date_mo_set && !date_yr_set && !date_dow_set) {
		date_mo++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && date_yr_set && !date_dow_set) {
		date_yr--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && date_yr_set && !date_dow_set) {
		date_yr++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && date_dow_set) {
		date_dow--;
		buttontime = HAL_GetTick();
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !time_config
			&& date_config && !time_hr_set && !time_mi_set && !date_dy_set
			&& !date_mo_set && !date_yr_set && date_dow_set) {
		date_dow++;
		buttontime = HAL_GetTick();
		ST7565_Update();
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
		buttontime = HAL_GetTick();
		switch (time_date_sel) {
		case 0:
			if (time_sel == 0)
				time_config = !time_config;
			break;
		case 1:
			if (date_sel == 0)
				date_config = !date_config;
			break;
		case 2:
			ST7565_Clear();
			ST7565_Print(3, 32, "Изменения сохр-ны", &Font_7x9, 1, PIX_ON);
			ST7565_Update();
			DS3231_SetFullTime(time_hr, time_mi, 0);
			DS3231_SetFullDate(date_dy, date_mo, date_dow, date_yr);
			HAL_Delay(2000);
			ST7565_Clear();
			break;
		case 3:
			menusel = 0;
			time_date_sel = 0;
			ST7565_Clear();
			ST7565_ClearBuffer();
			ST7565_Update();
			close_men_time = HAL_GetTick();
			time_date = false;
			break;
		}
	}
	ST7565_Update();
}
void about(void) {
	static uint_fast8_t page = 1;
	if (page > 16)
		page = 16;
	else if (page < 1)
		page = 1;
	ST7565_Print(0, 54, "Выход", &Font_7x9, 1, PIX_ON);
	ST7565_Print(36, 54, "<", &Font_7x9, 1, PIX_ON);
	switch (page) {
	case 1:
		ST7565_Print(0, 11, "STMeteo версия 1.0  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "От Morshu8800       ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "(c) 2024-2025       ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "Автор не несёт      ", &Font_6x8, 1, PIX_ON);
		break;
	case 2:
		ST7565_Print(0, 11, "От Morshu8800       ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "(c) 2024-2025       ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "Автор не несёт      ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "ответственности за  ", &Font_6x8, 1, PIX_ON);
		break;
	case 3:
		ST7565_Print(0, 11, "(c) 2024-2025       ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "Автор не несёт      ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "ответственности за  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "любые упущения или  ", &Font_6x8, 1, PIX_ON);
		break;
	case 4:
		ST7565_Print(0, 11, "Автор не несёт      ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "ответственности за  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "любые упущения или  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "недочёты в прошивке ", &Font_6x8, 1, PIX_ON);
		break;
	case 5:
		ST7565_Print(0, 11, "ответственности за  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "любые упущения или  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "недочёты в прошивке ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "данного устройства  ", &Font_6x8, 1, PIX_ON);
		break;
	case 6:
		ST7565_Print(0, 11, "любые упущения или  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "недочёты в прошивке ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "данного устройства! ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "                    ", &Font_6x8, 1, PIX_ON);
		break;
	case 7:
		ST7565_Print(0, 11, "недочёты в прошивке ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "данного устройства! ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "                    ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "Авторы исп.библиотек", &Font_6x8, 1, PIX_ON);
		break;
	case 8:
		ST7565_Print(0, 11, "данного устройства! ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "                    ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "Авторы исп.библиотек", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "                    ", &Font_6x8, 1, PIX_ON);
		break;
	case 9:
		ST7565_Print(0, 11, "                    ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "Авторы исп.библиотек", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "                    ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "Приходько Илья      ", &Font_6x8, 1, PIX_ON);
		break;
	case 10:
		ST7565_Print(0, 11, "Авторы исп.библиотек", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "                    ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "Приходько Илья      ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "STMicroelectronics  ", &Font_6x8, 1, PIX_ON);
		break;
	case 11:
		ST7565_Print(0, 11, "                    ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "Приходько Илья      ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "STMicroelectronics  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "Константин Голинский", &Font_6x8, 1, PIX_ON);
		break;
	case 12:
		ST7565_Print(0, 11, "Приходько Илья      ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "STMicroelectronics  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "Константин Голинский", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "Олег Волков         ", &Font_6x8, 1, PIX_ON);
		break;
	case 13:
		ST7565_Print(0, 11, "STMicroelectronics  ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "Константин Голинский", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "Олег Волков         ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "Ciastkolog          ", &Font_6x8, 1, PIX_ON);
		break;
	case 14:
		ST7565_Print(0, 11, "Константин Голинский", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "Олег Волков         ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "Ciastkolog          ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "Pan Eepj            ", &Font_6x8, 1, PIX_ON);
		break;
	case 15:
		ST7565_Print(0, 11, "Олег Волков         ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 21, "Ciastkolog          ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 31, "Pan Eepj            ", &Font_6x8, 1, PIX_ON);
		ST7565_Print(0, 41, "Henri Heimann       ", &Font_6x8, 1, PIX_ON);
		break;
	case 16:
		ST7565_DrawBitmap(0, 11, QR_code_128x42, 128, 42, PIX_ON);
		break;
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
		buttontime = HAL_GetTick();
		ST7565_ClearBuffer();
		ST7565_Clear();
		ST7565_Update();
		menusel = 0;
		page = 1;
		close_men_time = HAL_GetTick();
		about_prog = false;
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0) {
		buttontime = HAL_GetTick();
		page++;
		if (page > 15) {
			ST7565_Clear();
		}
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0) {
		buttontime = HAL_GetTick();
		if (page == 16) {
			ST7565_Clear();
		}
		page--;
		ST7565_Update();
	}
	ST7565_Update();
}

//top status bar in history!!))
void status_bar(void) {
	static char ST_HR[5], ST_MI[5];
	if (!mainmenu) {
		sprintf(ST_HR, "%ld", al_hr_trg);
		sprintf(ST_MI, "%ld", al_mi_trg);
		if (al_hr_trg < 10 && al_mi_trg < 10) {
			ST7565_Print(94, 0, "0", &Font_6x8, 1, PIX_ON);
			ST7565_Print(100, 0, ST_HR, &Font_6x8, 1, PIX_ON);
			ST7565_Print(107, 0, ":", &Font_6x8, 1, PIX_ON);
			ST7565_Print(114, 0, "0", &Font_6x8, 1, PIX_ON);
			ST7565_Print(120, 0, ST_MI, &Font_6x8, 1, PIX_ON);
		} else if (al_hr_trg < 10 && al_mi_trg > 9) {
			ST7565_Print(94, 0, "0", &Font_6x8, 1, PIX_ON);
			ST7565_Print(100, 0, ST_HR, &Font_6x8, 1, PIX_ON);
			ST7565_Print(107, 0, ":", &Font_6x8, 1, PIX_ON);
			ST7565_Print(114, 0, ST_MI, &Font_6x8, 1, PIX_ON);
		} else if (al_hr_trg > 9 && al_mi_trg < 10) {
			ST7565_Print(94, 0, ST_HR, &Font_6x8, 1, PIX_ON);
			ST7565_Print(107, 0, ":", &Font_6x8, 1, PIX_ON);
			ST7565_Print(114, 0, "0", &Font_6x8, 1, PIX_ON);
			ST7565_Print(120, 0, ST_MI, &Font_6x8, 1, PIX_ON);
		} else if (al_hr_trg > 9 && al_mi_trg > 9) {
			ST7565_Print(94, 0, ST_HR, &Font_6x8, 1, PIX_ON);
			ST7565_Print(107, 0, ":", &Font_6x8, 1, PIX_ON);
			ST7565_Print(114, 0, ST_MI, &Font_6x8, 1, PIX_ON);
		}
	}
	if (alarm1_set) {
		ST7565_DrawBitmap(13, 0, alarmbell1_12x8, 12, 8, PIX_ON);
	} else {
		ST7565_DrawRectangleFilled(12, 0, 13, 8, PIX_OFF);
	}
	if (alarm2_set) {
		ST7565_DrawBitmap(29, 0, alarmbell2_12x8, 12, 8, PIX_ON);
	} else {
		ST7565_DrawRectangleFilled(28, 0, 13, 8, PIX_OFF);
	}
	ST7565_DrawLine(0, 9, 127, 9, PIX_ON);
	battery_meashure();
}
void battery_meashure(void) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	bat = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	vol = (bat * volref) / 4095;
	ST7565_DrawLine(0, 9, 127, 9, PIX_ON);
	if (vol <= 2.10F) {
		ST7565_DrawBitmap(2, 0, batverylow_6x8, 6, 8, PIX_ON);
		ST7565_DrawRectangleFilled(3, 2, 3, 4, PIX_OFF);
		//PCD8544_DrawBitmap(78, 0, low_bat, 6, 8, PIX_ON);
	} else if (vol > 2.10F && vol <= 2.31F) {
		ST7565_DrawBitmap(2, 0, batlow_6x8, 6, 8, PIX_ON);
		ST7565_DrawRectangleFilled(3, 2, 3, 3, PIX_OFF);
		//PCD8544_DrawBitmap(78, 0, bat_25, 6, 8, PIX_ON);
	} else if (vol > 2.31F && vol <= 2.81F) {
		ST7565_DrawBitmap(2, 0, batav_6x8, 6, 8, PIX_ON);
		ST7565_DrawRectangleFilled(3, 2, 3, 2, PIX_OFF);
		//PCD8544_DrawBitmap(78, 0, center_bat, 6, 8, PIX_ON);
	} else if (vol > 2.81F && vol <= 3.21F) {
		ST7565_DrawBitmap(2, 0, batamost_6x8, 6, 8, PIX_ON);
		ST7565_DrawRectangleFilled(3, 2, 3, 1, PIX_OFF);
		//PCD8544_DrawBitmap(78, 0, bat_75, 6, 8, PIX_ON);
	} else if (vol > 3.21F) {
		ST7565_DrawBitmap(2, 0, fullbat_6x8, 6, 8, PIX_ON);
		//PCD8544_DrawBitmap(78, 0, full_bat, 6, 8, PIX_ON);
	}
}

//other settings
void other(void) {
	static int_fast8_t othersel = 0, ressel = 0;
	static bool reset_check = false, clear_think = false, reset_change = false;
	if (otherload) {
		if (!AT24Cxx_read_data(BRIGHTNESS_ADDR, (uint8_t*) &bright,
				sizeof(bright))) {
			Error_Handler();
		}
		AT24Cxx_read_data(INVERSION_ADDR, (uint8_t*) &invert_disp,
				sizeof(invert_disp));
		otherload = false;
	}
	ST7565_Print(8, 12, "Яркость:", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 22, "Тема:", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 33, "Сброс настроек,ресет", &Font_6x8, 1, PIX_ON);
	ST7565_Print(8, 43, "Сохр.изменения", &Font_7x9, 1, PIX_ON);
	ST7565_Print(8, 54, "Выход", &Font_7x9, 1, PIX_ON);
	if (othersel > 4)
		othersel = 0;
	else if (othersel < 0)
		othersel = 4;
	if (bright >= 20)
		bright = 20;
	else if (bright < 0)
		bright = 0;
	switch (bright) {
	case 0:
		ST7565_Print(65, 12, "0%  ", &Font_7x9, 1, PIX_ON);
		break;
	case 1:
		ST7565_Print(65, 12, "5%  ", &Font_7x9, 1, PIX_ON);
		break;
	case 2:
		ST7565_Print(65, 12, "10% ", &Font_7x9, 1, PIX_ON);
		break;
	case 3:
		ST7565_Print(65, 12, "15% ", &Font_7x9, 1, PIX_ON);
		break;
	case 4:
		ST7565_Print(65, 12, "20% ", &Font_7x9, 1, PIX_ON);
		break;
	case 5:
		ST7565_Print(65, 12, "25% ", &Font_7x9, 1, PIX_ON);
		break;
	case 6:
		ST7565_Print(65, 12, "30% ", &Font_7x9, 1, PIX_ON);
		break;
	case 7:
		ST7565_Print(65, 12, "35% ", &Font_7x9, 1, PIX_ON);
		break;
	case 8:
		ST7565_Print(65, 12, "40% ", &Font_7x9, 1, PIX_ON);
		break;
	case 9:
		ST7565_Print(65, 12, "45% ", &Font_7x9, 1, PIX_ON);
		break;
	case 10:
		ST7565_Print(65, 12, "50% ", &Font_7x9, 1, PIX_ON);
		break;
	case 11:
		ST7565_Print(65, 12, "55% ", &Font_7x9, 1, PIX_ON);
		break;
	case 12:
		ST7565_Print(65, 12, "60% ", &Font_7x9, 1, PIX_ON);
		break;
	case 13:
		ST7565_Print(65, 12, "65% ", &Font_7x9, 1, PIX_ON);
		break;
	case 14:
		ST7565_Print(65, 12, "70% ", &Font_7x9, 1, PIX_ON);
		break;
	case 15:
		ST7565_Print(65, 12, "75% ", &Font_7x9, 1, PIX_ON);
		break;
	case 16:
		ST7565_Print(65, 12, "80% ", &Font_7x9, 1, PIX_ON);
		break;
	case 17:
		ST7565_Print(65, 12, "85% ", &Font_7x9, 1, PIX_ON);
		break;
	case 18:
		ST7565_Print(65, 12, "90% ", &Font_7x9, 1, PIX_ON);
		break;
	case 19:
		ST7565_Print(65, 12, "95% ", &Font_7x9, 1, PIX_ON);
		break;
	case 20:
		ST7565_Print(65, 12, "100%", &Font_7x9, 1, PIX_ON);
		break;
	}
	switch (othersel) {
	case 0:
		if (!brightset) {
			ST7565_Print(0, 12, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 22, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 33, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(94, 12, " ", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(0, 12, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 22, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 33, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(94, 12, "<", &Font_7x9, 1, PIX_ON);
		}
		break;
	case 1:
		if (!invertset) {
			ST7565_Print(0, 12, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 22, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 33, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(94, 22, " ", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(0, 12, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 22, "?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 33, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(94, 22, "<", &Font_7x9, 1, PIX_ON);
		}
		break;
	case 2:
		ST7565_Print(0, 12, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 22, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 33, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 3:
		ST7565_Print(0, 12, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 22, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 33, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, ">", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, " ", &Font_7x9, 1, PIX_ON);
		break;
	case 4:
		ST7565_Print(0, 12, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 22, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 33, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 43, " ", &Font_7x9, 1, PIX_ON);
		ST7565_Print(0, 54, ">", &Font_7x9, 1, PIX_ON);
		break;
	}
	if (invert_disp) {
		ST7565_Print(44, 22, "Светлая", &Font_7x9, 1, PIX_ON);
	} else {
		ST7565_Print(44, 22, "Тёмная ", &Font_7x9, 1, PIX_ON);
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
		buttontime = HAL_GetTick();
		switch (othersel) {
		case 0:
			brightset = !brightset;
			break;
		case 1:
			invertset = !invertset;
			break;
		case 2:
			ST7565_Clear();
			reset_check = true;
			break;
		case 3:
			ST7565_Clear();
			ST7565_Print(3, 32, "Изменения сохр-ны", &Font_7x9, 1, PIX_ON);
			ST7565_Update();
			if (!AT24Cxx_write_data(BRIGHTNESS_ADDR, (uint8_t*) &bright,
					sizeof(bright))) {
				Error_Handler();
			}
			AT24Cxx_write_data(INVERSION_ADDR, (uint8_t*) &invert_disp,
					sizeof(invert_disp));
			HAL_Delay(2000);
			ST7565_InvertDisplay(invert_disp);
			TIM1->CCR1 = BRIGHTNESS[bright];
			ST7565_Clear();
			break;
		case 4:
			if (!AT24Cxx_read_data(BRIGHTNESS_ADDR, (uint8_t*) &bright,
					sizeof(bright))) {
				Error_Handler();
			}
			AT24Cxx_read_data(INVERSION_ADDR, (uint8_t*) &invert_disp,
					sizeof(invert_disp));
			ST7565_InvertDisplay(invert_disp);
			TIM1->CCR1 = BRIGHTNESS[bright];
			menusel = 0;
			othersel = 0;
			ST7565_Clear();
			ST7565_ClearBuffer();
			ST7565_Update();
			close_men_time = HAL_GetTick();
			other_settings = false;
			break;
		}
	}
	while (reset_check) {
		status_bar();
		alarm_check();
		if (!clear_think) {
			ST7565_Print(8, 11, "Сброс настроек", &Font_7x9, 1, PIX_ON);
		} else {
			ST7565_Print(8, 11, "Сброс?", &Font_7x9, 1, PIX_ON);
			ST7565_Print(51, 11, "Нет", &Font_7x9, 1, PIX_ON);
			ST7565_Print(81, 11, "Да     ", &Font_7x9, 1, PIX_ON);
		}
		ST7565_Print(8, 21, "Перезапуск устр.", &Font_7x9, 1, PIX_ON);
		ST7565_Print(8, 32, "Выход из меню", &Font_7x9, 1, PIX_ON);
		switch (ressel) {
		case 0:
			if (!clear_think) {
				ST7565_Print(0, 11, ">", &Font_7x9, 1, PIX_ON);
				ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
				ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			} else if (clear_think) {
				ST7565_Print(0, 11, "?", &Font_7x9, 1, PIX_ON);
				ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
				ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			}
			break;
		case 1:
			ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, ">", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, " ", &Font_7x9, 1, PIX_ON);
			break;
		case 2:
			ST7565_Print(0, 11, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 21, " ", &Font_7x9, 1, PIX_ON);
			ST7565_Print(0, 32, ">", &Font_7x9, 1, PIX_ON);
			break;
		}
		if (clear_think && reset_change) {
			ST7565_DrawLine(72, 11, 72, 20, PIX_OFF);
			ST7565_DrawLine(80, 11, 80, 20, PIX_OFF);
			ST7565_Print(73, 11, ">", &Font_7x9, 1, PIX_ON);
		} else if (clear_think && !reset_change) {
			ST7565_DrawLine(72, 11, 72, 20, PIX_OFF);
			ST7565_DrawLine(80, 11, 80, 20, PIX_OFF);
			ST7565_Print(73, 11, "<", &Font_7x9, 1, PIX_ON);
		}
		if (ressel > 2)
			ressel = 0;
		else if (ressel < 0)
			ressel = 2;
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0
				&& !clear_think) {
			buttontime = HAL_GetTick();
			switch (ressel) {
			case 0:
				clear_think = true;
				reset_change = false;
				break;
			case 1:
				NVIC_SystemReset();
				break;
			case 2:
				ST7565_Clear();
				othersel = 0;
				reset_check = false;
				break;
			}
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0
				&& !clear_think) {
			buttontime = HAL_GetTick();
			ressel--;
			ST7565_Update();
		} else if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0
				&& !clear_think) {
			buttontime = HAL_GetTick();
			ressel++;
			ST7565_Update();
		} else if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0
				&& clear_think) {
			buttontime = HAL_GetTick();
			reset_change = !reset_change;
			ST7565_Update();
		} else if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && clear_think) {
			buttontime = HAL_GetTick();
			reset_change = !reset_change;
			ST7565_Update();
		}
		if (HAL_GetTick() - buttontime > 200
				&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0 && clear_think) {
			buttontime = HAL_GetTick();
			ST7565_Update();
			if (reset_change) {
				DS3231_SetFullDate(1, 1, 6, 2000);
				DS3231_SetFullTime(0, 0, 0);
				AT24Cxx_write_data(KEY_ADDR, (uint8_t*) 255, sizeof(KEYCHECK));
				NVIC_SystemReset();
			} else if (!reset_change) {
				clear_think = false;
			}
		}
		ST7565_Update();
	}
	if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !invertset
			&& brightset) {
		buttontime = HAL_GetTick();
		TIM1->CCR1 = BRIGHTNESS[bright];
		bright++;
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && !invertset
			&& brightset) {
		buttontime = HAL_GetTick();
		TIM1->CCR1 = BRIGHTNESS[bright];
		bright--;
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && invertset
			&& !brightset) {
		buttontime = HAL_GetTick();
		invert_disp = !invert_disp;
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && invertset
			&& !brightset) {
		buttontime = HAL_GetTick();
		invert_disp = !invert_disp;
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(up_GPIO_Port, up_Pin) == 0 && !invertset
			&& !brightset) {
		buttontime = HAL_GetTick();
		othersel--;
		ST7565_Update();
	} else if (HAL_GetTick() - buttontime > 200
			&& HAL_GPIO_ReadPin(down_GPIO_Port, down_Pin) == 0 && !invertset
			&& !brightset) {
		buttontime = HAL_GetTick();
		othersel++;
		ST7565_Update();
	}
	ST7565_Update();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	CMSIS_Clock_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
//brightless=brightness*0.5; //- оптимизация, деление меняем на умножение, если это возможно...
	PERPERAL_INIT();
	if (read_alarms) {
		if (!AT24Cxx_read_data(ALARM1HR_ADDR, (uint8_t*) &al1_hr,
				sizeof(al1_hr))) {
			Error_Handler();
		}
		AT24Cxx_read_data(ALARM1MI_ADDR, (uint8_t*) &al1_mi, sizeof(al1_mi));
		AT24Cxx_read_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
				sizeof(alarm1_set));
		AT24Cxx_read_data(ALARM2HR_ADDR, (uint8_t*) &al2_hr, sizeof(al2_hr));
		AT24Cxx_read_data(ALARM2MI_ADDR, (uint8_t*) &al2_mi, sizeof(al2_mi));
		AT24Cxx_read_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
				sizeof(alarm2_set));
		AT24Cxx_read_data(INVERSION_ADDR, (uint8_t*) &invert_disp,
				sizeof(invert_disp));
		AT24Cxx_read_data(BRIGHTNESS_ADDR, (uint8_t*) &bright, sizeof(bright));
		if (bright > 20)
			bright = 20;
		else if (bright < 0)
			bright = 0;
		read_alarms = false;
		ST7565_InvertDisplay(invert_disp);
	}
	HAL_Delay(100);
	if (!alarm1_set && !alarm2_set) {
		HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
	}

//DS3231_SetFullTime(21, 53, 55); //Установка времени ТЕСТ
//DS3231_SetFullDate(10, 1, 5, 2025); //Установка даты ТЕСТ
	ST7565_Clear();
//ST7565_InvertDisplay(1); //- инверсия, если хватит памяти или времени
	ST7565_Update();
	//ST7565_DrawLine(0, 9, 127, 9, PIX_ON);
//	ST7565_DrawBitmap(0, 0, batverylow_6x8, 6, 8, PIX_ON);
//	ST7565_DrawBitmap(7, 0, batlow_6x8, 6, 8, PIX_ON);
//	ST7565_DrawBitmap(14, 0, batav_6x8, 6, 8, PIX_ON);
//	ST7565_DrawBitmap(21, 0, batamost_6x8, 6, 8, PIX_ON);
//ST7565_DrawBitmap(2, 0, fullbat_6x8, 6, 8, PIX_ON);
	setlocktime = settimelockdef;
	subtime = setlocktime - 5000;
	timelock = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		while (mainmenu == true && sleep != true) {
			time_disp();
			change_main_sel();
			alarm_check();
			status_bar();
			if (HAL_GetTick() - timelock >= setlocktime) {
				timelock = HAL_GetTick();
				//HAL_Delay(150);
				sleep = true;
			}
			if (HAL_GetTick() - timelock >= subtime) {
				if (bright > subbright) {
					TIM1->CCR1 = BRIGHTNESS[subbright];
				} else {
					TIM1->CCR1 = BRIGHTNESS[bright];
				}
			} else {
				TIM1->CCR1 = BRIGHTNESS[bright];
			}
			if (HAL_GetTick() - buttontime > 150
					&& HAL_GPIO_ReadPin(ok_GPIO_Port, ok_Pin) == 0) {
				timelock = HAL_GetTick();
				buttontime = HAL_GetTick();
				TIM1->CCR1 = BRIGHTNESS[bright];
			}
			if (HAL_GetTick() - buttontime > 150
					&& HAL_GPIO_ReadPin(alarm_disable_GPIO_Port,
					alarm_disable_Pin) == 0) {
				timelock = HAL_GetTick();
				buttontime = HAL_GetTick();
				TIM1->CCR1 = BRIGHTNESS[bright];
			}
		}
		while (mainmenu == false) {
			if (!alarm_set && !time_date && !other_settings && !about_prog) {
				disp_menu();
				change_menu_sel();
				alarm_check();
				status_bar();
				if (HAL_GetTick() - close_men_time > 30000) {
					dynew = 0;
					ST7565_Clear();
					ST7565_ClearBuffer();
					ST7565_Update();
					timelock = HAL_GetTick();
					close_men_time = HAL_GetTick();
					mainmenu = true;

				}
			}
			if (alarm_set && !time_date && !other_settings && !about_prog) {
				alarm_check();
				alarm_settings();
				status_bar();
			} else if (!alarm_set && time_date && !other_settings
					&& !about_prog) {
				alarm_check();
				time_date_set();
				status_bar();
			} else if (!alarm_set && !time_date && other_settings
					&& !about_prog) {
				alarm_check();
				status_bar();
				other();
			} else if (!alarm_set && !time_date && !other_settings
					&& about_prog) {
				alarm_check();
				about();
				status_bar();
			}
		}
		if (sleep) {
			if (alarm1_set && !alarm2_set) {
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
				sTime.Hours = 0;
				sTime.Minutes = 0;
				sTime.Seconds = 0;
				sTime.SubSeconds = 0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
				sDate.Month = RTC_MONTH_JANUARY;
				sDate.Date = 1;
				sDate.Year = 0;

				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sAlarm.AlarmTime.Hours = 0;
				sAlarm.AlarmTime.Minutes = 0;
				sAlarm.AlarmTime.Seconds = 10;
				sAlarm.AlarmTime.SubSeconds = 0;
				sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
				sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY
						| RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
				sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
				sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
				sAlarm.AlarmDateWeekDay = 1;
				sAlarm.Alarm = RTC_ALARM_A;
				if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN)
						!= HAL_OK) {
					Error_Handler();
				}
			} else if (alarm1_set && alarm2_set) {
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
				sTime.Hours = 0;
				sTime.Minutes = 0;
				sTime.Seconds = 0;
				sTime.SubSeconds = 0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
				sDate.Month = RTC_MONTH_JANUARY;
				sDate.Date = 1;
				sDate.Year = 0;

				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sAlarm.AlarmTime.Hours = 0;
				sAlarm.AlarmTime.Minutes = 0;
				sAlarm.AlarmTime.Seconds = 10;
				sAlarm.AlarmTime.SubSeconds = 0;
				sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
				sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY
						| RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
				sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
				sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
				sAlarm.AlarmDateWeekDay = 1;
				sAlarm.Alarm = RTC_ALARM_A;
				if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN)
						!= HAL_OK) {
					Error_Handler();
				}
			} else if (!alarm1_set && alarm2_set) {
				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
				sTime.Hours = 0;
				sTime.Minutes = 0;
				sTime.Seconds = 0;
				sTime.SubSeconds = 0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
				sDate.Month = RTC_MONTH_JANUARY;
				sDate.Date = 1;
				sDate.Year = 0;

				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				sAlarm.AlarmTime.Hours = 0;
				sAlarm.AlarmTime.Minutes = 0;
				sAlarm.AlarmTime.Seconds = 10;
				sAlarm.AlarmTime.SubSeconds = 0;
				sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
				sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY
						| RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
				sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
				sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
				sAlarm.AlarmDateWeekDay = 1;
				sAlarm.Alarm = RTC_ALARM_A;
				if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN)
						!= HAL_OK) {
					Error_Handler();
				}
			}
			//TIM3->CCR2 = BRIGHTNESS[0];
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(LCD_VDD_GPIO_Port, LCD_VDD_Pin, 0);
			HAL_GPIO_WritePin(METEO_VDD_GPIO_Port, METEO_VDD_Pin, 0);
			HAL_GPIO_WritePin(MEMORY_VDD_GPIO_Port, MEMORY_VDD_Pin, 0);
			HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
			HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 0);
			HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, 0);
			HAL_ADC_Stop(&hadc1);
			//			__HAL_RCC_CRC_CLK_DISABLE();
			//			__HAL_RCC_ADC_CLK_DISABLE();
			//			__HAL_RCC_TIM1_CLK_DISABLE();
			//			__HAL_RCC_TIM3_CLK_DISABLE();
			//			__HAL_RCC_I2C1_CLK_DISABLE();
			//			__HAL_RCC_I2C2_CLK_DISABLE();
			//			__HAL_RCC_SPI1_CLK_DISABLE();
			//			__HAL_RCC_GPIOB_CLK_DISABLE();
			//			__HAL_RCC_GPIOD_CLK_DISABLE();
			HAL_SuspendTick();
			PREPARE_TO_SLEEP();
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,
			PWR_SLEEPENTRY_WFI);

			//			SET_BIT(RCC->APBENR1, RCC_APBENR1_PWREN);
			//			SET_BIT(RCC->APBENR2, RCC_APBENR2_ADCEN);
			//			SET_BIT(RCC->APBENR2, RCC_APBENR2_SPI1EN);
			//			SET_BIT(RCC->APBENR1, RCC_APBENR1_I2C1EN);
			HAL_ResumeTick();
			MX_GPIO_Init();
			//SystemClock_Config();
			CMSIS_Clock_Config();
			MX_ADC1_Init();
			MX_I2C1_Init();
			MX_I2C2_Init();
			HAL_ADC_Init(&hadc1);
			//			__HAL_RCC_GPIOB_CLK_ENABLE();
			//			__HAL_RCC_GPIOD_CLK_ENABLE();
			//			__HAL_RCC_CRC_CLK_ENABLE();
			//			__HAL_RCC_ADC_CLK_ENABLE();
			//			__HAL_RCC_I2C1_CLK_ENABLE();
			//			__HAL_RCC_I2C2_CLK_ENABLE();
			//			__HAL_RCC_SPI1_CLK_ENABLE();
			//			__HAL_RCC_TIM1_CLK_ENABLE();
			//			__HAL_RCC_TIM3_CLK_ENABLE();
			read_alarms = true;
			if (ALARM_TRG) {
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				TIM3->CCR1 = 0;
				HAL_GPIO_WritePin(MEMORY_VDD_GPIO_Port, MEMORY_VDD_Pin, 1);
				HAL_GPIO_WritePin(METEO_VDD_GPIO_Port, METEO_VDD_Pin, 1);
				HAL_Delay(100);
				DS3231_Init(&hi2c1);
				HAL_Delay(200);
				if (read_alarms) {
					if (!AT24Cxx_read_data(ALARM2HR_ADDR, (uint8_t*) &al2_hr,
							sizeof(al2_hr))) {
						Error_Handler();
					}
					AT24Cxx_read_data(ALARM2MI_ADDR, (uint8_t*) &al2_mi,
							sizeof(al2_mi));
					AT24Cxx_read_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
							sizeof(alarm2_set));
					if (!AT24Cxx_read_data(ALARM1HR_ADDR, (uint8_t*) &al1_hr,
							sizeof(al1_hr))) {
						Error_Handler();
					}
					AT24Cxx_read_data(ALARM1MI_ADDR, (uint8_t*) &al1_mi,
							sizeof(al1_mi));
					AT24Cxx_read_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
							sizeof(alarm1_set));
					AT24Cxx_read_data(BRIGHTNESS_ADDR, (uint8_t*) &bright,
							sizeof(bright));
					AT24Cxx_read_data(INVERSION_ADDR, (uint8_t*) &invert_disp,
							sizeof(invert_disp));
					if (bright > 20)
						bright = 20;
					else if (bright < 0)
						bright = 0;
					ST7565_InvertDisplay(invert_disp);
					read_alarms = false;
				}
				for (uint32_t i = 0; i <= 5; i++) {
					al_hr_trg = DS3231_GetHour();
					al_mi_trg = DS3231_GetMinute();
					HAL_Delay(100);
				}
				if (al_hr_trg
						== al2_hr&& al_mi_trg == al2_mi && alarm2_set == true) {
					PERPERAL_SLEEP_INIT();
					HAL_Delay(150);
					timelock = HAL_GetTick();
					sleep = false;
					//HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);
					//isAlarm2 = true;
					dynew = 0;
					//ALARM_TRG2 = false;
				} else if (al_hr_trg
						== al1_hr&& al_mi_trg == al1_mi && alarm1_set == true) {
					PERPERAL_SLEEP_INIT();
					HAL_Delay(150);
					timelock = HAL_GetTick();
					sleep = false;
					dynew = 0;
				} else {
					ALARM_TRG = false;
					HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
					sTime.Hours = 0;
					sTime.Minutes = 0;
					sTime.Seconds = 0;
					sTime.SubSeconds = 0;
					sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
					sTime.StoreOperation = RTC_STOREOPERATION_RESET;
					if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN)
							!= HAL_OK) {
						Error_Handler();
					}
					sDate.WeekDay = RTC_WEEKDAY_MONDAY;
					sDate.Month = RTC_MONTH_JANUARY;
					sDate.Date = 1;
					sDate.Year = 0;

					if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN)
							!= HAL_OK) {
						Error_Handler();
					}
					sAlarm.AlarmTime.Hours = 0;
					sAlarm.AlarmTime.Minutes = 0;
					sAlarm.AlarmTime.Seconds = 10;
					sAlarm.AlarmTime.SubSeconds = 0;
					sAlarm.AlarmTime.DayLightSaving =
					RTC_DAYLIGHTSAVING_NONE;
					sAlarm.AlarmTime.StoreOperation =
					RTC_STOREOPERATION_RESET;
					sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY
							| RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
					sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
					sAlarm.AlarmDateWeekDaySel =
					RTC_ALARMDATEWEEKDAYSEL_DATE;
					sAlarm.AlarmDateWeekDay = 1;
					sAlarm.Alarm = RTC_ALARM_A;
					if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN)
							!= HAL_OK) {
						Error_Handler();
					}
				}
			} else {
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				TIM3->CCR1 = 0;
				PERPERAL_SLEEP_INIT();
				HAL_Delay(150);
				if (!AT24Cxx_read_data(BRIGHTNESS_ADDR, (uint8_t*) &bright,
						sizeof(bright))) {
					Error_Handler();
				}
				if (!AT24Cxx_read_data(ALARM2HR_ADDR, (uint8_t*) &al2_hr,
						sizeof(al2_hr))) {
					Error_Handler();
				}
				AT24Cxx_read_data(ALARM2MI_ADDR, (uint8_t*) &al2_mi,
						sizeof(al2_mi));
				AT24Cxx_read_data(ALARM2FLAG_ADDR, (uint8_t*) &alarm2_set,
						sizeof(alarm2_set));
				if (!AT24Cxx_read_data(ALARM1HR_ADDR, (uint8_t*) &al1_hr,
						sizeof(al1_hr))) {
					Error_Handler();
				}
				AT24Cxx_read_data(ALARM1MI_ADDR, (uint8_t*) &al1_mi,
						sizeof(al1_mi));
				AT24Cxx_read_data(ALARM1FLAG_ADDR, (uint8_t*) &alarm1_set,
						sizeof(alarm1_set));
				AT24Cxx_read_data(INVERSION_ADDR, (uint8_t*) &invert_disp,
						sizeof(invert_disp));
				if (bright > 20)
					bright = 20;
				else if (bright < 0)
					bright = 0;
				ST7565_InvertDisplay(invert_disp);
				timelock = HAL_GetTick();
				sleep = false;
				dynew = 0;
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	ALARM_TRG = true;
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
		HAL_Delay(1000);
		NVIC_SystemReset();
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
