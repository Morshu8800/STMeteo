/*
 *  AT24Cxx.h
 *
 *  Библиотека для работы с микросхемой памяти EEPROM типа AT24Cxx
 *  Created on: Oct 29, 2021
 *  Обновлено 09.08.2022(Решена проблема с ошибкой в вычислении CRC)
 *  Authors: Oleg Volkov & Konstantin Golinskiy
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md
 *  GitHub Константина: https://github.com/GolinskiyKonstantin
 *  Группа ВК: https://vk.com/solderingiron.stm32
 *  e-mail: golinskiy.konstantin@gmail.com
 */

/*
 Описание:
 Микросхема памяти EEPROM типа AT24Cxx имеет "хх" килобит данных ( т.е. хх/8 килобайта )
 Модели бывают следущие:

 (частота шины I2C зависит от напряжения питания 100 kHz (1.8V, 2.5V, 2.7V) and 400 kHz (5V)---------------------------
 AT24C01   128 	 слов по 8 бит 	(1 килобит)		16 страниц, каждая емкостью по 8 байт		размер адреса памяти 7-bit
 AT24C02   256 	 слов по 8 бит 	(2 килобит)		32 страниц, каждая емкостью по 8 байт		размер адреса памяти 8-bit
 AT24C04   512 	 слов по 8 бит 	(4 килобит)		32 страниц, каждая емкостью по 16 байт		размер адреса памяти 9-bit
 AT24C08   1024  слов по 8 бит 	(8 килобит)		64 страниц, каждая емкостью по 16 байт		размер адреса памяти 10-bit
 AT24C16   2048  слов по 8 бит 	(16 килобит)	128 страниц, каждая емкостью по 16 байт		размер адреса памяти 11-bit

 (частота шины I2C зависит от напряжения питания 100 kHz (1.8V, 2.5V, 2.7V) and 400 kHz (5V)---------------------------
 AT24C32   4096	 слов по 8 бит 	(32 килобит)	128 страниц, каждая емкостью по 32 байта	размер адреса памяти 12-bit
 AT24C64   8192	 слов по 8 бит	(64 килобит)	256 страниц, каждая емкостью по 32 байта	размер адреса памяти 13-bit

 (частота шины I2C зависит от напряжения питания 1 MHz (5V), 400 kHz (2.7V, 2.5V) and 100 kHz (1.8V )------------------
 AT24C128  16,384 слов по 8 бит	(128 килобит)	256 страниц, каждая емкостью по 64 байта	размер адреса памяти 14-bit
 AT24C256  32,768 слов по 8 бит	(256 килобит)	512 страниц, каждая емкостью по 64 байта	размер адреса памяти 15-bit

 (частота шины I2C зависит от напряжения питания 1 MHz (5V), 400 kHz (2.7V, 2.5V) and 100 kHz (1.8V )-----------------
 AT24C512  65,536 слов по 8 бит	(512 килобит)	512 страниц, каждая емкостью по 128 байта	размер адреса памяти 16-bit

 ВАЖНО!
 За 1 раз НЕЛЬЗЯ записать больше чем размер 1 страницы.
 Если записать больше, чем размер 1 страницы - то страница запишется до конца и все. На следующую страницу запись не произведется.

 Читать же можно хоть всю память целиком.
 Для указания адреса в памяти отправляем 1 или 2 байта, в зависимости от размера адреса памяти.

 В стертом состоянии, вся память EEPROM заполнена единицами (все ячейки содержат значение слова данных 0xFF).

 Vcc = от 1.7V до 5.5V

 Пин WP ( write protect ) - служит для блокировки записи.(Читать можно в любом случае).
 Когда вход защиты от записи подключен к Vcc, верхний квадрант памяти (1800-1FFFH)
 защищен от операций записи. Для нормальной работы записи пин WP должен быть заземлен.
 */

#ifndef _AT24CXX_H
#define _AT24CXX_H

#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include "crc.h"
#include <stdbool.h>
#include <string.h>

/*===============================================НАСТРОЙКИ=============================================*/

/*---------------------------------------------Выбор шины i2c------------------------------------------*/
#define AT24CXX_I2C 			hi2c2
extern I2C_HandleTypeDef AT24CXX_I2C;
/*---------------------------------------------Выбор шины i2c------------------------------------------*/

/*---------------------------------------------Выбор шины CRC------------------------------------------*/
#define AT24CXX_CRC             hcrc
extern CRC_HandleTypeDef AT24CXX_CRC;
/*---------------------------------------------Выбор шины CRC------------------------------------------*/

/*----------------------------------------Адрес EEPROM на шине i2c-------------------------------------*/
//Пины A0, A1, и A2 задают адрес микросхемы на шине i2c
//7-bit device address code - 1010  (A2)  (A1)  (A0). (A0, A1, A2 - это последние три бита)
//если все три пина притянуть к земле, то получим 1010000 адрес (0x50 << 1) = 0xA0)
#define AT24CXX_I2C_ADDR		(0x50 << 1)    //A0 A1 A2 припаяны к земле
/*----------------------------------------Адрес EEPROM на шине i2c-------------------------------------*/

/*-----------------------Указываем тип памяти(нужное раcкомментировать)--------------------------------*/
//#define AT24C01
//#define AT24C02
//#define AT24C04
//#define AT24C08
//#define AT24C16
#define AT24C32
//#define AT24C64
//#define AT24C128
//#define AT24C256
//#define AT24C512
/*-----------------------Указываем тип памяти(нужное раcкомментировать)--------------------------------*/

/*-----------------------------Нога для блокировки записи EEPROM---------------------------------------*/
//#define WP_PORT GPIOA
//#define WP_Pin	GPIO_PIN_4
/*-----------------------------Нога для блокировки записи EEPROM---------------------------------------*/

/*----------------------------Макросы для разных микросхем памяти--------------------------------------*/
#if defined(AT24C01)
    #define AT24CXX_PAGE_BYTE               8
    #define AT24CXX_MAX_MEM_ADDRESS         128
#elif defined(AT24C02)
    #define AT24CXX_PAGE_BYTE               8
    #define AT24CXX_MAX_MEM_ADDRESS         256
#elif defined(AT24C04)
    #define AT24CXX_PAGE_BYTE               16
    #define AT24CXX_MAX_MEM_ADDRESS         512
#elif defined(AT24C08)
    #define AT24CXX_PAGE_BYTE               16
    #define AT24CXX_MAX_MEM_ADDRESS         1024
#elif defined(AT24C16)
    #define AT24CXX_PAGE_BYTE               16
    #define AT24CXX_MAX_MEM_ADDRESS         2048
#elif defined(AT24C32)
    #define AT24CXX_PAGE_BYTE               32
    #define AT24CXX_MAX_MEM_ADDRESS         4096
#elif defined(AT24C64)
    #define AT24CXX_PAGE_BYTE               32
    #define AT24CXX_MAX_MEM_ADDRESS         8192
#elif defined(AT24C128)
#define AT24CXX_PAGE_BYTE               64
#define AT24CXX_MAX_MEM_ADDRESS         16384
#elif defined(AT24C256)
    #define AT24CXX_PAGE_BYTE               64
    #define AT24CXX_MAX_MEM_ADDRESS         32768
#elif defined(AT24C512)
    #define AT24CXX_PAGE_BYTE               128
    #define AT24CXX_MAX_MEM_ADDRESS         65536
#endif
/*----------------------------Макросы для разных микросхем памяти--------------------------------------*/

bool AT24xx_Connect_test(void);
void AT24Cxx_erase_chip(void);
uint16_t AT24Cxx_write(uint16_t addMem_write, uint8_t *data_write, uint16_t size_write);
uint16_t AT24Cxx_read(uint16_t addMem_read, uint8_t *data_read, uint16_t size_read);
bool AT24Cxx_write_data(uint16_t addMem_write, uint8_t *data, uint8_t len);
bool AT24Cxx_read_data(uint16_t addMem_read, uint8_t *data, uint8_t len);

#endif	/*	_AT24CXX_H */
