/*
 * pm7003.h
 *
 *  Created on: Sep 17, 2017
 *      Author: raek
 */

#ifndef PMS7003_H_
#define PMS7003_H_

#include <stdint.h>
#include "main.h"
#include "stm32l4xx_hal.h"

#define START_CHAR_1 0x42
#define START_CHAR_2 0x4d

typedef enum
{
	CMD_READ_PASSIVE       = 0xe2,
	CMD_CHANGE_MODE    = 0xe1,
	CMD_SLEEP_MODE     = 0xe4
} PMS7003_Commands;

typedef enum
{
	OP_MODE_PASSIVE = 0x00,
	OP_MODE_ACTIVE = 0x01
} PMS7003_Operation_Modes;

typedef enum {
	MODE_SLEEP = 0x00,
	MODE_WAKEUP = 0x01
} PMS7003_Sleep_Modes;

typedef enum {
	PMS7003_OK = 0,
	PMS7003_PASSIVE_RETRY_LIMIT_ERROR,
	PMS7003_UART_ERROR
} PMS7003_Return_Code;


struct pms7003_command_sequence {
	uint8_t start_char_1;
	uint8_t start_char_2;
	uint8_t command;
	uint8_t datah;
	uint8_t datal;
	uint8_t cksumh;
	uint8_t cksuml;
};

struct pms7003_readings_raw {
  uint8_t start1;
  uint8_t start2;
  uint8_t flh;
  uint8_t fll;
  uint8_t data01h;
  uint8_t data01l;
  uint8_t data02h;
  uint8_t data02l;
  uint8_t data03h;
  uint8_t data03l;
  uint8_t data04h;
  uint8_t data04l;
  uint8_t data05h;
  uint8_t data05l;
  uint8_t data06h;
  uint8_t data06l;
  uint8_t data07h;
  uint8_t data07l;
  uint8_t data08h;
  uint8_t data08l;
  uint8_t data09h;
  uint8_t data09l;
  uint8_t data10h;
  uint8_t data10l;
  uint8_t data11h;
  uint8_t data11l;
  uint8_t data12h;
  uint8_t data12l;
  uint8_t data13h;
  uint8_t data13l;
  uint8_t ckh;
  uint8_t ckl;
};

struct pms7003_readings {
	uint16_t pm1_0_cf;
	uint16_t pm2_5_cf;
	uint16_t pm10_0_cf;
	uint16_t pm1_0_atmo;
	uint16_t pm2_5_atmo;
	uint16_t pm10_0_atmo;
	uint16_t pm_0_3_raw;
	uint16_t pm_0_5_raw;
	uint16_t pm_1_0_raw;
	uint16_t pm_2_5_raw;
	uint16_t pm_5_0_raw;
	uint16_t pm_10_0_raw;
};


PMS7003_Return_Code PMS7003_Read(UART_HandleTypeDef *huart, struct pms7003_readings *readings, PMS7003_Operation_Modes op_mode);

PMS7003_Return_Code PMS7003_Set_Operation_Mode(UART_HandleTypeDef *huart, PMS7003_Operation_Modes op_mode);

PMS7003_Return_Code PMS7003_Set_Sleep(UART_HandleTypeDef *huart, PMS7003_Sleep_Modes sleep_mode);

#endif /* PMS7003_H_ */
