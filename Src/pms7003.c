/*
 * pms7003.c
 *
 *  Created on: Sep 17, 2017
 *      Author: raek
 */

#include "pms7003.h"

static void build_command_sequence(uint8_t cmd, uint8_t datal, struct pms7003_command_sequence *comseq);
static void read_reading(const struct pms7003_readings_raw *raw, struct pms7003_readings *human);
static PMS7003_Return_Code PMS7003_Passive_Request(UART_HandleTypeDef *huart);

static void build_command_sequence(uint8_t cmd, uint8_t datal, struct pms7003_command_sequence *comseq) {
	uint16_t verify_bits;

	comseq->start_char_1 = START_CHAR_1;
	comseq->start_char_2 = START_CHAR_2;
	comseq->command = cmd;
	comseq->datah = 0;
	comseq->datal = datal;

	verify_bits = START_CHAR_1 + START_CHAR_2 + cmd + 0 + datal;

	comseq->cksumh = (uint8_t) (verify_bits >> 8);
	comseq->cksuml = (uint8_t) (verify_bits);
}

static void read_reading(const struct pms7003_readings_raw *raw,
		struct pms7003_readings *human) {
	human->pm1_0_cf = raw->data01h << 8 | raw->data01l;
	human->pm2_5_cf = raw->data02h << 8 | raw->data02l;
	human->pm10_0_cf = raw->data03h << 8 | raw->data03l;
	human->pm1_0_atmo = raw->data04h << 8 | raw->data04l;
	human->pm2_5_atmo = raw->data05h << 8 | raw->data05l;
	human->pm10_0_atmo = raw->data06h << 8 | raw->data06l;
	human->pm_0_3_raw = raw->data07h << 8 | raw->data07l;
	human->pm_0_5_raw = raw->data08h << 8 | raw->data08l;
	human->pm_1_0_raw = raw->data09h << 8 | raw->data09l;
	human->pm_2_5_raw = raw->data10h << 8 | raw->data10l;
	human->pm_5_0_raw = raw->data11h << 8 | raw->data11l;
	human->pm_10_0_raw = raw->data12h << 8 | raw->data12l;
}

PMS7003_Return_Code PMS7003_Read(UART_HandleTypeDef *huart,
		struct pms7003_readings *readings, PMS7003_Operation_Modes op_mode) {

	struct pms7003_readings_raw raw_readings;
	uint8_t *p = (uint8_t*) &raw_readings;
	uint16_t cksum;
	uint8_t retry = 5;
	PMS7003_Return_Code ret;

	while (1) {
		__HAL_UART_FLUSH_DRREGISTER(huart);
		if (op_mode == OP_MODE_PASSIVE) {
			if ( (ret = PMS7003_Passive_Request(huart)) != PMS7003_OK) {
				return ret;
			}
			if (retry-- == 0) {
				return PMS7003_PASSIVE_RETRY_LIMIT_ERROR;
			}
		}

		if (HAL_UART_Receive(huart, (uint8_t*) &raw_readings.start1, 1, 5000) != HAL_OK)
			return PMS7003_UART_ERROR;

		if (raw_readings.start1 == START_CHAR_1) {
			if (HAL_UART_Receive(huart, (uint8_t*) &raw_readings.start2, 1, 5000) != HAL_OK)
					return PMS7003_UART_ERROR;

			if (raw_readings.start2 != START_CHAR_2)
				continue;

			if (HAL_UART_Receive(huart, (uint8_t*) &raw_readings.flh, 30, 5000) != HAL_OK)
				return PMS7003_UART_ERROR;

			cksum = 0;
			for (int i = 0; i < 30; i++) {
				cksum += p[i];
			}

			if (cksum != (raw_readings.ckh << 8 | raw_readings.ckl))
				continue;
			else
				break;
		} else {
			continue;
		}
	}

	read_reading(&raw_readings, readings);

	return PMS7003_OK;
}

PMS7003_Return_Code PMS7003_Set_Operation_Mode(UART_HandleTypeDef *huart, PMS7003_Operation_Modes op_mode) {
	struct pms7003_command_sequence cmdseq;

	build_command_sequence(CMD_CHANGE_MODE, op_mode, &cmdseq);

	if (HAL_UART_Transmit(huart, (uint8_t *)&cmdseq, 7, 2000) != HAL_OK)
		return PMS7003_UART_ERROR;

	return PMS7003_OK;
}

PMS7003_Return_Code PMS7003_Set_Sleep(UART_HandleTypeDef *huart, PMS7003_Sleep_Modes sleep_mode) {
	struct pms7003_command_sequence cmdseq;

	build_command_sequence(CMD_SLEEP_MODE, sleep_mode, &cmdseq);

	if (HAL_UART_Transmit(huart, (uint8_t *)&cmdseq, 7, 2000) != HAL_OK)
		return PMS7003_UART_ERROR;

	return PMS7003_OK;
}

static PMS7003_Return_Code PMS7003_Passive_Request(UART_HandleTypeDef *huart) {
	struct pms7003_command_sequence cmdseq;

	build_command_sequence(CMD_READ_PASSIVE, 0x0, &cmdseq);

	if (HAL_UART_Transmit(huart, (uint8_t *)&cmdseq, 7, 2000) != HAL_OK)
		return PMS7003_UART_ERROR;

	return PMS7003_OK;
}
