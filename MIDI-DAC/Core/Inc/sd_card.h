/*
 * SD_Card.h
 *
 *  Created on: Nov 24, 2025
 *      Author: nick
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_

#include "main.h"
#include "ff.h"

void play_sound_byte(const TCHAR* file_name, DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim);

void read_sound_file(const TCHAR* file_name, uint8_t *pData);





#endif /* INC_SD_CARD_H_ */
