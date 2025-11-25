/*
 * sd_card.c
 *
 *  Created on: Nov 24, 2025
 *      Author: nick
 */

#include "sd_card.h"

#define CHUNK_SIZE 64000

// Uses DMA to play sound byte from specified DAC
void play_sound_byte(const TCHAR* file_name, DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim) {

	uint8_t readBuf[CHUNK_SIZE] = {0};

	  //some variables for FatFs
	  FATFS FatFs; 	//Fatfs handle
	  FIL fil; 		//File handle
	  FRESULT
; //Result after operations

	  //Open the file system
	  fres = f_mount(&FatFs, "", 1); //1=mount now

	  //Let's get some statistics from the SD card
	  DWORD free_clusters, free_sectors, total_sectors;

	  FATFS* getFreeFs;

	  fres = f_getfree("", &free_clusters, &getFreeFs);

	  //Formula comes from ChaN's documentation
	  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	  free_sectors = free_clusters * getFreeFs->csize;

	  //Now let's try to open file "test.txt"
	  fres = f_open(&fil, file_name, FA_READ);

	  //We can either use f_read OR f_gets to get data out of files
	  //f_gets is a wrapper on f_read that does some string formatting for us
	  UINT bytesRead = 0;
	  FRESULT rres = f_read(&fil, readBuf, CHUNK_SIZE, &bytesRead);

	  //Be a tidy kiwi - don't forget to close your file!
	  f_close(&fil);

	  // fill the entire buffer to start
	  HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*)readBuf, CHUNK_SIZE, DAC_ALIGN_12B_R);
	  HAL_TIM_Base_Start(htim);
}


// reads from the sound file. must be a chunk size of sound or 64000 bytes
void read_sound_file(const TCHAR* file_name, uint8_t *pData) {

		  //some variables for FatFs
		  FATFS FatFs; 	//Fatfs handle
		  FIL fil; 		//File handle
		  FRESULT fres; //Result after operations

		  //Open the file system
		  fres = f_mount(&FatFs, "", 1); //1=mount now

		  //Let's get some statistics from the SD card
		  DWORD free_clusters, free_sectors, total_sectors;

		  FATFS* getFreeFs;

		  fres = f_getfree("", &free_clusters, &getFreeFs);

		  //Formula comes from ChaN's documentation
		  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
		  free_sectors = free_clusters * getFreeFs->csize;

		  //Now let's try to open file "test.txt"
		  fres = f_open(&fil, file_name, FA_READ);

		  //We can either use f_read OR f_gets to get data out of files
		  //f_gets is a wrapper on f_read that does some string formatting for us
		  UINT bytesRead = 0;
		  FRESULT rres = f_read(&fil, pData, CHUNK_SIZE, &bytesRead);

		  //Be a tidy kiwi - don't forget to close your file!
		  f_close(&fil);
}



