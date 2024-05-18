/*
 * font.h
 *
 *  Created on: Dec 22, 2023
 *      Author: phamh
 */

#ifndef FONTS_H_
#define FONTS_H_ 120

#include "stm32f1xx_hal.h"
#include "string.h"

typedef struct {
	uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef_t;

/**
 * @brief  String length and height
 */
typedef struct {
	uint16_t Length;      /*!< String length in units of pixels */
	uint16_t Height;      /*!< String height in units of pixels */
} FONTS_SIZE_t;


/**
 * @brief  7 x 10 pixels font size structure
 */
extern FontDef_t Font_7x10;

/**
 * @brief  11 x 18 pixels font size structure
 */
extern FontDef_t Font_11x18;

/**
 * @brief  16 x 26 pixels font size structure
 */
extern FontDef_t Font_16x26;

char* FONTS_GetStringSize(char* str, FONTS_SIZE_t* SizeStruct, FontDef_t* Font);


#endif /* FONTS_H_ */
