/*
 * util.h
 *
 *  Created on: 2022. 8. 12.
 *      Author: CSI
 */

#ifndef MISC_UTIL_H_
#define MISC_UTIL_H_


#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "Define_Global.h"
#include "cons_msg.h"

#include "lp5024_drv.h"
#include "ad7928_drv.h"


/* Private includes ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
uint16_t	UTIL_CalculateCRC16( uint8_t* p_buf, uint16_t len );

void	UTIL_SetCalcCRC16( void );
void	UTIL_CalcCRC16( uint8_t buf );
uint16_t	UTIL_GetCalcCRC16( void );

uint8_t		UTIL_CodeToByte( char* code );
uint16_t	UTIL_CodeToWord( char* code );
uint32_t	UTIL_CodeToDword( char* code );

int8_t		UTIL_CodeToInt8( char* code );
int16_t	    UTIL_CodeToInt16( char* code );
int32_t	    UTIL_CodeToInt32( char* code );

int8_t		UTIL_CodeToReal8( char* code, uint8_t _pt_num );
int16_t	    UTIL_CodeToReal16( char* code, uint8_t _pt_num );
int32_t	    UTIL_CodeToReal32( char* code, uint8_t _pt_num );
/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif


#endif /* MISC_UTIL_H_ */
