/*
 * sub_main.h
 *
 *  Created on: Apr 28, 2022
 *      Author: CSI
 */

#ifndef APPLICATION_USER_MODULE_SUB_MAIN_H_
#define APPLICATION_USER_MODULE_SUB_MAIN_H_


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
extern	HI2CINFO					g_hi2c_info_1;
extern	HI2CINFO					g_hi2c_info_2;
extern	HI2CINFO					g_hi2c_info_3;
extern	HI2CINFO					g_hi2c_info_4;


extern	SPI_HandleTypeDef *			g_p_hspi1;
extern	SPI_HandleTypeDef *			g_p_hspi2;
extern	SPI_HandleTypeDef *			g_p_hspi3;
extern	SPI_HandleTypeDef *			g_p_hspi4;

extern	UART_HandleTypeDef *		g_p_huart2;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void	SUBMain_Initial( void );

void	SUBMain_InitModule( void );

void	SUBMain_UARTRx_IT(UART_HandleTypeDef* _p_uart_info);
void	SUBMain_UARTTx_IT(UART_HandleTypeDef* _p_uart_info);

void	SUBMain_Process( void );
/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif


#endif /* APPLICATION_USER_MODULE_SUB_MAIN_H_ */

