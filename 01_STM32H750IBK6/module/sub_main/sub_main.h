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


/* Private includes ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
extern	I2C_HandleTypeDef *			g_p_hi2c1;
extern	I2C_HandleTypeDef *			g_p_hi2c2;
extern	I2C_HandleTypeDef *			g_p_hi2c3;
extern	I2C_HandleTypeDef *			g_p_hi2c4;

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

