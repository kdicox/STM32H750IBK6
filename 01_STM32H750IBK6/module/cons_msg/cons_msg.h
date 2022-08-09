/*
 * cons_msg.h
 *
 *  Created on: Aug 8, 2022
 *      Author: CSI
 */

#ifndef APPLICATION_USER_MODULE_CONS_MSG_H_
#define APPLICATION_USER_MODULE_CONS_MSG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "Define_Global.h"


/* Private includes ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
extern	UART_HandleTypeDef *		g_p_cons_ctrl;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define	PUTC		CONSMSG_Putc
#define	PUTS		CONSMSG_Puts
#define	PRINTF		CONSMSG_Printf


/* Exported functions prototypes ---------------------------------------------*/
void	CONSMSG_Initial( void );

void	CONSMSG_Rx_IT(UART_HandleTypeDef* _p_uart_info);
void	CONSMSG_Tx_IT(UART_HandleTypeDef* _p_uart_info);

void	CONSMSG_Send(const uint8_t _data);

void	CONSMSG_Putc(const char _ch);
void	CONSMSG_Puts(const char* _msg);

void	CONSMSG_Printf( const char* _msg, ... );

void	CONSMSG_Process( void );
/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif


#endif /* APPLICATION_USER_MODULE_CONS_MSG_H_ */

