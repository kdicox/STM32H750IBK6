/*
 * sub_main.c
 *
 *  Created on: Aug 8, 2022
 *      Author: CSI
 */


/* Includes ------------------------------------------------------------------*/
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	<stdarg.h>

#include	"sub_main.h"

#include	"lp5024_drv.h"
//#include	"config_data.h"
//#include	"util.h"


/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const	char*				g_build_date	= __DATE__;		//	build date
const	char*				g_build_time	= __TIME__;		//	build time

const	uint16_t			g_fw_ver		= 0x0100;		//	ver 01.00


HI2CINFO					g_hi2c_info_1 = { NULL, 0, 0, 0 };
HI2CINFO					g_hi2c_info_2 = { NULL, 0, 0, 0 };
HI2CINFO					g_hi2c_info_3 = { NULL, 0, 0, 0 };
HI2CINFO					g_hi2c_info_4 = { NULL, 0, 0, 0 };

SPI_HandleTypeDef *			g_p_hspi1 = NULL;
SPI_HandleTypeDef *			g_p_hspi2 = NULL;
SPI_HandleTypeDef *			g_p_hspi3 = NULL;
SPI_HandleTypeDef *			g_p_hspi4 = NULL;

UART_HandleTypeDef *		g_p_huart2 = NULL;


/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Exported user code --------------------------------------------------------*/
/**
  * @brief  서브 메인 초기화
  * @retval 없음
  */
void	SUBMain_Initial( void )
{
	g_p_cons_ctrl = g_p_huart2;
}


/**
  * @brief  서브 메인 모듈 초기화
  * @retval 없음
  */
void	SUBMain_InitModule( void )
{
	CONSMSG_Initial();

	PUTS( "\r\n\r\n\r\n\r\n" );
	PUTS( "==================================================\r\n" );
	PUTS( "=======     Motor Control Micom Board      =======\r\n" );
	PUTS( "==================================================\r\n" );
	PUTS( "\r\n" );
	PRINTF(
		   "FIRMWARE VERSION v%02X.%02X [%s %s]\r\n\r\n",
		   (g_fw_ver >> 8) & 0x00FF,
		   g_fw_ver & 0x00FF,
		   g_build_date,
		   g_build_time
		   );

	//PUTS( "  Configuration Config Data\r\n" );
	//CFGDATA_DeInit();

	PUTS( "  Configuration LP5024\r\n" );
	LP5024_Initial(&g_hi2c_info_1);
	LP5024_Initial(&g_hi2c_info_2);
	LP5024_Initial(&g_hi2c_info_3);
	LP5024_Initial(&g_hi2c_info_4);

	PUTS( "\r\n" );
	PUTS( "  Configuration AD7928\r\n" );
	//AD7928_Initial(g_p_hspi1);


	PUTS( "\r\n" );
	PUTS( "  Configuration COMPLETE\r\n" );
	//HAL_UART_Receive_IT( g_p_cons_uart, &g_cons_recv, 1);
	//HAL_UART_Receive_IT( g_p_ecom_uart, &g_ecom_recv, 1);

	//HAL_ADCEx_Calibration_Start( g_p_poten_adc );
	//HAL_NVIC_EnableIRQ( ADC1_2_IRQn );
	//HAL_ADC_Start_IT( g_p_poten_adc );
	//HAL_ADC_Start_DMA( g_p_poten_adc, &g_dma_data[0], 2 );

	PUTS( "\r\n> " );
}


/**
  * @brief UART 수신 입터렙트 처리
  * @param  _p_uart_info: 구조체 UART_HandleTypeDef의 포인터
  * @retval 없음
  */
void	SUBMain_UARTRx_IT(UART_HandleTypeDef* _p_uart_info)
{
	//	UART 콘솔 포트
	if ( _p_uart_info->Instance == g_p_huart2->Instance ) {
		CONSMSG_Rx_IT(_p_uart_info);
	}
}


/**
  * @brief UART 전송 입터렙트 처리
  * @param  _p_uart_info: 구조체 UART_HandleTypeDef의 포인터
  * @retval 없음
  */
void	SUBMain_UARTTx_IT(UART_HandleTypeDef* _p_uart_info)
{
	//	UART 콘솔 포트
	if ( _p_uart_info->Instance == g_p_huart2->Instance ) {
		CONSMSG_Tx_IT(_p_uart_info);
	}
}


/**
  * @brief  서브 메인 프로세스
  * @retval 없음
  */
void	SUBMain_Process( void )
{
	CONSMSG_Process();
}

