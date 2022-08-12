/*
 * lp5024_drv.h
 *
 *  Created on: 2022. 8. 9.
 *      Author: CSI
 */

#ifndef DEVICE_LP5024_LP5024_DRV_H_
#define DEVICE_LP5024_LP5024_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "Define_Global.h"


/* Private includes ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef	struct	__handle_i2c_inforamtion__
{
	I2C_HandleTypeDef*		p_hi2c;			//  I2C_HandleTypeDef의 포인터
	uint8_t					id;				//	디바이스 아이디

	uint8_t					f_probe_dev;	//	디바이스 발견 유무
	uint8_t					f_error;		//	에러 유무
}	HI2CINFO, * HI2CINFO_PTR;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void	LP5024_Initial(HI2CINFO_PTR _p_i2c_info);

void	LP5024_Output(HI2CINFO_PTR _p_i2c_info, uint8_t _rates);

uint8_t	LP5024_Read(HI2CINFO_PTR _p_i2c_info, uint8_t _addr);
void	LP5024_Write(HI2CINFO_PTR _p_i2c_info, uint8_t _addr, uint8_t _data);
/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif


#endif /* DEVICE_LP5024_LP5024_DRV_H_ */
