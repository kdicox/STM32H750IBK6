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
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void	LP5024_Initial(I2C_HandleTypeDef* _p_hi2c, uint8_t _id);

void	LP5024_Output(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _rates);

uint8_t	LP5024_Read(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _addr);
void	LP5024_Write(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _addr, uint8_t _data);
/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif


#endif /* DEVICE_LP5024_LP5024_DRV_H_ */
