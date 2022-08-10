/*
 * ad7928_drv.h
 *
 *  Created on: 2022. 8. 9.
 *      Author: CSI
 */

#ifndef DEVICE_AD7928_AD7928_DRV_H_
#define DEVICE_AD7928_AD7928_DRV_H_

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
void	AD7928_Initial(SPI_HandleTypeDef* _p_hspi);

uint16_t	AD7928_Read(SPI_HandleTypeDef* _p_hspi, uint16_t _addr);
void	AD7928_Write(SPI_HandleTypeDef* _p_hspi, uint16_t _addr, uint16_t _data);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_AD7928_AD7928_DRV_H_ */
