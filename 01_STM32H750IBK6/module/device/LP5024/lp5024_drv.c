/*
 * lp5024_drv.c
 *
 *  Created on: 2022. 8. 9.
 *      Author: CSI
 */

/* Includes ------------------------------------------------------------------*/
#include	"lp5024_drv.h"

#include	"cons_msg.h"


/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
uint8_t	i2c_reg_rd(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _addr);
void	i2c_reg_wd(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _addr, uint8_t _data);


/* Private user code ---------------------------------------------------------*/
/**
  * @brief  i2c 레지스터 읽기
  * @param  _p_hi2c: 구조체 I2C_HandleTypeDef의 포인터
  * @param  _id: 디바이스 아이디
  * @param  _addr: 레지스터 주소
  * @retval 레지스터 값
  */
uint8_t	i2c_reg_rd(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _addr)
{
	uint8_t		data = 0;


	if (HAL_OK != HAL_I2C_Master_Transmit(_p_hi2c, (_id << 1), &_addr, 1, 500)) {
	  while (1);
	}

	if (HAL_OK != HAL_I2C_Master_Receive(_p_hi2c, (_id << 1) |0x01, &data, 1, 500)) {
	  while (1);
	}

	return ( data );
}


/**
  * @brief  i2c 레지스터 쓰기
  * @param  _p_hi2c: 구조체 I2C_HandleTypeDef의 포인터
  * @param  _id: 디바이스 아이디
  * @param  _addr: 레지스터 주소
  * @param  _data: 데이터
  * @retval 없음
  */
void	i2c_reg_wd(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _addr, uint8_t _data)
{
	uint8_t		reg_data[2];


	reg_data[0] = _addr;
	reg_data[1] = _data;
	if (HAL_OK != HAL_I2C_Master_Transmit(_p_hi2c, (_id << 1), &reg_data[0], 2, 500)) {
	  while (1);
	}
}


/* Exported user code --------------------------------------------------------*/
/**
  * @brief  LP5024 초기화
  * @param  _p_hi2c: 구조체 I2C_HandleTypeDef의 포인터
  * @param  _id: 디바이스 아이디
  * @retval 없음
  */
void	LP5024_Initial(I2C_HandleTypeDef* _p_hi2c, uint8_t _id)
{
	PRINTF("REG00 %02x\r\n", i2c_reg_rd(_p_hi2c, _id, 0x00));
	PRINTF("REG01 %02x\r\n", i2c_reg_rd(_p_hi2c, _id, 0x01));
}


/**
  * @brief  LP5024 출력
  * @param  _p_hi2c: 구조체 I2C_HandleTypeDef의 포인터
  * @param  _id: 디바이스 아이디
  * @param  _rates: 출력 비율 (0: 출력 없음, 255: 최대 출력)
  * @retval 없음
  */
void	LP5024_Output(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _rates)
{

}


/**
  * @brief  LP5024 레지스터 읽기
  * @param  _p_hi2c: 구조체 I2C_HandleTypeDef의 포인터
  * @param  _id: 디바이스 아이디
  * @param  _addr: 레지스터 주소
  * @retval 레지스터 값
  */
uint8_t	LP5024_Read(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _addr)
{
	return ( 0 );
}


/**
  * @brief  LP5024 레지스터 쓰기
  * @param  _p_hi2c: 구조체 I2C_HandleTypeDef의 포인터
  * @param  _id: 디바이스 아이디
  * @param  _addr: 레지스터 주소
  * @param  _data: 데이터
  * @retval 없음
  */
void	LP5024_Write(I2C_HandleTypeDef* _p_hi2c, uint8_t _id, uint8_t _addr, uint8_t _data)
{

}


