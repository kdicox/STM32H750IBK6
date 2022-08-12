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
#define		DEVICE_CONFIG0			(0x00)
#define		DEVICE_CONFIG1			(0x01)
#define		LED_CONFIG0				(0x02)
#define		BANK_BRIGHTNESS			(0x03)
#define		BANK_A_COLOR			(0x04)
#define		BANK_B_COLOR			(0x05)
#define		BANK_C_COLOR			(0x06)
#define		LED0_BRIGHTNESS			(0x07)
#define		LED1_BRIGHTNESS			(0x08)
#define		LED2_BRIGHTNESS			(0x09)
#define		LED3_BRIGHTNESS			(0x0A)
#define		LED4_BRIGHTNESS			(0x0B)
#define		LED5_BRIGHTNESS			(0x0C)
#define		LED6_BRIGHTNESS			(0x0D)
#define		LED7_BRIGHTNESS			(0x0E)
#define		OUT0_COLOR				(0x0F)
#define		OUT1_COLOR				(0x10)
#define		OUT2_COLOR				(0x11)
#define		OUT3_COLOR				(0x12)
#define		OUT4_COLOR				(0x13)
#define		OUT5_COLOR				(0x14)
#define		OUT6_COLOR				(0x15)
#define		OUT7_COLOR				(0x16)
#define		OUT8_COLOR				(0x17)
#define		OUT9_COLOR				(0x18)
#define		OUT10_COLOR				(0x19)
#define		OUT11_COLOR				(0x1A)
#define		OUT12_COLOR				(0x1B)
#define		OUT13_COLOR				(0x1C)
#define		OUT14_COLOR				(0x1D)
#define		OUT15_COLOR				(0x1E)
#define		OUT16_COLOR				(0x1F)
#define		OUT17_COLOR				(0x20)
#define		OUT18_COLOR				(0x21)
#define		OUT19_COLOR				(0x22)
#define		OUT20_COLOR				(0x23)
#define		OUT21_COLOR				(0x24)
#define		OUT22_COLOR				(0x25)
#define		OUT23_COLOR				(0x26)
#define		DEVICE_RESET			(0x27)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
uint8_t	i2c_reg_rd(HI2CINFO_PTR _p_i2c_info, uint8_t _addr);
void	i2c_reg_wr(HI2CINFO_PTR _p_i2c_info, uint8_t _addr, uint8_t _data);

void	device_initial(HI2CINFO_PTR _p_i2c_info);

/* Private user code ---------------------------------------------------------*/
/**
  * @brief  i2c 레지스터 읽기
  * @param  _p_i2c_info: 구조체 HI2CINFO의 포인터
  * @param  _addr: 레지스터 주소
  * @retval 레지스터 값
  */
uint8_t	i2c_reg_rd(HI2CINFO_PTR _p_i2c_info, uint8_t _addr)
{
	uint8_t		data = 0;


	_p_i2c_info->f_error = 0;
	if (HAL_OK != HAL_I2C_Master_Transmit(_p_i2c_info->p_hi2c, (_p_i2c_info->id << 1), &_addr, 1, 500)) {
		_p_i2c_info->f_error = 1;
		return ( 0 );
	}

	if (HAL_OK != HAL_I2C_Master_Receive(_p_i2c_info->p_hi2c, (_p_i2c_info->id << 1) | 0x01, &data, 1, 500)) {
		_p_i2c_info->f_error = 1;
		return ( 0 );
	}

	return ( data );
}


/**
  * @brief  i2c 레지스터 쓰기
  * @param  _p_i2c_info: 구조체 HI2CINFO의 포인터
  * @param  _addr: 레지스터 주소
  * @param  _data: 데이터
  * @retval 없음
  */
void	i2c_reg_wr(HI2CINFO_PTR _p_i2c_info, uint8_t _addr, uint8_t _data)
{
	uint8_t		reg_data[2];


	reg_data[0] = _addr;
	reg_data[1] = _data;

	_p_i2c_info->f_error = 0;
	if (HAL_OK != HAL_I2C_Master_Transmit(_p_i2c_info->p_hi2c, (_p_i2c_info->id << 1), &reg_data[0], 2, 500)) {
		_p_i2c_info->f_error = 1;
	}
}


/**
  * @brief  디바이스 초기화
  * @param  _p_i2c_info: 구조체 HI2CINFO의 포인터
  * @retval 없음
  */
void	device_initial(HI2CINFO_PTR _p_i2c_info)
{
	i2c_reg_wr(_p_i2c_info, DEVICE_RESET, 0xFF);

	// LEDx_ BRIGHTNESS
	i2c_reg_wr(_p_i2c_info, 0x07, 0xFF);	// D1
	i2c_reg_wr(_p_i2c_info, 0x08, 0xFF);	// D2
	i2c_reg_wr(_p_i2c_info, 0x09, 0xFF);	// D3
	i2c_reg_wr(_p_i2c_info, 0x0A, 0x00);	// D4
	i2c_reg_wr(_p_i2c_info, 0x0B, 0x00);	// D5
	i2c_reg_wr(_p_i2c_info, 0x0C, 0x00);	// D6

	i2c_reg_wr(_p_i2c_info, 0x01, 0x26);	// 0x02 = Outputs maximum current IMAX = 35 mA
											// 0x00 = Outputs maximum current IMAX = 25.5 mA

	i2c_reg_wr(_p_i2c_info, 0x0F, 0xFF);
	i2c_reg_wr(_p_i2c_info, 0x10, 0xFF);
	i2c_reg_wr(_p_i2c_info, 0x11, 0xFF);      //D1

	i2c_reg_wr(_p_i2c_info, 0x12, 0xFF);
	i2c_reg_wr(_p_i2c_info, 0x13, 0xFF);
	i2c_reg_wr(_p_i2c_info, 0x14, 0xFF);      //D2

	i2c_reg_wr(_p_i2c_info, 0x15, 0xFF);
	i2c_reg_wr(_p_i2c_info, 0x16, 0xFF);
	i2c_reg_wr(_p_i2c_info, 0x17, 0x00);      //D3

	i2c_reg_wr(_p_i2c_info, 0x18, 0x00);
	i2c_reg_wr(_p_i2c_info, 0x19, 0x00);
	i2c_reg_wr(_p_i2c_info, 0x1A, 0x00);      //D4

	i2c_reg_wr(_p_i2c_info, 0x1B, 0x00);
	i2c_reg_wr(_p_i2c_info, 0x1C, 0x00);
	i2c_reg_wr(_p_i2c_info, 0x1D, 0x00);      //D5

	i2c_reg_wr(_p_i2c_info, 0x1E, 0x00);
	i2c_reg_wr(_p_i2c_info, 0x1F, 0x00);
	i2c_reg_wr(_p_i2c_info, 0x20, 0x00);      //D6

	i2c_reg_wr(_p_i2c_info, 0x03, 0x80);
	i2c_reg_wr(_p_i2c_info, 0x04, 0xFF);
	i2c_reg_wr(_p_i2c_info, 0x05, 0xFF);
	i2c_reg_wr(_p_i2c_info, 0x06, 0xFF);

	i2c_reg_wr(_p_i2c_info, 0x02, 0x00);
	i2c_reg_wr(_p_i2c_info, 0x00, 0x40);
}


/* Exported user code --------------------------------------------------------*/
/**
  * @brief  LP5024 초기화
  * @param  _p_i2c_info: 구조체 HI2CINFO의 포인터
  * @retval 없음
  */
void	LP5024_Initial(HI2CINFO_PTR _p_i2c_info)
{
	PRINTF("    LP5024 INITIALIZE [%08X:%02X]\r\n", _p_i2c_info->p_hi2c, _p_i2c_info->id);
	i2c_reg_wr(_p_i2c_info, DEVICE_RESET, 0xFF);

	if ( 1 == _p_i2c_info->f_error ) {
		_p_i2c_info->f_probe_dev = 0;

		PUTS("      DEVICE NOT FOUND\r\n");
	}
	else {
		_p_i2c_info->f_probe_dev = 1;

		device_initial(_p_i2c_info);

		PUTS("      DEVICE FOUND\r\n");
	}
}


/**
  * @brief  LP5024 출력
  * @param  _p_i2c_info: 구조체 HI2CINFO의 포인터
  * @param  _rates: 출력 비율 (0: 출력 없음, 255: 최대 출력)
  * @retval 없음
  */
void	LP5024_Output(HI2CINFO_PTR _p_i2c_info, uint8_t _rates)
{

}


/**
  * @brief  LP5024 레지스터 읽기
  * @param  _p_i2c_info: 구조체 HI2CINFO의 포인터
  * @param  _addr: 레지스터 주소
  * @retval 레지스터 값
  */
uint8_t	LP5024_Read(HI2CINFO_PTR _p_i2c_info, uint8_t _addr)
{
	return i2c_reg_rd(_p_i2c_info, _addr);
}


/**
  * @brief  LP5024 레지스터 쓰기
  * @param  _p_i2c_info: 구조체 HI2CINFO의 포인터
  * @param  _addr: 레지스터 주소
  * @param  _data: 데이터
  * @retval 없음
  */
void	LP5024_Write(HI2CINFO_PTR _p_i2c_info, uint8_t _addr, uint8_t _data)
{
	i2c_reg_wr(_p_i2c_info, _addr, _data);
}


