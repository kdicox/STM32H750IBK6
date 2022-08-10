/*
 * ad7928_drv.c
 *
 *  Created on: 2022. 8. 9.
 *      Author: CSI
 *
 *
 */

/*
 * AD7928 컨트롤 레지스터 설명
 *
 * D11 (WRITE)
 *
 * 		컨트롤 레지스터에서 이 비트에 기록된 값은 다음 11비트가 컨트롤 레지스터에 로드되는지 여부를 결정합니다.
 * 		이 비트가 1이면 다음 11비트가 컨트롤 레지스터에 기록됩니다.
 * 		0이면 나머지 11비트는 컨트롤 레지스터에 로드되지 않고 변경되지 않은 상태로 유지됩니다.
 *
 * D10 (SEQ)
 *
 *		컨트롤 레지스터의 SEQ 비트는 시퀀서 기능의 사용을 제어하고,
 *		SHADOW 레지스터에 액세스하기 위해 SHADOW 비트와 함께 사용됩니다(SHADOW 레지스터 비트 맵 참조).
 *
 * D9 (DON’TCARE)
 *
 *
 * D8 ~ D6 (ADD2 ~ ADD0)
 *
 *		이 3개의 주소 비트는 현재 변환 시퀀스의 끝에서 로드되고, 다음 시리얼 전송에서 변환할 아날로그 입력 채널을 선택하거나,
 *		표 10에 설명된 대로 연속 시퀀스에서 최종 채널을 선택할 수 있습니다.
 *		선택된 입력 채널은 표 8과 같이 디코딩됩니다.
 *		변환 결과에 해당하는 주소 비트도 12비트 데이터 이전에 DOUT에 출력됩니다. 시리얼 인터페이스 섹션을 참조하십시오.
 *		변환될 다음 채널은 14번째 SCLK 하강 에지에서 mux에 의해 선택됩니다.
 *
 * D5, D4 (PM1, PM0)
 *
 * 		전원 관리 비트. 이 두 비트는 표 9와 같이 AD7908/AD7918/AD7928의 작동 모드를 디코딩합니다.
 *
 * D3 (SHADOW)
 *
 *		컨트롤 레지스터의 SHADOW 비트는 SEQ 비트와 함께 사용되어,
 *		시퀀서 기능의 사용을 컨트롤하고 SHADOW 레지스터에 액세스합니다(표 10 참조).
 *
 * D2 (DON’TCARE)
 *
 *
 * D1 (RANGE)
 *
 * 		이 비트는 AD7908/AD7918/AD7928에서 사용할 아날로그 입력 범위를 선택합니다.
 * 		0으로 설정하면 아날로그 입력 범위가 0V에서 2 × REFIN으로 확장됩니다.
 * 		1로 설정하면 아날로그 입력 범위가 0V에서 REFIN(다음 변환용)까지 확장됩니다.
 * 		0V ~ 2 × REFIN의 경우 AVDD = 4.75V ~ 5.25V입니다.
 *
 * D0 (CODING)
 *
 * 		이 비트는 변환 결과에 대해 AD7908/AD7918/AD7928이 사용하는 출력 코딩 유형을 선택합니다.
 * 		이 비트가 0으로 설정되면 해당 부분의 출력 코딩은 2의 보수입니다.
 * 		이 비트가 1로 설정되면 부품의 출력 코딩은 스트레이트 바이너리(다음 변환용)입니다.
 *
 */



/* Includes ------------------------------------------------------------------*/
#include	"ad7928_drv.h"

#include	"cons_msg.h"


/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define		CTRL_BIT_WR			(0x0800)
#define		CTRL_BIT_SEQ		(0x0400)
#define		CTRL_BITS_CH0		(0x0000)
#define		CTRL_BITS_CH1		(0x0040)
#define		CTRL_BITS_CH2		(0x0080)
#define		CTRL_BITS_CH3		(0x00C0)
#define		CTRL_BITS_CH4		(0x0100)
#define		CTRL_BITS_CH5		(0x0140)
#define		CTRL_BITS_CH6		(0x0180)
#define		CTRL_BITS_CH7		(0x01C0)
#define		CTRL_BIT_PM1		(0x0020)
#define		CTRL_BIT_PM0		(0x0010)
#define		CTRL_BIT_SHADOW		(0x0008)
#define		CTRL_BIT_RANGE		(0x0002)
#define		CTRL_BIT_CODING		(0x0001)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Exported user code --------------------------------------------------------*/
/**
  * @brief  AD7928 초기화
  * @param  _p_hspi: 구조체 SPI_HandleTypeDef의 포인터
  * @retval 없음
  */
void	AD7928_Initial(SPI_HandleTypeDef* _p_hspi)
{
	//HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	//HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	//HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

	uint16_t		ctrl_reg[8] = {
			CTRL_BIT_WR | CTRL_BITS_CH0 | CTRL_BIT_PM1 | CTRL_BIT_PM0 | CTRL_BIT_RANGE | CTRL_BIT_CODING,
			CTRL_BIT_WR | CTRL_BITS_CH1 | CTRL_BIT_PM1 | CTRL_BIT_PM0 | CTRL_BIT_RANGE | CTRL_BIT_CODING,
			CTRL_BIT_WR | CTRL_BITS_CH2 | CTRL_BIT_PM1 | CTRL_BIT_PM0 | CTRL_BIT_RANGE | CTRL_BIT_CODING,
			CTRL_BIT_WR | CTRL_BITS_CH3 | CTRL_BIT_PM1 | CTRL_BIT_PM0 | CTRL_BIT_RANGE | CTRL_BIT_CODING,
			CTRL_BIT_WR | CTRL_BITS_CH4 | CTRL_BIT_PM1 | CTRL_BIT_PM0 | CTRL_BIT_RANGE | CTRL_BIT_CODING,
			CTRL_BIT_WR | CTRL_BITS_CH5 | CTRL_BIT_PM1 | CTRL_BIT_PM0 | CTRL_BIT_RANGE | CTRL_BIT_CODING,
			CTRL_BIT_WR | CTRL_BITS_CH6 | CTRL_BIT_PM1 | CTRL_BIT_PM0 | CTRL_BIT_RANGE | CTRL_BIT_CODING,
			CTRL_BIT_WR | CTRL_BITS_CH7 | CTRL_BIT_PM1 | CTRL_BIT_PM0 | CTRL_BIT_RANGE | CTRL_BIT_CODING
	};

	uint16_t		reg, data, id;


	id = 0;
	reg = ctrl_reg[id];
	while ( HAL_OK == HAL_SPI_TransmitReceive( _p_hspi, (uint8_t*)&ctrl_reg[id], (uint8_t*)&data, 1, 2000) ) {
		for (int i = 0; i < 5000000; i++) {
		}
		PRINTF("SPI %X %d %d %04X\r\n", (data >> 12)&0x000F, (int)data&0x0FFF, id, ctrl_reg[id]);
		id++;
		id %= 8;
		reg = ctrl_reg[id];
	}
}


/**
  * @brief  AD7928 레지스터 읽기
  * @param  _p_hspi: 구조체 SPI_HandleTypeDef의 포인터
  * @param  _addr: 레지스터 주소
  * @retval 레지스터 값
  */
uint16_t	AD7928_Read(SPI_HandleTypeDef* _p_hspi, uint16_t _addr)
{
	return ( 0 );
}


/**
  * @brief  AD7928 레지스터 쓰기
  * @param  _p_hspi: 구조체 SPI_HandleTypeDef의 포인터
  * @param  _addr: 레지스터 주소
  * @param  _data: 데이터
  * @retval 없음
  */
void	AD7928_Write(SPI_HandleTypeDef* _p_hspi, uint16_t _addr, uint16_t _data)
{

}

