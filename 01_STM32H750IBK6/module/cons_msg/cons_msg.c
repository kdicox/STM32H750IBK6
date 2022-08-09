/*
 * cons_msg.c
 *
 *  Created on: Aug 8, 2022
 *      Author: CSI
 */


/* Includes ------------------------------------------------------------------*/
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	<stdarg.h>

#include	"cons_msg.h"
//#include	"config_data.h"
//#include	"util.h"


/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define	TOKEN_DELIM				" "

#define IS_TOKEN(tok, s)		(strcmp(tok, s) == 0)
#define IS_HELP(tok)			(tok[0] == '?')

#define TOKEN(org, d, cmd)		strtok_r(org, d, &cmd)
#define NEXT_TOKEN(d, cmd)		strtok_r(NULL, d, &cmd)


/* Private variables ---------------------------------------------------------*/
static	uint8_t				g_cons_rx_buf[MAX_UART_RECVBUF];
static	uint16_t			g_cons_rx_wr = 0;
static	uint8_t				g_f_cons_rx = 0;

static	uint8_t				g_cons_tx_buf[MAX_UART_SENDBUF];
static	uint16_t			g_cons_tx_wr = 0;
static	uint16_t			g_cons_tx_rd = 0;

static	uint8_t				g_cmd_buf[MAX_UART_RECVBUF];
static	uint8_t				g_f_cmd_cmpt = 0;

static	uint8_t				g_msg_buf[MAX_UART_SENDBUF];

UART_HandleTypeDef *		g_p_cons_ctrl = NULL;

uint8_t						g_cons_recv;


/* Private function prototypes -----------------------------------------------*/
void	console_printf( const char* _msg, ... );
void	console_puts( const char* _msg );
void	console_putc( const char _ch );
void	console_send( const uint8_t _data );
uint8_t	console_is_empty_tx( void );

void	console_recv( uint8_t _data );

void	command_parser( char* _cmd );
void	console_process( void );


/* Private user code ---------------------------------------------------------*/
/**
  * @brief 콘솔 출력
  * @param  _msg: 포맷있는 메세지
  * @param  ...: 가변 인수
  * @retval 없음
  */
void	console_printf( const char* _msg, ... )
{
	va_list			arg;


	va_start( arg, _msg );
	vsprintf( (char*)g_msg_buf, _msg, arg );
	va_end( arg );

	console_puts( (char*)g_msg_buf );
}


/**
  * @brief 콘솔 출력
  * @param  _msg: 메세지
  * @retval 없음
  */
void	console_puts( const char* _msg )
{
	HAL_UART_Transmit( g_p_cons_ctrl, (uint8_t*)&_msg[0], strlen(_msg), 1000 );
#if 0
	while ( *_msg ) {
		console_putc( *_msg );
		_msg++;
	}
#endif
}


/**
  * @brief 콘솔 출력
  * @param  _data : 아스키코드
  * @retval 없음
  */
void	console_putc( const char _ch )
{
	HAL_UART_Transmit( g_p_cons_ctrl, (uint8_t*)&_ch, 1, 1000 );
#if 0
	g_cons_tx_buf[g_cons_tx_wr] = _data;
	g_cons_tx_wr++;
	g_cons_tx_wr &= 0x00FF;

	HAL_UART_Transmit_IT(g_p_cons_uart, &g_cons_tx_buf[g_cons_tx_rd], 1);
	g_cons_tx_rd++;
	g_cons_tx_rd &= 0x00FF;
#endif
}


/**
  * @brief 콘솔 출력
  * @param  _data : 아스키코드
  * @retval 없음
  */
void	console_send( const uint8_t _data )
{
	HAL_UART_Transmit( g_p_cons_ctrl, &_data, 1, 1000 );
#if 0
	g_cons_tx_buf[g_cons_tx_wr] = _data;
	g_cons_tx_wr++;
	g_cons_tx_wr &= 0x00FF;

	HAL_UART_Transmit_IT(g_p_cons_uart, &g_cons_tx_buf[g_cons_tx_rd], 1);
	g_cons_tx_rd++;
	g_cons_tx_rd &= 0x00FF;
#endif
}


/**
  * @brief  TX 버퍼가 비어있는지 확인
  * @retval TX 버퍼가 비어있으면 1, 아니면 0
  */
uint8_t	console_is_empty_tx( void )
{
	uint16_t		rd = g_cons_tx_rd;
	uint16_t		wr = g_cons_tx_wr;


	return (rd == wr);
}


/**
  * @brief  데이터 수신
  * @param  _data : 데이터
  * @retval 없음
  */
void	console_recv( uint8_t _data )
{
	//	콘솔 수신 모드를 사용하지 않거나, 커맨드가 처리 중이면 반환한다.
	if ( 0 == g_f_cons_rx )		{ return; }
	if ( 1 == g_f_cmd_cmpt )	{ return; }

	if ( ASCII_CR == _data ) {
		//	수신 데이터가 있으면 처리한다.
		if ( 0 < g_cons_rx_wr ) {
			uint16_t		lp;


			console_puts( "\r\n" );

			for ( lp = 0; lp < g_cons_rx_wr; lp++ ) {
				g_cmd_buf[lp] = g_cons_rx_buf[lp];
				g_cons_rx_buf[lp] = 0;
			}

			g_cmd_buf[lp] = 0;

			g_f_cmd_cmpt = 1;
			g_cons_rx_wr = 0;
		}

		return;
	}

	if ( ASCII_BS == _data ) {			//	백스페이스 입력
		if ( 0 < g_cons_rx_wr ) {		//	수신 데이터가 있으면 작업한다.
			g_cons_rx_wr--;
			g_cons_rx_buf[g_cons_rx_wr] = 0;

			console_putc( ASCII_BS );
			console_putc( ASCII_SP );
		}
	}
	else if ( ASCII_SP == _data ) {		//	스페이스 입력
		if ( 0 < g_cons_rx_wr ) {
			if ( ASCII_SP == g_cons_rx_buf[g_cons_rx_wr - 1] ) {	//	연속으로 스페이스가 생기지 않게 하다.
			}
			else {
				g_cons_rx_buf[g_cons_rx_wr] = _data;
				g_cons_rx_wr++;
			}
		}
		else {
		}
	}
	else {
		if ( '0' <= _data && '9' >= _data ) {
			g_cons_rx_buf[g_cons_rx_wr] = _data;
			g_cons_rx_wr++;
		}
		else if ( 'A' <= _data && 'Z' >= _data ) {
			g_cons_rx_buf[g_cons_rx_wr] = _data;
			g_cons_rx_wr++;
		}
		else if ( 'a' <= _data && 'z' >= _data ) {
			g_cons_rx_buf[g_cons_rx_wr] = _data;
			g_cons_rx_wr++;
		}
		else if ( '-' == _data || '*' == _data ) {
			g_cons_rx_buf[g_cons_rx_wr] = _data;
			g_cons_rx_wr++;
		}
		else if ( ':' == _data || '?' == _data || '.' == _data ) {
			g_cons_rx_buf[g_cons_rx_wr] = _data;
			g_cons_rx_wr++;
		}
	}

	console_putc( _data );

	//	버퍼를 다채우면 위치를 초기화한다.
	if ( MAX_UART_RECVBUF <= g_cons_rx_wr ) {
		g_cons_rx_wr = 0;
	}
}


/**
  * @brief  커맨드 파서
  * @param  _cmd : 커맨드
  * @retval 없음
  */
void	command_parser( char* _cmd )
{
	static	char*		cmd = NULL;
	static	char*		tok = NULL;


	cmd = NULL;
	tok = NULL;

	//	first token
	tok = TOKEN(_cmd, TOKEN_DELIM, cmd);

	//	compare command
	if ( IS_TOKEN(tok, "help") ) {				//	mode command
		console_puts( "==================================================\r\n" );
		console_puts( "mode          : firmware mode\r\n" );
		console_puts( "ver [mpu]     : firmware version\r\n" );
		console_puts( "facdef        : factory default\r\n" );
		console_puts( "==================================================\r\n" );
	}
	else if ( IS_TOKEN(tok, "mode") ) {			//	mode command
		console_puts("$MTRCTRLFW#\r\n");
	}
	else if ( IS_TOKEN(tok, "ver") ) {			//	version information
		extern const	char*				g_build_date;		//	sub_main.c에 선언
		extern const	char*				g_build_time;		//	sub_main.c에 선언
		extern const	uint16_t			g_fw_ver;			//	sub_main.c에 선언


		tok = NEXT_TOKEN(TOKEN_DELIM, cmd);

		if ( NULL == tok ) {
			console_printf("$ver[%04X]#\r\n", g_fw_ver);
		}
		else if ( IS_TOKEN(tok, "mpu") ) {
			console_printf("$FW VER:%04X [%s %s]#\r\n", g_fw_ver, g_build_date, g_build_time);
		}
	}
	else if ( IS_TOKEN(tok, "facdef") ) {       //  factory default
		console_puts("$OK[facdef]#\r\n");
	}
}


/**
  * @brief  콘솔 프로세스
  * @retval 없음
  */
void	console_process( void )
{
	static	uint8_t		proc_seq = 0;


	switch ( proc_seq ) {
	case	0	:	//	커맨드 수신 확인
		{
			if ( 0 == g_f_cmd_cmpt ) { return; }

			proc_seq++;
		}
		break;

	case	1	:	//	수신 커맨드 처리
		{
			command_parser( (char*)g_cmd_buf );
			proc_seq++;
		}
		break;

	case	2	:	//	초기 설정
		{
			g_f_cmd_cmpt	= 0;
			proc_seq		= 0;

			console_puts( "\r\n> " );
			//HAL_UART_Transmit( g_p_cons_uart, &CONS_PROMPT[0], 4, 1000 );
		}
		break;

	default		:
		{
			console_puts( "missing command process!\r\n" );
			g_f_cmd_cmpt	= 0;
			proc_seq		= 0;
			console_puts( "\r\n> " );
		}
		break;
	}
}


/* Exported user code --------------------------------------------------------*/
/**
  * @brief  서브 메인 초기화
  * @retval 없음
  */
void	CONSMSG_Initial( void )
{
	uint16_t		lp;


	for ( lp = 0; lp < MAX_UART_RECVBUF; lp++ ) {
		g_cons_rx_buf[lp]	= 0;
		g_cons_tx_buf[lp]	= 0;
		g_cmd_buf[lp]		= 0;
		g_msg_buf[lp]		= 0;
	}

	g_cons_rx_wr	= 0;
	g_f_cons_rx		= 1;

	g_cons_tx_wr	= 0;
	g_cons_tx_rd	= 0;

	g_f_cmd_cmpt = 0;

	HAL_UART_Receive_IT( g_p_cons_ctrl, &g_cons_recv, 1);
}


/**
  * @brief UART 수신 입터렙트 처리
  * @param  _p_uart_info: 구조체 UART_HandleTypeDef의 포인터
  * @retval 없음
  */
void	CONSMSG_Rx_IT(UART_HandleTypeDef* _p_uart_info)
{
	console_recv(g_cons_recv);
	HAL_UART_Receive_IT(g_p_cons_ctrl, &g_cons_recv, 1);
}


/**
  * @brief UART 송신 입터렙트 처리
  * @param  _p_uart_info: 구조체 UART_HandleTypeDef의 포인터
  * @retval 없음
  */
void	CONSMSG_Tx_IT(UART_HandleTypeDef* _p_uart_info)
{

}


/**
  * @brief 1 바이트 전송
  * @param  _data: 전송할 바이트 데이터
  * @retval 없음
  */
void	CONSMSG_Send(const uint8_t _data)
{
	console_send(_data);
}


/**
  * @brief 1 문자 전송
  * @param  _ch: 전송할 문자 데이터
  * @retval 없음
  */
void	CONSMSG_Putc(const char _ch)
{
	console_putc(_ch);
}


/**
  * @brief 문자열 전송
  * @param  _msg: 전송할 문자열 데이터의 포인터
  * @retval 없음
  */
void	CONSMSG_Puts(const char* _msg)
{
	console_puts(_msg);
}


/**
  * @brief 형식 있는 문자열 전송
  * @param  _msg: 전송할 문자열 데이터의 포인터
  * @param  ...: 가변 입력 데이터
  * @retval 없음
  */
void	CONSMSG_Printf( const char* _msg, ... )
{
	va_list			arg;


	va_start( arg, _msg );
	vsprintf( (char*)g_msg_buf, _msg, arg );
	va_end( arg );

	console_puts( (char*)g_msg_buf );
}


/**
  * @brief  서브 메인 프로세스
  * @retval 없음
  */
void	CONSMSG_Process( void )
{
	console_process();
}


