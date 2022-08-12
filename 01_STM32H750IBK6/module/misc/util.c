/*
 * util.c
 *
 *  Created on: 2022. 8. 12.
 *      Author: CSI
 */


/* Includes ------------------------------------------------------------------*/
#include	<stdlib.h>

#include	"string.h"
#include	"util.h"


/* External variables --------------------------------------------------------*/


/* External functions --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint16_t	CRC16TBL[256] =
{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,

	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};


uint16_t						g_calc_crc = 0;


/* Private function prototypes -----------------------------------------------*/
uint8_t		IsHexadecimal( char* code );
char*		CutPrefix( char* code );
uint8_t		AsciiToHex( char code );
uint8_t		AsciiToDec( char code );


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  UTIL Is Hexadecimal
  * @param  code : ascii code data
  * @retval flag result
  */
uint8_t		IsHexadecimal( char* code )
{
	if ( 2 > strlen( code ) ) { return ( 0 ); }

	if ( '0' == code[0] && 'x' == code[1] ) {
		return ( 1 );
	}

	return ( 0 );
}


/**
  * @brief  UTIL Cut Prefix
  * @param  code : ascii code data
  * @retval code data pointer
  */
char*		CutPrefix( char* code )
{
	if ( 2 > strlen( code ) ) { return ( code ); }

	if ( '0' == code[0] && 'x' == code[1] ) {
		return ( &code[2] );
	}

	return ( code );
}


/**
  * @brief  UTIL Ascii to hexadecimal
  * @param  code : ascii code data
  * @retval hexadecimal
  */
uint8_t		AsciiToHex( char code )
{
	if ( 'Z' < code ) { code -= ('a' - 'A'); }

	if ( '0' > code || 'Z' < code ) { return (0x80); }
	if ( '9' < code && 'A' > code ) { return (0x80); }

	if ( '9' >= code ) { return (code - '0'); }

	return ((code - 'A') + 10);
}


/**
  * @brief  UTIL Ascii to decimal
  * @param  code : ascii code data
  * @retval decimal
  */
uint8_t		AsciiToDec( char code )
{
	if ( '9' < code || '0' > code ) { return (0x80); }

	return (code - '0');
}


/* Public functions ----------------------------------------------------------*/
/**
  * @brief  UTIL Calculate CRC16
  * @param  p_buf : data buffer
  * @param  len   : data buffer length
  * @retval crc16
  */
uint16_t	UTIL_CalculateCRC16( uint8_t* p_buf, uint16_t len )
{
	uint16_t			crc;
	uint16_t			lp;


	crc = 0;
	for ( lp = 0; lp < len; lp++ ) {
		crc = (crc<<8) ^ CRC16TBL[((crc>>8) ^ p_buf[lp]) & 0x00FF];
	}

	return (crc);
}


/**
  * @brief  UTIL Set Calculate CRC16
  * @param  None
  * @retval None
  */
void	UTIL_SetCalcCRC16( void )
{
	g_calc_crc = 0;
}


/**
  * @brief  UTIL Calculate CRC16
  * @param  buf : calculate data
  * @retval None
  */
void	UTIL_CalcCRC16( uint8_t buf )
{
	g_calc_crc = (g_calc_crc<<8) ^ CRC16TBL[((g_calc_crc>>8) ^ buf) & 0x00FF];
}


/**
  * @brief  UTIL Get Calculate CRC16
  * @param  None
  * @retval CRC16
  */
uint16_t	UTIL_GetCalcCRC16( void )
{
	return ( g_calc_crc );
}


/**
  * @brief  UTIL Hex to Byte
  * @param  code : ascii string data
  * @retval decimal data 8 bits
  */
uint8_t		UTIL_CodeToByte( char* code )
{
	char*			ascii;

	uint8_t			flag, num;
	uint8_t			cnt = 0;
	uint16_t		dec = 0;


	flag	= IsHexadecimal( code );
	ascii	= CutPrefix( code );

	while ( *ascii ) {
		cnt++;

		if ( flag )	{ num = AsciiToHex( *ascii ); }
		else		{ num = AsciiToDec( *ascii ); }

		ascii++;

		if ( 0x80 != num ) {
			if ( flag )	{ dec *= 16; }
			else		{ dec *= 10; }

			dec += num;
		}
	}

	return (dec);
}


/**
  * @brief  UTIL Hex to Word
  * @param  code : ascii string data
  * @retval decimal data 16 bits
  */
uint16_t	UTIL_CodeToWord( char* code )
{
	char*			ascii;

	uint8_t			flag, num;
	uint8_t			cnt = 0;
	uint16_t		dec = 0;


	flag	= IsHexadecimal( code );
	ascii	= CutPrefix( code );

	while ( *ascii ) {
		cnt++;

		if ( flag )	{ num = AsciiToHex( *ascii ); }
		else		{ num = AsciiToDec( *ascii ); }

		ascii++;

		if ( 0x80 != num ) {
			if ( flag )	{ dec *= 16; }
			else		{ dec *= 10; }

			dec += num;
		}
	}

	return (dec);
}


/**
  * @brief  UTIL Hex to Word
  * @param  code : ascii string data
  * @retval decimal data 32 bits
  */
uint32_t	UTIL_CodeToDword( char* code )
{
	char*			ascii;

	uint8_t			flag, num;
	uint8_t			cnt = 0;
	uint32_t		dec = 0;


	flag	= IsHexadecimal( code );
	ascii	= CutPrefix( code );

	while ( *ascii ) {
		cnt++;

		if ( flag )	{ num = AsciiToHex( *ascii ); }
		else		{ num = AsciiToDec( *ascii ); }

		ascii++;

		if ( 0x80 != num ) {
			if ( flag )	{ dec *= 16; }
			else		{ dec *= 10; }

			dec += num;
		}
	}

	return (dec);
}


/**
  * @brief  UTIL Ascii to int8_t
  * @param  code : ascii string data
  * @retval decimal data 8 bits
  */
int8_t		UTIL_CodeToInt8( char* code )
{
    char*			ascii;

	uint8_t			num, sig = 0;
	uint8_t			cnt = 0;
	int8_t		    dec = 0;


	if ( '-' == code[0] ) {
    	ascii	= &code[1];
    	sig     = 1;
	}
	else {
        ascii	= &code[0];
	}

	while ( *ascii ) {
		cnt++;

		num = AsciiToDec( *ascii );
		ascii++;

		dec *= 10;
        dec += num;
	}

	if ( 1 == sig ) { dec *= -1; }

	return (dec);
}


/**
  * @brief  UTIL Ascii to int16_t
  * @param  code : ascii string data
  * @retval decimal data 16 bits
  */
int16_t	    UTIL_CodeToInt16( char* code )
{
    char*			ascii;

	uint8_t			num, sig = 0;
	uint8_t			cnt = 0;
	int16_t		    dec = 0;


	if ( '-' == code[0] ) {
    	ascii	= &code[1];
    	sig     = 1;
	}
	else {
        ascii	= &code[0];
	}

	while ( *ascii ) {
		cnt++;

		num = AsciiToDec( *ascii );
		ascii++;

		dec *= 10;
        dec += num;
	}

	if ( 1 == sig ) { dec *= -1; }

	return (dec);
}


/**
  * @brief  UTIL Ascii to int32_t
  * @param  code : ascii string data
  * @retval decimal data 32 bits
  */
int32_t	    UTIL_CodeToInt32( char* code )
{
    char*			ascii;

	uint8_t			num, sig = 0;
	uint8_t			cnt = 0;
	int32_t		    dec = 0;


	if ( '-' == code[0] ) {
    	ascii	= &code[1];
    	sig     = 1;
	}
	else {
        ascii	= &code[0];
	}

	while ( *ascii ) {
		cnt++;

		num = AsciiToDec( *ascii );
		ascii++;

		dec *= 10;
        dec += num;
	}

	if ( 1 == sig ) { dec *= -1; }

	return (dec);
}


/**
  * @brief  UTIL Ascii to real int8_t
  * @param  code : ascii string data
  * @param  _pt_num : point number
  * @retval decimal data 8 bits
  */
int8_t		UTIL_CodeToReal8( char* code, uint8_t _pt_num )
{
    char*			ascii;

	uint8_t			num, sig = 0;
	uint8_t			cnt = 0;
	int8_t		    dec = 0;


	if ( '-' == code[0] ) {
    	ascii	= &code[1];
    	sig     = 1;
	}
	else {
        ascii	= &code[0];
	}

	while ( *ascii ) {
        if ( '.' == *ascii ) {
            cnt = 1;
            ascii++;
            continue;
        }

		num = AsciiToDec( *ascii );
		ascii++;

		dec *= 10;
        dec += num;

        if ( 0 == cnt )         { continue; }
        if ( _pt_num <= cnt )   { break; }
        cnt++;
	}

	if ( 1 == sig ) { dec *= -1; }

	return (dec);
}


/**
  * @brief  UTIL Ascii to real int16_t
  * @param  code : ascii string data
  * @param  _pt_num : point number
  * @retval decimal data 16 bits
  */
int16_t	    UTIL_CodeToReal16( char* code, uint8_t _pt_num )
{
    char*			ascii;

	uint8_t			num, sig = 0;
	uint8_t			cnt = 0;
	int16_t		    dec = 0;


	if ( '-' == code[0] ) {
    	ascii	= &code[1];
    	sig     = 1;
	}
	else {
        ascii	= &code[0];
	}

	while ( *ascii ) {
        if ( '.' == *ascii ) {
            cnt = 1;
            ascii++;
            continue;
        }

		num = AsciiToDec( *ascii );
		ascii++;

		dec *= 10;
        dec += num;

        if ( 0 == cnt )         { continue; }
        if ( _pt_num <= cnt )   { break; }
        cnt++;
	}

	if ( 1 == sig ) { dec *= -1; }

	return (dec);
}


/**
  * @brief  UTIL Ascii to real int32_t
  * @param  code : ascii string data
  * @param  _pt_num : point number
  * @retval decimal data 32 bits
  */
int32_t	    UTIL_CodeToReal32( char* code, uint8_t _pt_num )
{
    char*			ascii;

	uint8_t			num, sig = 0;
	uint8_t			cnt = 0;
	int32_t		    dec = 0;


	if ( '-' == code[0] ) {
    	ascii	= &code[1];
    	sig     = 1;
	}
	else {
        ascii	= &code[0];
	}

	while ( *ascii ) {
        if ( '.' == *ascii ) {
            cnt = 1;
            ascii++;
            continue;
        }

		num = AsciiToDec( *ascii );
		ascii++;

		dec *= 10;
        dec += num;

        if ( 0 == cnt )         { continue; }
        if ( _pt_num <= cnt )   { break; }
        cnt++;
	}

	if ( 1 == sig ) { dec *= -1; }

	return (dec);
}

