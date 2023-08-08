#include "RW1063.h"

uint8_t CGRAM[3][8] = {
	{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F},//All black
	{0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F},//Square
	{0x0A, 0x15, 0x0A, 0x15, 0x0A, 0x15, 0x0A, 0x15},//Snowflake
};

//unsigned char CGRAM1[] = {
//0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,
//0x1f,0x11,0x11,0x11,0x11,0x11,0x11,0x1f,
//0x0a,0x15,0x0a,0x15,0x0a,0x15,0x0a,0x15,
//0x0a,0x15,0x0a,0x15,0x0a,0x15,0x0a,0x15
//};

uint8_t MSG1[16]    ="0123456789ABCDEF";
uint8_t MSG2[16]    ="Have nice day!!!";
uint8_t MSG3[16]    ="  Good   Luck!!!";
uint8_t MSG4[16]    ="G o o d  wawa!!!";

bool isTransmitActive = false;
SPI_HandleTypeDef *pRW1063_hspi;

GPIO_TypeDef *pRW1063_CS_Port = NULL;
uint16_t RW1063_CS_Pin = 0;

GPIO_TypeDef *pRW1063_RS_Port = NULL;
uint16_t RW1063_RS_Pin = 0;

//static void RST_SetLow( void )
//{
//	if( pRW1063_RST_Port != NULL )
//		HAL_GPIO_WritePin( pRW1063_RST_Port, RW1063_RST_Pin, GPIO_PIN_RESET );
//}
//static void RST_SetHigh( void )
//{
//	if( pRW1063_RST_Port != NULL )
//		HAL_GPIO_WritePin( pRW1063_RST_Port, RW1063_RST_Pin, GPIO_PIN_SET );
//}

static void CS_SetLow( void )
{
	if( pRW1063_CS_Port != NULL )
		HAL_GPIO_WritePin( pRW1063_CS_Port, RW1063_CS_Pin, GPIO_PIN_RESET );
}
static void CS_SetHigh( void )
{
	if( pRW1063_CS_Port != NULL )
		HAL_GPIO_WritePin( pRW1063_CS_Port, RW1063_CS_Pin, GPIO_PIN_SET );
}

static void RS_SetLow( void )
{
	if( pRW1063_RS_Port != NULL )
		HAL_GPIO_WritePin( pRW1063_RS_Port, RW1063_RS_Pin, GPIO_PIN_RESET );
}

static void RS_SetHigh( void )
{
	if( pRW1063_RS_Port != NULL )
		HAL_GPIO_WritePin( pRW1063_RS_Port, RW1063_RS_Pin, GPIO_PIN_SET );
}

static void spi_write_nbytes ( uint8_t *d, uint32_t len )
{
	if( pRW1063_hspi != NULL )
		HAL_SPI_Transmit( pRW1063_hspi, d, len, 100 );
}

static void spi_write_nbytes_IT ( uint8_t *d, uint32_t len )
{
	if( pRW1063_hspi != NULL )
		HAL_SPI_Transmit_IT( pRW1063_hspi, d, len );
}

static void spi_change_data_size ( uint8_t size )
{
	/* Confirm that the pointer of SPI handler is available */
	if( pRW1063_hspi == NULL )
		return;

	size = ( size > 16 ) ? 16 : size;
	size = ( size < 4 ) ? 4 : size;
	size--;
	
	pRW1063_hspi->Init.DataSize = size * 0x100;
	
	HAL_SPI_DeInit( pRW1063_hspi );
	HAL_SPI_Init( pRW1063_hspi );
}

static void spi_set_prescaler ( uint32_t prescaler )
{
	/* Confirm that the pointer of SPI handler is available */
	if( pRW1063_hspi == NULL )
		return;

	pRW1063_hspi->Init.BaudRatePrescaler = prescaler;
	
	HAL_SPI_DeInit( pRW1063_hspi );
	HAL_SPI_Init( pRW1063_hspi );
}

bool RW1063_init ( SPI_HandleTypeDef *hspi,
					GPIO_TypeDef *CS_PORT, uint16_t CS_PIN,
					GPIO_TypeDef *RS_PORT, uint16_t RS_PIN )
{

	unsigned char i, j;

	if( (hspi == NULL) || (CS_PORT == NULL) || (RS_PORT == NULL) )
		return false;

	pRW1063_hspi = hspi;

	pRW1063_CS_Port = CS_PORT;
	RW1063_CS_Pin = CS_PIN;

	pRW1063_RS_Port = RS_PORT;
	RW1063_RS_Pin = RS_PIN;

	/*****************************************************/

	RW1063_WrCmd(0x38);//SET 2 LINE,5*8 FONT
	HAL_Delay(1);
	RW1063_WrCmd(0x38);//SET 2 LINE,5*8 FONT
	HAL_Delay(1);
	RW1063_WrCmd(0x38);//SET 2 LINE,5*8 FONT
	HAL_Delay(1);
	RW1063_WrCmd(0x38);//SET 2 LINE,5*8 FONT
	HAL_Delay(1);
	RW1063_WrCmd(0x08);//Display off
	HAL_Delay(1);
	RW1063_WrCmd(0x06);//Entry mode set
	HAL_Delay(10);
	RW1063_WrCmd(0x01);//CLEAR DISPLAY
	HAL_Delay(1);
	RW1063_WrCmd(0x0C);//DISPLAY ON,Cursor OFF,Cursor Blink OFF

	/*****************************************************/
	// The first graph of the CGRAM Address start address
	//  is 000000 (0x00) CGRAM address is set to 0x40 + 0x00 = 0x40
	// The second graph's CGRAM Address starts with address 001000 (0x08)
	// etc.

	for (i = 0; i < 3; i ++)
	{
		RW1063_WrCmd(0x40 + (0x08 * i));

		for (j = 0; j < 8; j ++)
		{
			RW1063_WrData(CGRAM[i][j]);
		}
	}

	/*****************************************************/
	//Retrieve the customized font from CGRAM and write it into DDRAM

//	RW1063_WrCmd(LINE_ONE);
//	HAL_Delay(1);
//	for(int x = 0; x < 16 ; x++)
//	{
//		RW1063_WrData(0x00);
//		HAL_Delay(1);
//	}
//	RW1063_WrCmd(LINE_TWO);
//	HAL_Delay(1);
//	for(int y = 0; y < 16 ; y++)
//	{
//		RW1063_WrData(0x02);
//		HAL_Delay(1);
//	}

	//Write the default font into DDRAM.

	RW1063_WrCmd(LINE_ONE);
	RW1063_writeString(Cword,MSG1);
	RW1063_WrCmd(LINE_TWO);
	RW1063_writeString(Cword,MSG2);
//	RW1063_WrCmd(LINE_THREE);
//	RW1063_writeString(Cword,MSG3_1);
//	RW1063_WrCmd(LINE_FOUR);
//	RW1063_writeString(Cword,MSG4);

	return true;
}

void RW1063_WrCmd ( uint8_t d )
{
    CS_SetLow( );
    RS_SetLow( );  
	spi_write_nbytes( &d, 1 );
    CS_SetHigh( );
}

void RW1063_WrData ( uint8_t d )
{
	CS_SetLow( );
    RS_SetHigh( );
    spi_write_nbytes( &d, 1 );
    CS_SetHigh( );
}

void RW1063_writeString(uint8_t count,uint8_t *MSG)
{
  for(uint8_t i = 0; i<count;i++)
  {
	  RW1063_WrData(MSG[i]);
  }
}

