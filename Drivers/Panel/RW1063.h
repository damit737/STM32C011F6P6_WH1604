/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ST7735S_H
#define _ST7735S_H

#ifdef __cplusplus
extern "C" {
#endif

	
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"

#define Cword       0x10  //16

#define LINE_ONE    0x80  // DD RAM Address The starting position of the first line is 0x00
                          // set the DD RAM address to 0x80 + 0x00 = 0x80
#define LINE_TWO    0xc0  // DD RAM Address The starting position of the second line is 0x40
                          // set the DD RAM address to 0x80 + 0x40 = 0xc0
#define LINE_THREE  0x94  // DD RAM Address The starting position of the second line is 0x14
                          // set the DD RAM address to 0x80 + 0x14 = 0x94
#define LINE_FOUR   0xD4  // DD RAM Address The starting position of the second line is 0x54
                          // set the DD RAM address to 0x80 + 0x54 = 0xD4

	
bool RW1063_init ( SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN, GPIO_TypeDef *RS_PORT, uint16_t RS_PIN );
void RW1063_hardware_reset ( void );
void RW1063_WrCmd ( uint8_t d );
void RW1063_WrData ( uint8_t d );
void RW1063_writeString( uint8_t count,uint8_t *MSG );
#ifdef __cplusplus
}
#endif

#endif /* _ST7735S_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
