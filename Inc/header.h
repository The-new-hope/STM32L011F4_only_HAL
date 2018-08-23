#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include <inttypes.h>

#include "main.h"
#include "stm32l011xx.h"
#include "stm32l0xx_hal.h"
#include "BMP280/bmp280.h"

//#include "NEW_NRF/NRF24.h"

#include "NRF/NRF24L01.h"										//
//#include "PROTOCOL/protocol.h"									// 


#define SPIx_RCC		RCC_APB1Periph_SPI1
#define SPIx			SPI1
#define SPI_GPIO_RCC	RCC_APB2Periph_GPIOA
#define SPI_GPIO		GPIOA
#define SPI_PIN_MOSI	GPIO_PIN_7
#define SPI_PIN_MISO	GPIO_PIN_6
#define SPI_PIN_SCK		GPIO_PIN_5
#define SPI_PIN_CSN		GPIO_PIN_4
#define SPI_PIN_IRQ		GPIO_PIN_1
#define SPI_PIN_CE		GPIO_PIN_3

// ”правление линией CSN
#define CSN1 HAL_GPIO_WritePin(SPI_GPIO, SPI_PIN_CSN, GPIO_PIN_SET)
#define CSN0 HAL_GPIO_WritePin(SPI_GPIO, SPI_PIN_CSN, GPIO_PIN_RESET)
// ”правление линией CE
#define CE1   HAL_GPIO_WritePin(SPI_GPIO, SPI_PIN_CE, GPIO_PIN_SET)
#define CE0   HAL_GPIO_WritePin(SPI_GPIO, SPI_PIN_CE, GPIO_PIN_RESET)




//void Init_SPIx(void);
