#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include <inttypes.h>
#include "main.h"
#include "stm32l011xx.h"
#include "stm32l0xx_hal.h"
#include "BMP280/bmp280.h"	// Description of registers and commands BMP280
#include "NRF/NRF24L01.h"		// Description of registers and commands NRF24L01+

// Setting of NRF24L01+
#define CHAN		4 // channel number
#define RF_DATA_SIZE		10 // size of sendind data, bytes
#define TIME_SENDING		50 // Frequnce of sending data = 1/10 sek // 50 = 5 sek
#define TIME_FLASH_LED		10 // Frequnce of flash led = 1/10 sek // 10 = 1 sek

// SPI setting
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


// I2C setting
#define I2C_GPIO_RCC	RCC_APB2Periph_GPIOA
#define I2C_GPIO		GPIOA
#define I2C_SDA		GPIO_PIN_10
#define I2C_SCL		GPIO_PIN_9

//See also settings have in file stm32l0xx_hal.c
