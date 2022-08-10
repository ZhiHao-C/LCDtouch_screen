#ifndef __BPS_FLASH_H__
#define __BPS_FLASH_H__

#include "stm32f10x.h" 

#define FLASH_SPIx                    SPI1
#define FLASH_SPI_CLK                RCC_APB2Periph_SPI1
#define FLASH_SPI_APBxClkCmd         RCC_APB1PeriphClockCmd
#define FLASH_SPI_GPIOA_CLK       (RCC_APB2Periph_GPIOA)
#define FLASH_SPI_GPIOC_CLK       (RCC_APB2Periph_GPIOC)
#define FLASH_SPI_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd

#define SPI_FLASH_PageSize              256
#define SPI_FLASH_PerWritePageSize      256

#define SPITIMEout                    0x1000
#define DUMMY                         0X00
#define Read_ID                       0x9f
#define Sector_Erase                  0x20
#define Read_Status_Register1         0x05
#define Read_Data                     0x03 
#define Write_Enable                  0x06
#define Page_Program                  0x02  //–¥»Î ˝æ›√¸¡Ó

#define FLASH_SPI_CS_GPIO_PORT       GPIOC
#define FLASH_SPI_CS                 GPIO_Pin_0


#define FLASH_SPI_SCK_GPIO_PORT      GPIOA
#define FLASH_SPI_SCK                GPIO_Pin_5

#define FLASH_SPI_MOSI_GPIO_PORT      GPIOA
#define FLASH_SPI_MOSI                GPIO_Pin_7

#define FLASH_SPI_MISO_GPIO_PORT      GPIOA
#define FLASH_SPI_MISO                GPIO_Pin_6


#define CS(a)                 if(a==0)\
                             {GPIO_ResetBits(FLASH_SPI_CS_GPIO_PORT,FLASH_SPI_CS);}\
											 else\
                             {GPIO_SetBits(FLASH_SPI_CS_GPIO_PORT, FLASH_SPI_CS);} 

														 
void SPI_FLASH_Init(void);
uint32_t SPI_ReadID(void);
void Sector_erase(uint32_t addr);
void WaitnoBUSY(void);
void FLASH_ReadData(uint32_t addr,uint8_t*a,uint32_t numreadbyte);
void FLASH_WriteData(uint32_t addr,uint8_t *a,uint32_t numreadbyte);
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
#endif
