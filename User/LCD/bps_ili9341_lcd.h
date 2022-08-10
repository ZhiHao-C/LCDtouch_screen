#ifndef __BPS_ILI9341_LCD_H__
#define __BPS_ILI9341_LCD_H__

#include "stm32f10x.h" 
#include "bps_Font.h" 
/*
一个定义为volatile(__IO)的变量是说这变量可能会被意想不到的改变，这样，编译器就不会去假设这个变量的值了。
优化器在用到这个变量时必须每次都小心地重新读取这个变量的值，而不是使用保存在寄存器里的备份。
*/
#define Data_ADDR          ((__IO uint16_t *)(0x60020000))//A16高电平为数据
#define CMD_ADDR           ((__IO uint16_t *)(0x60000000))//A16高低平为命令

#define      CMD_SetCoordinateX		 		    (0x2A)	     //设置X坐标
#define      CMD_SetCoordinateY		 		    (0x2B)	     //设置Y坐标
#define      CMD_SetPixel		 		        (0x2C)	     //填充像素

#define      ILI9341_DispWindow_X_Star		    0     //起始点的X坐标
#define      ILI9341_DispWindow_Y_Star		    0     //起始点的Y坐标
#define 			ILI9341_LESS_PIXEL	  							240			//液晶屏较短方向的像素宽度
#define 			ILI9341_MORE_PIXEL	 								320			//液晶屏较长方向的像素宽度


/******************************* 定义 ILI934 显示屏常用颜色 ********************************/
#define      BACKGROUND		                WHITE   //默认背景颜色

#define      WHITE		 		           0xFFFF	   //白色
#define      BLACK                         0x0000	   //黑色 
#define      GREY                          0xF7DE	   //灰色 
#define      BLUE                          0x001F	   //蓝色 
#define      BLUE2                         0x051F	   //浅蓝色 
#define      RED                           0xF800	   //红色 
#define      MAGENTA                       0xF81F	   //红紫色，洋红色 
#define      GREEN                         0x07E0	   //绿色 
#define      CYAN                          0x7FFF	   //蓝绿色，青色 
#define      YELLOW                        0xFFE0	   //黄色 
#define      BRED                          0xF81F
#define      GRED                          0xFFE0
#define      GBLUE                         0x07FF

/* 定义 LCD 驱动芯片 ID */
#define     LCDID_UNKNOWN             0
#define     LCDID_ILI9341             0x9341
#define     LCDID_ST7789V             0x8552

/*************************************** 调试预用 ******************************************/
#define      DEBUG_DELAY()


#define FSMC_NE1               FSMC_Bank1_NORSRAM1
//控制引脚
#define ili9341_LCD_CS_CLK     RCC_APB2Periph_GPIOD
#define ili9341_LCD_CS_PORT    GPIOD
#define ili9341_LCD_CS_PIN     GPIO_Pin_7             


#define ili9341_LCD_RST_CLK     RCC_APB2Periph_GPIOE
#define ili9341_LCD_RST_PORT    GPIOE
#define ili9341_LCD_RST_PIN     GPIO_Pin_1             

#define ili9341_LCD_BK_CLK     RCC_APB2Periph_GPIOD
#define ili9341_LCD_BK_PORT    GPIOD
#define ili9341_LCD_BK_PIN     GPIO_Pin_12             

#define ili9341_LCD_RD_CLK     RCC_APB2Periph_GPIOD
#define ili9341_LCD_RD_PORT    GPIOD
#define ili9341_LCD_RD_PIN     GPIO_Pin_4              

#define ili9341_LCD_WE_CLK     RCC_APB2Periph_GPIOD
#define ili9341_LCD_WE_PORT    GPIOD
#define ili9341_LCD_WE_PIN     GPIO_Pin_5             

#define ili9341_LCD_RS_CLK     RCC_APB2Periph_GPIOD
#define ili9341_LCD_RS_PORT    GPIOD
#define ili9341_LCD_RS_PIN     GPIO_Pin_11            



//数据信号线
#define      ili9341_LCD_D0_CLK                RCC_APB2Periph_GPIOD   
#define      ili9341_LCD_D0_PORT               GPIOD
#define      ili9341_LCD_D0_PIN                GPIO_Pin_14

#define      ili9341_LCD_D1_CLK                RCC_APB2Periph_GPIOD   
#define      ili9341_LCD_D1_PORT               GPIOD
#define      ili9341_LCD_D1_PIN                GPIO_Pin_15

#define      ili9341_LCD_D2_CLK                RCC_APB2Periph_GPIOD   
#define      ili9341_LCD_D2_PORT               GPIOD
#define      ili9341_LCD_D2_PIN                GPIO_Pin_0

#define      ili9341_LCD_D3_CLK                RCC_APB2Periph_GPIOD  
#define      ili9341_LCD_D3_PORT               GPIOD
#define      ili9341_LCD_D3_PIN                GPIO_Pin_1

#define      ili9341_LCD_D4_CLK                RCC_APB2Periph_GPIOE   
#define      ili9341_LCD_D4_PORT               GPIOE
#define      ili9341_LCD_D4_PIN                GPIO_Pin_7

#define      ili9341_LCD_D5_CLK                RCC_APB2Periph_GPIOE   
#define      ili9341_LCD_D5_PORT               GPIOE
#define      ili9341_LCD_D5_PIN                GPIO_Pin_8

#define      ili9341_LCD_D6_CLK                RCC_APB2Periph_GPIOE   
#define      ili9341_LCD_D6_PORT               GPIOE
#define      ili9341_LCD_D6_PIN                GPIO_Pin_9

#define      ili9341_LCD_D7_CLK                RCC_APB2Periph_GPIOE  
#define      ili9341_LCD_D7_PORT               GPIOE
#define      ili9341_LCD_D7_PIN                GPIO_Pin_10

#define      ili9341_LCD_D8_CLK                RCC_APB2Periph_GPIOE   
#define      ili9341_LCD_D8_PORT               GPIOE
#define      ili9341_LCD_D8_PIN                GPIO_Pin_11

#define      ili9341_LCD_D9_CLK                RCC_APB2Periph_GPIOE   
#define      ili9341_LCD_D9_PORT               GPIOE
#define      ili9341_LCD_D9_PIN                GPIO_Pin_12

#define      ili9341_LCD_D10_CLK                RCC_APB2Periph_GPIOE   
#define      ili9341_LCD_D10_PORT               GPIOE
#define      ili9341_LCD_D10_PIN                GPIO_Pin_13

#define      ili9341_LCD_D11_CLK                RCC_APB2Periph_GPIOE   
#define      ili9341_LCD_D11_PORT               GPIOE
#define      ili9341_LCD_D11_PIN                GPIO_Pin_14

#define      ili9341_LCD_D12_CLK                RCC_APB2Periph_GPIOE   
#define      ili9341_LCD_D12_PORT               GPIOE
#define      ili9341_LCD_D12_PIN                GPIO_Pin_15

#define      ili9341_LCD_D13_CLK                RCC_APB2Periph_GPIOD   
#define      ili9341_LCD_D13_PORT               GPIOD
#define      ili9341_LCD_D13_PIN                GPIO_Pin_8

#define      ili9341_LCD_D14_CLK                RCC_APB2Periph_GPIOD   
#define      ili9341_LCD_D14_PORT               GPIOD
#define      ili9341_LCD_D14_PIN                GPIO_Pin_9

#define      ili9341_LCD_D15_CLK                RCC_APB2Periph_GPIOD   
#define      ili9341_LCD_D15_PORT               GPIOD
#define      ili9341_LCD_D15_PIN                GPIO_Pin_10

//根据液晶扫描方向而变化的XY像素宽度
//调用ILI9341_GramScan函数设置方向时会自动更改
extern uint16_t LCD_X_LENGTH,LCD_Y_LENGTH; 

//液晶屏扫描模式
//参数可选值为0-7
extern uint8_t LCD_SCAN_MODE;

void GPIO_Config(void);
void ili9341_FSMC_Config(void);
void Init_ILI9341(void);
uint16_t read_Pixel_Format(void);
__inline void ILI9341_Write_Cmd ( uint16_t usCmd );
__inline void ILI9341_Write_Data ( uint16_t usData );
__inline uint16_t ILI9341_Read_Data ( void );
void Backlight_control(uint8_t Switch);
void LCD_REST(void);
void ILI9341_REG_Config ( void );
void LCD_SetBackColor(uint16_t Color);
void LCD_SetTextColor(uint16_t Color);
void LCD_GetColors(uint16_t *TextColor, uint16_t *BackColor);
void LCD_SetColors(uint16_t TextColor, uint16_t BackColor);
void ILI9341_DrawRectangle ( uint16_t usX_Start, uint16_t usY_Start, uint16_t usWidth, uint16_t usHeight, uint8_t ucFilled );
void ILI9341_DrawCircle ( uint16_t usX_Center, uint16_t usY_Center, uint16_t usRadius, uint8_t ucFilled );
void ILI9341_DrawLine ( uint16_t usX1, uint16_t usY1, uint16_t usX2, uint16_t usY2 );
uint16_t ILI9341_GetPointPixel ( uint16_t usX, uint16_t usY );
void ILI9341_SetPointPixel ( uint16_t usX, uint16_t usY);
void ILI9341_Clear ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight );
void open_window(uint16_t X,uint16_t Y,uint16_t width,uint16_t high);
void ILI9341_GramScan ( uint8_t ucOption );
void ILI9341_DispChar_EN ( uint16_t usX, uint16_t usY, const char cChar );
void ILI9341_DispString_EN ( 	uint16_t usX ,uint16_t usY,  char * pStr );
void ILI9341_DispChar_CH ( uint16_t usX, uint16_t usY, uint16_t usChar );
void ILI9341_DispString_EN_CH ( 	uint16_t usX , uint16_t usY, char * pStr );
void LCD_SetFont (sFONT *fonts);
#endif

