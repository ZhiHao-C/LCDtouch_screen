#ifndef __BPS_ILI9341_LCD_H__
#define __BPS_ILI9341_LCD_H__

#include "stm32f10x.h" 
#include "bps_Font.h" 
/*
һ������Ϊvolatile(__IO)�ı�����˵��������ܻᱻ���벻���ĸı䣬�������������Ͳ���ȥ�������������ֵ�ˡ�
�Ż������õ��������ʱ����ÿ�ζ�С�ĵ����¶�ȡ���������ֵ��������ʹ�ñ����ڼĴ�����ı��ݡ�
*/
#define Data_ADDR          ((__IO uint16_t *)(0x60020000))//A16�ߵ�ƽΪ����
#define CMD_ADDR           ((__IO uint16_t *)(0x60000000))//A16�ߵ�ƽΪ����

#define      CMD_SetCoordinateX		 		    (0x2A)	     //����X����
#define      CMD_SetCoordinateY		 		    (0x2B)	     //����Y����
#define      CMD_SetPixel		 		        (0x2C)	     //�������

#define      ILI9341_DispWindow_X_Star		    0     //��ʼ���X����
#define      ILI9341_DispWindow_Y_Star		    0     //��ʼ���Y����
#define 			ILI9341_LESS_PIXEL	  							240			//Һ�����϶̷�������ؿ��
#define 			ILI9341_MORE_PIXEL	 								320			//Һ�����ϳ���������ؿ��


/******************************* ���� ILI934 ��ʾ��������ɫ ********************************/
#define      BACKGROUND		                WHITE   //Ĭ�ϱ�����ɫ

#define      WHITE		 		           0xFFFF	   //��ɫ
#define      BLACK                         0x0000	   //��ɫ 
#define      GREY                          0xF7DE	   //��ɫ 
#define      BLUE                          0x001F	   //��ɫ 
#define      BLUE2                         0x051F	   //ǳ��ɫ 
#define      RED                           0xF800	   //��ɫ 
#define      MAGENTA                       0xF81F	   //����ɫ�����ɫ 
#define      GREEN                         0x07E0	   //��ɫ 
#define      CYAN                          0x7FFF	   //����ɫ����ɫ 
#define      YELLOW                        0xFFE0	   //��ɫ 
#define      BRED                          0xF81F
#define      GRED                          0xFFE0
#define      GBLUE                         0x07FF

/* ���� LCD ����оƬ ID */
#define     LCDID_UNKNOWN             0
#define     LCDID_ILI9341             0x9341
#define     LCDID_ST7789V             0x8552

/*************************************** ����Ԥ�� ******************************************/
#define      DEBUG_DELAY()


#define FSMC_NE1               FSMC_Bank1_NORSRAM1
//��������
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



//�����ź���
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

//����Һ��ɨ�跽����仯��XY���ؿ��
//����ILI9341_GramScan�������÷���ʱ���Զ�����
extern uint16_t LCD_X_LENGTH,LCD_Y_LENGTH; 

//Һ����ɨ��ģʽ
//������ѡֵΪ0-7
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

