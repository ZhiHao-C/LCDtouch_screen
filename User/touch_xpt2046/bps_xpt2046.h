#ifndef __BPS_XPT2046_H__
#define __BPS_XPT2046_H__

#include "stm32f10x.h" 

/******************************* XPT2046 触摸屏触摸信号指示引脚定义(不使用中断) ***************************/
#define             XPT2046_PENIRQ_GPIO_CLK                        RCC_APB2Periph_GPIOE   
#define             XPT2046_PENIRQ_GPIO_PORT                       GPIOE
#define             XPT2046_PENIRQ_GPIO_PIN                        GPIO_Pin_4

//触屏信号有效电平
#define             XPT2046_PENIRQ_ActiveLevel                     0
#define             XPT2046_PENIRQ_Read()                          GPIO_ReadInputDataBit ( XPT2046_PENIRQ_GPIO_PORT, XPT2046_PENIRQ_GPIO_PIN )

/******触摸状态机相关******/
typedef enum
{
	XPT2046_STATE_RELEASE  = 0,	//触摸释放
	XPT2046_STATE_WAITING,			//触摸按下
	XPT2046_STATE_PRESSED,			//触摸按下
}TouchState	;

#define TOUCH_PRESSED 				1
#define TOUCH_NOT_PRESSED			0

#define	            XPT2046_CHANNEL_X 	                          0x90 	          //通道Y+的选择控制字	
#define	            XPT2046_CHANNEL_Y 	                          0xd0	          //通道X+的选择控制字
//触摸参数写到FLASH里的标志
#define							FLASH_TOUCH_PARA_FLAG_VALUE					0xA5

//触摸标志写到FLASH里的地址
#define 							FLASH_TOUCH_PARA_FLAG_ADDR						(1*1024)

//触摸参数写到FLASH里的地址
#define 							FLASH_TOUCH_PARA_ADDR									(2*1024)
/******************************* XPT2046 触摸屏模拟SPI引脚定义 ***************************/
#define             XPT2046_SPI_GPIO_CLK                         RCC_APB2Periph_GPIOE| RCC_APB2Periph_GPIOD

#define             XPT2046_SPI_CS_PIN		                      GPIO_Pin_13
#define             XPT2046_SPI_CS_PORT		                      GPIOD

#define	            XPT2046_SPI_CLK_PIN	                        GPIO_Pin_0
#define             XPT2046_SPI_CLK_PORT	                      GPIOE

#define	            XPT2046_SPI_MOSI_PIN	                      GPIO_Pin_2
#define	            XPT2046_SPI_MOSI_PORT	                      GPIOE

#define	            XPT2046_SPI_MISO_PIN	                      GPIO_Pin_3
#define	            XPT2046_SPI_MISO_PORT	                      GPIOE


//#define             XPT2046_CS_ENABLE()              GPIO_SetBits ( XPT2046_SPI_CS_PORT, XPT2046_SPI_CS_PIN )    
//#define             XPT2046_CS_DISABLE()             GPIO_ResetBits ( XPT2046_SPI_CS_PORT, XPT2046_SPI_CS_PIN )  

//#define             XPT2046_CLK_HIGH()               GPIO_SetBits ( XPT2046_SPI_CLK_PORT, XPT2046_SPI_CLK_PIN )    
//#define             XPT2046_CLK_LOW()                GPIO_ResetBits ( XPT2046_SPI_CLK_PORT, XPT2046_SPI_CLK_PIN ) 

//#define             XPT2046_MOSI_1()                 GPIO_SetBits ( XPT2046_SPI_MOSI_PORT, XPT2046_SPI_MOSI_PIN ) 
//#define             XPT2046_MOSI_0()                 GPIO_ResetBits ( XPT2046_SPI_MOSI_PORT, XPT2046_SPI_MOSI_PIN )

#define             XPT2046_MISO()                   GPIO_ReadInputDataBit ( XPT2046_SPI_MISO_PORT, XPT2046_SPI_MISO_PIN )


#define Xpt2046_CS(a)        if(a)\
                             {GPIO_SetBits(XPT2046_SPI_CS_PORT,XPT2046_SPI_CS_PIN);}\
											 else\
                             {GPIO_ResetBits(XPT2046_SPI_CS_PORT, XPT2046_SPI_CS_PIN);} 	

#define CLK(a)        if(a)\
                             {GPIO_SetBits(XPT2046_SPI_CLK_PORT,XPT2046_SPI_CLK_PIN);}\
											 else\
                             {GPIO_ResetBits(XPT2046_SPI_CLK_PORT, XPT2046_SPI_CLK_PIN);} 			

#define MOSI(a)        if(a)\
                             {GPIO_SetBits(XPT2046_SPI_MOSI_PORT,XPT2046_SPI_MOSI_PIN);}\
											 else\
                             {GPIO_ResetBits(XPT2046_SPI_MOSI_PORT, XPT2046_SPI_MOSI_PIN);} 																			 

/******************************* 声明 XPT2046 相关的数据类型 ***************************/
typedef	struct          //液晶坐标结构体 
{		
	/*负数值表示无新数据*/
   int16_t x;			//记录最新的触摸参数值
   int16_t y; 
	
	/*用于记录连续触摸时(长按)的上一次触摸位置*/
	 int16_t pre_x;		
   int16_t pre_y;
	
} strType_XPT2046_Coordinate;   

typedef struct         //校准因子结构体 
{
	 float An,  		 //注:sizeof(long double) = 8
					Bn,     
					Cn,   
					Dn,    
					En,    
					Fn,     
					Divider;
	
} strType_XPT2046_Calibration;

														 
typedef struct         //校准系数结构体（最终使用）
{
	 float dX_X,  	//A1		 
					dX_Y,   //B1	
					dX,     //C1	
					dY_X,   //A2	
					dY_Y,   //B2	
					dY;     //C2	

} strType_XPT2046_TouchPara;														 
														 
														 
void touch_screen_XPT2046_Init(void);
void XPT2046_WriteCMD(uint8_t cmd);														 
int16_t XPT2046_ReadCMD(void);
uint8_t touch_detect(void);
void GetAD_X_Y(int16_t * AD_x,int16_t * AD_y);										 
uint8_t XPT2046_Get_Touch_AD_average( strType_XPT2046_Coordinate * pScreenCoordinate );
uint8_t XPT2046_Calculate_CalibrationFactor ( strType_XPT2046_Coordinate * pDisplayCoordinate, strType_XPT2046_Coordinate * pScreenSample, strType_XPT2046_Calibration * pCalibrationFactor );
uint8_t XPT2046_Touch_Calibrate ( uint8_t LCD_Mode ) ;
void Calibrate_or_Get_TouchParaWithFlash(uint8_t LCD_Mode,uint8_t forceCal);
void XPT2046_TouchEvenHandler(void );
void XPT2046_TouchDown(strType_XPT2046_Coordinate * touch);
void XPT2046_TouchUp(strType_XPT2046_Coordinate * touch);
#endif
