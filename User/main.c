#include "stm32f10x.h"                  // Device header
#include "bps_led.h"
#include "bps_usart.h"
#include "bps_ili9341_lcd.h"
#include "bps_Font.h"
#include "bps_xpt2046.h"
#include "bps_drawing.h"
#include <stdio.h>
#include <string.h>






extern void delay_ms(unsigned int n);
extern void Delay_ms( __IO uint32_t ulCount );




int main()
{
	
	LED_B_GPIO_Config();
	led_B(0);
	Init_ILI9341();
	USART_Config();
	touch_screen_XPT2046_Init();
	Calibrate_or_Get_TouchParaWithFlash(6,0);//计算校准参数并存入flash中
	ILI9341_GramScan (6);	
	
	//绘制触摸画板界面
	Palette_Init(LCD_SCAN_MODE);

// //测试将AD值转化为实际坐标误差
//	LCD_SetBackColor(BACKGROUND);
//	ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);
//	LCD_SetTextColor(BLACK);
//	cross_curve(120,160);
//	cross_curve(100,200);
//	cross_curve(50,50);
	
	printf("\r\n ********** 触摸画板程序 *********** \r\n"); 

	while(1)
	{
		XPT2046_TouchEvenHandler();
	}

		
		
}
	
	
	
	
//	/*要使FSMC_A【16】为高电平，先左移16位，再左移一位是因为LCD一个地址是对应两个字节而HADDR
//	是一个地址对应一个字节所以对其方式为HADDR[25:1]与FSMC_A[24:0]对应相连，HADDR[0]未接 ，temp
//	相当于往HADDR[25:0]写数据然后再转移到FSMC_A[24:0]因为现在是HADDR[1]接FSMC_A[0]所以还要左移一位
//	*/
//	temp|=(0x01<<(16+1));
//	//A16位为高电平表示数据
//	printf("A16为高电平=%x\n",temp);
//	temp=0x60000000;
//	temp&=~(0x01<<(16+1));
//	//A16位为低电平表示命令
//	printf("A16为低电平=%x\n",temp);
//	//发送命令
//	*CMD_ADDR=0xABCD;
//	//发送数据
//	*Data_ADDR=0x1234;
//	//A16位为高电平读取数据
//	read_data=*Data_ADDR;
	
	
	
	

