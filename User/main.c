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
	Calibrate_or_Get_TouchParaWithFlash(6,0);//����У׼����������flash��
	ILI9341_GramScan (6);	
	
	//���ƴ����������
	Palette_Init(LCD_SCAN_MODE);

// //���Խ�ADֵת��Ϊʵ���������
//	LCD_SetBackColor(BACKGROUND);
//	ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);
//	LCD_SetTextColor(BLACK);
//	cross_curve(120,160);
//	cross_curve(100,200);
//	cross_curve(50,50);
	
	printf("\r\n ********** ����������� *********** \r\n"); 

	while(1)
	{
		XPT2046_TouchEvenHandler();
	}

		
		
}
	
	
	
	
//	/*ҪʹFSMC_A��16��Ϊ�ߵ�ƽ��������16λ��������һλ����ΪLCDһ����ַ�Ƕ�Ӧ�����ֽڶ�HADDR
//	��һ����ַ��Ӧһ���ֽ����Զ��䷽ʽΪHADDR[25:1]��FSMC_A[24:0]��Ӧ������HADDR[0]δ�� ��temp
//	�൱����HADDR[25:0]д����Ȼ����ת�Ƶ�FSMC_A[24:0]��Ϊ������HADDR[1]��FSMC_A[0]���Ի�Ҫ����һλ
//	*/
//	temp|=(0x01<<(16+1));
//	//A16λΪ�ߵ�ƽ��ʾ����
//	printf("A16Ϊ�ߵ�ƽ=%x\n",temp);
//	temp=0x60000000;
//	temp&=~(0x01<<(16+1));
//	//A16λΪ�͵�ƽ��ʾ����
//	printf("A16Ϊ�͵�ƽ=%x\n",temp);
//	//��������
//	*CMD_ADDR=0xABCD;
//	//��������
//	*Data_ADDR=0x1234;
//	//A16λΪ�ߵ�ƽ��ȡ����
//	read_data=*Data_ADDR;
	
	
	
	

