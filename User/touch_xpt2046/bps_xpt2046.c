#include "bps_xpt2046.h"
#include "bps_led.h"
#include "bps_Font.h" 
#include "bps_drawing.h"
#include "bps_ili9341_lcd.h"
#include "bps_flash.h"
#include <stdio.h> 
#include <string.h>


//X（显示坐标（读到的AD值））=A1×AD_x+B1×AD_y+C1
//Y（显示坐标（读到的AD值））=A2×AD_x+B2×AD_y+C2

/******************************* 定义 XPT2046 全局变量 ***************************/
//默认触摸参数，不同的屏幕稍有差异，可重新调用触摸校准函数获取
strType_XPT2046_TouchPara strXPT2046_TouchPara[] = { 	
 -0.006464,   -0.073259,  280.358032,    0.074878,    0.002052,   -6.545977,//扫描方式0
	0.086314,    0.001891,  -12.836658,   -0.003722,   -0.065799,  254.715714,//扫描方式1
	0.002782,    0.061522,  -11.595689,    0.083393,    0.005159,  -15.650089,//扫描方式2
	0.089743,   -0.000289,  -20.612209,   -0.001374,    0.064451,  -16.054003,//扫描方式3
	0.000767,   -0.068258,  250.891769,   -0.085559,   -0.000195,  334.747650,//扫描方式4
 -0.084744,    0.000047,  323.163147,   -0.002109,   -0.066371,  260.985809,//扫描方式5
 -0.001848,    0.066984,  -12.807136,   -0.084858,   -0.000805,  333.395386,//扫描方式6
 -0.085470,   -0.000876,  334.023163,   -0.003390,    0.064725,   -6.211169,//扫描方式7
};


volatile uint8_t ucXPT2046_TouchFlag = 0;
/**
  * @brief  用于 XPT2046 的简单微秒级延时函数
  * @param  nCount ：延时计数值，单位为微妙
  * @retval 无
  */	
void Delay_ms( __IO uint32_t ulCount )
{
	uint32_t i;


	for ( i = 0; i < ulCount; i ++ )
	{
		uint8_t uc = 12;     //设置值为12，大约延1微秒  
	      
		while ( uc -- );     //延1微秒	

	}
	
}

//初始化XPT2046液晶控制芯片相关的GPIO
void touch_screen_XPT2046_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//打开时钟
	RCC_APB2PeriphClockCmd (XPT2046_SPI_GPIO_CLK, ENABLE );
	//配置CS
	GPIO_InitStructure.GPIO_Pin=XPT2046_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz ;	  
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_Init(XPT2046_SPI_CS_PORT, &GPIO_InitStructure);
	//CLK
	GPIO_InitStructure.GPIO_Pin=XPT2046_SPI_CLK_PIN;
  GPIO_Init(XPT2046_SPI_CLK_PORT, &GPIO_InitStructure);
	//MOSI
	GPIO_InitStructure.GPIO_Pin=XPT2046_SPI_MOSI_PIN;
  GPIO_Init(XPT2046_SPI_MOSI_PORT, &GPIO_InitStructure);
	//MISO（浮空输入）
	GPIO_InitStructure.GPIO_Pin=XPT2046_SPI_MISO_PIN;  
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  GPIO_Init(XPT2046_SPI_MISO_PORT, &GPIO_InitStructure);
	//PENIRQ浮空输入
	GPIO_InitStructure.GPIO_Pin=XPT2046_PENIRQ_GPIO_PIN;  
	GPIO_Init(XPT2046_PENIRQ_GPIO_PORT, &GPIO_InitStructure);
}

//发送命令
void XPT2046_WriteCMD(uint8_t cmd)
{
	uint8_t i;
	Xpt2046_CS(1);
	Delay_ms(10);
	Xpt2046_CS(0);
	MOSI(0);
	for(i=0;i<8;i++)
	{
		CLK(0);
		if((cmd&0x80)==0)
		{
			MOSI(0);
		}
		else
		{
			MOSI(1);
		}
		cmd=cmd<<1;
		Delay_ms(1);
		CLK(1);
		Delay_ms(5);
		CLK(0);
//		Delay_ms(5);
	}
	
	Delay_ms(1);
}

//接收数据之前还要产生一个时钟
int16_t XPT2046_ReadCMD(void)//读取到的数据为12位
{
	uint8_t i;
	uint16_t Data=0;
//	Delay_ms(10);
	MOSI(0);
	CLK(1);
	CLK(0);
	
	for(i=0;i<12;i++)
	{
		CLK(1);
		Delay_ms(5);
		if(XPT2046_MISO()==1)
		{
			Data|=(0x800>>i);
		}
		CLK(0);
		Delay_ms(5);
	}
	
	Delay_ms(1);
	return Data;
}

//官方读写函数
///**
//  * @brief  XPT2046 的写入命令
//  * @param  ucCmd ：命令
//  *   该参数为以下值之一：
//  *     @arg 0x90 :通道Y+的选择控制字
//  *     @arg 0xd0 :通道X+的选择控制字
//  * @retval 无
//  */
//static void XPT2046_WriteCMD ( uint8_t ucCmd ) 
//{
//	uint8_t i;


//	XPT2046_MOSI_0();
//	
//	XPT2046_CLK_LOW();

//	for ( i = 0; i < 8; i ++ ) 
//	{
//		( ( ucCmd >> ( 7 - i ) ) & 0x01 ) ? XPT2046_MOSI_1() : XPT2046_MOSI_0();
//		
//	  XPT2046_DelayUS ( 5 );
//		
//		XPT2046_CLK_HIGH();

//	  XPT2046_DelayUS ( 5 );

//		XPT2046_CLK_LOW();
//	}
//	
//}


///**
//  * @brief  XPT2046 的读取命令
//  * @param  无
//  * @retval 读取到的数据
//  */
//static uint16_t XPT2046_ReadCMD ( void ) 
//{
//	uint8_t i;
//	uint16_t usBuf=0, usTemp;
//	


//	XPT2046_MOSI_0();

//	XPT2046_CLK_HIGH();

//	for ( i=0;i<12;i++ ) 
//	{
//		XPT2046_CLK_LOW();    
//	
//		usTemp = XPT2046_MISO();
//		
//		usBuf |= usTemp << ( 11 - i );
//	
//		XPT2046_CLK_HIGH();
//		
//	}
//	
//	return usBuf;

//}



//官方读取X的AD值和Y的AD值函数
///**
//  * @brief  对 XPT2046 选择一个模拟通道后，启动ADC，并返回ADC采样结果
//  * @param  ucChannel
//  *   该参数为以下值之一：
//  *     @arg 0x90 :通道Y+的选择控制字
//  *     @arg 0xd0 :通道X+的选择控制字
//  * @retval 该通道的ADC采样结果
//  */
////读取X方向电压值和Y方向电压值来确认X，Y坐标
//static uint16_t XPT2046_ReadAdc ( uint8_t ucChannel )
//{
//	XPT2046_WriteCMD ( ucChannel );

//  return 	XPT2046_ReadCMD ();
//	
//}

///**
//  * @brief  读取 XPT2046 的X通道和Y通道的AD值（12 bit，最大是4096）
//  * @param  sX_Ad ：存放X通道AD值的地址
//  * @param  sY_Ad ：存放Y通道AD值的地址
//  * @retval 无
//  */
//static void XPT2046_ReadAdc_XY ( int16_t * sX_Ad, int16_t * sY_Ad )  
//{ 
//	int16_t sX_Ad_Temp, sY_Ad_Temp; 
//	
//	sX_Ad_Temp = XPT2046_ReadAdc ( XPT2046_CHANNEL_X );

//	Delay_ms ( 1 ); 

//	sY_Ad_Temp = XPT2046_ReadAdc ( XPT2046_CHANNEL_Y ); 
//	
//	
//	* sX_Ad = sX_Ad_Temp; 
//	* sY_Ad = sY_Ad_Temp; 
//	
//	
//}


//触摸检测状态机

uint8_t touch_detect(void)
{
	static uint8_t i;
	uint8_t result;
	static TouchState touch_state=XPT2046_STATE_RELEASE;
	switch(touch_state)
	{
		case XPT2046_STATE_RELEASE:
			if(XPT2046_PENIRQ_Read()==0)
			{
				touch_state=XPT2046_STATE_WAITING;
				result=TOUCH_NOT_PRESSED;
			}
			else
			{
				touch_state=XPT2046_STATE_RELEASE;
				result=TOUCH_NOT_PRESSED;
			}
		break;
			
		case XPT2046_STATE_WAITING:
			if(XPT2046_PENIRQ_Read()==0)
			{
				i++;
				if(i>=3)
				{
					if(XPT2046_PENIRQ_Read()==0)
					{
						touch_state=XPT2046_STATE_PRESSED;
						result=TOUCH_PRESSED;
					}
				}
				else
				{
					touch_state=XPT2046_STATE_WAITING;
					result=TOUCH_NOT_PRESSED;
				}
			}
			else
			{
				touch_state=XPT2046_STATE_RELEASE;
				i=0;
			}
		
		break;
			
		case XPT2046_STATE_PRESSED:
			if(XPT2046_PENIRQ_Read()==0)
			{
				touch_state=XPT2046_STATE_PRESSED;
				result=TOUCH_PRESSED;
			}
			else
			{
				touch_state=XPT2046_STATE_RELEASE;
				result=TOUCH_NOT_PRESSED;
			}
		
		break;
	}
	return result;
}
//读取到的数存到AD_x，AD_y指针中
void GetAD_X_Y(int16_t * AD_x,int16_t * AD_y)
{
	int16_t X=0;
	int16_t Y=0;
	XPT2046_WriteCMD(0xd0);//0xD0为读取X坐标
	Delay_ms(10);
	X=XPT2046_ReadCMD();
	Delay_ms(10);
	XPT2046_WriteCMD(0x90);//0x90为读取Y坐标。
	Delay_ms(10);
	Y=XPT2046_ReadCMD();
	* AD_x=X;
	* AD_y=Y;
}


//主要功能；得到触摸位置的AD值
//触摸时得到的AD值采取十个数据去掉最大和最小值然后求平均值
uint8_t XPT2046_Get_Touch_AD_average( strType_XPT2046_Coordinate * pScreenCoordinate )
{
	uint8_t ucCount = 0, i;
	
	int16_t sAD_X, sAD_Y;
	int16_t sBufferArray [ 2 ] [ 10 ] = { { 0 },{ 0 } };  //坐标X和Y进行多次采样
	
	//存储采样中的最小值、最大值
	int32_t lX_Min, lX_Max, lY_Min, lY_Max;


	/* 循环采样10次 */ 
	do					       				
	{		  
		GetAD_X_Y ( & sAD_X, & sAD_Y );  
		
		sBufferArray [ 0 ] [ ucCount ] = sAD_X;  
		sBufferArray [ 1 ] [ ucCount ] = sAD_Y;
		
		ucCount ++;  
		
	}	while ( ( XPT2046_PENIRQ_Read() == XPT2046_PENIRQ_ActiveLevel ) && ( ucCount < 10 ) );//用户点击触摸屏时即TP_INT_IN信号为低 并且 ucCount<10
	
	
	/*如果触笔弹起*/
	if ( XPT2046_PENIRQ_Read() != XPT2046_PENIRQ_ActiveLevel )
		ucXPT2046_TouchFlag = 0;			//中断标志复位

	
	/*如果成功采样10个样本*/
	if ( ucCount ==10 )		 					
	{
		lX_Max = lX_Min = sBufferArray [ 0 ] [ 0 ];
		lY_Max = lY_Min = sBufferArray [ 1 ] [ 0 ];       
		
		for ( i = 1; i < 10; i ++ )
		{
			if ( sBufferArray[ 0 ] [ i ] < lX_Min )
				lX_Min = sBufferArray [ 0 ] [ i ];
			
			else if ( sBufferArray [ 0 ] [ i ] > lX_Max )
				lX_Max = sBufferArray [ 0 ] [ i ];

		}
		
		for ( i = 1; i < 10; i ++ )
		{
			if ( sBufferArray [ 1 ] [ i ] < lY_Min )
				lY_Min = sBufferArray [ 1 ] [ i ];
			
			else if ( sBufferArray [ 1 ] [ i ] > lY_Max )
				lY_Max = sBufferArray [ 1 ] [ i ];

		}
		
		
		/*去除最小值和最大值之后求平均值*/
		pScreenCoordinate ->x =  ( sBufferArray [ 0 ] [ 0 ] + sBufferArray [ 0 ] [ 1 ] + sBufferArray [ 0 ] [ 2 ] + sBufferArray [ 0 ] [ 3 ] + sBufferArray [ 0 ] [ 4 ] + 
		                           sBufferArray [ 0 ] [ 5 ] + sBufferArray [ 0 ] [ 6 ] + sBufferArray [ 0 ] [ 7 ] + sBufferArray [ 0 ] [ 8 ] + sBufferArray [ 0 ] [ 9 ] - lX_Min-lX_Max ) >> 3;
		
		pScreenCoordinate ->y =  ( sBufferArray [ 1 ] [ 0 ] + sBufferArray [ 1 ] [ 1 ] + sBufferArray [ 1 ] [ 2 ] + sBufferArray [ 1 ] [ 3 ] + sBufferArray [ 1 ] [ 4 ] + 
		                           sBufferArray [ 1 ] [ 5 ] + sBufferArray [ 1 ] [ 6 ] + sBufferArray [ 1 ] [ 7 ] + sBufferArray [ 1 ] [ 8 ] + sBufferArray [ 1 ] [ 9 ] - lY_Min-lY_Max ) >> 3; 
		
		
		return 1;		
		
	}   	
	return 0;    	
}

//两点校准


//三点校准
//关系：
//X = A*x+B*y+C;
//Y = D*x+E*y+F;
//其中X,Y是屏的物理坐标(液晶屏的坐标)，x,y是屏逻辑坐标（触屏的坐标）
/**
  * @brief  计算 XPT2046 触摸坐标校正系数（注意：只有在LCD和触摸屏间的误差角度非常小时,才能运用下面公式）
  * @param  pDisplayCoordinate ：屏幕人为显示的已知坐标
  * @param  pstrScreenSample ：对已知坐标点触摸时 XPT2046 产生的AD坐标
  * @param  pCalibrationFactor ：根据人为设定坐标和采样回来的坐标计算出来的屏幕触摸校正系数
  * @retval 计算状态
	*   该返回值为以下值之一：
  *     @arg 1 :计算成功
  *     @arg 0 :计算失败
  */
uint8_t XPT2046_Calculate_CalibrationFactor ( strType_XPT2046_Coordinate * pDisplayCoordinate, strType_XPT2046_Coordinate * pScreenSample, strType_XPT2046_Calibration * pCalibrationFactor )
{
	uint8_t ucRet = 1;

	/* K＝ ( X0－X2 )  ( Y1－Y2 )－ ( X1－X2 )  ( Y0－Y2 ) */
	pCalibrationFactor -> Divider =  ( ( pScreenSample [ 0 ] .x - pScreenSample [ 2 ] .x ) *  ( pScreenSample [ 1 ] .y - pScreenSample [ 2 ] .y ) ) - 
									                 ( ( pScreenSample [ 1 ] .x - pScreenSample [ 2 ] .x ) *  ( pScreenSample [ 0 ] .y - pScreenSample [ 2 ] .y ) ) ;
	
	
	if (  pCalibrationFactor -> Divider == 0  )
		ucRet = 0;

	else
	{
		/* A＝ (  ( XD0－XD2 )  ( Y1－Y2 )－ ( XD1－XD2 )  ( Y0－Y2 ) )／K	*/
		pCalibrationFactor -> An =  ( ( pDisplayCoordinate [ 0 ] .x - pDisplayCoordinate [ 2 ] .x ) *  ( pScreenSample [ 1 ] .y - pScreenSample [ 2 ] .y ) ) - 
								                ( ( pDisplayCoordinate [ 1 ] .x - pDisplayCoordinate [ 2 ] .x ) *  ( pScreenSample [ 0 ] .y - pScreenSample [ 2 ] .y ) );
		
		/* B＝ (  ( X0－X2 )  ( XD1－XD2 )－ ( XD0－XD2 )  ( X1－X2 ) )／K	*/
		pCalibrationFactor -> Bn =  ( ( pScreenSample [ 0 ] .x - pScreenSample [ 2 ] .x ) *  ( pDisplayCoordinate [ 1 ] .x - pDisplayCoordinate [ 2 ] .x ) ) - 
								                ( ( pDisplayCoordinate [ 0 ] .x - pDisplayCoordinate [ 2 ] .x ) *  ( pScreenSample [ 1 ] .x - pScreenSample [ 2 ] .x ) );
		
		/* C＝ ( Y0 ( X2XD1－X1XD2 )+Y1 ( X0XD2－X2XD0 )+Y2 ( X1XD0－X0XD1 ) )／K */
		pCalibrationFactor -> Cn =  ( pScreenSample [ 2 ] .x * pDisplayCoordinate [ 1 ] .x - pScreenSample [ 1 ] .x * pDisplayCoordinate [ 2 ] .x ) * pScreenSample [ 0 ] .y +
								                ( pScreenSample [ 0 ] .x * pDisplayCoordinate [ 2 ] .x - pScreenSample [ 2 ] .x * pDisplayCoordinate [ 0 ] .x ) * pScreenSample [ 1 ] .y +
								                ( pScreenSample [ 1 ] .x * pDisplayCoordinate [ 0 ] .x - pScreenSample [ 0 ] .x * pDisplayCoordinate [ 1 ] .x ) * pScreenSample [ 2 ] .y ;
		
		/* D＝ (  ( YD0－YD2 )  ( Y1－Y2 )－ ( YD1－YD2 )  ( Y0－Y2 ) )／K	*/
		pCalibrationFactor -> Dn =  ( ( pDisplayCoordinate [ 0 ] .y - pDisplayCoordinate [ 2 ] .y ) *  ( pScreenSample [ 1 ] .y - pScreenSample [ 2 ] .y ) ) - 
								                ( ( pDisplayCoordinate [ 1 ] .y - pDisplayCoordinate [ 2 ] .y ) *  ( pScreenSample [ 0 ] .y - pScreenSample [ 2 ] .y ) ) ;
		
		/* E＝ (  ( X0－X2 )  ( YD1－YD2 )－ ( YD0－YD2 )  ( X1－X2 ) )／K	*/
		pCalibrationFactor -> En =  ( ( pScreenSample [ 0 ] .x - pScreenSample [ 2 ] .x ) *  ( pDisplayCoordinate [ 1 ] .y - pDisplayCoordinate [ 2 ] .y ) ) - 
								                ( ( pDisplayCoordinate [ 0 ] .y - pDisplayCoordinate [ 2 ] .y ) *  ( pScreenSample [ 1 ] .x - pScreenSample [ 2 ] .x ) ) ;
		
		
		/* F＝ ( Y0 ( X2YD1－X1YD2 )+Y1 ( X0YD2－X2YD0 )+Y2 ( X1YD0－X0YD1 ) )／K */
		pCalibrationFactor -> Fn =  ( pScreenSample [ 2 ] .x * pDisplayCoordinate [ 1 ] .y - pScreenSample [ 1 ] .x * pDisplayCoordinate [ 2 ] .y ) * pScreenSample [ 0 ] .y +
								                ( pScreenSample [ 0 ] .x * pDisplayCoordinate [ 2 ] .y - pScreenSample [ 2 ] .x * pDisplayCoordinate [ 0 ] .y ) * pScreenSample [ 1 ] .y +
								                ( pScreenSample [ 1 ] .x * pDisplayCoordinate [ 0 ] .y - pScreenSample [ 0 ] .x * pDisplayCoordinate [ 1 ] .y ) * pScreenSample [ 2 ] .y;
			
	}
	
	
	return ucRet;
	
	
}


/**
  * @brief  XPT2046 触摸屏校准
	* @param	LCD_Mode：指定要校正哪种液晶扫描模式的参数
  * @note  本函数调用后会把液晶模式设置为LCD_Mode
  * @retval 校准结果
	*   该返回值为以下值之一：
  *     @arg 1 :校准成功
  *     @arg 0 :校准失败
  */
//通过这个函数得到逻辑坐标转化为物理坐标的参数
//X = A*x+B*y+C;
//Y = D*x+E*y+F;
uint8_t XPT2046_Touch_Calibrate ( uint8_t LCD_Mode ) 
{

	uint8_t i;
	
	char cStr [ 100 ];
	
	uint16_t usTest_x = 0, usTest_y = 0, usGap_x = 0, usGap_y = 0;
	
	char * pStr = 0;

	strType_XPT2046_Coordinate strCrossCoordinate[4], strScreenSample[4];
	
	strType_XPT2046_Calibration CalibrationFactor;
			
	LCD_SetFont(&Font8x16);
	LCD_SetColors(BLUE,BLACK);

	//设置扫描方向，横屏
	ILI9341_GramScan ( LCD_Mode );
	
	
	/* 设定“十”字交叉点的坐标 */ 
	strCrossCoordinate [0].x = LCD_X_LENGTH >> 2;
	strCrossCoordinate[0].y = LCD_Y_LENGTH >> 2;
	
	strCrossCoordinate[1].x = strCrossCoordinate[0].x;
	strCrossCoordinate[1].y = ( LCD_Y_LENGTH * 3 ) >> 2;
	
	strCrossCoordinate[2].x = ( LCD_X_LENGTH * 3 ) >> 2;
	strCrossCoordinate[2].y = strCrossCoordinate[1].y;
	
	strCrossCoordinate[3].x = strCrossCoordinate[2].x;
	strCrossCoordinate[3].y = strCrossCoordinate[0].y;	
	
	
	for ( i = 0; i < 4; i ++ )
	{ 
		ILI9341_Clear ( 0, 0, LCD_X_LENGTH, LCD_Y_LENGTH );       
		
		pStr = "Touch Calibrate ......";		
		//插入空格，居中显示
		sprintf(cStr,"%*c%s",(LCD_X_LENGTH/(WIDTH_CH_CHAR/2)-strlen(pStr))/2,' ',pStr);	
		ILI9341_DispString_EN_CH(0,152,cStr);			
	
		//插入空格，居中显示
		sprintf ( cStr, "%*c%d",(LCD_X_LENGTH/(WIDTH_CH_CHAR/2)-strlen(pStr))/2,' ',i + 1 );
		ILI9341_DispString_EN_CH(0,152-16*1,cStr);		
	
		Delay_ms ( 300000 );		                     //适当的延时很有必要
		
		cross_curve ( strCrossCoordinate[i] .x, strCrossCoordinate[i].y );  //显示校正用的“十”字
		
		if(i>=1)
		{
			while(XPT2046_PENIRQ_Read()==0);
		}
		while ( ! XPT2046_Get_Touch_AD_average ( & strScreenSample [i] ) );               //读取XPT2046数据到变量pCoordinate，当ptr为空时表示没有触点被按下
		
	}
	
	
	XPT2046_Calculate_CalibrationFactor ( strCrossCoordinate, strScreenSample, & CalibrationFactor ) ;  	 //用原始参数计算出 原始参数与坐标的转换系数

	
	if ( CalibrationFactor.Divider == 0 ) goto Failure;
	
	//前面三个点用于三点校准法计算A1B1C1A2B2C2第四个点得出的AD值基于前面三个点的准度	
	usTest_x = ( ( CalibrationFactor.An * strScreenSample[3].x ) + ( CalibrationFactor.Bn * strScreenSample[3].y ) + CalibrationFactor.Cn ) / CalibrationFactor.Divider;		//取一个点计算X值	 
	usTest_y = ( ( CalibrationFactor.Dn * strScreenSample[3].x ) + ( CalibrationFactor.En * strScreenSample[3].y ) + CalibrationFactor.Fn ) / CalibrationFactor.Divider;    //取一个点计算Y值
		
	usGap_x = ( usTest_x > strCrossCoordinate[3].x ) ? ( usTest_x - strCrossCoordinate[3].x ) : ( strCrossCoordinate[3].x - usTest_x );   //实际X坐标与计算坐标的绝对差
	usGap_y = ( usTest_y > strCrossCoordinate[3].y ) ? ( usTest_y - strCrossCoordinate[3].y ) : ( strCrossCoordinate[3].y - usTest_y );   //实际Y坐标与计算坐标的绝对差
		
	printf("误差usGap_x=%d   误差usGap_y=%d\n",usGap_x,usGap_y);
	if ( ( usGap_x > 15 ) || ( usGap_y > 15 ) ) goto Failure;       //可以通过修改这两个值的大小来调整精度    
	

	/* 校准系数为全局变量 */ 
	strXPT2046_TouchPara[LCD_Mode].dX_X = ( CalibrationFactor.An * 1.0 ) / CalibrationFactor.Divider;
	strXPT2046_TouchPara[LCD_Mode].dX_Y = ( CalibrationFactor.Bn * 1.0 ) / CalibrationFactor.Divider;
	strXPT2046_TouchPara[LCD_Mode].dX   = ( CalibrationFactor.Cn * 1.0 ) / CalibrationFactor.Divider;
	
	strXPT2046_TouchPara[LCD_Mode].dY_X = ( CalibrationFactor.Dn * 1.0 ) / CalibrationFactor.Divider;
	strXPT2046_TouchPara[LCD_Mode].dY_Y = ( CalibrationFactor.En * 1.0 ) / CalibrationFactor.Divider;
	strXPT2046_TouchPara[LCD_Mode].dY   = ( CalibrationFactor.Fn * 1.0 ) / CalibrationFactor.Divider;

	#if 0  //输出调试信息，注意要先初始化串口
		{
					float * ulHeadAddres ;
			/* 打印校校准系数 */ 
			XPT2046_INFO ( "显示模式【%d】校准系数如下：", LCD_Mode);
			
			ulHeadAddres = ( float* ) ( & strXPT2046_TouchPara[LCD_Mode] );
			
			for ( i = 0; i < 6; i ++ )
			{					
				printf ( "%12f,", *ulHeadAddres++  );			
			}	
			printf("\r\n");
		}
	#endif
		
	ILI9341_Clear ( 0, 0, LCD_X_LENGTH, LCD_Y_LENGTH );
	
	LCD_SetTextColor(GREEN);
	
	pStr = "Calibrate Succed";
	//插入空格，居中显示	
	sprintf(cStr,"%*c%s",(LCD_X_LENGTH/(WIDTH_CH_CHAR/2)-strlen(pStr))/2,' ',pStr);		
  ILI9341_DispString_EN_CH(0,152,cStr);	

  Delay_ms ( 1000000 );

	return 1;    
	

Failure:
	
	ILI9341_Clear ( 0, 0, LCD_X_LENGTH, LCD_Y_LENGTH ); 
	
	LCD_SetTextColor(RED);
	
	pStr = "Calibrate fail";	
	//插入空格，居中显示	
	sprintf(cStr,"%*c%s",(LCD_X_LENGTH/(WIDTH_CH_CHAR/2)-strlen(pStr))/2,' ',pStr);		
  ILI9341_DispString_EN_CH(0,152,cStr);	

	pStr = "try again";
	//插入空格，居中显示		
	sprintf(cStr,"%*c%s",(LCD_X_LENGTH/(WIDTH_CH_CHAR/2)-strlen(pStr))/2,' ',pStr);		
  ILI9341_DispString_EN_CH(0,152+16*1,cStr);			

	Delay_ms ( 1000000 );		
	
	return 0; 
		
}

/**
  * @brief  从FLASH中获取 或 重新校正触摸参数（校正后会写入到SPI FLASH中）
  * @note		若FLASH中从未写入过触摸参数，
	*						会触发校正程序校正LCD_Mode指定模式的触摸参数，此时其它模式写入默认值
  *
	*					若FLASH中已有触摸参数，且不强制重新校正
	*						会直接使用FLASH里的触摸参数值
  *
	*					每次校正时只会更新指定的LCD_Mode模式的触摸参数，其它模式的不变
  * @note  本函数调用后会把液晶模式设置为LCD_Mode
  *
	* @param  LCD_Mode:要校正触摸参数的液晶模式
	* @param  forceCal:是否强制重新校正参数，可以为以下值：
	*		@arg 1：强制重新校正
	*		@arg 0：只有当FLASH中不存在触摸参数标志时才重新校正
  * @retval 无
  */	
void Calibrate_or_Get_TouchParaWithFlash(uint8_t LCD_Mode,uint8_t forceCal)
{
	uint8_t para_flag=0;
	uint8_t temp=0;
	
	//初始化FLASH
	SPI_FLASH_Init();
	
	//读取触摸参数标志
	FLASH_ReadData(FLASH_TOUCH_PARA_FLAG_ADDR,&para_flag,1);
	printf("初始flash中的para_flag值为：%d\n",para_flag);

	//若不存在标志或florceCal=1时，重新校正参数
	if(para_flag != FLASH_TOUCH_PARA_FLAG_VALUE || forceCal ==1)
	{ 		
		//若标志存在，说明原本FLASH内有触摸参数，
		//先读回所有LCD模式的参数值，以便稍后强制更新时只更新指定LCD模式的参数,其它模式的不变
		if(  para_flag == FLASH_TOUCH_PARA_FLAG_VALUE && forceCal == 1)
		{
			FLASH_ReadData(FLASH_TOUCH_PARA_ADDR,(uint8_t *)&strXPT2046_TouchPara,4*6*8);	
		}
		
		//等待触摸屏校正完毕,更新指定LCD模式的触摸参数值
		while( ! XPT2046_Touch_Calibrate (LCD_Mode) );     

		//擦除扇区
		Sector_erase(0);
		WaitnoBUSY();
		//设置触摸参数标志
		para_flag = FLASH_TOUCH_PARA_FLAG_VALUE;
		printf("给para_flag赋值为：%d\n",para_flag);
		//写入触摸参数标志
		FLASH_WriteData(FLASH_TOUCH_PARA_FLAG_ADDR,&para_flag,1);
		WaitnoBUSY();
		FLASH_ReadData(FLASH_TOUCH_PARA_FLAG_ADDR,&temp,1);
		Delay_ms(5);
		printf("写入flash中的para_flag值为：%d\n",temp);
		//写入最新的触摸参数
		SPI_FLASH_BufferWrite((uint8_t *)&strXPT2046_TouchPara,FLASH_TOUCH_PARA_ADDR,4*6*8);
		Delay_ms(5);
 
	}
	else	//若标志存在且不强制校正，则直接从FLASH中读取
	{
		FLASH_ReadData(FLASH_TOUCH_PARA_ADDR,(uint8_t *)&strXPT2046_TouchPara,4*6*8);	 

			#if 0	//输出调试信息，注意要初始化串口
				{
					
					uint8_t para_flag=0,i;
					float *ulHeadAddres  ;
					
					/* 打印校校准系数 */ 
					XPT2046_INFO ( "从FLASH里读取得的校准系数如下：" );
					
					ulHeadAddres = ( float* ) ( & strXPT2046_TouchPara );

					for ( i = 0; i < 6*8; i ++ )
					{				
						if(i%6==0)
							printf("\r\n");			
									
						printf ( "%12f,", *ulHeadAddres );
						ulHeadAddres++;				
					}
					printf("\r\n");
				}
			#endif
	}
	

}
   
/**
  * @brief  获取 XPT2046 触摸点（校准后）的坐标
  * @param  pDisplayCoordinate ：该指针存放获取到的触摸点坐标
  * @param  pTouchPara：坐标校准系数
  * @retval 获取情况
	*   该返回值为以下值之一：
  *     @arg 1 :获取成功
  *     @arg 0 :获取失败
  */
uint8_t XPT2046_Get_TouchedPoint ( strType_XPT2046_Coordinate * pDisplayCoordinate, strType_XPT2046_TouchPara * pTouchPara )
{
	uint8_t ucRet = 1;           //若正常，则返回0
	
	strType_XPT2046_Coordinate strScreenCoordinate; 
	

  if ( XPT2046_Get_Touch_AD_average ( & strScreenCoordinate ) )
  {    
		pDisplayCoordinate ->x = ( ( pTouchPara[LCD_SCAN_MODE].dX_X * strScreenCoordinate.x ) + ( pTouchPara[LCD_SCAN_MODE].dX_Y * strScreenCoordinate.y ) + pTouchPara[LCD_SCAN_MODE].dX );        
		pDisplayCoordinate ->y = ( ( pTouchPara[LCD_SCAN_MODE].dY_X * strScreenCoordinate.x ) + ( pTouchPara[LCD_SCAN_MODE].dY_Y * strScreenCoordinate.y ) + pTouchPara[LCD_SCAN_MODE].dY );

  }
	 
	else ucRet = 0;            //如果获取的触点信息有误，则返回0
		
  return ucRet;
} 




/**
  * @brief   触摸屏被按下的时候会调用本函数
  * @param  touch包含触摸坐标的结构体
  * @note  请在本函数中编写自己的触摸按下处理应用
  * @retval 无
  */
void XPT2046_TouchDown(strType_XPT2046_Coordinate * touch)
{
	//若为负值表示之前已处理过
	if(touch->pre_x == -1 && touch->pre_x == -1)
		return;
	
	/***在此处编写自己的触摸按下处理应用***/
  
	/*处理触摸画板的选择按钮*/
  Touch_Button_Down(touch->x,touch->y);
  
  /*处理描绘轨迹*/
  Draw_Trail(touch->pre_x,touch->pre_y,touch->x,touch->y,&brush);
	
	/***在上面编写自己的触摸按下处理应用***/
	
	
}

/**
  * @brief   触摸屏释放的时候会调用本函数
  * @param  touch包含触摸坐标的结构体
  * @note  请在本函数中编写自己的触摸释放处理应用
  * @retval 无
  */
void XPT2046_TouchUp(strType_XPT2046_Coordinate * touch) 
{
	
	//若为负值表示之前已处理过
	if(touch->pre_x == -1 && touch->pre_x == -1)
		return;
		
	/***在此处编写自己的触摸释放处理应用***/
  
	/*处理触摸画板的选择按钮*/
  Touch_Button_Up(touch->pre_x,touch->pre_y);	
	
	/***在上面编写自己的触摸释放处理应用***/
}









/**
	* @brief   检测到触摸中断时调用的处理函数,通过它调用tp_down 和tp_up汇报触摸点
	*	@note 	 本函数需要在while循环里被调用，也可使用定时器定时调用
	*			例如，可以每隔5ms调用一次，消抖阈值宏DURIATION_TIME可设置为2，这样每秒最多可以检测100个点。
	*						可在XPT2046_TouchDown及XPT2046_TouchUp函数中编写自己的触摸应用
	* @param   none
	* @retval  none
	*/
void XPT2046_TouchEvenHandler(void )
{
	  static strType_XPT2046_Coordinate cinfo={-1,-1,-1,-1};
	
		if(touch_detect() == TOUCH_PRESSED)
		{
			led_B(1);
			
			//获取触摸坐标
			XPT2046_Get_TouchedPoint(&cinfo,strXPT2046_TouchPara);
			
			//调用触摸被按下时的处理函数，可在该函数编写自己的触摸按下处理过程
			XPT2046_TouchDown(&cinfo);


			/*更新触摸信息到pre xy*/
			cinfo.pre_x = cinfo.x; cinfo.pre_y = cinfo.y;  
		}
		else
		{
			led_B(0);
			
			//调用触摸被释放时的处理函数，可在该函数编写自己的触摸释放处理过程
			XPT2046_TouchUp(&cinfo); 
			
			/*触笔释放，把 xy 重置为负*/
			cinfo.x = -1;
			cinfo.y = -1; 
			cinfo.pre_x = -1;
			cinfo.pre_y = -1;
		}

}

