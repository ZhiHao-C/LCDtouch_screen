#ifndef __BPS_FONT_H__
#define __BPS_FONT_H__

#include "stm32f10x.h" 


#define      WIDTH_CH_CHAR		                16	    //中文字符宽度 
#define      HEIGHT_CH_CHAR		              	16		  //中文字符高度

typedef struct _tFont
{    
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
  
} sFONT;

//0表示使用SD卡字模，非零表示FLASH字模,由于SD卡字模有文件系统，速度慢很多。

#define Font_source_FLASH_or_SD 						1

#if Font_source_FLASH_or_SD
	/*使用FLASH字模*/
	/*中文字库存储在FLASH的起始地址*/
	/*FLASH*/
	#define GBKCODE_START_ADDRESS   387*4096


	/*获取字库的函数*/
	//定义获取中文字符字模数组的函数名，ucBuffer为存放字模数组名，usChar为中文字符（国标码）
	#define      GetGBKCode( ucBuffer, usChar )  GetGBKCode_from_EXFlash( ucBuffer, usChar )  
	int GetGBKCode_from_EXFlash( uint8_t * pBuffer, uint16_t c);

#else
	/*使用SD字模*/


	/*SD卡字模路径*/
	#define GBKCODE_FILE_NAME			"0:/Font/GB2312_H1616.FON"


	/*获取字库的函数*/
	//定义获取中文字符字模数组的函数名，ucBuffer为存放字模数组名，usChar为中文字符（国标码）

	#define GetGBKCode( ucBuffer, usChar )  GetGBKCode_from_sd( ucBuffer, usChar )
	int GetGBKCode_from_sd ( uint8_t * pBuffer, uint16_t c);

#endif



extern sFONT Font8x16;
extern sFONT Font24x32;
extern sFONT Font16x24;


#endif
