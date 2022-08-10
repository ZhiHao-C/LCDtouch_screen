#ifndef __BPS_FONT_H__
#define __BPS_FONT_H__

#include "stm32f10x.h" 


#define      WIDTH_CH_CHAR		                16	    //�����ַ���� 
#define      HEIGHT_CH_CHAR		              	16		  //�����ַ��߶�

typedef struct _tFont
{    
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
  
} sFONT;

//0��ʾʹ��SD����ģ�������ʾFLASH��ģ,����SD����ģ���ļ�ϵͳ���ٶ����ܶࡣ

#define Font_source_FLASH_or_SD 						1

#if Font_source_FLASH_or_SD
	/*ʹ��FLASH��ģ*/
	/*�����ֿ�洢��FLASH����ʼ��ַ*/
	/*FLASH*/
	#define GBKCODE_START_ADDRESS   387*4096


	/*��ȡ�ֿ�ĺ���*/
	//�����ȡ�����ַ���ģ����ĺ�������ucBufferΪ�����ģ��������usCharΪ�����ַ��������룩
	#define      GetGBKCode( ucBuffer, usChar )  GetGBKCode_from_EXFlash( ucBuffer, usChar )  
	int GetGBKCode_from_EXFlash( uint8_t * pBuffer, uint16_t c);

#else
	/*ʹ��SD��ģ*/


	/*SD����ģ·��*/
	#define GBKCODE_FILE_NAME			"0:/Font/GB2312_H1616.FON"


	/*��ȡ�ֿ�ĺ���*/
	//�����ȡ�����ַ���ģ����ĺ�������ucBufferΪ�����ģ��������usCharΪ�����ַ��������룩

	#define GetGBKCode( ucBuffer, usChar )  GetGBKCode_from_sd( ucBuffer, usChar )
	int GetGBKCode_from_sd ( uint8_t * pBuffer, uint16_t c);

#endif



extern sFONT Font8x16;
extern sFONT Font24x32;
extern sFONT Font16x24;


#endif
