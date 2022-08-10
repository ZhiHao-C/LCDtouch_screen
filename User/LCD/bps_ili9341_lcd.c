#include "bps_Font.h" 
#include "bps_ili9341_lcd.h"


uint16_t lcdid = 0;

static sFONT *LCD_Currentfonts = &Font8x16;  //Ӣ������
static uint16_t CurrentTextColor   = BLACK;//ǰ��ɫ
static uint16_t CurrentBackColor   = WHITE;//����ɫ
//����Һ��ɨ�跽����仯��XY���ؿ��
//����ILI9341_GramScan�������÷���ʱ���Զ�����
uint16_t LCD_X_LENGTH = ILI9341_LESS_PIXEL;
uint16_t LCD_Y_LENGTH = ILI9341_MORE_PIXEL;

//Һ����ɨ��ģʽ����������Ҫ���ڷ���ѡ�������ļ������
//������ѡֵΪ0-7
//����ILI9341_GramScan�������÷���ʱ���Զ�����
//LCD�ճ�ʼ�����ʱ��ʹ�ñ�Ĭ��ֵ
uint8_t LCD_SCAN_MODE = 6;


void ILI9341_Write_Cmd ( uint16_t usCmd );
void ILI9341_Write_Data ( uint16_t usData );
uint16_t ILI9341_Read_Data ( void );

void delay_ms(unsigned int n)//��ʱ����
{
	int i;
	while (n--)
	{
		for(i=0;i<2500;i++);
	}
}


static void ILI9341_Delay ( __IO uint32_t nCount )
{
  for ( ; nCount != 0; nCount -- );
	
}



void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE,ENABLE);
	//����BK
	GPIO_InitStructure.GPIO_Pin = ili9341_LCD_BK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ili9341_LCD_BK_PORT, &GPIO_InitStructure);
	
	//����RET
	GPIO_InitStructure.GPIO_Pin = ili9341_LCD_RST_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ili9341_LCD_RST_PORT, &GPIO_InitStructure);
	
	//����ʣ�µ�����
	GPIO_InitStructure.GPIO_Pin = ili9341_LCD_CS_PIN|ili9341_LCD_RD_PIN|ili9341_LCD_WE_PIN|ili9341_LCD_RS_PIN|
	ili9341_LCD_D0_PIN|ili9341_LCD_D1_PIN|ili9341_LCD_D2_PIN|ili9341_LCD_D3_PIN|ili9341_LCD_D13_PIN|ili9341_LCD_D14_PIN|
	ili9341_LCD_D15_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = ili9341_LCD_D4_PIN|ili9341_LCD_D5_PIN|ili9341_LCD_D6_PIN|ili9341_LCD_D7_PIN|
	ili9341_LCD_D8_PIN|ili9341_LCD_D9_PIN|ili9341_LCD_D10_PIN|ili9341_LCD_D11_PIN|ili9341_LCD_D12_PIN;
	
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void ili9341_FSMC_Config(void)
{
	
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming;
	
	/* ʹ��FSMCʱ��*/
	RCC_AHBPeriphClockCmd (RCC_AHBPeriph_FSMC, ENABLE);

	//��ַ����ʱ�䣨ADDSET��Ϊ1��HCLK 2/72M=28ns
	readWriteTiming.FSMC_AddressSetupTime      = 0x01;	 //��ַ����ʱ��
	//���ݱ���ʱ�䣨DATAST��+ 1��HCLK = 5/72M=70ns	
	readWriteTiming.FSMC_DataSetupTime         = 0x04;	 //���ݽ���ʱ��
	//ѡ����Ƶ�ģʽ
	//ģʽB,�첽NOR FLASHģʽ����ILI9341��8080ʱ��ƥ��
	readWriteTiming.FSMC_AccessMode            = FSMC_AccessMode_B;	
	
	/*����������ģʽB�޹�*/
	//��ַ����ʱ�䣨ADDHLD��ģʽAδ�õ�
	readWriteTiming.FSMC_AddressHoldTime       = 0x00;	 //��ַ����ʱ��
	//��������ת�����ڣ������ڸ���ģʽ��NOR����
	readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
	//����ʱ�ӷ�Ƶ��������ͬ�����͵Ĵ洢��
	readWriteTiming.FSMC_CLKDivision           = 0x00;
	//���ݱ���ʱ�䣬������ͬ���͵�NOR	
	readWriteTiming.FSMC_DataLatency           = 0x00;	

	
	FSMC_NORSRAMInitStructure.FSMC_Bank                  = FSMC_NE1;
	//���õ�ַ���������������Ƿ��ã�������NOR
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux        = FSMC_DataAddressMux_Disable;
	//����Ҫ���ƵĴ洢������
	FSMC_NORSRAMInitStructure.FSMC_MemoryType            = FSMC_MemoryType_NOR;
	//�洢�����ݿ�ȣ�16λ
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;
	//�����Ƿ�ʹ��ͻ������ģʽ��������ͬ�����͵Ĵ洢��
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;
	//���õȴ��źŵ���Ч���ԣ�������ͬ�����͵Ĵ洢��
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
	//�����Ƿ�ʹ�ܵȴ��źţ�������ͬ�����͵Ĵ洢��
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable;
	//�����Ƿ�֧�ְѷǶ����ͻ��������������ͬ�����͵Ĵ洢��
	FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;
	//���õȴ��źŲ����ʱ�䣬������ͬ�����͵Ĵ洢��
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;
	//�洢��дʹ��
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;
	//��ʹ�õȴ��ź�
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;
	//��չģʽ
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Disable;
	//ͻ��д����
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable;
	
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;
	
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &readWriteTiming;  
	
	FSMC_NORSRAMInit ( & FSMC_NORSRAMInitStructure ); //��ʼ��FSMC����
	
	/* ʹ�� FSMC_Bank1_NORSRAM4 */
	FSMC_NORSRAMCmd ( FSMC_NE1, ENABLE );  

}

__inline void ILI9341_Write_Cmd ( uint16_t usCmd )
{
	* ( CMD_ADDR ) = usCmd;
	
}

__inline  void ILI9341_Write_Data ( uint16_t usData )
{
	*  ( Data_ADDR ) = usData;
	
}

__inline  uint16_t ILI9341_Read_Data ( void )
{
	return ( * Data_ADDR );
	
}


//����ƿ��ƣ�1��0�رգ�
void Backlight_control(uint8_t Switch)
{
	if(Switch>=1)
	{
		GPIO_ResetBits(ili9341_LCD_BK_PORT,ili9341_LCD_BK_PIN);
	}
	else
	{
		GPIO_SetBits(ili9341_LCD_BK_PORT,ili9341_LCD_BK_PIN);
	}
}
//��λ
void LCD_REST(void)
{
	GPIO_ResetBits(ili9341_LCD_RST_PORT,ili9341_LCD_RST_PIN);//�͵�ƽ��λ
	delay_ms(1); 
	GPIO_SetBits(ili9341_LCD_RST_PORT,ili9341_LCD_RST_PIN);
	delay_ms(1); 
}




//��ȡ���ظ�ʽ
uint16_t read_Pixel_Format(void)
{
	ILI9341_Write_Cmd(0x0c);
	ILI9341_Read_Data();
	return ILI9341_Read_Data();
}

/**
 * @brief  ��ȡLCD����оƬID�����������ڲ��Եײ�Ķ�д����
 * @param  ��
 * @retval ����ʱ����ֵΪLCD����оƬID: LCDID_ILI9341/LCDID_ST7789V
 *         ���򷵻�: LCDID_UNKNOWN
 */
uint16_t ILI9341_ReadID(void)
{
	uint16_t id = 0;
	
	ILI9341_Write_Cmd(0x04);
	ILI9341_Read_Data();
	ILI9341_Read_Data();
	id = ILI9341_Read_Data();
	id <<= 8;
	id |= ILI9341_Read_Data();
	
  if(id == LCDID_ST7789V)
  {
    return id;
  }
  else
  {
    ILI9341_Write_Cmd(0xD3);
    ILI9341_Read_Data();
    ILI9341_Read_Data();
    id = ILI9341_Read_Data();
    id <<= 8;
    id |= ILI9341_Read_Data();
    if(id == LCDID_ILI9341)
    {
      return id;
    }
  }
  
	return 0;
}


void ILI9341_REG_Config ( void )
{
	lcdid=ILI9341_ReadID();
	if(lcdid==LCDID_ILI9341)
	{
		/*  Power control B (CFh)  */
		delay_ms  (5);
		ILI9341_Write_Cmd ( 0xCF  );
		ILI9341_Write_Data ( 0x00  );
		ILI9341_Write_Data ( 0x81  );
		ILI9341_Write_Data ( 0x30  );//

		/*  Power on sequence control (EDh) */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0xED );
		ILI9341_Write_Data ( 0x64 );
		ILI9341_Write_Data ( 0x03 );
		ILI9341_Write_Data ( 0x12 );
		ILI9341_Write_Data ( 0x81 );//

		/*  Driver timing control A (E8h) */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0xE8 );
		ILI9341_Write_Data ( 0x85 );
		ILI9341_Write_Data ( 0x10 );
		ILI9341_Write_Data ( 0x78 );

		/*  Power control A (CBh) */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0xCB );
		ILI9341_Write_Data ( 0x39 );
		ILI9341_Write_Data ( 0x2C );
		ILI9341_Write_Data ( 0x00 );
		ILI9341_Write_Data ( 0x34 );
		//ILI9341_Write_Data ( 0x02 );
		ILI9341_Write_Data ( 0x06 ); //ԭ����0x02��Ϊ0x06�ɷ�ֹҺ����ʾ����ʱ�����Ƶ����

		/* Pump ratio control (F7h) */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0xF7 );
		ILI9341_Write_Data ( 0x20 );

		/* Driver timing control B */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0xEA );
		ILI9341_Write_Data ( 0x00 );
		ILI9341_Write_Data ( 0x00 );

		/* Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0xB1 );
		ILI9341_Write_Data ( 0x00 );
		ILI9341_Write_Data ( 0x1B );

		/*  Display Function Control (B6h) */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0xB6 );
		ILI9341_Write_Data ( 0x0A );
		ILI9341_Write_Data ( 0xA2 );

		/* Power Control 1 (C0h) */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0xC0 );
		ILI9341_Write_Data ( 0x35 );

		/* Power Control 2 (C1h) */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0xC1 );
		ILI9341_Write_Data ( 0x11 );

		/* VCOM Control 1 (C5h) */
		ILI9341_Write_Cmd ( 0xC5 );
		ILI9341_Write_Data ( 0x45 );
		ILI9341_Write_Data ( 0x45 );

		/*  VCOM Control 2 (C7h)  */
		ILI9341_Write_Cmd ( 0xC7 );
		ILI9341_Write_Data ( 0xA2 );

		/* Enable 3G (F2h) */
		ILI9341_Write_Cmd ( 0xF2 );
		ILI9341_Write_Data ( 0x00 );

		/* Gamma Set (26h) */
		ILI9341_Write_Cmd ( 0x26 );
		ILI9341_Write_Data ( 0x01 );
		delay_ms (5);

		/* Positive Gamma Correction */
		ILI9341_Write_Cmd ( 0xE0 ); //Set Gamma
		ILI9341_Write_Data ( 0x0F );
		ILI9341_Write_Data ( 0x26 );
		ILI9341_Write_Data ( 0x24 );
		ILI9341_Write_Data ( 0x0B );
		ILI9341_Write_Data ( 0x0E );
		ILI9341_Write_Data ( 0x09 );
		ILI9341_Write_Data ( 0x54 );
		ILI9341_Write_Data ( 0xA8 );
		ILI9341_Write_Data ( 0x46 );
		ILI9341_Write_Data ( 0x0C );
		ILI9341_Write_Data ( 0x17 );
		ILI9341_Write_Data ( 0x09 );
		ILI9341_Write_Data ( 0x0F );
		ILI9341_Write_Data ( 0x07 );
		ILI9341_Write_Data ( 0x00 );

		/* Negative Gamma Correction (E1h) */
		ILI9341_Write_Cmd ( 0XE1 ); //Set Gamma
		ILI9341_Write_Data ( 0x00 );
		ILI9341_Write_Data ( 0x19 );
		ILI9341_Write_Data ( 0x1B );
		ILI9341_Write_Data ( 0x04 );
		ILI9341_Write_Data ( 0x10 );
		ILI9341_Write_Data ( 0x07 );
		ILI9341_Write_Data ( 0x2A );
		ILI9341_Write_Data ( 0x47 );
		ILI9341_Write_Data ( 0x39 );
		ILI9341_Write_Data ( 0x03 );
		ILI9341_Write_Data ( 0x06 );
		ILI9341_Write_Data ( 0x06 );
		ILI9341_Write_Data ( 0x30 );
		ILI9341_Write_Data ( 0x38 );
		ILI9341_Write_Data ( 0x0F );

		/* memory access control set */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0x36 ); 	
		ILI9341_Write_Data ( 0xC8 );    /*����  ���Ͻǵ� (���)�����½� (�յ�)ɨ�跽ʽ*/
		delay_ms(5);

		/* column address control set */
		ILI9341_Write_Cmd ( CMD_SetCoordinateX ); 
		ILI9341_Write_Data ( 0x00 );
		ILI9341_Write_Data ( 0x00 );
		ILI9341_Write_Data ( 0x00 );
		ILI9341_Write_Data ( 0xEF );

		/* page address control set */
		delay_ms(5);
		ILI9341_Write_Cmd ( CMD_SetCoordinateY ); 
		ILI9341_Write_Data ( 0x00 );
		ILI9341_Write_Data ( 0x00 );
		ILI9341_Write_Data ( 0x01 );
		ILI9341_Write_Data ( 0x3F );

		/*  Pixel Format Set (3Ah)  */
		delay_ms(5);
		ILI9341_Write_Cmd ( 0x3a ); 
		ILI9341_Write_Data ( 0x55 );

		/* Sleep Out (11h)  */
		ILI9341_Write_Cmd ( 0x11 );	
		delay_ms ( 100 );
		delay_ms(5);

		/* Display ON (29h) */
		ILI9341_Write_Cmd ( 0x29 );
		
	}
	else if(lcdid == LCDID_ST7789V)
  {
    /*  Power control B (CFh)  */
    DEBUG_DELAY  ();
    ILI9341_Write_Cmd ( 0xCF  );
    ILI9341_Write_Data ( 0x00  );
    ILI9341_Write_Data ( 0xC1  );
    ILI9341_Write_Data ( 0x30  );
    
    /*  Power on sequence control (EDh) */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0xED );
    ILI9341_Write_Data ( 0x64 );
    ILI9341_Write_Data ( 0x03 );
    ILI9341_Write_Data ( 0x12 );
    ILI9341_Write_Data ( 0x81 );
    
    /*  Driver timing control A (E8h) */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0xE8 );
    ILI9341_Write_Data ( 0x85 );
    ILI9341_Write_Data ( 0x10 );
    ILI9341_Write_Data ( 0x78 );
    
    /*  Power control A (CBh) */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0xCB );
    ILI9341_Write_Data ( 0x39 );
    ILI9341_Write_Data ( 0x2C );
    ILI9341_Write_Data ( 0x00 );
    ILI9341_Write_Data ( 0x34 );
    ILI9341_Write_Data ( 0x02 );
    
    /* Pump ratio control (F7h) */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0xF7 );
    ILI9341_Write_Data ( 0x20 );
    
    /* Driver timing control B */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0xEA );
    ILI9341_Write_Data ( 0x00 );
    ILI9341_Write_Data ( 0x00 );
    
    
    /* Power Control 1 (C0h) */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0xC0 );   //Power control
    ILI9341_Write_Data ( 0x21 );  //VRH[5:0]
    
    /* Power Control 2 (C1h) */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0xC1 );   //Power control
    ILI9341_Write_Data ( 0x11 );  //SAP[2:0];BT[3:0]
    
    /* VCOM Control 1 (C5h) */
    ILI9341_Write_Cmd ( 0xC5 );
    ILI9341_Write_Data ( 0x2D );
    ILI9341_Write_Data ( 0x33 );
    
    /*  VCOM Control 2 (C7h)  */
//	ILI9341_Write_Cmd ( 0xC7 );
//	ILI9341_Write_Data ( 0XC0 );
    
    /* memory access control set */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0x36 );   //Memory Access Control
    ILI9341_Write_Data ( 0x00 );  /*����  ���Ͻǵ� (���)�����½� (�յ�)ɨ�跽ʽ*/
    DEBUG_DELAY ();
    
    ILI9341_Write_Cmd(0x3A);   
    ILI9341_Write_Data(0x55); 
    
      /* Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0xB1 );
    ILI9341_Write_Data ( 0x00 );
    ILI9341_Write_Data ( 0x17 );
    
    /*  Display Function Control (B6h) */
    DEBUG_DELAY ();
    ILI9341_Write_Cmd ( 0xB6 );
    ILI9341_Write_Data ( 0x0A );
    ILI9341_Write_Data ( 0xA2 );
    
    ILI9341_Write_Cmd(0xF6);    			
    ILI9341_Write_Data(0x01); 
    ILI9341_Write_Data(0x30); 
    
    /* Enable 3G (F2h) */
    ILI9341_Write_Cmd ( 0xF2 );
    ILI9341_Write_Data ( 0x00 );
    
    /* Gamma Set (26h) */
    ILI9341_Write_Cmd ( 0x26 );
    ILI9341_Write_Data ( 0x01 );
    DEBUG_DELAY ();
    
    /* Positive Gamma Correction */
    ILI9341_Write_Cmd(0xe0); //Positive gamma
    ILI9341_Write_Data(0xd0);         
    ILI9341_Write_Data(0x00); 
    ILI9341_Write_Data(0x02); 
    ILI9341_Write_Data(0x07); 
    ILI9341_Write_Data(0x0b); 
    ILI9341_Write_Data(0x1a); 
    ILI9341_Write_Data(0x31); 
    ILI9341_Write_Data(0x54); 
    ILI9341_Write_Data(0x40); 
    ILI9341_Write_Data(0x29); 
    ILI9341_Write_Data(0x12); 
    ILI9341_Write_Data(0x12); 
    ILI9341_Write_Data(0x12); 
    ILI9341_Write_Data(0x17);

    /* Negative Gamma Correction (E1h) */
    ILI9341_Write_Cmd(0xe1); //Negative gamma
    ILI9341_Write_Data(0xd0); 
    ILI9341_Write_Data(0x00); 
    ILI9341_Write_Data(0x02); 
    ILI9341_Write_Data(0x07); 
    ILI9341_Write_Data(0x05); 
    ILI9341_Write_Data(0x25); 
    ILI9341_Write_Data(0x2d); 
    ILI9341_Write_Data(0x44); 
    ILI9341_Write_Data(0x45); 
    ILI9341_Write_Data(0x1c); 
    ILI9341_Write_Data(0x18); 
    ILI9341_Write_Data(0x16); 
    ILI9341_Write_Data(0x1c); 
    ILI9341_Write_Data(0x1d); 
  
	
//	/* column address control set */
//	ILI9341_Write_Cmd ( CMD_SetCoordinateX ); 
//	ILI9341_Write_Data ( 0x00 );
//	ILI9341_Write_Data ( 0x00 );
//	ILI9341_Write_Data ( 0x00 );
//	ILI9341_Write_Data ( 0xEF );
//	
//	/* page address control set */
//	DEBUG_DELAY ();
//	ILI9341_Write_Cmd ( CMD_SetCoordinateY ); 
//	ILI9341_Write_Data ( 0x00 );
//	ILI9341_Write_Data ( 0x00 );
//	ILI9341_Write_Data ( 0x01 );
//	ILI9341_Write_Data ( 0x3F );
	
	
    /* Sleep Out (11h)  */
    ILI9341_Write_Cmd ( 0x11 );	  //Exit Sleep
    ILI9341_Delay ( 0xAFFf<<2 );
    DEBUG_DELAY ();
    
    /* Display ON (29h) */
    ILI9341_Write_Cmd ( 0x29 );   //Display on
    
    ILI9341_Write_Cmd(0x2c);
  }
	
	 
}


/**
 * @brief  ����ILI9341��GRAM��ɨ�跽�� 
 * @param  ucOption ��ѡ��GRAM��ɨ�跽�� 
 *     @arg 0-7 :������ѡֵΪ0-7��˸�����
 *
 *	����������0��3��5��6 ģʽ�ʺϴ���������ʾ���֣�
 *				���Ƽ�ʹ������ģʽ��ʾ����	����ģʽ��ʾ���ֻ��о���Ч��			
 *		
 *	����0��2��4��6 ģʽ��X��������Ϊ240��Y��������Ϊ320
 *	����1��3��5��7 ģʽ��X��������Ϊ320��Y��������Ϊ240
 *
 *	���� 6 ģʽΪ�󲿷�Һ�����̵�Ĭ����ʾ����
 *	���� 3 ģʽΪ����ͷ����ʹ�õķ���
 *	���� 0 ģʽΪBMPͼƬ��ʾ����ʹ�õķ���
 *
 * @retval ��
 * @note  ����ͼ����A��ʾ���ϣ�V��ʾ���£�<��ʾ����>��ʾ����
					X��ʾX�ᣬY��ʾY��

------------------------------------------------------------
ģʽ0��				.		ģʽ1��		.	ģʽ2��			.	ģʽ3��					
					A		.					A		.		A					.		A									
					|		.					|		.		|					.		|							
					Y		.					X		.		Y					.		X					
					0		.					1		.		2					.		3					
	<--- X0 o		.	<----Y1	o		.		o 2X--->  .		o 3Y--->	
------------------------------------------------------------	
ģʽ4��				.	ģʽ5��			.	ģʽ6��			.	ģʽ7��					
	<--- X4 o		.	<--- Y5 o		.		o 6X--->  .		o 7Y--->	
					4		.					5		.		6					.		7	
					Y		.					X		.		Y					.		X						
					|		.					|		.		|					.		|							
					V		.					V		.		V					.		V		
---------------------------------------------------------				
											 LCD��ʾ��
								|-----------------|
								|			Ұ��Logo		|
								|									|
								|									|
								|									|
								|									|
								|									|
								|									|
								|									|
								|									|
								|-----------------|
								��Ļ���棨��240����320��

 *******************************************************/
void ILI9341_GramScan ( uint8_t ucOption )
{	
	//������飬ֻ������0-7
	if(ucOption >7 )
		return;
	
	//����ģʽ����LCD_SCAN_MODE��ֵ����Ҫ���ڴ�����ѡ��������
	LCD_SCAN_MODE = ucOption;
	
	//����ģʽ����XY��������ؿ��
	if(ucOption%2 == 0)	
	{
		//0 2 4 6ģʽ��X�������ؿ��Ϊ240��Y����Ϊ320
		LCD_X_LENGTH = ILI9341_LESS_PIXEL;
		LCD_Y_LENGTH =	ILI9341_MORE_PIXEL;
	}
	else				
	{
		//1 3 5 7ģʽ��X�������ؿ��Ϊ320��Y����Ϊ240
		LCD_X_LENGTH = ILI9341_MORE_PIXEL;
		LCD_Y_LENGTH =	ILI9341_LESS_PIXEL; 
	}

	//0x36��������ĸ�3λ����������GRAMɨ�跽��	
	ILI9341_Write_Cmd ( 0x36 );
  if(lcdid == LCDID_ILI9341)
  {
    ILI9341_Write_Data ( 0x08 |(ucOption<<5));//����ucOption��ֵ����LCD��������0-7��ģʽ
  }
  else if(lcdid == LCDID_ST7789V)
  {
    ILI9341_Write_Data ( 0x00 |(ucOption<<5));//����ucOption��ֵ����LCD��������0-7��ģʽ
  }
	ILI9341_Write_Cmd ( CMD_SetCoordinateX ); 
	ILI9341_Write_Data ( 0x00 );		/* x ��ʼ�����8λ */
	ILI9341_Write_Data ( 0x00 );		/* x ��ʼ�����8λ */
	ILI9341_Write_Data ( ((LCD_X_LENGTH-1)>>8)&0xFF ); /* x ���������8λ */	
	ILI9341_Write_Data ( (LCD_X_LENGTH-1)&0xFF );				/* x ���������8λ */

	ILI9341_Write_Cmd ( CMD_SetCoordinateY ); 
	ILI9341_Write_Data ( 0x00 );		/* y ��ʼ�����8λ */
	ILI9341_Write_Data ( 0x00 );		/* y ��ʼ�����8λ */
	ILI9341_Write_Data ( ((LCD_Y_LENGTH-1)>>8)&0xFF );	/* y ���������8λ */	 
	ILI9341_Write_Data ( (LCD_Y_LENGTH-1)&0xFF );				/* y ���������8λ */

	/* write gram start */
	ILI9341_Write_Cmd ( CMD_SetPixel );	
}





void Init_ILI9341(void)
{
	GPIO_Config();
	ili9341_FSMC_Config();
	Backlight_control(1);
	LCD_REST();
	ILI9341_REG_Config();
	
	ILI9341_GramScan(LCD_SCAN_MODE);

}

/**
 * @brief  ��ILI9341��ʾ���Ͽ���һ������
 * @param  usX �����ض�ɨ�跽���´��ڵ����X����
 * @param  usY �����ض�ɨ�跽���´��ڵ����Y����
 * @param  usWidth �����ڵĿ��
 * @param  usHeight �����ڵĸ߶�
 * @retval ��
 */
void open_window(uint16_t X,uint16_t Y,uint16_t width,uint16_t high)
{
	ILI9341_Write_Cmd ( CMD_SetCoordinateX ); //����x�������ʼ�����λ��
	ILI9341_Write_Data ( (X&0xff00)>>8 );
	ILI9341_Write_Data ( (X&0x00ff) );
	ILI9341_Write_Data ( ((width+X-1)&0xff00)>>8 );
	ILI9341_Write_Data ( ((width+X-1)&0x00ff) );
	
	ILI9341_Write_Cmd ( CMD_SetCoordinateY ); //����y�������ʼ�����λ��
	ILI9341_Write_Data ( (Y&0xff00)>>8 );
	ILI9341_Write_Data ( (Y&0x00ff) );
	ILI9341_Write_Data ( ((high+Y-1)&0xff00)>>8 );
	ILI9341_Write_Data ( ((high+Y-1)&0x00ff) );
	
}




/**
 * @brief  �趨ILI9341�Ĺ������
 * @param  usX �����ض�ɨ�跽���¹���X����
 * @param  usY �����ض�ɨ�跽���¹���Y����
 * @retval ��
 */
static void ILI9341_SetCursor ( uint16_t usX, uint16_t usY )	
{
	open_window ( usX, usY, 1, 1 );
}

/**
 * @brief  ��ILI9341��ʾ������ĳһ��ɫ������ص�
 * @param  ulAmout_Point ��Ҫ�����ɫ�����ص������Ŀ
 * @param  usColor ����ɫ
 * @retval ��
 */
static __inline void ILI9341_FillColor ( uint32_t ulAmout_Point, uint16_t usColor )
{
	uint32_t i = 0;
	
	
	/* memory write */
	ILI9341_Write_Cmd ( CMD_SetPixel );	
		
	for ( i = 0; i < ulAmout_Point; i ++ )
		ILI9341_Write_Data ( usColor );
		
	
}

/**
 * @brief  ��ILI9341��ʾ����ĳһ������ĳ����ɫ��������
 * @param  usX �����ض�ɨ�跽���´��ڵ����X����
 * @param  usY �����ض�ɨ�跽���´��ڵ����Y����
 * @param  usWidth �����ڵĿ��
 * @param  usHeight �����ڵĸ߶�
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void ILI9341_Clear ( uint16_t usX, uint16_t usY, uint16_t usWidth, uint16_t usHeight )
{
	open_window ( usX, usY, usWidth, usHeight );

	ILI9341_FillColor ( usWidth * usHeight, CurrentBackColor );		
	
}



/**
 * @brief  ��ILI9341��ʾ����ĳһ����ĳ����ɫ�������
 * @param  usX �����ض�ɨ�跽���¸õ��X����
 * @param  usY �����ض�ɨ�跽���¸õ��Y����
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void ILI9341_SetPointPixel ( uint16_t usX, uint16_t usY )	
{	
	if ( ( usX < LCD_X_LENGTH ) && ( usY < LCD_Y_LENGTH ) )
  {
		ILI9341_SetCursor ( usX, usY );//��һ�����صĴ�
		
		ILI9341_FillColor ( 1, CurrentTextColor );
	}
	
}

/**
 * @brief  ��ȡ GRAM ��һ����������
 * @param  ��
 * @retval ��������
 */
static uint16_t ILI9341_Read_PixelData ( void )	
{	
	uint16_t usRG=0, usB=0 ;

	
	ILI9341_Write_Cmd ( 0x2E );   /* ������ */
	//ȥ��ǰһ�ζ�ȡ���
	ILI9341_Read_Data (); 	      /*FIRST READ OUT DUMMY DATA*/
	
	//��ȡ��ɫͨ������ɫͨ����ֵ
	usRG = ILI9341_Read_Data ();  	/*READ OUT RED AND GREEN DATA  */
	usB = ILI9341_Read_Data ();  		/*READ OUT BLUE DATA*/

  return ( (usRG&0xF800)| ((usRG<<3)&0x7E0) | (usB>>11) );
}


/**
 * @brief  ��ȡ ILI9341 ��ʾ����ĳһ����������������
 * @param  usX �����ض�ɨ�跽���¸õ��X����
 * @param  usY �����ض�ɨ�跽���¸õ��Y����
 * @retval ��������
 */
uint16_t ILI9341_GetPointPixel ( uint16_t usX, uint16_t usY )
{ 
	uint16_t usPixelData;

	
	ILI9341_SetCursor ( usX, usY );
	
	usPixelData = ILI9341_Read_PixelData ();
	
	return usPixelData;
	
}


/**
 * @brief  �� ILI9341 ��ʾ����ʹ�� Bresenham �㷨���߶� 
 * @param  usX1 �����ض�ɨ�跽�����߶ε�һ���˵�X����
 * @param  usY1 �����ض�ɨ�跽�����߶ε�һ���˵�Y����
 * @param  usX2 �����ض�ɨ�跽�����߶ε���һ���˵�X����
 * @param  usY2 �����ض�ɨ�跽�����߶ε���һ���˵�Y����
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void ILI9341_DrawLine ( uint16_t usX1, uint16_t usY1, uint16_t usX2, uint16_t usY2 )
{
	uint16_t us; 
	uint16_t usX_Current, usY_Current;
	
	int32_t lError_X = 0, lError_Y = 0, lDelta_X, lDelta_Y, lDistance; 
	int32_t lIncrease_X, lIncrease_Y; 	
	
	
	lDelta_X = usX2 - usX1; //������������ 
	lDelta_Y = usY2 - usY1; 
	
	usX_Current = usX1; 
	usY_Current = usY1; 
	
	
	if ( lDelta_X > 0 ) 
		lIncrease_X = 1; //���õ������� 
	
	else if ( lDelta_X == 0 ) 
		lIncrease_X = 0;//��ֱ�� 
	
	else 
  { 
    lIncrease_X = -1;
    lDelta_X = - lDelta_X;
  } 

	
	if ( lDelta_Y > 0 )
		lIncrease_Y = 1; 
	
	else if ( lDelta_Y == 0 )
		lIncrease_Y = 0;//ˮƽ�� 
	
	else 
  {
    lIncrease_Y = -1;
    lDelta_Y = - lDelta_Y;
  } 

	
	if (  lDelta_X > lDelta_Y )
		lDistance = lDelta_X; //ѡȡ�������������� 
	
	else 
		lDistance = lDelta_Y; 

	
	for ( us = 0; us <= lDistance + 1; us ++ )//������� 
	{  
		ILI9341_SetPointPixel ( usX_Current, usY_Current );//���� 
		
		lError_X += lDelta_X ; 
		lError_Y += lDelta_Y ; 
		
		if ( lError_X > lDistance ) 
		{ 
			lError_X -= lDistance; 
			usX_Current += lIncrease_X; 
		}  
		
		if ( lError_Y > lDistance ) 
		{ 
			lError_Y -= lDistance; 
			usY_Current += lIncrease_Y; 
		} 
		
	}  
	
	
}   


/**
 * @brief  �� ILI9341 ��ʾ���ϻ�һ������
 * @param  usX_Start �����ض�ɨ�跽���¾��ε���ʼ��X����
 * @param  usY_Start �����ض�ɨ�跽���¾��ε���ʼ��Y����
 * @param  usWidth�����εĿ�ȣ���λ�����أ�
 * @param  usHeight�����εĸ߶ȣ���λ�����أ�
 * @param  ucFilled ��ѡ���Ƿ����þ���
  *   �ò���Ϊ����ֵ֮һ��
  *     @arg 0 :���ľ���
  *     @arg 1 :ʵ�ľ��� 
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void ILI9341_DrawRectangle ( uint16_t usX_Start, uint16_t usY_Start, uint16_t usWidth, uint16_t usHeight, uint8_t ucFilled )
{
	if ( ucFilled )
	{
		open_window ( usX_Start, usY_Start, usWidth, usHeight );
		ILI9341_FillColor ( usWidth * usHeight ,CurrentTextColor);	
	}
	else
	{
		ILI9341_DrawLine ( usX_Start, usY_Start, usX_Start + usWidth - 1, usY_Start );
		ILI9341_DrawLine ( usX_Start, usY_Start + usHeight - 1, usX_Start + usWidth - 1, usY_Start + usHeight - 1 );
		ILI9341_DrawLine ( usX_Start, usY_Start, usX_Start, usY_Start + usHeight - 1 );
		ILI9341_DrawLine ( usX_Start + usWidth - 1, usY_Start, usX_Start + usWidth - 1, usY_Start + usHeight - 1 );		
	}

}

/**
 * @brief  �� ILI9341 ��ʾ����ʹ�� Bresenham �㷨��Բ
 * @param  usX_Center �����ض�ɨ�跽����Բ�ĵ�X����
 * @param  usY_Center �����ض�ɨ�跽����Բ�ĵ�Y����
 * @param  usRadius��Բ�İ뾶����λ�����أ�
 * @param  ucFilled ��ѡ���Ƿ�����Բ
  *   �ò���Ϊ����ֵ֮һ��
  *     @arg 0 :����Բ
  *     @arg 1 :ʵ��Բ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void ILI9341_DrawCircle ( uint16_t usX_Center, uint16_t usY_Center, uint16_t usRadius, uint8_t ucFilled )
{
	int16_t sCurrentX, sCurrentY;
	int16_t sError;
	
	
	sCurrentX = 0; sCurrentY = usRadius;	  
	
	sError = 3 - ( usRadius << 1 );     //�ж��¸���λ�õı�־
	
	
	while ( sCurrentX <= sCurrentY )
	{
		int16_t sCountY;
		
		
		if ( ucFilled ) 			
			for ( sCountY = sCurrentX; sCountY <= sCurrentY; sCountY ++ ) 
			{                      
				ILI9341_SetPointPixel ( usX_Center + sCurrentX, usY_Center + sCountY );           //1���о����� 
				ILI9341_SetPointPixel ( usX_Center - sCurrentX, usY_Center + sCountY );           //2       
				ILI9341_SetPointPixel ( usX_Center - sCountY,   usY_Center + sCurrentX );           //3
				ILI9341_SetPointPixel ( usX_Center - sCountY,   usY_Center - sCurrentX );           //4
				ILI9341_SetPointPixel ( usX_Center - sCurrentX, usY_Center - sCountY );           //5    
        ILI9341_SetPointPixel ( usX_Center + sCurrentX, usY_Center - sCountY );           //6
				ILI9341_SetPointPixel ( usX_Center + sCountY,   usY_Center - sCurrentX );           //7 	
        ILI9341_SetPointPixel ( usX_Center + sCountY,   usY_Center + sCurrentX );           //0				
			}
		
		else
		{          
			ILI9341_SetPointPixel ( usX_Center + sCurrentX, usY_Center + sCurrentY );             //1���о�����
			ILI9341_SetPointPixel ( usX_Center - sCurrentX, usY_Center + sCurrentY );             //2      
			ILI9341_SetPointPixel ( usX_Center - sCurrentY, usY_Center + sCurrentX );             //3
			ILI9341_SetPointPixel ( usX_Center - sCurrentY, usY_Center - sCurrentX );             //4
			ILI9341_SetPointPixel ( usX_Center - sCurrentX, usY_Center - sCurrentY );             //5       
			ILI9341_SetPointPixel ( usX_Center + sCurrentX, usY_Center - sCurrentY );             //6
			ILI9341_SetPointPixel ( usX_Center + sCurrentY, usY_Center - sCurrentX );             //7 
			ILI9341_SetPointPixel ( usX_Center + sCurrentY, usY_Center + sCurrentX );             //0
    }			
		
		
		sCurrentX ++;

		
		if ( sError < 0 ) 
			sError += 4 * sCurrentX + 6;	  
		
		else
		{
			sError += 10 + 4 * ( sCurrentX - sCurrentY );   
			sCurrentY --;
		} 	
		
		
	}
	
	
}


/**
  * @brief  ����LCD��ǰ��(����)��������ɫ,RGB565
  * @param  TextColor: ָ��ǰ��(����)��ɫ
  * @param  BackColor: ָ��������ɫ
  * @retval None
  */
void LCD_SetColors(uint16_t TextColor, uint16_t BackColor) 
{
  CurrentTextColor = TextColor; 
  CurrentBackColor = BackColor;
}

/**
  * @brief  ��ȡLCD��ǰ��(����)��������ɫ,RGB565
  * @param  TextColor: �����洢ǰ��(����)��ɫ��ָ�����
  * @param  BackColor: �����洢������ɫ��ָ�����
  * @retval None
  */
void LCD_GetColors(uint16_t *TextColor, uint16_t *BackColor)
{
  *TextColor = CurrentTextColor;
  *BackColor = CurrentBackColor;
}


/**
  * @brief  ����LCD��ǰ��(����)��ɫ,RGB565
  * @param  Color: ָ��ǰ��(����)��ɫ 
  * @retval None
  */
void LCD_SetTextColor(uint16_t Color)
{
  CurrentTextColor = Color;
}

/**
  * @brief  ����LCD�ı�����ɫ,RGB565
  * @param  Color: ָ��������ɫ 
  * @retval None
  */
void LCD_SetBackColor(uint16_t Color)
{
  CurrentBackColor = Color;
}


void ILI9341_DispChar_EN ( uint16_t usX, uint16_t usY, const char cChar )
{
	uint8_t  byteCount, bitCount,fontLength;	
	uint16_t ucRelativePositon;
	uint8_t *Pfont;
	
	//��ascii���ƫ�ƣ���ģ������ASCII���ǰ32����ͼ�η��ţ�
	ucRelativePositon = cChar - ' ';
	
	//ÿ����ģ���ֽ���
	fontLength = (LCD_Currentfonts->Width*LCD_Currentfonts->Height)/8;
		
	//��ģ�׵�ַ
	/*ascii���ƫ��ֵ����ÿ����ģ���ֽ����������ģ��ƫ��λ��*/
	Pfont = (uint8_t *)&LCD_Currentfonts->table[ucRelativePositon * fontLength];
	
	//������ʾ����
	open_window ( usX, usY, LCD_Currentfonts->Width, LCD_Currentfonts->Height);
	
	ILI9341_Write_Cmd ( CMD_SetPixel );			

	//���ֽڶ�ȡ��ģ����
	//����ǰ��ֱ����������ʾ���ڣ���ʾ���ݻ��Զ�����
	for ( byteCount = 0; byteCount < fontLength; byteCount++ )
	{
			//һλһλ����Ҫ��ʾ����ɫ
			for ( bitCount = 0; bitCount < 8; bitCount++ )
			{
					if ( Pfont[byteCount] & (0x80>>bitCount) )
						ILI9341_Write_Data ( CurrentTextColor );			
					else
						ILI9341_Write_Data ( CurrentBackColor );
			}	
	}	
}
/**
 * @brief  �� ILI9341 ��ʾ������ʾӢ���ַ���
 * @param  usX �����ض�ɨ�跽�����ַ�����ʼX����
 * @param  usY �����ض�ɨ�跽�����ַ�����ʼY����
 * @param  pStr ��Ҫ��ʾ��Ӣ���ַ������׵�ַ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void ILI9341_DispString_EN ( 	uint16_t usX ,uint16_t usY,  char * pStr )
{
	while(*pStr!=0)
	{
		if(( usX - ILI9341_DispWindow_X_Star + LCD_Currentfonts->Width )>LCD_X_LENGTH)
		{
			usX=0;
			usY=usY+LCD_Currentfonts->Height;
		}
		if((usY-ILI9341_DispWindow_Y_Star+LCD_Currentfonts->Height)>LCD_Y_LENGTH)
		{
			usX=0;
			usY=0;
		}
		ILI9341_DispChar_EN(usX,usY,*pStr);
		usX+=LCD_Currentfonts->Width;
		pStr++;

	}
		
}
	
/**
 * @brief  �� ILI9341 ��ʾ������ʾһ�������ַ�
 * @param  usX �����ض�ɨ�跽�����ַ�����ʼX����
 * @param  usY �����ض�ɨ�跽�����ַ�����ʼY����
 * @param  usChar ��Ҫ��ʾ�������ַ��������룩
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */ 
void ILI9341_DispChar_CH ( uint16_t usX, uint16_t usY, uint16_t usChar )
{
	uint8_t byteCount, bitCount;
	uint8_t ucBuffer [ WIDTH_CH_CHAR*HEIGHT_CH_CHAR/8 ];	 	

	//������ʾ����
	open_window ( usX, usY, WIDTH_CH_CHAR, HEIGHT_CH_CHAR );
	
	ILI9341_Write_Cmd ( CMD_SetPixel );
	
	//ȡ��ģ����  
  GetGBKCode ( ucBuffer, usChar );	
	for(byteCount=0;byteCount<WIDTH_CH_CHAR*HEIGHT_CH_CHAR/8;byteCount++)
	{
		for(bitCount=0;bitCount<8;bitCount++)
		{
			if(ucBuffer[byteCount]&(0x80>>bitCount))
			{
				ILI9341_Write_Data ( CurrentTextColor );
			}
			else
			{
				ILI9341_Write_Data ( CurrentBackColor );
			}
		}
	}
	
}

/**
 * @brief  �� ILI9341 ��ʾ������ʾ��Ӣ���ַ���
 * @param  usX �����ض�ɨ�跽�����ַ�����ʼX����
 * @param  usY �����ض�ɨ�跽�����ַ�����ʼY����
 * @param  pStr ��Ҫ��ʾ���ַ������׵�ַ
 * @note ��ʹ��LCD_SetBackColor��LCD_SetTextColor��LCD_SetColors����������ɫ
 * @retval ��
 */
void ILI9341_DispString_EN_CH ( 	uint16_t usX , uint16_t usY, char * pStr )
{
	uint16_t usCh;

	
	while( * pStr != '\0' )
	{
		if ( * pStr <= 126 )	           	//Ӣ���ַ�
		{
			if ( ( usX - ILI9341_DispWindow_X_Star + LCD_Currentfonts->Width ) > LCD_X_LENGTH )
			{
				usX = ILI9341_DispWindow_X_Star;
				usY += LCD_Currentfonts->Height;
			}
			
			if ( ( usY - ILI9341_DispWindow_Y_Star + LCD_Currentfonts->Height ) > LCD_Y_LENGTH )
			{
				usX = ILI9341_DispWindow_X_Star;
				usY = ILI9341_DispWindow_Y_Star;
			}			
		
		  ILI9341_DispChar_EN ( usX, usY, * pStr );
			
			usX +=  LCD_Currentfonts->Width;
		
		  pStr ++;

		}
		
		else	                            //�����ַ�
		{
			if ( ( usX - ILI9341_DispWindow_X_Star + WIDTH_CH_CHAR ) > LCD_X_LENGTH )
			{
				usX = ILI9341_DispWindow_X_Star;
				usY += HEIGHT_CH_CHAR;
			}
			
			if ( ( usY - ILI9341_DispWindow_Y_Star + HEIGHT_CH_CHAR ) > LCD_Y_LENGTH )
			{
				usX = ILI9341_DispWindow_X_Star;
				usY = ILI9341_DispWindow_Y_Star;
			}	
			
			usCh = * pStr;
			usCh=usCh << 8;
			pStr++;
			usCh|=* pStr;
			pStr++;
			ILI9341_DispChar_CH ( usX, usY, usCh );
			usX += WIDTH_CH_CHAR;
			
			
			
			
			
			
			
			
//			usCh = * ( uint16_t * ) pStr;	
//			
//			usCh = ( usCh << 8 ) + ( usCh >> 8 );		

//			ILI9341_DispChar_CH ( usX, usY, usCh );
//			
//			usX += WIDTH_CH_CHAR;
//			
//			pStr += 2;           //һ�����������ֽ� 
		
    }
		
  }	
} 


/**
  * @brief  ����Ӣ����������
  * @param  fonts: ָ��Ҫѡ�������
	*		����Ϊ����ֵ֮һ
  * 	@arg��Font24x32;
  * 	@arg��Font16x24;
  * 	@arg��Font8x16;
  * @retval None
  */
void LCD_SetFont(sFONT *fonts)
{
  LCD_Currentfonts = fonts;
}




