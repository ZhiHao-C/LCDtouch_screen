#include "bps_flash.h"

static void SPI_GPIO_Config()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//ʹ��GPIOA
	RCC_APB2PeriphClockCmd(FLASH_SPI_GPIOA_CLK,ENABLE);
	//ʹ��GPIOC
	RCC_APB2PeriphClockCmd(FLASH_SPI_GPIOC_CLK,ENABLE);
	//ʹ��SPI1
	RCC_APB2PeriphClockCmd(FLASH_SPI_CLK,ENABLE);
	//����SCK����
	GPIO_InitStruct.GPIO_Pin=FLASH_SPI_SCK;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStruct);
	//����MOSI����
	GPIO_InitStruct.GPIO_Pin=FLASH_SPI_MOSI;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);
	//����MISO����
	GPIO_InitStruct.GPIO_Pin=FLASH_SPI_MISO;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(FLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);
	
	//����CS���ţ����ģ�⣩
	GPIO_InitStruct.GPIO_Pin=FLASH_SPI_CS;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_CS_GPIO_PORT, &GPIO_InitStruct);
	//��cs���ڿ���״̬
	CS(1)
}

static void SPI_Config()
{
	SPI_InitTypeDef SPI_InitStruct;
	SPI_InitStruct.SPI_Direction=SPI_Direction_2Lines_FullDuplex;//˫��ȫ˫��
	SPI_InitStruct.SPI_Mode=SPI_Mode_Master;//����ģʽ
	SPI_InitStruct.SPI_DataSize=SPI_DataSize_8b;//һ������֡Ϊ8λ
	SPI_InitStruct.SPI_CPOL=SPI_CPOL_Low;//CPOL��ʱ�Ӽ��� (Clock polarity) 0�� ����״̬ʱ��SCK���ֵ͵�ƽ��1�� ����״̬ʱ��SCK���ָߵ�ƽ��
	SPI_InitStruct.SPI_CPHA=SPI_CPHA_1Edge;//CPHA��ʱ����λ ��һ��ʱ�ӱ��ؿ�ʼ����
	SPI_InitStruct.SPI_NSS=SPI_NSS_Soft;//���ģʽ
	SPI_InitStruct.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_2;//��Ƶ
	SPI_InitStruct.SPI_FirstBit=SPI_FirstBit_MSB;//��λ����
	SPI_InitStruct.SPI_CRCPolynomial=0;//������CRCУ��
	
	SPI_Init(SPI1, &SPI_InitStruct);
	
	SPI_Cmd(SPI1,ENABLE);
}

void SPI_FLASH_Init(void)//��ʼ�����ź�SPI
{
	SPI_GPIO_Config();
	SPI_Config();
}
//STM32ͨ��SPI������д��flash��
uint8_t SPI_SendRead_byte(uint8_t Data)
{
	uint32_t n=SPITIMEout;
	//�жϷ��ͱ�־λ�Ƿ�Ϊ1��Ϊ1ʱ���ͻ�����Ϊ�տ���������д���ݣ�Ϊ0��ǿգ��üĴ�����stm32�У�
	while (SPI_I2S_GetFlagStatus(FLASH_SPIx, SPI_I2S_FLAG_TXE)==0)//ͣ��̫����Ȼû����һ��������ֹ������
	{
		n--;
		if(n==0) 
		{return 0;}
	}
	SPI_I2S_SendData(FLASH_SPIx, Data);
	n=SPITIMEout;
//���ڽ���һλ�ͷ���һλͬʱ���е����ձ�־λ��1ʱҲ��������һ���ֽںͽ�����һ���ֽ�
	while(SPI_I2S_GetFlagStatus(FLASH_SPIx, SPI_I2S_FLAG_RXNE)==0);
	{
		n--;
		if(n==0) 
		{return 0;}
	}
	return SPI_I2S_ReceiveData(FLASH_SPIx);
}

//ͨ��SPI��������
uint8_t SPI_Readbyte()
{
	return SPI_SendRead_byte(DUMMY);
}


//��ȡflash��id��
uint32_t SPI_ReadID(void)
{
	uint32_t flash_ID=0;
	CS(0);//csΪ0�൱��ʹ��
	SPI_SendRead_byte(Read_ID);//���������flashҪ��ȡ����id(24λ)
	//��ȡ��д����ͬ�����У����ж�ȡʱ�����д�뷽�����ʱ���ź�������ȡ
	flash_ID|=SPI_SendRead_byte(DUMMY);
	flash_ID<<=8;
	
	flash_ID|=SPI_SendRead_byte(DUMMY);
	flash_ID<<=8;
	
	flash_ID|=SPI_SendRead_byte(DUMMY);
	
	CS(1);
	return flash_ID;
}

void Flash_WriteEnable(void)
{
	CS(0);
	SPI_SendRead_byte(Write_Enable);
	CS(1);
}

//������ѡ��������СΪ4kb��
void Sector_erase(uint32_t addr)
{
	Flash_WriteEnable();//��������֮ǰҪִ��д����ָ��,λ�ñ�����CSΪ0֮ǰ
	CS(0);
	SPI_SendRead_byte(Sector_Erase);
	SPI_SendRead_byte((addr>>16)&0xff);
	SPI_SendRead_byte((addr>>8)&0xff);
	SPI_SendRead_byte((addr>>0)&0xff);
	CS(1);
}

void WaitnoBUSY(void)//�ȴ�busyΪ0
{
	
	uint8_t state=0;
	CS(0);
	SPI_SendRead_byte(Read_Status_Register1);
	do
	{
		state=SPI_SendRead_byte(DUMMY);
	}while((state&0x01)==1);
	
	CS(1);
}


//��ȡflash��ĳ����ַ������,��ȡ�������ݴ洢��a��ַ����ȡnumreadbyte���ֽ�
void FLASH_ReadData(uint32_t addr,uint8_t *a,uint32_t numreadbyte)
{
	CS(0);//csΪ0�൱��ʹ��
	SPI_SendRead_byte(Read_Data);//���������flashҪ��ȡ��addr��ַ����
	SPI_SendRead_byte((addr>>16)&0xff);
	SPI_SendRead_byte((addr>>8)&0xff);
	SPI_SendRead_byte((addr>>0)&0xff);
	while(numreadbyte--)
	{
		*a=SPI_SendRead_byte(DUMMY);
		a++;
	}
	CS(1);
}

//��flashд�����ݣ��������д256�ֽڣ�
void FLASH_WriteData(uint32_t addr,uint8_t *a,uint32_t numreadbyte)
{
	Flash_WriteEnable();//дʹ��
	CS(0);//csΪ0�൱��ʹ��
	SPI_SendRead_byte(Page_Program);//���������flashҪ��ȡ��addr��ַ����
	SPI_SendRead_byte((addr>>16)&0xff);
	SPI_SendRead_byte((addr>>8)&0xff);
	SPI_SendRead_byte((addr>>0)&0xff);
	while(numreadbyte--)
	{
		SPI_SendRead_byte(*a);
		a++;
	}
	CS(1);
}

/**
  * @brief  ��FLASHд�����ݣ����ñ�����д������ǰ��Ҫ�Ȳ�������
  * @param	pBuffer��Ҫд�����ݵ�ָ��
  * @param  WriteAddr��д���ַ
  * @param  NumByteToWrite��д�����ݳ���
  * @retval ��
  */
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
	
	/*mod�������࣬��writeAddr��SPI_FLASH_PageSize��������������AddrֵΪ0*/
  Addr = WriteAddr % SPI_FLASH_PageSize;
	
	/*��count������ֵ���պÿ��Զ��뵽ҳ��ַ*/
  count = SPI_FLASH_PageSize - Addr;
	/*�����Ҫд��������ҳ*/
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
	/*mod�������࣬�����ʣ�಻��һҳ���ֽ���*/
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;
	
	/* Addr=0,��WriteAddr �պð�ҳ���� aligned  */
  if (Addr == 0)
  {
		/* NumByteToWrite < SPI_FLASH_PageSize */
    if (NumOfPage == 0) 
    {
      FLASH_WriteData( WriteAddr,pBuffer, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    { 
			/*�Ȱ�����ҳ��д��*/
      while (NumOfPage--)
      {
        FLASH_WriteData( WriteAddr,pBuffer, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }
			/*���ж���Ĳ���һҳ�����ݣ�����д��*/
      FLASH_WriteData( WriteAddr,pBuffer, NumOfSingle);
    }
  }
	/* ����ַ�� SPI_FLASH_PageSize ������  */
  else 
  {
		/* NumByteToWrite < SPI_FLASH_PageSize */
    if (NumOfPage == 0)
    {
			/*��ǰҳʣ���count��λ�ñ�NumOfSingleС��һҳд����*/
      if (NumOfSingle > count) 
      {
        temp = NumOfSingle - count;
				/*��д����ǰҳ*/
        FLASH_WriteData( WriteAddr,pBuffer, count);
				
        WriteAddr +=  count;
        pBuffer += count;
				/*��дʣ�������*/
        FLASH_WriteData( WriteAddr,pBuffer, temp);
      }
      else /*��ǰҳʣ���count��λ����д��NumOfSingle������*/
      {
        FLASH_WriteData( WriteAddr,pBuffer, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
			/*��ַ����������count�ֿ������������������*/
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;
			
			/* ��д��count�����ݣ�Ϊ��������һ��Ҫд�ĵ�ַ���� */
      FLASH_WriteData( WriteAddr,pBuffer, count);
			
			/* ���������ظ���ַ�������� */
      WriteAddr +=  count;
      pBuffer += count;
			/*������ҳ��д��*/
      while (NumOfPage--)
      {
        FLASH_WriteData( WriteAddr,pBuffer, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }
			/*���ж���Ĳ���һҳ�����ݣ�����д��*/
      if (NumOfSingle != 0)
      {
        FLASH_WriteData( WriteAddr,pBuffer, NumOfSingle);
      }
    }
  }
}
