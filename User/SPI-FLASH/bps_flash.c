#include "bps_flash.h"

static void SPI_GPIO_Config()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//使能GPIOA
	RCC_APB2PeriphClockCmd(FLASH_SPI_GPIOA_CLK,ENABLE);
	//使能GPIOC
	RCC_APB2PeriphClockCmd(FLASH_SPI_GPIOC_CLK,ENABLE);
	//使能SPI1
	RCC_APB2PeriphClockCmd(FLASH_SPI_CLK,ENABLE);
	//配置SCK引脚
	GPIO_InitStruct.GPIO_Pin=FLASH_SPI_SCK;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_SCK_GPIO_PORT, &GPIO_InitStruct);
	//配置MOSI引脚
	GPIO_InitStruct.GPIO_Pin=FLASH_SPI_MOSI;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);
	//配置MISO引脚
	GPIO_InitStruct.GPIO_Pin=FLASH_SPI_MISO;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(FLASH_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);
	
	//配置CS引脚（软件模拟）
	GPIO_InitStruct.GPIO_Pin=FLASH_SPI_CS;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(FLASH_SPI_CS_GPIO_PORT, &GPIO_InitStruct);
	//让cs处于空闲状态
	CS(1)
}

static void SPI_Config()
{
	SPI_InitTypeDef SPI_InitStruct;
	SPI_InitStruct.SPI_Direction=SPI_Direction_2Lines_FullDuplex;//双线全双工
	SPI_InitStruct.SPI_Mode=SPI_Mode_Master;//主机模式
	SPI_InitStruct.SPI_DataSize=SPI_DataSize_8b;//一个数据帧为8位
	SPI_InitStruct.SPI_CPOL=SPI_CPOL_Low;//CPOL：时钟极性 (Clock polarity) 0： 空闲状态时，SCK保持低电平；1： 空闲状态时，SCK保持高电平。
	SPI_InitStruct.SPI_CPHA=SPI_CPHA_1Edge;//CPHA：时钟相位 第一个时钟边沿开始采样
	SPI_InitStruct.SPI_NSS=SPI_NSS_Soft;//软件模式
	SPI_InitStruct.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_2;//分频
	SPI_InitStruct.SPI_FirstBit=SPI_FirstBit_MSB;//高位先行
	SPI_InitStruct.SPI_CRCPolynomial=0;//不采用CRC校验
	
	SPI_Init(SPI1, &SPI_InitStruct);
	
	SPI_Cmd(SPI1,ENABLE);
}

void SPI_FLASH_Init(void)//初始化引脚和SPI
{
	SPI_GPIO_Config();
	SPI_Config();
}
//STM32通过SPI读或者写入flash中
uint8_t SPI_SendRead_byte(uint8_t Data)
{
	uint32_t n=SPITIMEout;
	//判断发送标志位是否为1（为1时发送缓存器为空可以往里面写数据，为0则非空，该寄存器在stm32中）
	while (SPI_I2S_GetFlagStatus(FLASH_SPIx, SPI_I2S_FLAG_TXE)==0)//停留太久仍然没有置一则跳出防止程序卡死
	{
		n--;
		if(n==0) 
		{return 0;}
	}
	SPI_I2S_SendData(FLASH_SPIx, Data);
	n=SPITIMEout;
//由于接收一位和发送一位同时进行当接收标志位置1时也代表发送完一个字节和接收完一个字节
	while(SPI_I2S_GetFlagStatus(FLASH_SPIx, SPI_I2S_FLAG_RXNE)==0);
	{
		n--;
		if(n==0) 
		{return 0;}
	}
	return SPI_I2S_ReceiveData(FLASH_SPIx);
}

//通过SPI读回数据
uint8_t SPI_Readbyte()
{
	return SPI_SendRead_byte(DUMMY);
}


//读取flash的id号
uint32_t SPI_ReadID(void)
{
	uint32_t flash_ID=0;
	CS(0);//cs为0相当于使能
	SPI_SendRead_byte(Read_ID);//发送命令给flash要读取它的id(24位)
	//读取与写入是同步进行，进行读取时可随便写入方便产生时钟信号用来读取
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

//擦除所选扇区（大小为4kb）
void Sector_erase(uint32_t addr)
{
	Flash_WriteEnable();//擦除扇区之前要执行写启用指令,位置必须在CS为0之前
	CS(0);
	SPI_SendRead_byte(Sector_Erase);
	SPI_SendRead_byte((addr>>16)&0xff);
	SPI_SendRead_byte((addr>>8)&0xff);
	SPI_SendRead_byte((addr>>0)&0xff);
	CS(1);
}

void WaitnoBUSY(void)//等待busy为0
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


//读取flash的某个地址的内容,读取到的内容存储到a地址，读取numreadbyte个字节
void FLASH_ReadData(uint32_t addr,uint8_t *a,uint32_t numreadbyte)
{
	CS(0);//cs为0相当于使能
	SPI_SendRead_byte(Read_Data);//发送命令给flash要读取它addr地址内容
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

//往flash写入数据（最多连续写256字节）
void FLASH_WriteData(uint32_t addr,uint8_t *a,uint32_t numreadbyte)
{
	Flash_WriteEnable();//写使能
	CS(0);//cs为0相当于使能
	SPI_SendRead_byte(Page_Program);//发送命令给flash要读取它addr地址内容
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
  * @brief  对FLASH写入数据，调用本函数写入数据前需要先擦除扇区
  * @param	pBuffer，要写入数据的指针
  * @param  WriteAddr，写入地址
  * @param  NumByteToWrite，写入数据长度
  * @retval 无
  */
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
	
	/*mod运算求余，若writeAddr是SPI_FLASH_PageSize整数倍，运算结果Addr值为0*/
  Addr = WriteAddr % SPI_FLASH_PageSize;
	
	/*差count个数据值，刚好可以对齐到页地址*/
  count = SPI_FLASH_PageSize - Addr;
	/*计算出要写多少整数页*/
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
	/*mod运算求余，计算出剩余不满一页的字节数*/
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;
	
	/* Addr=0,则WriteAddr 刚好按页对齐 aligned  */
  if (Addr == 0)
  {
		/* NumByteToWrite < SPI_FLASH_PageSize */
    if (NumOfPage == 0) 
    {
      FLASH_WriteData( WriteAddr,pBuffer, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    { 
			/*先把整数页都写了*/
      while (NumOfPage--)
      {
        FLASH_WriteData( WriteAddr,pBuffer, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }
			/*若有多余的不满一页的数据，把它写完*/
      FLASH_WriteData( WriteAddr,pBuffer, NumOfSingle);
    }
  }
	/* 若地址与 SPI_FLASH_PageSize 不对齐  */
  else 
  {
		/* NumByteToWrite < SPI_FLASH_PageSize */
    if (NumOfPage == 0)
    {
			/*当前页剩余的count个位置比NumOfSingle小，一页写不完*/
      if (NumOfSingle > count) 
      {
        temp = NumOfSingle - count;
				/*先写满当前页*/
        FLASH_WriteData( WriteAddr,pBuffer, count);
				
        WriteAddr +=  count;
        pBuffer += count;
				/*再写剩余的数据*/
        FLASH_WriteData( WriteAddr,pBuffer, temp);
      }
      else /*当前页剩余的count个位置能写完NumOfSingle个数据*/
      {
        FLASH_WriteData( WriteAddr,pBuffer, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
			/*地址不对齐多出的count分开处理，不加入这个运算*/
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;
			
			/* 先写完count个数据，为的是让下一次要写的地址对齐 */
      FLASH_WriteData( WriteAddr,pBuffer, count);
			
			/* 接下来就重复地址对齐的情况 */
      WriteAddr +=  count;
      pBuffer += count;
			/*把整数页都写了*/
      while (NumOfPage--)
      {
        FLASH_WriteData( WriteAddr,pBuffer, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }
			/*若有多余的不满一页的数据，把它写完*/
      if (NumOfSingle != 0)
      {
        FLASH_WriteData( WriteAddr,pBuffer, NumOfSingle);
      }
    }
  }
}
