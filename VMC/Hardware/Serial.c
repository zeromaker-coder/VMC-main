#include "Serial.h"

uint32_t DMA_Rece_Buf1[100] = {0};
void Serial_SendFloatNumber(float FloatNumber,uint8_t Int_Length, uint8_t Float_Length);

void Serial_Init(uint32_t bound)
{
//GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级  
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE); //使能USART1，GPIOA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输

    USART_DeInit(USART1);  //复位串口1
   //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
   
    //USART1_RX  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置
		USART_InitStructure.USART_BaudRate = bound;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启空闲中断
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);   //使能串口1 DMA接收
    USART_Cmd(USART1, ENABLE);                   //使能串口 
 
    //相应的DMA配置
		DMA_DeInit(DMA1_Channel5);   //将DMA1的通道5寄存器重设为缺省值  串口1对应的是DMA通道5
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR; //DMA外设usart基地址
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)DMA_Rece_Buf1;  //DMA内存基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
		DMA_InitStructure.DMA_BufferSize = DMA_Rec_Len1;  //DMA通道的DMA缓存的大小
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为8位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
		DMA_Init(DMA1_Channel5, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道

    DMA_Cmd(DMA1_Channel5, ENABLE);  //正式驱动DMA传输
}

//重新恢复DMA指针
void MYDMA1_Enable(DMA_Channel_TypeDef*DMA_CHx)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART1 TX DMA1所指示的通道    
 	DMA_SetCurrDataCounter(DMA_CHx,DMA_Rec_Len1);//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA_CHx, ENABLE);  //打开USART1 TX DMA1所指示的通道  
}	


/*  串口一中断执行函数   */
int Usart1_Rec_Cnt;
float M0_Target =20.0; //串口目标值
float M1_Target =20.0; //串口目标值
char str[10];
void USART1_IRQHandler(void)                //串口1中断服务程序
{
	 char  num[10]={0};
	 int i = 1; 
   int j = 0;
	 if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //接收中断
		{
			 USART_ReceiveData(USART1);//读取数据
			 Usart1_Rec_Cnt =DMA_Rec_Len1-DMA_GetCurrDataCounter(DMA1_Channel5); //算出接本帧数据长度
			 sprintf(str,"%s",(char *)DMA_Rece_Buf1);
			 memset(DMA_Rece_Buf1, 0, Usart1_Rec_Cnt);      //清空
			 //***********帧数据处理函数************/
			 if(str[0] == 'A' && str[Usart1_Rec_Cnt-1] == '\n')//数据第一位与数据的最后一位
			 {
					 if(str[1] != '-')//接受到的数据为正数
					 {
						 while (str[i] != '\n' && (isdigit(str[i]) || str[i] == '.')) {num[j++] = str[i++];}
						 M0_Target = atof(num);
//						 Serial_SendFloatNumber(M0_Target,3,2);
					 } 
					 else//接收到的数据为负数
					 {
						 i=2;
						 while (str[i] != '\n' && (isdigit(str[i]) || str[i] == '.')) {num[j++] = str[i++];}
						 M0_Target = -(atof(num));
//						 Serial_SendFloatNumber(M0_Target,3,2);
					 }
					 Serial_SendString(str);
			 }
			 else if(str[0] == 'B' && str[Usart1_Rec_Cnt-1] == '\n')//数据第一位与数据的最后一位
			 {
					 if(str[1] != '-')//接受到的数据为正数
					 {
						 while (str[i] != '\n' && (isdigit(str[i]) || str[i] == '.')) {num[j++] = str[i++];}
						 M1_Target = atof(num);
//						 Serial_SendFloatNumber(M1_Target,3,2);
					 } 
					 else//接收到的数据为负数
					 {
						 i=2;
						 while (str[i] != '\n' && (isdigit(str[i]) || str[i] == '.')) {num[j++] = str[i++];}
						 M1_Target = -(atof(num));
//						 Serial_SendFloatNumber(M1_Target,3,2);
					 }
					 Serial_SendString(str);
			 }
			 else
			 {
				   Serial_SendString((char *)"Input format error\n");
			 }
			 USART_ClearITPendingBit(USART1,USART_IT_IDLE);         //清除中断标志
			 MYDMA1_Enable(DMA1_Channel5);                  //恢复DMA指针，等待下一次的接收
			 Usart1_Rec_Cnt=0;
		}
		memset(str, 0, Usart1_Rec_Cnt);      //清空
		memset(num, 0, Usart1_Rec_Cnt);      //清空
}


//单片机串口发送一个字节
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);//等待数据转运完整，等待标志位值1，没传完数据前都是0。传完后寄存器被清空，当下次执行SendData时，标志位自动清零
}

//将数字每一位拆开
uint32_t Serial_Pow(uint32_t x, uint32_t y)
{
	uint32_t Result = 1;
	while(y--)
		{
			Result = Result * x;
		}
		return Result;
}

//发送数组
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for(i = 0; i < Length; i++)
	{
		Serial_SendByte(Array[i]);
	}
}

/*发送字符串*/
void Serial_SendString(const char* str) {
    while (*str) {
        Serial_SendByte(*str++);
    }
}

//发送一串数字
void Serial_SendNumber(uint32_t Number,uint8_t Length)
{
	uint8_t i;
	for(i = 0 ; i < Length; i++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1)%10 + '0');
	}
}

//发送正负浮点数
void Serial_SendFloatNumber(float FloatNumber,uint8_t Int_Length, uint8_t Float_Length)
{
	if (FloatNumber > 0 || FloatNumber == 0)
	{
		uint32_t Int_num = (uint32_t)FloatNumber;
		uint32_t Float_num = FloatNumber * Serial_Pow(10,Float_Length);
		
//		Float_num = Float_num % 600;
		for(int i = 0 ; i < Int_Length; i++)
		{
			Serial_SendByte(Int_num / Serial_Pow(10, Int_Length - i - 1)%10 + '0');
		}
		Serial_SendString(".");
		for(int j = 0 ; j < Float_Length; j++)
		{
			Serial_SendByte(Float_num / Serial_Pow(10, Float_Length - j - 1)%10 + '0');
		}
	}
	else if(FloatNumber < 0)
	{
		FloatNumber = -FloatNumber;
		uint32_t Int_num = (uint32_t)FloatNumber;
		uint32_t Float_num = FloatNumber * Serial_Pow(10,Float_Length);
//		Float_num = Float_num % 600;
		Serial_SendString("-");
		for(int i = 0 ; i < Int_Length; i++)
		{
			Serial_SendByte(Int_num / Serial_Pow(10, Int_Length - i - 1)%10 + '0');
		}
		Serial_SendString(".");
		for(int j = 0 ; j < Float_Length; j++)
		{
			Serial_SendByte(Float_num / Serial_Pow(10, Float_Length - j - 1)%10 + '0');
		}
	}
	Serial_SendString("\n");
}
