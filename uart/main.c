
#include "stm32f4xx.h"
#include "system_timetick.h"
#include "stdio.h"
#include "string.h"
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
	
PUTCHAR_PROTOTYPE //include sdtio.h 
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
void print_f(char text[])
{
	for(int i = 0; i<strlen(text); i++)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) // trong example nhung thi lay co USART_FLAG_TXE
		{}
		USART_SendData(USART2, (uint8_t)text[i]);
	}
}

#define		BUFF_SIZE			100

uint8_t 	rxbuff[BUFF_SIZE];
uint8_t 	a[4*BUFF_SIZE];
uint16_t	indexArray = 0;
uint16_t 	rcv_flag = 0;
uint16_t pre_NDTR;
uint8_t getFlag;
void init_main(void);
uint16_t get_mess(void);


int main(void)
{
	/* Enable SysTick at 10ms interrupt */
	SysTick_Config(SystemCoreClock/100); //su dung de do thoi gian he thong

	//pre_NDTR = DMA1_Stream5->NDTR;
	init_main();
	

	
	while(1){
		
}
}
uint16_t get_mess_from_rx(void)
{
	if(pre_NDTR != DMA1_Stream5->NDTR) 
		{
			pre_NDTR = DMA1_Stream5->NDTR;
			return 0;
		}
		if((pre_NDTR == DMA1_Stream5->NDTR)&&(pre_NDTR == BUFF_SIZE))
		{	
			return 0;
		}
		DMA_Cmd(DMA1_Stream5, DISABLE);		
		//while( DMA_GetCmdStatus(DMA1_Stream5) );
		DMA1_Stream5->NDTR = (uint32_t)BUFF_SIZE;
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
		DMA_Cmd(DMA1_Stream5, ENABLE);
		//getFlag = DMA_GetCmdStatus(DMA1_Stream5);
		return (uint16_t)(BUFF_SIZE - pre_NDTR);
}

void init_main(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure; 
	DMA_InitTypeDef   DMA_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
	NVIC_InitTypeDef  NVIC_UART_InitStructure;
	NVIC_InitTypeDef	NVIC_TIM_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   
  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable USART2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* TIM7 Peripheral clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	
	/* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 839; //clk = SystemCoreClock /2/(PSC+1) = 1MHz
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 9999;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	/* Configure PD12, PD13, PD14, PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
  /* Connect UASRT2 pins to AF7 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); 

  /* GPIO Configuration for USART2 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);

  /* Enable USART */
  USART_Cmd(USART2, ENABLE);
	
	/* Enable USART2 DMA */
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	
	/* DMA1 Stream5 Channel4 for USART2 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff; //DMA_Memory1BaseAddr s? d?ng cho mode double-buffer
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // không tang d?a ch? ngo?i vi
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // tang d?a ch? memory
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         //khúc này xu?ng d? c?u hình FIFO nhwmg hình nhu không xài
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream5, ENABLE);
	
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);
	
	//
	NVIC_UART_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_UART_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_UART_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_UART_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_UART_InitStructure);
	
	/* Enable TIM Interrupt */
	NVIC_TIM_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_TIM_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_TIM_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_TIM_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_TIM_InitStructure);
	
	/* Enable TIM7 */
	TIM_Cmd(TIM7, ENABLE);
	
	/* Enable IT TIM7 */
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	
	//
	//USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
  /* Transfer complete (TC) interrupt mask */
  //MA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
	
	//DMA_ITConfig(DMA1_Stream5, DMA_IT_HT, ENABLE);
	
}

void DMA1_Stream5_IRQHandler(void)
{
  uint16_t i;

  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
	DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_HTIF5);
  for(i=0; i<BUFF_SIZE; i++)
    a[indexArray + i] = rxbuff[i];
	indexArray = indexArray + BUFF_SIZE;
  rcv_flag = 1;
	DMA_Cmd(DMA1_Stream5, ENABLE);
}

void USART2_IRQHandler(void) 
{
	
}
void TIM7_IRQHandler(void) 
{
	//TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_ClearFlag(TIM7, TIM_IT_Update); //difference between this func with TIM_ClearITPendingBit
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	get_mess_from_rx();
	//printf("\n\rConfig completed!!!\n\r");
	print_f("hello\n");
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
