

#include <misc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_iwdg.h>

#include "servo1.h"
#include "data.h"


extern	const	PwmChannel	pwmCh[32];

void SystemInit (void);

void
init(void);

//	tools
void
cleanUSARTBuffer(void);

void
cleanLogUSARTBuffer(void);

int
parseUsartMsg(void);

void
doCntrl(void);

void
logBuffer(void);




int
main(void)
{
	int	p;
  	SystemInit();
	init();

	for ( ; ; )	{
		/* Reload IWDG counter */
		IWDG_ReloadCounter();

		if (rxPos > 0)	{
			if (rxPos < (BUFSZ - 1))	{
				p	=	parseUsartMsg();
				if (p == ACK)	{
						doCntrl();
				}
			}
			else	{
				cleanUSARTBuffer();
			}
		}
	}
}


void
initTIM(void)
{
	TIM_TimeBaseInitTypeDef	timInit;
	TIM_OCInitTypeDef	timOCInit;

	RCC->APB1ENR |= (RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM2);

	RCC->APB1RSTR |= RCC_APB1Periph_TIM3;
	RCC->APB1RSTR &= ~RCC_APB1Periph_TIM3;
	
	RCC->APB1RSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1RSTR &= ~RCC_APB1Periph_TIM2;

	timInit.TIM_Period	=	20000 - 1;	//	20 ms
	timInit.TIM_Prescaler	=	71;		//
	timInit.TIM_ClockDivision	=	0;
	timInit.TIM_CounterMode	=	TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &timInit);

	/* PWM1 Mode configuration: Channel1 */
	timOCInit.TIM_OCMode = TIM_OCMode_Inactive;
	timOCInit.TIM_OutputState = TIM_OutputState_Enable;
	timOCInit.TIM_Pulse = 999;
	timOCInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &timOCInit);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	timOCInit.TIM_OutputState = TIM_OutputState_Enable;
	timOCInit.TIM_Pulse = 1799;

	TIM_OC2Init(TIM3, &timOCInit);

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* Set the ARR Preload Bit */
    TIM3->CR1 |= TIM_CR1_ARPE;
	/* Enable the Interrupt sources */
    TIM3->DIER |= (TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2);
	/* TIM3 enable counter */
    TIM3->CR1 |= TIM_CR1_CEN;
	
	
	timInit.TIM_Period	=	(999);		//	1 ms
	timInit.TIM_Prescaler	=	71;		//
	timInit.TIM_ClockDivision	=	0;
	timInit.TIM_CounterMode	=	TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2, &timInit);
	/* Enable the Interrupt sources */
    TIM2->DIER |= (TIM_IT_Update);
	/* TIM2 enable counter */
	TIM2->CR1 |= TIM_CR1_CEN;
}


void
initUSART(void)
{
	USART_InitTypeDef usartInit;

	rxPos	=	BUFSZ;
	cleanUSARTBuffer();

	RCC->APB1ENR |= RCC_APB1Periph_USART3;

	RCC->APB1RSTR |= RCC_APB1Periph_USART3;
	RCC->APB1RSTR &= ~RCC_APB1Periph_USART3;


	/* USART3 configuration ------------------------------------------------------*/
	/* USART3 configured as follow:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- parity bit
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/
	usartInit.USART_BaudRate	=	115200;
	usartInit.USART_WordLength	=	USART_WordLength_8b;
	usartInit.USART_StopBits	=	USART_StopBits_1;
	usartInit.USART_Parity		=	USART_Parity_No;
	usartInit.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
	usartInit.USART_Mode		=	USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART3, &usartInit);

	/* Enable USART */
    USART3->CR1 |= CR1_UE_Set;
}

void
initGPIO(void)
{
	GPIO_InitTypeDef	gpioInit;
	int	i;

	//RCC_APB2PeriphClockCmd(, ENABLE);
	RCC->APB2ENR |=	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO);
	
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);

	gpioInit.GPIO_Mode	=	GPIO_Mode_Out_PP;
	gpioInit.GPIO_Speed	=	GPIO_Speed_10MHz;
	gpioInit.GPIO_Pin	=	0;
	
	for (i = 0; i < 20; i++)	{
	  if (pwmCh[i].gpio == (GPIO_TypeDef*) 0)
		break;
	  if (pwmCh[i].gpio == GPIOA)
		gpioInit.GPIO_Pin	|=	pwmCh[i].pin;
	}

	GPIO_Init(GPIOA, &gpioInit);
	
	gpioInit.GPIO_Pin	=	0;
	
	for ( ; i < 20; i++)	{
	  if (pwmCh[i].gpio == (GPIO_TypeDef*) 0)
		break;
	  if (pwmCh[i].gpio == GPIOB)
		gpioInit.GPIO_Pin	|=	pwmCh[i].pin;
	}
	
	GPIO_Init(GPIOB, &gpioInit);
	
	//	init USART pins
	gpioInit.GPIO_Mode	=	GPIO_Mode_AF_PP;
	gpioInit.GPIO_Pin	=	GPIO_Pin_10;
	gpioInit.GPIO_Speed	=	GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &gpioInit);

	gpioInit.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	gpioInit.GPIO_Pin	=	GPIO_Pin_11;
	gpioInit.GPIO_Speed	=	GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &gpioInit);
}


void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  /* Enable the TIM3 Interrupts */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the TIM3 Interrupts */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  NVIC_EnableIRQ(USART3_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_EnableIRQ(TIM2_IRQn);
}

void
initIWDG(void)
{
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz */
  IWDG_SetPrescaler(IWDG_Prescaler_32);

  /* Set counter reload value to 349 */
  IWDG_SetReload(349);

  /* Reload IWDG counter */
  IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
}


void
init(void)
{
	int	i;
	
	/* Check if the system has resumed from IWDG reset */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{
		/* IWDGRST flag set */
		/* Clear reset flags */
		//RCC_ClearFlag();
		/* Set RMVF bit to clear the reset flags */
		RCC->CSR |= CSR_RMVF_Set;
	}

	for (i = 0; i < 20; i++)	{
		channelState[i].chVal	=	0;
		channelState[i].chState	=	0;
	}
	
	initGPIO();

	initUSART();

	initTIM();
	
	NVIC_Configuration();

	initIWDG();
}

/////////////////////////////////////////////////////////////////////////
//	eof
