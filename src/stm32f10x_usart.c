
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"


#define CR1_CLEAR_Mask            ((uint16_t)0xE9F3)  /*!< USART CR1 Mask */
#define CR2_STOP_CLEAR_Mask       ((uint16_t)0xCFFF)  /*!< USART CR2 STOP Bits Mask */
#define CR3_CLEAR_Mask            ((uint16_t)0xFCFF)  /*!< USART CR3 Mask */
#define IT_Mask                   ((uint16_t)0x001F)  /*!< USART Interrupt Mask */

/* USART OverSampling-8 Mask */
#define CR1_OVER8_Set             ((u16)0x8000)  /* USART OVER8 mode Enable Mask */

/**
  * @brief  Initializes the USARTx peripheral according to the specified
  *         parameters in the USART_InitStruct .
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *         that contains the configuration information for the specified USART 
  *         peripheral.
  * @retval None
  */
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
  uint32_t tmpreg = 0x00, apbclock = 0x00;
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  uint32_t usartxbase = 0;
  RCC_ClocksTypeDef RCC_ClocksStatus;

  usartxbase = (uint32_t)USARTx;

/*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;
  /* Clear STOP[13:12] bits */
  tmpreg &= CR2_STOP_CLEAR_Mask;
  /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
  /* Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_StopBits;
  
  /* Write to USART CR2 */
  USARTx->CR2 = (uint16_t)tmpreg;

/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USARTx->CR1;
  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= CR1_CLEAR_Mask;
  /* Configure the USART Word Length, Parity and mode ----------------------- */
  /* Set the M bits according to USART_WordLength value */
  /* Set PCE and PS bits according to USART_Parity value */
  /* Set TE and RE bits according to USART_Mode value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity |
            USART_InitStruct->USART_Mode;
  /* Write to USART CR1 */
  USARTx->CR1 = (uint16_t)tmpreg;

/*---------------------------- USART CR3 Configuration -----------------------*/  
  tmpreg = USARTx->CR3;
  /* Clear CTSE and RTSE bits */
  tmpreg &= CR3_CLEAR_Mask;
  /* Configure the USART HFC -------------------------------------------------*/
  /* Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
  tmpreg |= USART_InitStruct->USART_HardwareFlowControl;
  /* Write to USART CR3 */
  USARTx->CR3 = (uint16_t)tmpreg;

/*---------------------------- USART BRR Configuration -----------------------*/
  /* Configure the USART Baud Rate -------------------------------------------*/
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  if (usartxbase == USART1_BASE)
  {
    apbclock = RCC_ClocksStatus.PCLK2_Frequency;
  }
  else
  {
    apbclock = RCC_ClocksStatus.PCLK1_Frequency;
  }
  
  /* Determine the integer part */
  if ((USARTx->CR1 & CR1_OVER8_Set) != 0)
  {
    /* Integer part computing in case Oversampling mode is 8 Samples */
    integerdivider = ((25 * apbclock) / (2 * (USART_InitStruct->USART_BaudRate)));    
  }
  else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
  {
    /* Integer part computing in case Oversampling mode is 16 Samples */
    integerdivider = ((25 * apbclock) / (4 * (USART_InitStruct->USART_BaudRate)));    
  }
  tmpreg = (integerdivider / 100) << 4;

  /* Determine the fractional part */
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

  /* Implement the fractional part in the register */
  if ((USARTx->CR1 & CR1_OVER8_Set) != 0)
  {
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  }
  else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
  {
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
  }
  
  /* Write to USART BRR */
  USARTx->BRR = (uint16_t)tmpreg;
}

/**
  * @brief  Enables or disables the specified USART interrupts.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TXE:  Transmit Data Register empty interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *     @arg USART_IT_IDLE: Idle line detection interrupt
  *     @arg USART_IT_PE:   Parity Error interrupt
  *     @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @param  NewState: new state of the specified USARTx interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState)
{
  uint32_t usartreg = 0x00, itpos = 0x00, itmask = 0x00;
  uint32_t usartxbase = 0x00;
  /* Check the parameters */
  
  usartxbase = (uint32_t)USARTx;

  /* Get the USART register index */
  usartreg = (((uint8_t)USART_IT) >> 0x05);

  /* Get the interrupt position */
  itpos = USART_IT & IT_Mask;
  itmask = (((uint32_t)0x01) << itpos);
    
  if (usartreg == 0x01) /* The IT is in CR1 register */
  {
    usartxbase += 0x0C;
  }
  else if (usartreg == 0x02) /* The IT is in CR2 register */
  {
    usartxbase += 0x10;
  }
  else /* The IT is in CR3 register */
  {
    usartxbase += 0x14; 
  }
  if (NewState != DISABLE)
  {
    *(__IO uint32_t*)usartxbase  |= itmask;
  }
  else
  {
    *(__IO uint32_t*)usartxbase &= ~itmask;
  }
}
