

#include <stdint.h>
#include <stm32f10x_gpio.h>


#define	BUFSZ	512

#define	LEN		0
#define	CMD		1
#define	DATA		2
#define	CH_SUM_L	3
#define	CH_SUM_H	4
#define	ACK		5

#define CR1_UE_Set                ((uint16_t)0x2000)  /*!< USART Enable Mask */
/* CSR register bit mask */
#define CSR_RMVF_Set              ((uint32_t)0x01000000)


typedef struct	{
	GPIO_TypeDef	*gpio;
	uint16_t		pin;
}	PwmChannel;


typedef struct  {
	uint32_t	chVal		:	16;
	uint32_t	chState		:	1;
	uint32_t				:	15;
}	Chann;


