#include "usart1.h"
#include <string.h>
volatile u8 rx_buffer[RX_BUFFER_SIZE];
volatile u8 tx_buffer[TX_BUFFER_SIZE];

volatile u8 rx_bfr_head, rx_bfr_tail = 0;
volatile u8 tx_bfr_head, tx_bfr_tail = 0;



void USART1_IRQHandler(void) {
	volatile u32 usart1_isr = USART1->ISR;

	if (usart1_isr & USART_ISR_RXNE) {
		rx_buffer[rx_bfr_head] = USART1->RDR;
	
		if (rx_bfr_head == rx_bfr_tail)
			rx_bfr_tail++;

		if (rx_bfr_tail >= RX_BUFFER_SIZE)
			rx_bfr_tail = 0;

		rx_bfr_head++;

		if (rx_bfr_head >= RX_BUFFER_SIZE)
			rx_bfr_head = 0;
	}

	if ( (usart1_isr & USART_ISR_TXE) ) {
		USART1->TDR = (unsigned char) tx_buffer[tx_bfr_tail] & 0xFF;
		tx_bfr_tail = (tx_bfr_tail + 1) % TX_BUFFER_SIZE;

		if (tx_bfr_tail == tx_bfr_head) {
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}

	}
}

int usart1_puts(char *ptr, u8 size){
	u8 i;
	u8 count = 0;

	for (i = 0; i < size; i++) {
		tx_buffer[tx_bfr_head] = *ptr++;
		tx_bfr_head = (tx_bfr_head + 1) % TX_BUFFER_SIZE;
		count ++;
		
		while (tx_bfr_head == tx_bfr_tail)
			__ASM("nop"); /* Block until there is buffer space available*/

	}

	USART1->CR1 |= USART_CR1_TXEIE;

	return count;

}

int usart1_read(char *ptr, u8 size) {
	return 0;
}

void usart1_init(void){
  /* Configure USART1 */
  
  /* GPIO Pins set to 'Alternate function' */
  GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); 
  /* Configure alternate function register */
  GPIOB->AFR[0] |= (7 << 24);
  GPIOB->AFR[0] |= (7 << 28);

  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  RCC->CFGR3 |= RCC_CFGR3_USART1SW_0; /* Use System CLK for USART1*/

  USART1->CR1 = (0xE0000000 & USART1->CR1 ) | (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE  );
  USART1->BRR = ((USART1->BRR & 0xFFFF0000) | 625); /* 115200 baud */

  	bzero ((void *)&rx_buffer[0],RX_BUFFER_SIZE);
  	bzero ((void *)&tx_buffer[0],TX_BUFFER_SIZE);

	NVIC_EnableIRQ(USART1_IRQn);

}