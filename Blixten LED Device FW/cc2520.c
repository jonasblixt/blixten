#include <stdio.h>
#include <string.h>
#include "stm32f30x.h"
#include "cc2520.h"
#include "cmsis_os.h"

#define BUFFER_SIZE 128

volatile u8 rx_buffer[BUFFER_SIZE];
volatile u8 tx_buffer[BUFFER_SIZE];
volatile u8 dma_trx_complete = true;

void cc2520_thread (void const *argument);                             // thread function
osThreadId tid_cc2520_thread;                                          // thread id
osThreadDef (cc2520_thread, osPriorityNormal, 1, 0);                   // thread object

#define CC2520_STATE_INIT 0
#define CC2520_STATE_XTALON 1
#define CC2520_STATE_IDLE 2

void cc2520_dma_trx(u8 txlen, u8 rxlen) {
	dma_trx_complete = false;

	/* Kanske onödigt att nollställa helt för varje förfrågan?! */
	bzero((void *) &rx_buffer[0],BUFFER_SIZE);
	bzero((void *) &tx_buffer[txlen],BUFFER_SIZE-txlen);

	/* Setup DMA */

	/* Poll transfer complete ISR flag */
	/* osThreadYeild() */
}

void cc2520_thread (void const *argument) {
	u8 state = CC2520_STATE_INIT;
	u8 status = 0 ;
	u16 timeout = 0;

	while (1) {
		switch (state) {
			case CC2520_STATE_INIT:
				timeout = 1000;
			break;
			case CC2520_STATE_XTALON:
				cc2520_dma_trx(CC2520_CMD_SXOSCON,&status,1);

				while (dma_trx_complete)
					osThreadYield();

				timeout--;

				if (timeout <= 0) {
					printf ("cc2520: timeout initializing oscillator!\n\r");
					state = CC2520_STATE_INIT;
				}

				if (status & CC2520_STATUS_XOSC32M_STABLE) {
					printf ("cc2520: oscillator stable.\n\r");
					state = CC2520_STATE_IDLE;
				}

			break;
			case CC2520_STATE_IDLE:
			break;
			default:
				printf ("CC2520: Unknown state (%i)\r\n",(state));
				state = CC2520_STATE_INIT;
			break;
		}
  		osThreadYield();
  	}
}

void cc2520_init(void) {
	printf ("CC2520 Init...\r\n");

	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;			/* Enable SPI3 clock */

	GPIOA->MODER |= GPIO_MODER_MODER15_1; 		/* Configure Alternate function on PA15 (CS_N) */
 	GPIOA->AFR[1] |= (6 << 28);					/* AF6 = SPI3/CS_N */

	GPIOB->MODER |= GPIO_MODER_MODER3_1; 		/* Configure Alternate function on PB3 (SCK) */
 	GPIOB->AFR[0] |= (6 << 12);					/* AF6 = SPI3/SCK */

	GPIOB->MODER |= GPIO_MODER_MODER4_1; 		/* Configure Alternate function on PB4 (MISO) */
 	GPIOB->AFR[0] |= (6 << 16);					/* AF6 = SPI3/MISO */

	GPIOB->MODER |= GPIO_MODER_MODER5_1; 		/* Configure Alternate function on PB5 (MOSI) */
 	GPIOB->AFR[0] |= (6 << 20);					/* AF6 = SPI3/MOSI */

	//SPI3->CR1 |= SPI_CR1_SSM;
	SPI3->CR1 |= SPI_CR1_BR_2; /* f_pclk / 32 */
	SPI3->CR1 |= SPI_CR1_MSTR;
	


	SPI3->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

	SPI3->CR2 |= SPI_CR2_SSOE;


	/* Configure DMA */ 
	//DMA1_Channel7->CPAR = (int) &(TIM2->DMAR);
	/* Source data is 8-bit destination register is word sized. */
	//DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC|DMA_CCR_TCIE|DMA_CCR_PSIZE_1;
	//DMA1->IFCR = 0xFFFFFFFF;
	//NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	//SPI3->CR1 |= SPI_CR1_SPE;

	 tid_cc2520_thread = osThreadCreate (osThread(cc2520_thread), NULL);

}