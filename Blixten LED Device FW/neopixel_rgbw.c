/**
 * NRZ data generator for the RGBW Neopixel.
 * Implemented for the STM32F302K8
 *
 * (C) 2016 Jonas Persson <jonpe960@gmail.com>
 * 					   ____
 *  PWM Output:   |___|    |
 *  
 *  
 *
 */


#include "neopixel_rgbw.h"
#include <string.h>

#define NEOPIXEL_N_LEDS 3
#define NEOPIXEL_PWM_F (u8) (72000000 / 833333)
#define NEOPIXEL_H (u8) (NEOPIXEL_PWM_F/2)
#define NEOPIXEL_L (u8) (NEOPIXEL_PWM_F/4)


static u8 data_buffer[2][32*NEOPIXEL_N_LEDS]; /* DMA ping-pong buffers for NRZ data */
volatile u8 active_buffer = 0;
volatile u8 buffer_changed = 0;
volatile u16 frame_count = 0;
static u32 rgbw_buffer[NEOPIXEL_N_LEDS]; 

/* TIM6 is used to start and update LED's at a constant rate */
void TIM6_DAC_IRQHandler(void) {

	TIM6->SR &= ~TIM_SR_UIF;

	frame_count++;

	if (buffer_changed) {
		buffer_changed = 0;
		active_buffer = active_buffer>0?0:1;
	}

	DMA1_Channel7->CMAR = (int) (data_buffer[active_buffer]);
	DMA1_Channel7->CNDTR = 32*NEOPIXEL_N_LEDS;
	DMA1_Channel7->CCR |= DMA_CCR_EN;

	TIM2->CNT = 0;
	TIM2->CCR4 = 0; 
	TIM2->CCR2 = NEOPIXEL_PWM_F;
	TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

	/* Force DMA update */
	TIM2->EGR =  TIM_EGR_CC2G;
	TIM2->CR1 |= TIM_CR1_CEN;		

}


/* TIM2 CH4 is available on DMA1 Channel 7 */
void DMA1_Channel7_IRQHandler(void) {

	u32 dma_isr = DMA1->ISR;

	/*  Buffer transfer completed. At this point
		all of the data is transferred but the timer is
		still generating the last NRZ bit.

		To allow for the last bit to be generated a
		'dummy' (0) is appended to the buffer.

		To generate the 80us lead -out (reset) and produce
		 a constant refresh rate TIM6 is activated.
	*/
	if (dma_isr & DMA_ISR_TCIF7) {
		DMA1->IFCR = DMA_IFCR_CTCIF7;
		TIM2->CR1 &= ~TIM_CR1_CEN;
		TIM2->CNT = 0;
		TIM2->CCR4 = 0; 
		TIM2->CCR2 = NEOPIXEL_PWM_F;
		DMA1_Channel7->CCR &= ~DMA_CCR_EN;

		TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;
		TIM2->CCMR2 |= TIM_CCMR2_OC4M_2;
	}


}
/* Generate NRZ data in the buffer not being used 
  for an active DMA transfer */
void neopixel_update_buffer(void) {
	u8 i,n = 0;
	u32 rgbw_val = 0;
	u8* update_buffer_ptr;

	if (active_buffer)
		update_buffer_ptr = data_buffer[0];
	else
		update_buffer_ptr = data_buffer[1];


	for (i = 0; i < NEOPIXEL_N_LEDS; i++) {
		rgbw_val = rgbw_buffer[i];

		for (n = 0; n < 32; n++) {
			if (rgbw_val & 0x80000000)
				*update_buffer_ptr = NEOPIXEL_H;
			else
				*update_buffer_ptr = NEOPIXEL_L;

			update_buffer_ptr++;
			rgbw_val = (rgbw_val << 1);
		}

	}

	buffer_changed= 1;
}
/* Update a specific LED value */
int neopixel_rgbw_set_led(u8 led_index, u32 rgbw_value) {
	if (led_index < NEOPIXEL_N_LEDS) {
		rgbw_buffer[led_index] = rgbw_value;
		return 0;
	}
	return -1;
}

u16 neopixel_get_framecount(void) {
	return frame_count;
}

u8 neopixel_buffer_available(void) {
	return !buffer_changed;
}


void neopixel_rgbw_init(void){


	/*DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP; // Stop TIM2 when cpu is halted, for debuging */

 	GPIOA->MODER |= GPIO_MODER_MODER10_1; 		/* Configure Alternate function on PA10 */
 	GPIOA->AFR[1] |= (10 << 8); 				/* Connect Timer 2 CH4 to PA10*/
 	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10; 	/* Set high drive strength*/

	/* Enable TIM2, TIM6 clock on APB1 x2 (72 MHz) */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN; 

	/* Disable timer to enable configuration*/
	TIM2->CR1 &= ~TIM_CR1_CKD;
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM1->CR1 &= ~TIM_CR1_ARPE;


	/*  Timer2 CH4 is used to generate the PWM signal
		The neopixel RGBW -diode timing requirement is :
		1.20us +/- 300ns (the datasheet is a bit conflicting on this)
		the table suggests the above timing. On another place in the
		data sheet this timing is suggested: 1.25us +/- 600ns.

		This code will meet the tougher timing requirement, at lest
		on the STM32F302K8, with 72MHz timer clock input.
	 */

	TIM2->ARR = NEOPIXEL_PWM_F; 	/* 1/72e6 * 86 = 1.194 us period time */
	TIM2->CNT = 0;
	TIM2->CCMR2 &= ~TIM_CCMR2_CC4S; /* Configure Timer 2 CH4 as output */
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; /* PWM Mode 2 */
	TIM2->CCR4 = 0x0;
	TIM2->CCER |= TIM_CCER_CC4E; 	/* Enable output pin */
	

	/* Configure TIM2 CH2 to generate DMA requests for every complete PWM cycle */
	TIM2->CCR2 = NEOPIXEL_PWM_F;
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE; /* Automatically re-load CCR2 */
	TIM2->DCR |= (0 << 8); 			/* DMA Data burst length = 1 */
	TIM2->DCR |= 16; 				/* Location of CCR4 for DMA Update to update PWM */
	TIM2->DIER |= TIM_DIER_CC2DE; 	/* Generate DMA req on CH2 */	
	TIM2->SR = 0;

	/* Configure DMA */ 
	DMA1_Channel7->CPAR = (int) &(TIM2->DMAR);
	/* Source data is 8-bit destination register is word sized. */
	DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC|DMA_CCR_TCIE|DMA_CCR_PSIZE_1;
	DMA1->IFCR = 0xFFFFFFFF;
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	/* Configure refresh timer, 25Hz */
	TIM6->PSC = 72;
	TIM6->CR1 |= TIM_CR1_ARPE;
	TIM6->ARR = 20000;
	TIM6->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->CR1 |= TIM_CR1_CEN;
	/* Reset buffers, all LED's off */
}
