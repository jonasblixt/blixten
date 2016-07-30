


void neopixel_rgbw_init(void){


	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;

 	GPIOA->MODER |= GPIO_MODER_MODER10_1; /* Configure Alternate function on PA10 */
 	GPIOA->AFR[1] |= (10 << 8); /* Connect Timer 2 CH4 to PA10*/
 	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10; /* Set high drive strength*/

	/* Enable TIM2 clock on APB1 x2 (72 MHz) */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 

	
	TIM2->CR1 &= ~TIM_CR1_CKD;
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM1->CR1 &= ~TIM_CR1_ARPE;


	/* The neopixel RGBW -diode timing requirement is :
		1.25us +/- 600ns
	 */
	TIM2->ARR = 86; /* 1/72e6 * 86 = 1.194 us period time */
	TIM2->CNT = 0;
	TIM2->CCMR2 &= ~TIM_CCMR2_CC4S; /* Configure Timer 2 CH4 as output */
	TIM2->CR1 |= TIM_CR1_CMS_1 ;
	//TIM2->CCMR2 |= TIM_CCMR2_OC4PE; /* Automatically re-load CCR4 */
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; /* PWM Mode 1 */
	TIM2->CCR4 = 40;
	TIM2->CCER |= TIM_CCER_CC4E;
	TIM2->EGR =  TIM_EGR_CC4G;
	/* Configure TIM2 to generate DMA requests when one PWM cycle is complete */

	TIM2->DCR |= (0 << 8); /* DMA Data burst length = 1 */
	TIM2->DCR |= 16; /* Location of CCR4 for DMA Update */
	TIM2->DIER |= TIM_DIER_CC4DE | TIM_DIER_UDE; /* Generate DMA req on CH4 */	
	TIM2->SR = 0;

	/* Configure DMA */


	DMA1_Channel7->CMAR =  &led_data[0]; 
	DMA1_Channel7->CPAR =  TIM2->DMAR;
	DMA1_Channel7->CNDTR = 32;
	DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE |DMA_CCR_MSIZE_0  | DMA_CCR_PSIZE_0 ;
	DMA1->IFCR = 0xFFFFFFFF;
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	

	DMA1_Channel7->CCR |= DMA_CCR_EN;
	TIM2->CR1 |= TIM_CR1_CEN; /* Enable timer*/

	TIM2->EGR =  TIM_EGR_CC4G | TIM_EGR_UG; /* Trigger update */



}