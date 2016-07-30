#include <stdio.h>
#include "stm32f30x.h"
#include "syscalls.h"
#include "usart1.h"
#include "neopixel_rgbw.h"
#include "timers.h"
#include "cc2520.h"
#include "arm_math.h"
#include "arm_common_tables.h"
#include "cmsis_os.h"
 

/*----------------------------------------------------------------------------
 *      RTX User configuration part BEGIN
 *---------------------------------------------------------------------------*/
 
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
//
// <h>Thread Configuration
// =======================
//
//   <o>Number of concurrent running user threads <1-250>
//   <i> Defines max. number of user threads that will run at the same time.
//   <i> Default: 6
#ifndef OS_TASKCNT
 #define OS_TASKCNT     6
#endif
 
//   <o>Default Thread stack size [bytes] <64-4096:8><#/4>
//   <i> Defines default stack size for threads with osThreadDef stacksz = 0
//   <i> Default: 200
#ifndef OS_STKSIZE
 #define OS_STKSIZE     500      // this stack size value is in words
#endif
 
//   <o>Main Thread stack size [bytes] <64-32768:8><#/4>
//   <i> Defines stack size for main thread.
//   <i> Default: 200
#ifndef OS_MAINSTKSIZE
 #define OS_MAINSTKSIZE 500      // this stack size value is in words
#endif
 
//   <o>Number of threads with user-provided stack size <0-250>
//   <i> Defines the number of threads with user-provided stack size.
//   <i> Default: 0
#ifndef OS_PRIVCNT
 #define OS_PRIVCNT     0
#endif
 
//   <o>Total stack size [bytes] for threads with user-provided stack size <0-1048576:8><#/4>
//   <i> Defines the combined stack size for threads with user-provided stack size.
//   <i> Default: 0
#ifndef OS_PRIVSTKSIZE
 #define OS_PRIVSTKSIZE 0       // this stack size value is in words
#endif
 
//   <q>Stack overflow checking
//   <i> Enable stack overflow checks at thread switch.
//   <i> Enabling this option increases slightly the execution time of a thread switch.
#ifndef OS_STKCHECK
 #define OS_STKCHECK    1
#endif
 
//   <q>Stack usage watermark
//   <i> Initialize thread stack with watermark pattern for analyzing stack usage (current/maximum) in System and Thread Viewer.
//   <i> Enabling this option increases significantly the execution time of osThreadCreate.
#ifndef OS_STKINIT
#define OS_STKINIT      0
#endif
 
//   <o>Processor mode for thread execution 
//     <0=> Unprivileged mode 
//     <1=> Privileged mode
//   <i> Default: Privileged mode
#ifndef OS_RUNPRIV
 #define OS_RUNPRIV     1
#endif
 

#define OS_SYSTICK     1

//   <o>RTOS Kernel Timer input clock frequency [Hz] <1-1000000000>
//   <i> Defines the input frequency of the RTOS Kernel Timer.  
//   <i> When the Cortex-M SysTick timer is used, the input clock 
//   <i> is on most systems identical with the core clock.
#ifndef OS_CLOCK
 #define OS_CLOCK       72000000
#endif
 
//   <o>RTX Timer tick interval value [us] <1-1000000>
//   <i> The RTX Timer tick interval value is used to calculate timeout values.
//   <i> When the Cortex-M SysTick timer is enabled, the value also configures the SysTick timer.
//   <i> Default: 1000  (1ms)
#ifndef OS_TICK
 #define OS_TICK        1000
#endif
 
// </h>
 
// <h>System Configuration
// =======================
//
// <e>Round-Robin Thread switching
// ===============================
//
// <i> Enables Round-Robin Thread switching.
#ifndef OS_ROBIN
 #define OS_ROBIN       1
#endif
 
//   <o>Round-Robin Timeout [ticks] <1-1000>
//   <i> Defines how long a thread will execute before a thread switch.
//   <i> Default: 5
#ifndef OS_ROBINTOUT
 #define OS_ROBINTOUT   5
#endif
 
// </e>
 
// <e>User Timers
// ==============
//   <i> Enables user Timers
#ifndef OS_TIMERS
 #define OS_TIMERS      1
#endif
 
//   <o>Timer Thread Priority
//                        <1=> Low
//     <2=> Below Normal  <3=> Normal  <4=> Above Normal
//                        <5=> High
//                        <6=> Realtime (highest)
//   <i> Defines priority for Timer Thread
//   <i> Default: High
#ifndef OS_TIMERPRIO
 #define OS_TIMERPRIO   5
#endif
 
//   <o>Timer Thread stack size [bytes] <64-4096:8><#/4>
//   <i> Defines stack size for Timer thread.
//   <i> Default: 200
#ifndef OS_TIMERSTKSZ
 #define OS_TIMERSTKSZ  50     // this stack size value is in words
#endif
 
//   <o>Timer Callback Queue size <1-32>
//   <i> Number of concurrent active timer callback functions.
//   <i> Default: 4
#ifndef OS_TIMERCBQS
 #define OS_TIMERCBQS   4
#endif
 
// </e>
 
//   <o>ISR FIFO Queue size<4=>   4 entries  <8=>   8 entries
//                         <12=> 12 entries  <16=> 16 entries
//                         <24=> 24 entries  <32=> 32 entries
//                         <48=> 48 entries  <64=> 64 entries
//                         <96=> 96 entries
//   <i> ISR functions store requests to this buffer,
//   <i> when they are called from the interrupt handler.
//   <i> Default: 16 entries
#ifndef OS_FIFOSZ
 #define OS_FIFOSZ      16
#endif
 
// </h>
 
//------------- <<< end of configuration section >>> -----------------------
 
// Standard library system mutexes
// ===============================
//  Define max. number system mutexes that are used to protect 
//  the arm standard runtime library. For microlib they are not used.
#ifndef OS_MUTEXCNT
 #define OS_MUTEXCNT    8
#endif
 
/*----------------------------------------------------------------------------
 *      RTX User configuration part END
 *---------------------------------------------------------------------------*/
 
#define OS_TRV          ((uint32_t)(((double)OS_CLOCK*(double)OS_TICK)/1E6)-1)
 

/*----------------------------------------------------------------------------
 *      Global Functions
 *---------------------------------------------------------------------------*/
 
/*--------------------------- os_idle_demon ---------------------------------*/

/// \brief The idle demon is running when no other thread is ready to run
void os_idle_demon (void) {
 
  for (;;) {
    /* HERE: include optional user code to be executed when no thread runs.*/
  }
}
 
 
/*--------------------------- os_error --------------------------------------*/
 
/* OS Error Codes */
#define OS_ERROR_STACK_OVF      1
#define OS_ERROR_FIFO_OVF       2
#define OS_ERROR_MBX_OVF        3
#define OS_ERROR_TIMER_OVF      4
 
extern osThreadId svcThreadGetId (void);
 
/// \brief Called when a runtime error is detected
/// \param[in]   error_code   actual error code that has been detected
void os_error (uint32_t error_code) {
 
  /* HERE: include optional code to be executed on runtime error. */
  switch (error_code) {
    case OS_ERROR_STACK_OVF:
      /* Stack overflow detected for the currently running task. */
      /* Thread can be identified by calling svcThreadGetId().   */
      break;
    case OS_ERROR_FIFO_OVF:
      /* ISR FIFO Queue buffer overflow detected. */
      break;
    case OS_ERROR_MBX_OVF:
      /* Mailbox overflow detected. */
      break;
    case OS_ERROR_TIMER_OVF:
      /* User Timer Callback Queue overflow detected. */
      break;
    default:
      break;
  }
  for (;;);
}
 

/*----------------------------------------------------------------------------
 *      RTX Configuration Functions
 *---------------------------------------------------------------------------*/
 
#include "RTX_CM_lib.h"

volatile u32 systick=0;

unsigned int system_get_tick() {
  return systick;
}

/*void SysTick_Handler(void) {
  systick++;
}*/

void NMI_Handler(void) {for(;;);}
void HardFault_Handler(void) {
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}
void MemManage_Handler(void) {for(;;);}
void BusFault_Handler(void) {for(;;);}
void UsageFault_Handler(void) {for(;;);}
//void SVC_Handler(void) {for(;;);}
void DebugMon_Handler(void) {for(;;);}
//void PendSV_Handler(void) {for(;;);}
void WWDG_IRQHandler(void) {for(;;);}
void PVD_IRQHandler(void) {for(;;);}
void TAMPER_STAMP_IRQHandler(void) {for(;;);}
void RTC_WKUP_IRQHandler(void) {for(;;);}
void FLASH_IRQHandler(void) {for(;;);}
void RCC_IRQHandler(void) {for(;;);}
void EXTI0_IRQHandler(void) {for(;;);}
void EXTI1_IRQHandler(void) {for(;;);}
void EXTI2_TS_IRQHandler(void) {for(;;);}
void EXTI3_IRQHandler(void) {for(;;);}
void EXTI4_IRQHandler(void) {for(;;);}
void DMA1_Channel1_IRQHandler(void) {for(;;);}
void DMA1_Channel2_IRQHandler(void) {for(;;);}
void DMA1_Channel3_IRQHandler(void) {for(;;);}
void DMA1_Channel4_IRQHandler(void) {for(;;);}
void DMA1_Channel5_IRQHandler(void) {for(;;);}
void DMA1_Channel6_IRQHandler(void) {for(;;);}

void ADC1_2_IRQHandler(void) {for(;;);}
void USB_HP_CAN1_TX_IRQHandler(void) {for(;;);}
void USB_LP_CAN1_RX0_IRQHandler(void) {for(;;);}
void CAN1_RX1_IRQHandler(void) {for(;;);}
void CAN1_SCE_IRQHandler(void) {for(;;);}
void EXTI9_5_IRQHandler(void) {for(;;);}
void TIM1_BRK_TIM15_IRQHandler(void) {for(;;);}
void TIM1_UP_TIM16_IRQHandler(void) {for(;;);}
void TIM1_TRG_COM_TIM17_IRQHandler(void) {for(;;);}
void TIM1_CC_IRQHandler(void) {for(;;);}
void TIM4_IRQHandler(void) {for(;;);}
void TIM3_IRQHandler(void) {for(;;);}
void TIM2_IRQHandler(void) { for(;;); }
void I2C1_EV_IRQHandler(void) {for(;;);}
void I2C1_ER_IRQHandler(void) {for(;;);}
void I2C2_EV_IRQHandler(void) {for(;;);}
void I2C2_ER_IRQHandler(void) {for(;;);}
void SPI1_IRQHandler(void) {for(;;);}
void SPI2_IRQHandler(void) {for(;;);}

void USART2_IRQHandler(void) {for(;;);}
void USART3_IRQHandler(void) {for(;;);}
void EXTI15_10_IRQHandler(void) {for(;;);}
void RTC_Alarm_IRQHandler(void) {for(;;);}
void USBWakeUp_IRQHandler(void) {for(;;);}
void TIM8_BRK_IRQHandler(void) {for(;;);}
void TIM8_UP_IRQHandler(void) {for(;;);}
void TIM8_TRG_COM_IRQHandler(void) {for(;;);}
void TIM8_CC_IRQHandler(void) {for(;;);}
void ADC3_IRQHandler(void) {for(;;);}
void SPI3_IRQHandler(void) {for(;;);}
void UART4_IRQHandler(void) {for(;;);}
void UART5_IRQHandler(void) {for(;;);}

void TIM7_IRQHandler(void) {for(;;);}
void DMA2_Channel1_IRQHandler(void) {for(;;);}
void DMA2_Channel2_IRQHandler(void) {for(;;);}
void DMA2_Channel3_IRQHandler(void) {for(;;);}
void DMA2_Channel4_IRQHandler(void) {for(;;);}
void DMA2_Channel5_IRQHandler(void) {for(;;);}
void ADC4_IRQHandler(void) {for(;;);}
void COMP1_2_3_IRQHandler(void) {for(;;);}
void COMP4_5_6_IRQHandler(void) {for(;;);}
void COMP7_IRQHandler(void) {for(;;);}
void USB_HP_IRQHandler(void) {for(;;);}
void USB_LP_IRQHandler(void) {for(;;);}
void USBWakeUp_RMP_IRQHandler(void) {for(;;);}
void FPU_IRQHandler(void) {for(;;);}

/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
volatile uint32_t prvGetr0;
volatile uint32_t prvGetr1;
volatile uint32_t prvGetr2;
volatile uint32_t prvGetr3;
volatile uint32_t prvGetr12;
volatile uint32_t prvGetlr; /* Link register. */
volatile uint32_t prvGetpc; /* Program counter. */
volatile uint32_t prvGetpsr;/* Program status register. */

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
    prvGetr0  = pulFaultStackAddress[ 0 ];
    prvGetr1  = pulFaultStackAddress[ 1 ];
    prvGetr2  = pulFaultStackAddress[ 2 ];
    prvGetr3  = pulFaultStackAddress[ 3 ];
    prvGetr12 = pulFaultStackAddress[ 4 ];
    prvGetlr  = pulFaultStackAddress[ 5 ];
    prvGetpc  = pulFaultStackAddress[ 6 ];
    prvGetpsr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );
}


void SystemInit(void)
{


  RCC->CR &= ~( RCC_CR_HSEON | RCC_CR_PLLON );
  while ( (RCC->CR & RCC_CR_PLLRDY) );
  RCC->CIR = 0x0;



  /* External XTAL: 8 MHz 
   * XTAL / 1 -> PLL -> 72MHz
   * AHB: 72MHz
   * APB1: 36 MHz
   * APB2: 72 MHz
   */


  RCC->CR |= RCC_CR_HSEON | RCC_CR_HSION;

  while ( !(RCC->CR & RCC_CR_HSERDY) );

  /* Enable Prefetch Buffer and set Flash Latency */
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1 | FLASH_ACR_LATENCY_0;

  /* HCLK = SYSCLK */
  RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

  /* PCLK2 = HCLK */
  RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

  /* PCLK1 = HCLK */
  RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;


  /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                                        RCC_CFGR_PLLMULL));
  RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1  | RCC_CFGR_PLLMULL9 | RCC_CFGR_MCO_PLL );

  RCC->CFGR2 = RCC_CFGR2_PREDIV1_DIV1 ;
  /* Enable PLL */
  RCC->CR |= RCC_CR_PLLON;

  while ( !(RCC->CR & RCC_CR_PLLRDY) );
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN ;

  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

  while (! (RCC->CFGR & RCC_CFGR_SWS_PLL));

  /* Configure GPIO */

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

  GPIOA->MODER |= GPIO_MODER_MODER0_0;
  GPIOA->ODR |= GPIO_ODR_0;

  GPIOA->MODER |= GPIO_MODER_MODER5_0;
  GPIOA->ODR |= GPIO_ODR_5;


  GPIOA->MODER |= GPIO_MODER_MODER2_0;
  GPIOA->ODR |= GPIO_ODR_2;



  GPIOA->MODER |= GPIO_MODER_MODER8_0; /* MCO Output*/
 
  GPIOA->MODER |= GPIO_MODER_MODER7_0; 
  GPIOA->ODR |= GPIO_ODR_7;
  GPIOA->MODER |= GPIO_MODER_MODER9_0; 
  GPIOA->ODR |= GPIO_ODR_9;

  DMA1_Channel1->CCR &= ~DMA_CCR_EN;
  DMA1_Channel2->CCR &= ~DMA_CCR_EN;
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;
  DMA1_Channel4->CCR &= ~DMA_CCR_EN;
  DMA1_Channel5->CCR &= ~DMA_CCR_EN;
  DMA1_Channel6->CCR &= ~DMA_CCR_EN;
  DMA1_Channel7->CCR &= ~DMA_CCR_EN;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* Enable DMA Clock */


  SCB->CPACR |= (0x3 << 10 * 2 | 0x3 << 11 * 2);

  usart1_init();

}

void Thread (void const *argument);                             // thread function
osThreadId tid_Thread;                                          // thread id
osThreadDef (Thread, osPriorityNormal, 1, 0);                   // thread object

  
void Thread_StsLED (void const *argument);                             // thread function
osThreadId tid_StsLEDThread;                                          // thread id
osThreadDef (Thread_StsLED, osPriorityNormal, 1, 0);                   // thread object


void Thread (void const *argument) {
  float32_t theta = 0.0;
  u8 r,g,b,w = 0;

  while (1) {
    if (neopixel_buffer_available()){
      theta = theta + 0.01;
      r = (u8) (127 * (1.0f + arm_sin_f32(theta)));
      g = (u8) (127 * (1.0f + arm_sin_f32(1.5*theta)));
      b = (u8) (127 * (1.0f + arm_sin_f32(2*theta)));
      w = (u8) (32 * (1.0f + arm_sin_f32(0.1*theta)));
      w += (u8) (128 * (1.0f + arm_sin_f32(0.3*theta)));
      neopixel_rgbw_set_led(0,(u32) g<<24|r<<16|b<<8|w);
      neopixel_rgbw_set_led(1,(u32) g<<24|r<<16|b<<8|w);
      neopixel_rgbw_set_led(2,(u32) g<<24|r<<16|b<<8|w);
      neopixel_update_buffer();
      //osDelay(10);
    } else {
      osThreadYield();
    }
  }
}

void Thread_StsLED (void const *argument) {
  u8 onoff = 0;
  u8 count = 0;
  while (1) {

    onoff = !onoff;

    if (onoff)
      GPIOA->ODR |= GPIO_ODR_0;
    else
      GPIOA->ODR &= ~GPIO_ODR_0;

    printf ("Alive: %i\r\n",count++);

    osDelay(1000);
  }
}

int main(void)
{  
  osKernelInitialize ();
  printf ("\r\n---- BLIXTEN ----\r\n");
  printf("CPU: STM32F302K8, 72MHz, running RTX\r\n");
  neopixel_rgbw_init();
  cc2520_init();
  tid_Thread = osThreadCreate (osThread(Thread), NULL);
  tid_StsLEDThread = osThreadCreate (osThread(Thread_StsLED), NULL);
  osKernelStart (); 
}


