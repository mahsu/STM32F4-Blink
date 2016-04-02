/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2015 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>

#include "Board_LED.h"                  // ::Board Support:LED
#include "Board_Buttons.h"              // ::Board Support:Buttons

#include "stm32f4xx.h"                  // Device header
#include "Serial.h"


volatile uint32_t msTicks;                                 // counts 1ms timeTicks
/*----------------------------------------------------------------------------
 * SysTick_Handler:
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}

/*----------------------------------------------------------------------------
 * Delay: delays a number of Systicks
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) { __NOP(); }
}

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using HSI
                             (HSE is not populated on Nucleo board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) {

  RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI is system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

  FLASH->ACR  = FLASH_ACR_PRFTEN;                          // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_ICEN;                            // Instruction cache enable
  FLASH->ACR |= FLASH_ACR_DCEN;                            // Data cache enable
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS;                     // Flash 5 wait state

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;                        // APB1 = HCLK/4
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;                        // APB2 = HCLK/2

  RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

  // PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P
  RCC->PLLCFGR = ( 16ul                   |                // PLL_M =  16
                 (384ul <<  6)            |                // PLL_N = 384
                 (  3ul << 16)            |                // PLL_P =   8
                 (RCC_PLLCFGR_PLLSRC_HSI) |                // PLL_SRC = HSI
                 (  8ul << 24)             );              // PLL_Q =   8

  RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready

  RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src
}

/*----------------------------------------------------------------------------
 * main: blink LED and check button state
 *----------------------------------------------------------------------------*/
int main (void) {
  int32_t max_num = LED_GetCount();
  int32_t num = 0;

  SystemCoreClockConfigure();                              // configure HSI as System Clock
  SystemCoreClockUpdate();

  LED_Initialize();
  Buttons_Initialize();
  SER_Initialize();

  SysTick_Config(SystemCoreClock / 1000);                  // SysTick 1 msec interrupts

	//Enable the GPIO clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	//D8 is PA9
	GPIOA->MODER |= GPIO_MODER_MODER9_0 ;
	GPIOA->OSPEEDR |= ~GPIO_OSPEEDER_OSPEEDR9; //max speed
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9; //default - push/pull mode
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9); //disable pull up/push down resistors
	
  for (;;) {
    LED_On(num);                                           // Turn specified LED on
    Delay(250);                                            // Wait 500ms
    while (Buttons_GetState() & (1 << 0));                 // Wait while holding USER button
    LED_Off(num);                                          // Turn specified LED off
    Delay(250);                                            // Wait 500ms
    while (Buttons_GetState() & (1 << 0));                 // Wait while holding USER button

		GPIOA->BSRR = GPIO_BSRR_BS_9; //HIGH to PA9 - atomic
		Delay(250);
		GPIOA->BSRR = GPIO_BSRR_BR_9; //LOW to PA9 - atomic
		Delay(250);
    num++;                                                 // Change LED number
    if (num >= max_num) {
      num = 0;                                             // Restart with first LED
    }
		
    printf ("Hello World\n\r");
  }

}
