/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <stdlib.h>
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* Function processing DMA Rx data. Counts how many capital and small letters are in sentence.
 * Result is supposed to be stored in global variable of type "letter_count_" that is defined in "main.h"
 *
 * @param1 - received sign
 */
void proccesDmaData(uint8_t sign);


/* Space for your global variables. */

	// type your global variables here:
int posDma=0;

void printMessageAboutBufferCapacity(){
	size_t size = snprintf(NULL, 0,"Buffer capacity: %d bytes, occupied memory: %d bytes, load in %%: %d \n \n",DMA_USART2_BUFFER_SIZE, posDma, 100*posDma/DMA_USART2_BUFFER_SIZE);
	char* info = (char*)malloc(size);
	snprintf(info, size,"Buffer capacity: %d bytes, occupied memory: %d bytes, load in %%: %d \n \n",DMA_USART2_BUFFER_SIZE, posDma, 100*posDma/DMA_USART2_BUFFER_SIZE);
	USART2_PutBuffer((uint8_t *)info, size);
	free(info);
}



int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  /* Space for your local variables, callback registration ...*/

  	  //type your code here:
  USART2_RegisterCallback(proccesDmaData);

  while (1)
  {
	  /* Periodic transmission of information about DMA Rx buffer state.
	   * Transmission frequency - 5Hz.
	   * Message format - "Buffer capacity: %d bytes, occupied memory: %d bytes, load [in %]: %f%"
	   * Example message (what I wish to see in terminal) - Buffer capacity: 1000 bytes, occupied memory: 231 bytes, load [in %]: 23.1%
	   */

  	  	  	  //type your code here:
	//	USART2_CheckDmaReception();
	//  LL_mDelay(1000);

	//  LL_USART_TransmitData8(USART2, upper);
	//  LL_USART_TransmitData8(USART2, lower);
    //  LL_mDelay(5000);

	  printMessageAboutBufferCapacity();
		LL_mDelay(1000);

  }
  /* USER CODE END 3 */
}


void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  LL_Init1msTick(8000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(8000000);
}

/*
 * Implementation of function processing data received via USART.
 */
void proccesDmaData(uint8_t sign)
{
	/* Process received data */

		// type your algorithm here:
	static uint8_t count = 0;
	 static uint8_t upper = 0;
	  static uint8_t lower = 0;
		if (sign=='#'){
			count=1;
			upper=0;
			lower=0;

		}
		if(count==0){
			lower=0;
			upper=0;
		}

		if(count>0){
			if(sign!='#'&&sign!='$'){
				count=count+1;
				if (sign >= 'A' && sign <= 'Z'){
					upper=upper+1;
				}
				if (sign >= 'a' && sign <= 'z'){
					lower=lower+1;
				}
			}
		}


		if (sign=='$'&&count>0&&count<=35){
		size_t size = snprintf(NULL, 0,"number of lower: %d, number of upper: %d \n \n",lower,upper);
		char* info = (char*)malloc(size);
		snprintf(info, size,"number of lower: %d, number of upper: %d \n \n",lower,upper);
		USART2_PutBuffer((uint8_t *)info, size);
		count=0;
		lower=0;
		upper=0;
		}

		if(count>35){
			count=0;
			upper=0;
			lower=0;
		}
}


void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(char *file, uint32_t line)
{ 

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
