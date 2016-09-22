/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.4
  * @date    06-May-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Global Variables */
USBD_HandleTypeDef USBD_Device;
TIM_HandleTypeDef htim2;


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

static void MX_TIM2_Init(void);

// void fill_buffer (int buffer);
void Audio_Preparation (void);

// extern int next_buff;
USBD_HandleTypeDef USBD_Device;
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  MX_TIM2_Init();

  // Roman's code - TMR2_Config(fs*mult_factor/2, RCC_Clocks.PCLK1_Frequency);


  /* Configure LED3, LED4, LED5 and LED6 */
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    BSP_LED_Init(LED5);
    BSP_LED_Init(LED6);

    /* Initialize Button */
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

    /* Init Device Library */
    USBD_Init(&USBD_Device, &AUDIO_Desc, 0);

    /* Add Supported Class */
    USBD_RegisterClass(&USBD_Device, USBD_AUDIO_CLASS);

    /* Add Interface callbacks for AUDIO Class */
    USBD_AUDIO_RegisterInterface(&USBD_Device, &USBD_AUDIO_fops);

    /* Start Device Process */
    USBD_Start(&USBD_Device);

    Audio_Preparation();

    /* Run Application (Interrupt mode) */
    while (1)
    {
  	  // Wait as long as next_buff == 1, then fill first half
  	  //while (next_buff == 1);
  	  //fill_buffer (0);

  	  // Wait as long as next_buff == 0, then fill second half
  	  //while (next_buff == 0);
  	  //fill_buffer (1);
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}


//timer2 is clocked via the MCLK frequency and captures its counter value by SOF event
void MX_TIM2_Init(void)
{
    //TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    //TIM_ICInitTypeDef TIM_ICInitStructure;
    //TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;

    //GPIO_InitTypeDef GPIO_InitStructure;

    // TIM2_CH1_ETR pin (PA.15) configuration
    // => defined now in stm32f4xx_hal_msp.c
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    //GPIO_Init(GPIOA, &GPIO_InitStructure);
    //GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

    /* Enable the TIM2 clock */

    //OK
    htim2.Instance = TIM2;
    //OK
    htim2.Init.Prescaler = 0;
    //OK
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    //Seems OK initial MX value was 0 => as in Roman's code
    htim2.Init.Period = 0xffffffff;
    //OK
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    //OK
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
      Error_Handler();
    }

    //clock TIM2 via ETR pin
    //TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
    {
      Error_Handler();
    }

    /* TIM2 input trigger selection */
    /* It is necessary for the coming of SOF signal to capture the value of the timer 2 counter capture register, and the timer counter 2 - fold
         The processing procedure SOF'a capture the flag is cleared, and the value is stored, in order to give the value of feedback rate
         Since the system has three independent clock source - generator MCLK, USB SOF rate from host and HSE PLL,
         timer 2 is clocked by the frequency of MCLK = 12288 kHz. Takm way between SOFami
         Timer 2 should total approximately 12200-12400. This value must fall in the capture register 2. Since the timer according to standard
         value feedback value should be issued in the format ratio of 10.14 and contain fs / fsof, and accumulation is 2 ^ SOF_VALUE periods
         we obtain for the period SOF - 12288 Pulse
         6 need to shift bits to the left to obtain feedback_value
     */


        // Programmable TMR2 to capture USB frame period
        //TIM_RemapConfig(TIM2,TIM2_USBFS_SOF);
        //TIM_SelectInputTrigger(TIM2, TIM_TS_ITR1);
        //TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Reset);


      sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
      sSlaveConfig.InputTrigger = TIM_TS_ITR1;
      if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
      {
        Error_Handler();
      }

      sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
      sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
      if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
      {
        Error_Handler();
      }

      if (HAL_TIMEx_RemapConfig(&htim2, TIM_TIM2_USBFS_SOF) != HAL_OK)
      {
        Error_Handler();
      }
      /* Enable capture*/
      //TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
      //TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      //TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;
      //TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
      //TIM_ICInitStructure.TIM_ICFilter = 0;
      //TIM_ICInit(TIM2, &TIM_ICInitStructure);
      sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
      sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
      sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
      sConfigIC.ICFilter = 0;
      if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
      {
        Error_Handler();
      }

    //## TIM_Cmd(TIM2, ENABLE);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
