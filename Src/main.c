/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma2d.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"
#include "stm32f429i_discovery_ts.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	uint8_t uartDados[30];
	uint8_t hora;
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	extern ApplicationTypeDef Appli_state;
	FIL fp; //file handle
	FATFS fatfs; //structure with file system information
	char text[100]="test";//text which will be written into file
	char filename[100]="log.csv";//name of the file
	char filebuffer[100];
	char text2[100];//buffer for data read from file
	
	uint32_t ret;//return variable 
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		sTime.Hours = (uartDados[0] - 0x30) * 10 + (uartDados[1] - 0x30);
		sTime.Minutes = (uartDados[3] - 0x30) * 10 + (uartDados[4] - 0x30);
		sTime.Seconds = (uartDados[6] - 0x30) * 10 + (uartDados[7] - 0x30);
		
		sDate.Date = (uartDados[9] - 0x30) * 10 + (uartDados[10] - 0x30);
		sDate.Month = (uartDados[12] - 0x30) * 10 + (uartDados[13] - 0x30);
		sDate.Year = (uartDados[15] - 0x30) * 10 + (uartDados[16] - 0x30);
		
		HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BIN);
		
		
		HAL_UART_Receive_IT(&huart1,uartDados,17);
	}
	float le_umidade()
	{
		uint8_t dado[2];
		uint16_t H0_rH,H1_rH;
		int16_t H0_T0_OUT,H1_T0_OUT,H_OUT ;
		dado[0] = 0x82;
		HAL_I2C_Mem_Write(&hi2c3,0xBE,0x20,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x30,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x31,I2C_MEMADD_SIZE_8BIT,&dado[1],1,30);
		H0_rH = dado[0]/2;
		H1_rH = dado[1]/2;
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x36,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x37,I2C_MEMADD_SIZE_8BIT,&dado[1],1,30);
		H0_T0_OUT = (dado[1] << 8) + dado[0];
		
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3A,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3B,I2C_MEMADD_SIZE_8BIT,&dado[1],1,30);
		H1_T0_OUT = (dado[1] << 8) + dado[0];
		
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x28,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x29,I2C_MEMADD_SIZE_8BIT,&dado[1],1,30);
		H_OUT = (dado[1] << 8) + dado[0];
		
		return (((H1_rH - H0_rH) * (H_OUT - H0_T0_OUT))/(H1_T0_OUT - H0_T0_OUT)) + H0_rH;
	}
	float le_temperatura()
	{
		uint8_t dado[2];
		uint16_t t0_deg = 0, t1_deg = 0;
		int16_t  t1_out = 0, t0_out = 0,t_out= 0;
		dado[0] = 0x82;
		HAL_I2C_Mem_Write(&hi2c3,0xBE,0x20,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x32,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x33,I2C_MEMADD_SIZE_8BIT,&dado[1],1,30);
		t0_deg = dado[0]; 
		t1_deg = dado[1];
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x35,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		t1_deg = ((dado[0] & 0xC) << 6) + t1_deg; 
		t0_deg = ((dado[0]  & 3) << 8) + t0_deg;
		t0_deg = 	t0_deg /8; 
		t1_deg = t1_deg / 8;
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3C,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3D,I2C_MEMADD_SIZE_8BIT,&dado[1],1,30);
		t0_out = (dado[1] << 8) + dado[0];
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3E,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3F,I2C_MEMADD_SIZE_8BIT,&dado[1],1,30);
		t1_out = (dado[1] << 8) + dado[0];
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x2A,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		HAL_I2C_Mem_Read(&hi2c3,0xBF,0x2B,I2C_MEMADD_SIZE_8BIT,&dado[1],1,30);
		t_out = (dado[1] << 8) + dado[0];
		return (((t1_deg - t0_deg) * (t_out - t0_out))/(t1_out - t0_out) + t0_deg);
	}
	float le_pressao()
	{
		uint8_t dado[3];
		dado[0] = 0x30;
		uint8_t PRESS_OUT_XL, PRESS_OUT_L,PRESS_OUT_H;
		int press;
		HAL_I2C_Mem_Write(&hi2c3,0xBA,0x10,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
			dado[0] = 0x0;
		HAL_I2C_Mem_Write(&hi2c3,0xBA,0x11,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		HAL_I2C_Mem_Read(&hi2c3,0xBB,0x28,I2C_MEMADD_SIZE_8BIT,&dado[0],1,30);
		PRESS_OUT_XL = dado[0];
		HAL_I2C_Mem_Read(&hi2c3,0xBB,0x29,I2C_MEMADD_SIZE_8BIT,&dado[1],1,30);
		PRESS_OUT_L = dado[1];
		HAL_I2C_Mem_Read(&hi2c3,0xBB,0x2A,I2C_MEMADD_SIZE_8BIT,&dado[2],1,30);
		PRESS_OUT_H = dado[2];
		press = PRESS_OUT_XL + (PRESS_OUT_L << 8) + (PRESS_OUT_H << 16);
		//press = (~press) + 1;
		
		return press/4096;
	}

	void pen_drive(void)
	{	
		static int flag=0;
		HAL_GPIO_TogglePin(GPIOG,1<<14);
		while(flag==0) // abre o arquivo para escrita na primeira vez e deixa aberto
		{
			MX_USB_HOST_Process();
			if(Appli_state==APPLICATION_READY)
			{
				/*open or create file for writing*/
				if(f_open(&fp,filename,FA_CREATE_ALWAYS | FA_WRITE)!=FR_OK)
					while(1);
				else
					flag=1;
			}
		}
		if(f_write(&fp,filebuffer,strlen((char*)filebuffer),&ret)!=FR_OK) // vai escrevendo at� pressionar o botao azul
			while(1);
		if(HAL_GPIO_ReadPin(GPIOA,1)==1) // quando pressiona botao azul para de gravar e fecha o
			{
				f_close(&fp);
				GPIOG->BSRR=1<<13;
			}
	}

	
	/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	TS_StateTypeDef TsState;
	int Pot = 0;
	int Current = 0;
	float umidade, temp;
	int pressao;
	unsigned char print_vector[30];
	unsigned char tt[30];
	

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_FMC_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER,LCD_FRAME_BUFFER);
	BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER,LCD_FRAME_BUFFER);
	BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
	BSP_LCD_DisplayOn();
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_SetFont(&Font16);
	
	sTime.Hours = 18;
	sTime.Minutes = 30;
	sTime.Seconds = 0;
	sDate.Date = 11; //(Dia do m�s de 1 a 31)
	sDate.Month = RTC_MONTH_JANUARY; //(M�s de 1 a 12)
	sDate.WeekDay = RTC_WEEKDAY_MONDAY; //(Dia da semana de 1 a 7)
	sDate.Year = 19; //(Ano de 0 a 99)
	HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BIN);
	HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BIN);
	
	HAL_UART_Receive_IT(&huart1,uartDados,17);

	BSP_TS_Init(240, 320);
	

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);



	if ( f_mount( &p.fatfs,"" ,0) != FR_OK )
	{
		BSP_LCD_SetFont(&Font12);
		sprintf((char*)c.vetor_print,"USB mount failed, try again");
		BSP_LCD_DisplayStringAtLine(0,c.vetor_print);
		while(1);
}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
		
		pressao = le_pressao();
		umidade = le_umidade();
		temp = le_temperatura();
		
		BSP_LCD_DisplayStringAtLine(4,tt);
		
		
		HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);
				
		sprintf((char*)print_vector,"%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
    BSP_LCD_DisplayStringAtLine(8,print_vector);
		sprintf((char*)print_vector,"%02d/%02d/%02d",sDate.Date,sDate.Month,sDate.Year);
    BSP_LCD_DisplayStringAtLine(9,print_vector);
		
		sprintf((char*)print_vector,"umid: %02.1f%%",umidade);
    BSP_LCD_DisplayStringAtLine(16,print_vector);
		sprintf((char*)print_vector,"temp:%02.1fC",temp);
    BSP_LCD_DisplayStringAtLine(15,print_vector);
		sprintf((char*)print_vector,"pressao:%04dhPa",pressao);
    BSP_LCD_DisplayStringAtLine(14,print_vector);
		
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		Pot = HAL_ADC_GetValue(&hadc1); //LEITURA DO CANAL 5
		HAL_ADC_PollForConversion(&hadc1,100);
		Current = HAL_ADC_GetValue(&hadc1); //LEITURA DO CANAL 13 (na ordem RANK) //Pino PC3
		HAL_ADC_Stop(&hadc1);
		
		
		sprintf((char*)print_vector,"%04d",Pot);
		BSP_LCD_DisplayStringAtLine(6,print_vector);
		sprintf((char*)print_vector,"corrente: %04d",Current);
		BSP_LCD_DisplayStringAtLine(7,print_vector);
		
		if(Pot > 2000 & Pot < 2095)
		{
			BSP_LCD_SetFont(&Font12);
			BSP_LCD_DisplayStringAtLine(1,(uint8_t*)"motor desligado");
			BSP_LCD_SetFont(&Font16);
			TIM3->CCR1 = 0;
			TIM3->CCR3 = 0;
		}
		else if(Pot >= 2095)
		{
			sprintf((char*)print_vector,"Motor Direita : %04d",((Pot-2095)*100)/2000);
			BSP_LCD_SetFont(&Font12);
			BSP_LCD_DisplayStringAtLine(2,print_vector);
			BSP_LCD_SetFont(&Font16);
			TIM3->CCR1 = Pot;
			TIM3->CCR3 = 0;
		}
		else if(Pot <= 2000)
		{
			sprintf((char*)print_vector,"Motor Esquerda : %04d",((2000-Pot)*100)/2000);
			BSP_LCD_SetFont(&Font12);
			BSP_LCD_DisplayStringAtLine(3,print_vector);
			BSP_LCD_SetFont(&Font16);
			TIM3->CCR1 = 0;
			TIM3->CCR3 = Pot; 
		}
		
		
		sprintf(filebuffer,"%d,%.1f,%.1f,%d,%02d:%02d:%02d,%02d/%02d/%02d\r\n",pressao,temp,umidade,Current,sTime.Hours,sTime.Minutes,sTime.Seconds,sDate.Date,sDate.Month,sDate.Year);
		
		pen_drive();
		HAL_Delay(180);
		
		
		
		/*BSP_TS_GetState(&TsState);
		if(TsState.TouchDetected)
		{
			//sprintf((char*)vetor,"X=%03d, Y=%03d",TsState.X,TsState.Y);
			//BSP_LCD_DisplayStringAtLine(4,(uint8_t*)vetor);
			TsState.X=0;
			TsState.Y=0;
		}*/
		
		//uint8_t vectorNha[4] = "nha\n";
		//HAL_UART_Transmit(&huart1,vectorNha, 4, 60);
		
		//HAL_Delay(30);
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 216;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
