/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int time;
	uint8_t level;
} ClickInfoDef;

typedef struct {
	uint8_t f;
	uint8_t h;
	uint8_t m;
	uint8_t s;
} SETTIME;

typedef struct FLAHTIME {
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t format;
	uint8_t hour;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t alarmFormat;
	uint8_t alarmHour;
	uint8_t alarmMinutes;
	uint8_t alarmSeconds;

} FT;

FT flashTime;
FT flashTime2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//EXTI
#define LONG_CLICK_MIN 700
#define LONG_CLICK_MAX 5000
#define DOUBLE_CLICK_MIN 40
#define DOUBLE_CLICK_MAX 120
//ADC
#define UP_KEY_MIN 0
#define UP_KEY_MAX 15
#define DOWN_KEY_MIN 830
#define DOWN_KEY_MAX 870
#define LEFT_KEY_MIN 1910
#define LEFT_KEY_MAX 1960
#define RIGHT_KEY_MIN 2920
#define RIGHT_KEY_MAX 3010
//LCD
#define LCD_ADDR (0x27 << 1)
#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)
#define LCD_DELAY_MS 5


// flash memory
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_12   /* Start @ of user Flash area */
/* End @ of user Flash area : sector start address + sector size -1 */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_23  +  GetSectorSize(ADDR_FLASH_SECTOR_23) - 1
#define DATA_32                 ((uint32_t)0x00001011) // flah메모리 매직넘버 해당 값을 바꾸면 초기 값으로 셋팅 할 수 있따.

#define NOISE 15
// 멜로디 소리 조절 용도 값이 클수록 소리는 작아진다.
#define VOLUME 300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx = 0;
char rx_buf[20];
int bufindex = 0;
uint8_t receive_data_type = 0;
char *p;

ClickInfoDef click[3];
int current_time = 0;
int time_interval = 0;
int last_time = 0;

// 0이면 알람 ON, 1 이면 OFF
int alarmMode = 1;
uint8_t longClick = 0;
int ADC_value;
uint8_t rx;
uint8_t lcd_display_number = 0;
uint8_t mode = 1;
RTC_TimeTypeDef sTime = { 0 };
RTC_DateTypeDef sDate = { 0 };
char buf[25]; // printf
char temp[25]; // set Time

char ampm[2][3] = { "AM", "PM" };
char alarmSet[2][4] = { "ON ", "OFF" };

// ?��?��?�� 켜졌?���? Display1_line1?�� A?���? ?��?��?��?��.
char alarmOnOff[2][2] = { "A", " " };


int ADC_flag = 0;

// ---------------------------------------------------------------------------------
SETTIME st;
SETTIME at;

uint16_t bicycle[] = { E, G, G, M, E, G, G, M, A, A, A, A, A, M }; //, G, G, M, A, A, A, A, A};
uint16_t interval[] = { 125, 125, 250, 250, 125, 125, 250, 0, 125, 125, 125,
		125, 375, 625 };
uint16_t mute[] = { 125, 125, 0, 0, 125, 125, 0, 0, 125, 125, 125, 125, 125, 0 };

uint16_t bicycle_2[] = { G, G, G, G, F, F, F, F, E, E, E, E, E, M };
uint16_t interval_2[] = { 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
		125, 375, 625 };
uint16_t mute_2[] = { 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
		125, 125, 0 };

uint16_t bicycle_3[] = { E, G, G, G, E, G, G, M, A, A, E, E, G, M };
uint16_t interval_3[] = { 125, 125, 125, 125, 125, 125, 250, 250, 200, 75, 125,
		125, 375, 625 };
uint16_t mute_3[] = { 125, 125, 125, 125, 125, 125, 0, 0, 125, 50, 75, 125, 125,
		125, 0 };

uint16_t bicycle_4[] = { F, F, F, F, E, E, E, E, D, D, G, G, C, M };
uint16_t interval_4[] = { 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
		125, 375, 625 };
uint16_t mute_4[] = { 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
		125, 125, 0 };

uint8_t melody_number;


uint8_t bell_length = sizeof(bicycle) / sizeof(uint16_t);
uint8_t bell_length_2 = sizeof(bicycle_2) / sizeof(uint16_t);
uint8_t bell_length_3 = sizeof(bicycle_3) / sizeof(uint16_t);
uint8_t bell_length_4 = sizeof(bicycle_4) / sizeof(uint16_t);

uint16_t underworld_melody[] = { C4, C5, A3, A4, AS3, AS4, M, M,
		C4, C5, A3, A4, AS3, AS4, M, M, F3, F4, D3, D4, DS3, DS4, M, M,
		F3, F4, D3, D4, DS3, DS4, M, M, DS4, CS4, D4, CS4, DS4, DS4, GS3,
		G3, CS4, C4, FS4, F4, E3, AS4, A4, GS4, DS4, B3, AS3, A3, GS3, M, M, M };
//Underwolrd tempo
uint16_t underworld_tempo[] = { 12, 12, 12, 12, 12, 12, 6, 3, 12, 12, 12, 12,
		12, 12, 6, 3, 12, 12, 12, 12, 12, 12, 6, 3, 12, 12, 12, 12, 12, 12, 6,
		6, 18, 18, 18, 6, 6, 6, 6, 6, 6, 18, 18, 18, 18, 18, 18, 10, 10, 10, 10,
		10, 10, 3, 3, 3 };

uint8_t underworld_length = sizeof(underworld_melody) / sizeof(uint16_t);

// flash memory ---------------------------------------------------
static FLASH_EraseInitTypeDef EraseInitStruct;
static uint32_t GetSector(uint32_t Address);

uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = 0, SECTORError = 0;
__IO uint32_t data32 = 0, MemoryProgramStatus = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void Ready_Receive_IT();
void Set_receive_datas();

//LCD
void I2C_Scan();
void loop();
void init();
void LCD_SendString(uint8_t lcd_addr, char *str);
void LCD_Init(uint8_t lcd_addr);
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd);
void LCD_SendData(uint8_t lcd_addr, uint8_t data);
HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags);

void AdcSwitch(uint8_t *adc_point, int *location);
void bufferState();
void SetTimeUp(const int *location);


void SaveAlarm();
void SetClock();


void PlayToAlarm();
void Bicyclemelody();
void underworld();
void SaveMelody();


void MF_Init_flash();
void SetUpflash();

void Display0_line1();
void Dispaly0_line2();
void Display1_line1();
void Display1_line2();
void Display2_line1();
void Display2_line2();
void Display3_line1();
void Display3_line2(uint8_t *num);
void Display3_ADC_switch_select_melody_number(uint8_t *num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 100);
	return ch;
}*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // 커서위치를 기억하고 커서 이동에 규칙을 주기 위해 만들었다.
  int location = 0;
  // LCD 보드에 부착되어있는 스위치가 연속해서 동작하지 않게 하기 위해서 만들었다.
  uint8_t adc_point = 0;
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
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
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
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE BEGIN Init */
  MF_Init_flash();
  // LCD 사용을 위한 초기화
  init();
  HAL_TIM_Base_Init(&htim3);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END Init */

  at.f = flashTime.alarmFormat;
  at.h = flashTime.alarmHour;
  at.m = flashTime.alarmMinutes;
  at.s = flashTime.alarmSeconds;


  sDate.Year = 22;
  sDate.Month = 6;
  sDate.Date = 20;
  sTime.TimeFormat = 0;
  sTime.Hours = 12;


  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  memset(buf, 0, sizeof(buf));
  memset(rx_buf, 0, sizeof(rx_buf));
  HAL_UART_Receive_IT(&huart3, &rx, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (lcd_display_number == 0) {
		  // 커서 위치 초기화 ************* cursor *************************************
		  bufferState();
		  location = 0;
		  //********************* Display ******************************************
		  while (lcd_display_number == 0) {
			  Display0_line1();
			  Dispaly0_line2();
			  Set_receive_datas();
		  //************************************************************************
		  // 알람이 울리면 longClick으로 멜로디 종료 시킬 수 있다.
			  if (alarmMode == 0) {
				  if (at.f == sTime.TimeFormat) {
					  if (at.h == sTime.Hours && at.m == sTime.Minutes && at.s == sTime.Seconds) {
						  alarmMode = 1;
						  while (longClick == 0) {
							  PlayToAlarm();
						  }
						  longClick = 0;
		    			  Display0_line1();
					  }
				  }
			  }
		    //**********************************************************************
		  }
	  } //set Time
	  else if (lcd_display_number == 1) {
		  //********************* Display ******************************************
		  Display1_line1();
		  Display1_line2();
		  //********************** cursor ******************************************
		  bufferState();
		  location = 0;
		  //***********************st 초기?�� *****************************************
		  st.f = sTime.TimeFormat;
		  st.h = sTime.Hours;
		  st.m = sTime.Minutes;
		  st.s = sTime.Seconds;
		  //************************************************************************
		  longClick = 0;
		  while (lcd_display_number == 1) {
			  AdcSwitch(&adc_point, &location);
			  SetClock();
			  Set_receive_datas();
		  }
		 //*************************************************************************
		} //alarm
		else if (lcd_display_number == 2) {
			//********************* Display ****************************************
			Display2_line1();
			Display2_line2();
			//********************** cursor ****************************************
			bufferState();
			location = 0;
			//***********************st 초기?�� ***************************************
			st.f = at.f;
			st.h = at.h;
			st.m = at.m;
			st.s = at.s;
			//**********************************************************************
			longClick = 0;
			while (lcd_display_number == 2) {
				AdcSwitch(&adc_point, &location);
				SaveAlarm();
			}
		}
		else if (lcd_display_number == 3) {
			//********************* Display *****************************************
			Display3_line1();
			while(lcd_display_number == 3){
				Display3_line2(&melody_number);
				SaveMelody();
				Set_receive_datas();
			}
		}
		else if (lcd_display_number > 3)
			lcd_display_number = 0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void MF_Init_flash()
{
	HAL_FLASH_Unlock();

	if (*((uint32_t*) 0x08104000) == DATA_32) {

		FirstSector = GetSector(FLASH_USER_START_ADDR);
		NbOfSectors = 1;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Sector = FirstSector;
		EraseInitStruct.NbSectors = NbOfSectors;

		flashTime.alarmFormat = *((uint32_t*) 0x08100014);
		flashTime.alarmHour = *((uint32_t*) 0x08100018);
		flashTime.alarmMinutes = *((uint32_t*) 0x0810001C);
		flashTime.alarmSeconds = *((uint32_t*) 0x08100020);
		alarmMode = *((uint32_t*) 0x08100024);
		melody_number = *((uint32_t*) 0x08100028);

	} else {

		FirstSector = GetSector(FLASH_USER_START_ADDR);
		// flash 메모리의 key 값은 데이터와 다른 섹터에 저장되어있다. 따라서 2섹터를 사용한다.
		NbOfSectors = 2;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Sector = FirstSector;
		EraseInitStruct.NbSectors = NbOfSectors;

		if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {

		}

		NbOfSectors = 1;
		EraseInitStruct.NbSectors = NbOfSectors;

		Address = ADDR_FLASH_SECTOR_12;

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08104000),
				((uint32_t) DATA_32));

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100014), 0);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100018), 12);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x0810001C), 0);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100020), 0);

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100024), 1); // alarmMode off

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100028), 1);

		flashTime.alarmFormat = *((uint32_t*) 0x08100014);
		flashTime.alarmHour = *((uint32_t*) 0x08100018);
		flashTime.alarmMinutes = *((uint32_t*) 0x0810001C);
		flashTime.alarmSeconds = *((uint32_t*) 0x08100020);
		alarmMode = *((uint32_t*) 0x08100024);
		melody_number = *((uint32_t*) 0x08100028);
	}

	HAL_FLASH_Lock();
}



void Display0_line1()
{
	sprintf(buf, " %s   LCD Clock  ", alarmOnOff[alarmMode]);
	LCD_SendCommand(LCD_ADDR, 0b10000000);
	LCD_SendString(LCD_ADDR, buf);
}
void Dispaly0_line2()
{
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	//HAL_UART_Transmit(&huart3, (uint8_t*) buf, sizeof(buf), 2000);
	sprintf(buf, "%s %02d:%02d:%02d     ", ampm[sTime.TimeFormat],
			sTime.Hours, sTime.Minutes, sTime.Seconds);
	printf("\r\n");
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, buf);
}

void Display1_line1()
{
	sprintf(buf, " %s   Set Time   ", alarmOnOff[alarmMode]);
	LCD_SendCommand(LCD_ADDR, 0b10000000);
	LCD_SendString(LCD_ADDR, buf);
}
void Display1_line2()
{
	sprintf(buf, "%s %02d:%02d:%02d     ", ampm[sTime.TimeFormat],
			sTime.Hours, sTime.Minutes, sTime.Seconds);
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, buf);
}
void Display2_line1()
{
	sprintf(buf, " %s   alarm      ", alarmOnOff[alarmMode]);
	LCD_SendCommand(LCD_ADDR, 0b10000000);
	LCD_SendString(LCD_ADDR, buf);
}
void Display2_line2()
{
	sprintf(buf, "%s %02d:%02d:%02d %s ", ampm[at.f], at.h, at.m, at.s,
			alarmSet[alarmMode]);
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, buf);
}

void Display3_line1()
{
	memset(buf, 0, sizeof(buf));
	sprintf(buf, " select melody  ");
	LCD_SendCommand(LCD_ADDR, 0b10000000);
	LCD_SendString(LCD_ADDR, buf);
}

void Display3_line2(uint8_t *num) // melodyNumber
{
	memset(buf, 0, sizeof(buf));

	Display3_ADC_switch_select_melody_number(num);

	if(*num == 1)
		sprintf(buf, "<-> 1.Bicycle    ");
	else if(*num == 2)
		sprintf(buf, "<-> 2.underworld ");

	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, buf);
}

void Display3_ADC_switch_select_melody_number(uint8_t *num)
{
	if(ADC_flag == 3)
	{
		*num = 1;
	}
	else if(ADC_flag == 4)
	{
		*num = 2;
	}
}


void bufferState() {
	if (lcd_display_number == 2 || lcd_display_number == 1) {
		LCD_SendCommand(LCD_ADDR, 0b11000000);
		LCD_SendCommand(LCD_ADDR, 0b00001111);
	} else
		LCD_SendCommand(LCD_ADDR, 0b00001110);
}
// SET+T+A+00+00+00\n
// SET+A+P+00+00+00\n
// SET+M+0\n

void Set_receive_datas() {
	if (receive_data_type != 0) {
		if (receive_data_type == 1) {
			if (*(p + 6) == 'A') {
				int format = 0;
				sTime.TimeFormat = format;
			} else if (*(p + 6) == 'P') {
				int format = 1;
				sTime.TimeFormat = format;
			}
			int hours_1 = *(p + 8) - '0';
			int hours_2 = *(p + 9) - '0';
			sTime.Hours = (hours_1 * 10) + hours_2;

			int minutes_1 = *(p + 11) - '0';
			int minutes_2 = *(p + 12) - '0';
			sTime.Minutes = (minutes_1 * 10) + minutes_2;

			int seconds_1 = *(p + 14) - '0';
			int seconds_2 = *(p + 15) - '0';
			sTime.Seconds = (seconds_1 * 10) + seconds_2;

			HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		} else if (receive_data_type == 2) {
			if (*(p + 6) == 'A') {
				int format = 0;
				at.f = format;
			} else if (*(p + 6) == 'P') {
				int format = 1;
				at.f = format;
			}
			int hours_1 = *(p +8) - '0';
			int hours_2 = *(p +9) - '0';
			at.h = (hours_1 * 10) + hours_2;

			int minutes_1 = *(p + 11) - '0';
			int minutes_2 = *(p + 12) - '0';
			at.m = (minutes_1 * 10) + minutes_2;

			int seconds_1 = *(p + 14) - '0';
			int seconds_2 = *(p + 15) - '0';
			at.s = (seconds_1 * 10) + seconds_2;

			SetUpflash();

			alarmMode = 0;

		} else if (receive_data_type == 3) {
			int melody = *(p + 6) - '0';
			if (melody == 1 || melody == 2){
				melody_number = melody;
				SetUpflash();
			}
		}
		Ready_Receive_IT();
	}
}

void Ready_Receive_IT()
{
	receive_data_type = 0;
	memset(rx_buf, 0, sizeof(rx_buf));
	bufindex = 0;
	HAL_UART_Receive_IT(&huart3, &rx, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART3) {
		// SET+T+A+00+00+00\n
		// SET+A+P+00+00+00\n
		// SET+M+0\n
		if(rx == '\n')
		{
			p = strstr((char*)rx_buf, "SET");
			if(p != 0)
			{
				if (*(p + 4) == 'T')
				{
					receive_data_type = 1;
				}
				else if(*(p + 4) == 'A')
				{
					receive_data_type = 2;
				}
				else if(*(p + 4) == 'M')
				{
					receive_data_type = 3;
				}
				else
				{
					Ready_Receive_IT();
				}
			}
			else
				Ready_Receive_IT();
		}
		else if (bufindex < 19)
		{
			rx_buf[bufindex++] = rx;
			HAL_UART_Receive_IT(&huart3, &rx, 1);
		}

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	GPIO_PinState pin;

	if (GPIO_Pin == GPIO_PIN_13) {
		current_time = HAL_GetTick();
		time_interval = current_time - last_time;
		last_time = current_time;

		pin = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

		if (time_interval <= NOISE) {
			printf("Noise %d, %d\r\n", pin, time_interval);
		} else {

			click[1].time = click[0].time;
			click[1].level = click[0].level;

			click[0].time = time_interval;
			click[0].level = pin;
		}
		if (click[0].level == GPIO_PIN_RESET && click[0].time >= LONG_CLICK_MIN) // long click
		{
			printf("\r\nLong Key\r\n");
			longClick = 1;

		} else if (click[0].level == GPIO_PIN_RESET
				&& click[1].level == GPIO_PIN_SET) {
			printf("\r\nSelect Key, %d\r\n", click[0].time);
			lcd_display_number++;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		HAL_ADC_Start(&hadc1);
		ADC_value = HAL_ADC_GetValue(&hadc1);
		printf("ADC_value = %d\r\n", ADC_value);
		HAL_ADC_Stop(&hadc1);
		ADC_flag = 0;
		//***************** UP ***********************************************
		if ((ADC_value <= UP_KEY_MAX) ) {
			ADC_flag = 1;
		}
		//***************** DOWN ***********************************************
		else if ((ADC_value >= DOWN_KEY_MIN && ADC_value <= DOWN_KEY_MAX) ) {
			ADC_flag = 2;
		}
		//****************** LEFT **********************************************
		else if ((ADC_value >= LEFT_KEY_MIN && ADC_value <= LEFT_KEY_MAX)) {
			ADC_flag = 3;
		}
		//***************** RIGHT **********************************************
		else if ((ADC_value >= RIGHT_KEY_MIN && ADC_value <= RIGHT_KEY_MAX)) {
			ADC_flag = 4;
		}
	}
}

void I2C_Scan() {
	char info[] = "Scanning I2C bus...\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*) info, strlen(info), HAL_MAX_DELAY);

	HAL_StatusTypeDef res;
	for (uint16_t i = 0; i < 128; i++) {
		res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
		if (res == HAL_OK) {
			char msg[64];
			snprintf(msg, sizeof(msg), "0x%02X", i);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);
		} else {
			HAL_UART_Transmit(&huart2, (uint8_t*) ".", 1, HAL_MAX_DELAY);
		}
	}

	HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data,
		uint8_t flags) {
	HAL_StatusTypeDef res;
	for (;;) {
		res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1, HAL_MAX_DELAY);
		if (res == HAL_OK)
			break;
	}

	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	data_arr[0] = up | flags | BACKLIGHT | PIN_EN;
	data_arr[1] = up | flags | BACKLIGHT;
	data_arr[2] = lo | flags | BACKLIGHT | PIN_EN;
	data_arr[3] = lo | flags | BACKLIGHT;

	res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, sizeof(data_arr),
	HAL_MAX_DELAY);
	HAL_Delay(LCD_DELAY_MS);
	return res;
}

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
	LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
	LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void LCD_Init(uint8_t lcd_addr) {
	// 4-bit mode, 2 lines, 5x7 format
	LCD_SendCommand(lcd_addr, 0b00110000);
	// display & cursor home (keep this!)
	LCD_SendCommand(lcd_addr, 0b00000010);
	// display on, right shift, underline off, blink off
	LCD_SendCommand(lcd_addr, 0b00001100);
	// clear display (optional here)
	LCD_SendCommand(lcd_addr, 0b00000001);
}

void LCD_SendString(uint8_t lcd_addr, char *str) {
	while (*str) {
		LCD_SendData(lcd_addr, (uint8_t) (*str));
		str++;
	}
}

void init() {
	I2C_Scan();
	LCD_Init(LCD_ADDR);
}

void loop() {
	HAL_Delay(100);
}
void SaveAlarm() {
	/***************** Save 기능 **************************/
	if (longClick == 1) {
		at.f = st.f;
		at.h = st.h;
		at.m = st.m;
		at.s = st.s;
		longClick = 0;
		lcd_display_number = 0;
		SetUpflash();
	}
}

void SaveMelody()
{
	if (longClick == 1) {
		longClick = 0;
		lcd_display_number = 0;
		SetUpflash();
	}
}

void SetClock() {

	/***************** Save 기능 **************************/
	if (longClick == 1) {
		sTime.TimeFormat = st.f;
		sTime.Hours = st.h;
		sTime.Minutes = st.m;
		sTime.Seconds = st.s;

		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		longClick = 0;
		lcd_display_number = 0;
	}
}
void SetTimeDown(const int *location) {
	if (*location == 0) {
		st.f ^= 1;
	}
	else if (*location == 3){
		if(st.h >= 10) st.h -= 10;
		else if (st.h == 0)
			st.h = 10;
		else
			st.h = 0;
	}
	else if (*location == 4) {
		if (st.h == 0) st.h = 12;
		else st.h -= 1;
	}
	else if (*location == 6) {
		if (st.m < 10) st.m += 50;
		else st.m -= 10;
	}
	else if (*location == 7) {
		if (st.m % 10 == 0) st.m += 9;
		else st.m--;
	}
	else if (*location == 9) {
		if (st.s < 10) st.s += 50;
		else st.s -= 10;
	}
	else if (*location == 10) {
		if (st.s % 10 == 0) st.s += 9;
		else st.s--;
	}
	else if (*location == 12 && lcd_display_number == 2)
		alarmMode ^= 1;


	//-------------------------------------------------------
	char format[3];
	if (st.f == 0)
		strcpy(format, "AM");
	else if (st.f == 1)
		strcpy(format, "PM");
	/****************** Display *************************************/
	if (lcd_display_number == 1)
		sprintf(temp, "%s %02d:%02d:%02d     ", format, st.h, st.m, st.s);
	else if (lcd_display_number == 2)
		sprintf(temp, "%s %02d:%02d:%02d %s  ", format, st.h, st.m, st.s,
				alarmSet[alarmMode]);
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, temp);
	/***************** SetTime ?�� 커서 ?��?��리기 **************************/
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	for (int j = 0; j < *location; j++) {
		LCD_SendCommand(LCD_ADDR, 0b00010100);
	}
}

void SetTimeUp(const int *location) {
	if (*location == 0) {
		st.f ^= 1;
	}
	else if (*location == 3) {
		if(st.h > 2 && st.h != 12)
			st.h = 12;
		else if (st.h == 12)
			st.h = 0;
		else
			st.h += 10;
	}
	else if (*location == 4) {
		if (st.h == 12)
			st.h = 0;
		else
			st.h += 1;

	} else if (*location == 6) {
		if (st.m >= 50)
			st.m -= 50;
		else if (st.m < 50) {
			st.m += 10;
		}
	} else if (*location == 7) {
		if (st.m % 10 == 9)
			st.m -= 9;
		else
			st.m++;
	} else if (*location == 9) {
		if (st.s >= 50)
			st.s -= 50;
		else if (st.s < 50)
			st.s += 10;
	} else if (*location == 10) {
		if (st.s % 10 == 9)
			st.s -= 9;
		else
			st.s++;
	} else if (*location == 12 && lcd_display_number == 2)
		alarmMode ^= 1;
	char format[3];
	if (st.f == 0)
		strcpy(format, "AM");
	else if (st.f == 1)
		strcpy(format, "PM");
/****************** Display *************************************/
	if (lcd_display_number == 1)
		sprintf(temp, "%s %02d:%02d:%02d     ", format, st.h, st.m, st.s);
	else if (lcd_display_number == 2)
		sprintf(temp, "%s %02d:%02d:%02d %s  ", format, st.h, st.m, st.s,
				alarmSet[alarmMode]);
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, temp);

	/***************** SetTime ?�� 커서 ?��?��리기 **************************/
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	for (int j = 0; j < *location; j++) {
		LCD_SendCommand(LCD_ADDR, 0b00010100);
	}
}

void AdcSwitch(uint8_t *adc_point, int *location) {
	//***************** UP *************************************************
	if (ADC_flag == 1 && *adc_point != 1) {
		*adc_point = 1;
		printf("ADC_value = %d\r\n", ADC_value);
		SetTimeUp(location);
	}
	//***************** DOWN ***********************************************
	else if (ADC_flag == 2 && *adc_point != 2) {
		*adc_point = 2;
		printf("ADC_value = %d\r\n", ADC_value);
		SetTimeDown(location);
	}
	//****************** LEFT **********************************************
	else if (ADC_flag == 3  && *adc_point != 3) {
		printf("ADC_value = %d\r\n", ADC_value);
		*adc_point = 3;
		//****************** LEFT **********************************************
		if (*location <= 0) {
			for (int r = 0; r < 12; r++) {
				LCD_SendCommand(LCD_ADDR, 0b00010100);
			}
			*location = 12;
		}
		else {
			if (*location == 3) {
				LCD_SendCommand(LCD_ADDR, 0b00010000);
				LCD_SendCommand(LCD_ADDR, 0b00010000);
				(*location) -= 2;
			}
			if (*location == 6 || *location == 9 || *location == 12) {
				LCD_SendCommand(LCD_ADDR, 0b00010000);
				(*location) -= 1;
			}
			LCD_SendCommand(LCD_ADDR, 0b00010000);
			(*location)--;
			printf("2||%d\r\n", *adc_point );
		}
	}
	//***************** RIGHT **********************************************
	else if (ADC_flag == 4  && *adc_point != 4){
		printf("ADC_value = %d\r\n", ADC_value);
		*adc_point = 4;
		//***************** RIGHT **********************************************
		if (12 <= *location ) {
			for (int l = 12; l > 0; l--) {
				LCD_SendCommand(LCD_ADDR, 0b00010000);
			}
			*location = 0;
		} else {
			if (*location == 0) {
				LCD_SendCommand(LCD_ADDR, 0b00010100);
				LCD_SendCommand(LCD_ADDR, 0b00010100);
				(*location) += 2;
			}
			if (*location == 4 || *location == 7 || *location == 10) {
				LCD_SendCommand(LCD_ADDR, 0b00010100);
				(*location) += 1;
			}
			LCD_SendCommand(LCD_ADDR, 0b00010100);
			(*location)++;

		}
	}
	if (ADC_value > RIGHT_KEY_MAX  && *adc_point != 0) {
		*adc_point = 0;
	}
//**********************************************************************

}

void Bicyclemelody() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	for (int i = 0; i < bell_length; i++) {
		if (longClick == 0) {
			TIM2->ARR = bicycle[i];
			TIM2->CCR1 = TIM2->ARR / VOLUME;
			HAL_Delay(interval[i]);
			TIM2->CCR1 = 0;
			HAL_Delay(mute[i]);
		}
	}
	for (int i = 0; i < bell_length_2; i++) {
		if (longClick == 0) {
			TIM2->ARR = bicycle_2[i];
			TIM2->CCR1 = TIM2->ARR / VOLUME;
			HAL_Delay(interval_2[i]);
			TIM2->CCR1 = 0;
			HAL_Delay(mute_2[i]);
		}
	}
	for (int i = 0; i < bell_length_3; i++) {
		if (longClick == 0) {
			TIM2->ARR = bicycle_3[i];
			TIM2->CCR1 = TIM2->ARR / VOLUME;
			HAL_Delay(interval_3[i]);
			TIM2->CCR1 = 0;
			HAL_Delay(mute_3[i]);
		}
	}
	for (int i = 0; i < bell_length_4; i++) {
		if (longClick == 0) {
			TIM2->ARR = bicycle_4[i];
			TIM2->CCR1 = TIM2->ARR / VOLUME;
			HAL_Delay(interval_4[i]);
			TIM2->CCR1 = 0;
			HAL_Delay(mute_4[i]);
		}
	}
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void underworld() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	if (longClick == 0) {
		for (int i = 0; i < 10; i++) {
			TIM2->ARR = underworld_melody[i];
			TIM2->CCR1 = TIM2->ARR / VOLUME;
			HAL_Delay(1500 / underworld_tempo[i]);
		}
	}
	if (longClick == 0) {
		for (int i = 10; i < 20; i++) {
			TIM2->ARR = underworld_melody[i];
			TIM2->CCR1 = TIM2->ARR / VOLUME;
			HAL_Delay(1500 / underworld_tempo[i]);
		}
	}
	if (longClick == 0) {
		for (int i = 20; i < 30; i++) {
			TIM2->ARR = underworld_melody[i];
			TIM2->CCR1 = TIM2->ARR / VOLUME;
			HAL_Delay(1500 / underworld_tempo[i]);
		}
	}
	if (longClick == 0) {
		for (int i = 30; i < underworld_length; i++) {
			TIM2->ARR = underworld_melody[i];
			TIM2->CCR1 = TIM2->ARR / VOLUME;
			HAL_Delay(1500 / underworld_tempo[i]);
		}
	}
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

//********************************************************** flash

static uint32_t GetSector(uint32_t Address) {
	uint32_t sector = 0;

	if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0)) {
		sector = FLASH_SECTOR_0;
	} else if ((Address < ADDR_FLASH_SECTOR_2)
			&& (Address >= ADDR_FLASH_SECTOR_1)) {
		sector = FLASH_SECTOR_1;
	} else if ((Address < ADDR_FLASH_SECTOR_3)
			&& (Address >= ADDR_FLASH_SECTOR_2)) {
		sector = FLASH_SECTOR_2;
	} else if ((Address < ADDR_FLASH_SECTOR_4)
			&& (Address >= ADDR_FLASH_SECTOR_3)) {
		sector = FLASH_SECTOR_3;
	} else if ((Address < ADDR_FLASH_SECTOR_5)
			&& (Address >= ADDR_FLASH_SECTOR_4)) {
		sector = FLASH_SECTOR_4;
	} else if ((Address < ADDR_FLASH_SECTOR_6)
			&& (Address >= ADDR_FLASH_SECTOR_5)) {
		sector = FLASH_SECTOR_5;
	} else if ((Address < ADDR_FLASH_SECTOR_7)
			&& (Address >= ADDR_FLASH_SECTOR_6)) {
		sector = FLASH_SECTOR_6;
	} else if ((Address < ADDR_FLASH_SECTOR_8)
			&& (Address >= ADDR_FLASH_SECTOR_7)) {
		sector = FLASH_SECTOR_7;
	} else if ((Address < ADDR_FLASH_SECTOR_9)
			&& (Address >= ADDR_FLASH_SECTOR_8)) {
		sector = FLASH_SECTOR_8;
	} else if ((Address < ADDR_FLASH_SECTOR_10)
			&& (Address >= ADDR_FLASH_SECTOR_9)) {
		sector = FLASH_SECTOR_9;
	} else if ((Address < ADDR_FLASH_SECTOR_11)
			&& (Address >= ADDR_FLASH_SECTOR_10)) {
		sector = FLASH_SECTOR_10;
	} else if ((Address < ADDR_FLASH_SECTOR_12)
			&& (Address >= ADDR_FLASH_SECTOR_11)) {
		sector = FLASH_SECTOR_11;
	} else if ((Address < ADDR_FLASH_SECTOR_13)
			&& (Address >= ADDR_FLASH_SECTOR_12)) {
		sector = FLASH_SECTOR_12;
	} else if ((Address < ADDR_FLASH_SECTOR_14)
			&& (Address >= ADDR_FLASH_SECTOR_13)) {
		sector = FLASH_SECTOR_13;
	} else if ((Address < ADDR_FLASH_SECTOR_15)
			&& (Address >= ADDR_FLASH_SECTOR_14)) {
		sector = FLASH_SECTOR_14;
	} else if ((Address < ADDR_FLASH_SECTOR_16)
			&& (Address >= ADDR_FLASH_SECTOR_15)) {
		sector = FLASH_SECTOR_15;
	} else if ((Address < ADDR_FLASH_SECTOR_17)
			&& (Address >= ADDR_FLASH_SECTOR_16)) {
		sector = FLASH_SECTOR_16;
	} else if ((Address < ADDR_FLASH_SECTOR_18)
			&& (Address >= ADDR_FLASH_SECTOR_17)) {
		sector = FLASH_SECTOR_17;
	} else if ((Address < ADDR_FLASH_SECTOR_19)
			&& (Address >= ADDR_FLASH_SECTOR_18)) {
		sector = FLASH_SECTOR_18;
	} else if ((Address < ADDR_FLASH_SECTOR_20)
			&& (Address >= ADDR_FLASH_SECTOR_19)) {
		sector = FLASH_SECTOR_19;
	} else if ((Address < ADDR_FLASH_SECTOR_21)
			&& (Address >= ADDR_FLASH_SECTOR_20)) {
		sector = FLASH_SECTOR_20;
	} else if ((Address < ADDR_FLASH_SECTOR_22)
			&& (Address >= ADDR_FLASH_SECTOR_21)) {
		sector = FLASH_SECTOR_21;
	} else if ((Address < ADDR_FLASH_SECTOR_23)
			&& (Address >= ADDR_FLASH_SECTOR_22)) {
		sector = FLASH_SECTOR_22;
	} else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23) */
	{
		sector = FLASH_SECTOR_23;
	}
	return sector;
}

void PlayToAlarm(){
	if(melody_number == 1)
		Bicyclemelody();
	else if(melody_number == 2)
		underworld();
}
void SetUpflash() {

	flashTime.alarmFormat = at.f;
	flashTime.alarmHour = at.h;
	flashTime.alarmMinutes = at.m;
	flashTime.alarmSeconds = at.s;

	HAL_FLASH_Unlock();

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {

	}

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100014),
			flashTime.alarmFormat);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100018),
			flashTime.alarmHour);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x0810001C),
			flashTime.alarmMinutes);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100020),
			flashTime.alarmSeconds);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100024),
			alarmMode );

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ((uint32_t) 0x08100028),
			melody_number );

	HAL_FLASH_Lock();

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
