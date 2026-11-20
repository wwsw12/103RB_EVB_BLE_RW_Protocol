/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "clcd.h"
#include "hdc1080.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STX 0x02
#define R 0x52
#define W 0x57
#define VAR 'V'
#define CDS 'P'
#define TEM 'T'
#define ENCO 'E'
#define SERVO 'S'
#define HD1080 'I'
#define DAC 'D'
#define LED 'L'
#define RGBLED 'M'
#define TSW 'R'
#define CLCD1 '1'
#define CLCD2 '2'
#define ETX 0x03
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//알고리즘 변수
uint8_t sendbuf[10] = {0};
uint8_t P_index = 0;
uint8_t s_index = 0;
uint8_t count =0;

char debouncing_flag = 0;
uint32_t last_time = 0;

uint8_t rx_data1;
uint8_t rx_data2;
uint8_t rx_buffer1[50];
char rx_buffer2[50];
uint8_t rx_index2 = 0;
uint8_t rx_index1 = 0;
volatile uint8_t RX_flag1 = 0;
volatile uint8_t RX_flag2 = 0;

uint16_t DAC_data;
short SERVO_data;
int ENCO_data;
uint32_t W_led_data; // W
uint32_t led_data;

uint8_t led1;
uint8_t led2;
uint8_t led3;
uint8_t led4;

uint16_t servo_pulse = 0;
float DAC_V = 0.0;

char CLCD1_data[17];
char CLCD2_data[17];

char sbuf1[17] = {0};
char sbuf2[17] = {0};
int j = 0;

unsigned char R_LED_data;
unsigned char G_LED_data;
unsigned char B_LED_data;

uint32_t print_pretime =0;

//TC1047
float temp_send;
uint32_t TC1047_temp_data;
int adc_done_flag =0;

//ADC
volatile uint16_t ADC_value[3] = {0};

//HDC1080
float HDC1080_temp;
float temp;
uint8_t humi, bat_stat;
uint32_t HDC1080_temp_data;

//SW_LED
uint8_t LED_data[4];
char sw1_flag = 0 ;
char sw2_flag = 0 ;
char sw3_flag = 0 ;
char sw4_flag = 0 ;
uint32_t sw1_pretime = 0 ;
uint32_t sw2_pretime = 0 ;
uint32_t sw3_pretime = 0 ;
uint32_t sw4_pretime = 0 ;

//encoder
volatile int E_count;
volatile int preE_count =0;

//led
uint8_t T_SW[4];
uint8_t LED_data[4];

//3rgb
uint16_t gtimepulse[3];

//time
uint32_t TimerCount;
char act_mode = 0;

uint16_t packet_length = 0;  // ETX가 있어야 할 인덱스 = 길이 + 4
uint8_t calculate_cout = 0;  // 길이 바이트로 ETX 위치를 계산했는지
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void __io_putchar(int ch)
{
	if(ch == '\n')
	{
		HAL_UART_Transmit (&huart2, (uint8_t*) "\r", 1, 0xffff);
	}

	HAL_UART_Transmit (&huart2, (uint8_t*) &ch, 1, 0xffff);
}
void DAC7512_write (unsigned short data)
{
    uint8_t buff[2];
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); //cs chip selct을 low로 선택
    buff[0] = (uint8_t) (data >> 8);
    buff[1] = (uint8_t) data;
    HAL_SPI_Transmit(&hspi2, buff, 2, 1000); // 할함수 spi로 전송 spi2로 buff 배열을 2바이트만큼 1000ms 기둘
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); //cs chip selct을 high로 선택
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// R 패킷(MCU → 앱) 확인용 디버그 로그: 1이면 보낸 패킷을 USART2(Tera Term)에 16진수로 출력
// 평소에는 0 (50 ms마다 출력되면 W 패킷 로그가 묻힘)
#define R_LOG 0
static void R_LOG_PRINT(uint8_t *buf, uint8_t len)
{
#if R_LOG
	printf("R> ");
	for (int i = 0; i < len; i++) printf("%02X ", buf[i]);
	printf("\n");
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  printf("88-2.EVB_DEMO2_RW\n");
  HAL_UART_Receive_IT(&huart1, &rx_data1, 1); //uart1 it

  HAL_TIM_Base_Start_IT (&htim2); // 50ms 마다 packet 보내기

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_value, 3);

  CLCD_Init(16,2);
  hdc1080_init (&hi2c1, HDC1080_T_RES_14, HDC1080_RH_RES_14,1,&bat_stat);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2); //SERVO
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //삼색 led
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  int SW1_state_pre = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
  int SW2_state_pre = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
  int SW3_state_pre = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
  int SW4_state_pre = HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin);

  DAC7512_write(0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //R 부분
	  hdc1080_measure(&hi2c1, &temp, &humi);
	  HDC1080_temp_data = *(uint32_t*) &temp;// 메모리커피 함수와 같은 논리 주소값을 그대로 가져오되, 포인터 형변환을 통해 시프트 연산은 되게하는 코드

	  temp_send = (((float)ADC_value[2] / 4096) * 3.3 - 0.5) * 100;
	  TC1047_temp_data = *(uint32_t*) &temp_send;

//	  if(HAL_GetTick() - print_pretime > 300)
//	  {
//		  printf("temp = %2.2f humid = %3d\n", temp,humi);
//		  printf("ADC= %d ",ADC_value[0]);
//		  printf("CDS= %d ",ADC_value[1]);
//		  printf("TEM= %f\n ",temp_send);
//		  print_pretime = HAL_GetTick();
//	  }

	  int SW1_state_cur = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
	  int SW2_state_cur = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
	  int SW3_state_cur = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
	  int SW4_state_cur = HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin);

	  if((SW1_state_pre != SW1_state_cur) || (SW2_state_pre != SW2_state_cur)||(SW3_state_pre != SW3_state_cur)||(SW4_state_pre != SW4_state_cur))
	  {
		  uint32_t current_time = HAL_GetTick();
		  if(debouncing_flag == 0)
		  {
			  debouncing_flag = 1;
			  last_time = current_time;
		  }
		  else
		  {
			  if((current_time - last_time) >= 100)
			  {
				  debouncing_flag = 0;
				  SW1_state_pre = SW1_state_cur;
				  SW2_state_pre = SW2_state_cur;
				  SW3_state_pre = SW3_state_cur;
				  SW4_state_pre = SW4_state_cur;

				  if(SW1_state_cur == GPIO_PIN_RESET)
				  {
					  printf("Pushed SW1\n");
					  T_SW[0] = 0x01;
					  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
				  }
				  else T_SW[0] = 0x00;

				  if(SW2_state_cur == GPIO_PIN_RESET)
				  {
					  printf("Pushed SW2\n");
					  T_SW[1] = 0x01;
					  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
				  }
				  else T_SW[1] = 0x00;

				  if(SW3_state_cur == GPIO_PIN_RESET)
				  {
					  printf("Pushed SW3\n");
					  T_SW[2] = 0x01;
					  HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
				  }
				  else T_SW[2] = 0x00;

				  if(SW4_state_cur == GPIO_PIN_RESET)
				  {
					  printf("Pushed SW4\n");
					  T_SW[3] = 0x01;
					  HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
				  }
				  else T_SW[3] = 0x00;
			  }
		  }
	  }

	  if(HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) == GPIO_PIN_SET)
	  {
		  LED_data[0] = 0x01;
	  }
	  else
	  {
		  LED_data[0] = 0x00;
	  }

	  if(HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin) == GPIO_PIN_SET)
	  {
		  LED_data[1] = 0x01;
	  }
	  else
	  {
		  LED_data[1] = 0x00;
	  }

	  if(HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin) == GPIO_PIN_SET)
	  {
		  LED_data[2] = 0x01;
	  }
	  else
	  {
		  LED_data[2] = 0x00;
	  }

	  if(HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin) == GPIO_PIN_SET)
	  {
		  LED_data[3] = 0x01;
	  }
	  else
	  {
		  LED_data[3] = 0x00;
	  }

	  if(act_mode == 1)
	  {
		  act_mode=0;

		  switch (count)
		 {
			 case 0:// 가변저항
				 P_index = 0;
				 sendbuf[P_index++] = STX;
				 sendbuf[P_index++] = R;
				 sendbuf[P_index++] = VAR;
				 sendbuf[P_index++] = 2;
				 sendbuf[P_index++] = ADC_value[0];
				 sendbuf[P_index++] = ADC_value[0] >> 8;
				 sendbuf[P_index++] = ETX;
				 HAL_UART_Transmit(&huart1, sendbuf, P_index, 50);
				 R_LOG_PRINT(sendbuf, P_index);
				 count++;
				 break;

			 case 1:// CDS
				 P_index = 0;
				 sendbuf[P_index++] = STX;
				 sendbuf[P_index++] = R;
				 sendbuf[P_index++] = CDS;
				 sendbuf[P_index++] = 2;
				 sendbuf[P_index++] = ADC_value[1];
				 sendbuf[P_index++] = ADC_value[1] >> 8;
				 sendbuf[P_index++] = ETX;
				 HAL_UART_Transmit(&huart1, sendbuf, P_index, 50);
				 R_LOG_PRINT(sendbuf, P_index);
				 count++;
				 break;

			 case 2:// TC1047
				 P_index = 0;
				 sendbuf[P_index++] = STX;
				 sendbuf[P_index++] = R;
				 sendbuf[P_index++] = TEM;
				 sendbuf[P_index++] = 4;
				 sendbuf[P_index++] = TC1047_temp_data;
				 sendbuf[P_index++] = TC1047_temp_data >> 8;
				 sendbuf[P_index++] = TC1047_temp_data >> 16;
				 sendbuf[P_index++] = TC1047_temp_data >> 24;
				 sendbuf[P_index++] = ETX;
				 HAL_UART_Transmit(&huart1, sendbuf, P_index, 50);
				 R_LOG_PRINT(sendbuf, P_index);
				 count++;
				 break;

			 case 3:// HDC1080
				 P_index = 0;
				 sendbuf[P_index++] = STX;
				 sendbuf[P_index++] = R;
				 sendbuf[P_index++] = HD1080;
				 sendbuf[P_index++] = 5;
				 sendbuf[P_index++] = HDC1080_temp_data;
				 sendbuf[P_index++] = HDC1080_temp_data >> 8;
				 sendbuf[P_index++] = HDC1080_temp_data >> 16;
				 sendbuf[P_index++] = HDC1080_temp_data >> 24;
				 sendbuf[P_index++] = humi;
				 sendbuf[P_index++] = ETX;
				 HAL_UART_Transmit(&huart1, sendbuf, P_index, 50);
				 R_LOG_PRINT(sendbuf, P_index);
				 count++;
				 break;

			 case 4:// 택트 스위치
				 P_index = 0;
				 sendbuf[P_index++] = STX;
				 sendbuf[P_index++] = R;
				 sendbuf[P_index++] = TSW;
				 sendbuf[P_index++] = 4;
				 sendbuf[P_index++] = T_SW[3];
				 sendbuf[P_index++] = T_SW[2];
				 sendbuf[P_index++] = T_SW[1];
				 sendbuf[P_index++] = T_SW[0];
				 sendbuf[P_index++] = ETX;
				 HAL_UART_Transmit(&huart1, sendbuf, P_index, 50);
				 R_LOG_PRINT(sendbuf, P_index);
				 count++;
				 break;

			 case 5:// LED
				 P_index = 0;
				 sendbuf[P_index++] = STX;
				 sendbuf[P_index++] = R;
				 sendbuf[P_index++] = 'L';
				 sendbuf[P_index++] = 4;
				 sendbuf[P_index++] = LED_data[3];
				 sendbuf[P_index++] = LED_data[2];
				 sendbuf[P_index++] = LED_data[1];
				 sendbuf[P_index++] = LED_data[0];
				 sendbuf[P_index++] = ETX;
				 HAL_UART_Transmit(&huart1, sendbuf, P_index, 50);
				 R_LOG_PRINT(sendbuf, P_index);
				 count++;
				 break;

			 case 6:// 엔코더
				 P_index = 0;
				 sendbuf[P_index++] = STX;
				 sendbuf[P_index++] = R;
				 sendbuf[P_index++] = ENCO;
				 sendbuf[P_index++] = 4;
				 sendbuf[P_index++] = E_count;
				 sendbuf[P_index++] = E_count >> 8;
				 sendbuf[P_index++] = E_count >> 16;
				 sendbuf[P_index++] = E_count >> 24;
				 sendbuf[P_index++] = ETX;
				 HAL_UART_Transmit(&huart1, sendbuf, P_index, 50);
				 R_LOG_PRINT(sendbuf, P_index);
				 count = 0;
				 break;

			 default:
				 count = 0;
				 break;
		 }
	  }

	  //W 부분
	  if (RX_flag1 == 1)
	  {
		  P_index =0;
		  printf("\n\n");
		  for (int i = 0; i < rx_index1; i++)
		  {
			  printf("%02X ", rx_buffer1[i]);
		  }
		  printf("\n");
		  // rx_index1 = 0, RX_flag1 = 0은 해석이 끝난 뒤(switch 아래)로 옮김
		  // → 해석하는 동안 ISR이 같은 버퍼에 다음 패킷을 덮어쓰지 않음

		  if(rx_buffer1[P_index++] == STX) printf("STX_OK\n");
		  if(rx_buffer1[P_index++] == W) printf("W_OK\n");

		  switch (rx_buffer1[P_index++])
		  {
			  case LED:
				  printf("LED_OK\n");
				  if(rx_buffer1[P_index++] == 4) printf("DATABYTE_OK\n");
				  if(rx_buffer1[P_index + rx_buffer1[P_index-1]] == ETX) printf("ETX_OK\n");

				  led4 = rx_buffer1[P_index++];
				  if(led4 == 1) HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
				  else HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);

				  led3 = rx_buffer1[P_index++];
				  if(led3 == 1) HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
				  else HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);

				  led2 = rx_buffer1[P_index++];
				  if(led2 == 1) HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
				  else HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);

				  led1 =  rx_buffer1[P_index];
				  if(led1 == 1) HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
				  else HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);

				  break;

//				  W_led_data = (uint32_t)(rx_buffer1[P_index])
//							|(uint32_t)(rx_buffer1[P_index + 1] << 8)
//							|(uint32_t)(rx_buffer1[P_index + 2] << 16)
//							|(uint32_t)(rx_buffer1[P_index + 3] << 24);
//				  led1 = (uint8_t)(W_led_data);
//				  led2 = (uint8_t)(W_led_data >> 8);
//				  led3 = (uint8_t)(W_led_data >> 16);
//				  led4 = (uint8_t)(W_led_data >> 24);
//				  break;

			case ENCO:
				printf("ENDCODER_OK\n");
				if(rx_buffer1[P_index++] == 4) printf("DATABYTE_OK\n");
				if(rx_buffer1[P_index + rx_buffer1[P_index-1]] == ETX) printf("ETX_OK\n");
				ENCO_data =  (int)(rx_buffer1[P_index])
							|(int)(rx_buffer1[P_index + 1] << 8)
							|(int)(rx_buffer1[P_index + 2] << 16)
							|(int)(rx_buffer1[P_index + 3] << 24); // 순서바꿔서
				sendbuf[s_index++] = STX;
				sendbuf[s_index++] = R;
				sendbuf[s_index++] = ENCO;
				sendbuf[s_index++] = 4;
				sendbuf[s_index++] = ENCO_data;
				sendbuf[s_index++] = ENCO_data >> 8;
				sendbuf[s_index++] = ENCO_data >> 16;
				sendbuf[s_index++] = ENCO_data >> 24;
				sendbuf[s_index++] = ETX;
				HAL_UART_Transmit(&huart1, sendbuf, s_index, 50);
				E_count = ENCO_data;
				s_index = 0;
				break;

			case SERVO:
				printf("SERVO_OK\n");
				if(rx_buffer1[P_index++] == 2) printf("DATABYTE_OK\n");
				if(rx_buffer1[P_index + rx_buffer1[P_index-1]] == ETX) printf("ETX_OK\n");
				SERVO_data =   (short)(rx_buffer1[P_index])
							  |(short)(rx_buffer1[P_index + 1] << 8);

				servo_pulse = 550 + ((SERVO_data + 90) * (2360 - 550) / 180); //공식은 변수로 1degree =  펄스값 // 550 = angle_0 define 해서 시각적으로
				 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2, servo_pulse);
				break;

			case DAC:
				printf("DAC_OK\n");
				if(rx_buffer1[P_index++] == 2) printf("DATABYTE_OK\n");
				if(rx_buffer1[P_index + rx_buffer1[P_index-1]] == ETX) printf("ETX_OK\n");
				DAC_data =   (uint16_t)(rx_buffer1[P_index])
							|(uint16_t)(rx_buffer1[P_index + 1] << 8);
				DAC7512_write(DAC_data);
				DAC_V = ((float)DAC_data * 3.3) / 4096.0;
				printf("%.2fV\n",DAC_V);
				break;

			  case RGBLED: // 'M'
				  printf("RGBLED_OK\n");
				  if(rx_buffer1[P_index++] == 3) printf("DATABYTE_OK\n");
				  if(rx_buffer1[P_index + rx_buffer1[P_index-1]] == ETX) printf("ETX_OK\n");

				  B_LED_data = (unsigned char)(rx_buffer1[P_index]);
				  G_LED_data = (unsigned char)(rx_buffer1[P_index + 1]);
				  R_LED_data = (unsigned char)(rx_buffer1[P_index + 2]);

				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,B_LED_data);
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,R_LED_data);
				  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,G_LED_data);
				  break;

			case CLCD1:
				printf("CLCD1_OK\n");
				CLCD_Puts(0, 0, "                ");
				j = 0;
				if(rx_buffer1[P_index++] == 16) printf("DATABYTE_OK\n");
				if(rx_buffer1[P_index + rx_buffer1[P_index-1]] == ETX) printf("ETX_OK\n");
				for(int i = 15; i >= 0; i--)
				{
					CLCD1_data[j++] = rx_buffer1[P_index + i];
				}
				CLCD1_data[j] = '\0';
				printf("CLCD1 = %s\n", CLCD1_data);
				CLCD_Puts(0, 0, CLCD1_data);
			  break;

			case CLCD2:
				printf("CLCD2_OK\n");
				CLCD_Puts(0, 1, "                ");
				j=0;
				if(rx_buffer1[P_index++] == 16) printf("DATABYTE_OK\n");
				if(rx_buffer1[P_index + rx_buffer1[P_index-1]] == ETX) printf("ETX_OK\n");
				for(int i = 15; i >= 0; i--)
				{
					CLCD2_data[j++] = rx_buffer1[P_index + i];
				}
				CLCD2_data[j] = '\0';
				printf("CLCD2 = %s\n", CLCD2_data);
				CLCD_Puts(0, 1, CLCD2_data);
			  break;

			default:
				printf("NO_DEVICE\n");
				break;
		  }

		  // 해석이 끝난 뒤에 다음 패킷 수신 허용 (순서 중요: 인덱스 먼저, 플래그 나중)
		  rx_index1 = 0;
		  RX_flag1 = 0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CLCD_D0_Pin|CLCD_D1_Pin|CLCD_D2_Pin|CLCD_D3_Pin
                          |CLCD_EN_Pin|CLCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CLCD_D0_Pin CLCD_D1_Pin CLCD_D2_Pin CLCD_D3_Pin
                           CLCD_EN_Pin CLCD_RS_Pin */
  GPIO_InitStruct.Pin = CLCD_D0_Pin|CLCD_D1_Pin|CLCD_D2_Pin|CLCD_D3_Pin
                          |CLCD_EN_Pin|CLCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_SW_Pin */
  GPIO_InitStruct.Pin = ENCODER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B_SIG_Pin */
  GPIO_InitStruct.Pin = B_SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_SIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A_SIG_Pin */
  GPIO_InitStruct.Pin = A_SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A_SIG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
	    if (RX_flag1 == 1)
	    {
	        // 메인 루프가 아직 이전 패킷을 해석 중 → 버퍼를 덮어쓰지 않도록 이 바이트는 버림
	    }
	    else if (rx_index1 == 0)
	    {
	        if (rx_data1 == 0x02)
	        {
	            rx_buffer1[rx_index1++] = rx_data1;
	            calculate_cout = 0; // 새 패킷 시작 → 이전 패킷의 길이 계산 상태를 지움
	        }
	    }
	    else
	    {
	        rx_buffer1[rx_index1++] = rx_data1;

	        // 데이터 안에 0x03이 섞여도 잘리지 않도록, 길이 바이트(rx_buffer1[3])로 ETX 위치를 계산
	        if (rx_index1 >= 4)
	        {
	            if (calculate_cout == 0)
	            {
	                packet_length = rx_buffer1[3] + 4; // STX, R/W, 장치, 길이 4바이트 + 데이터
	                calculate_cout = 1;

	                if (packet_length >= sizeof(rx_buffer1)) // 길이가 버퍼보다 크면 버리고 다시 STX부터
	                {
	                    rx_index1 = 0;
	                    calculate_cout = 0;
	                }
	            }

	            // ETX 자리까지 다 받았을 때만 판단 (이전 패킷의 값이 남아 있는 칸을 읽지 않도록)
	            if ((calculate_cout == 1) && (rx_index1 == packet_length + 1))
	            {
	                if (rx_buffer1[packet_length] == 0x03)
	                {
	                    RX_flag1 = 1;
	                }
	                else
	                {
	                    rx_index1 = 0; // ETX 자리가 아니면 버리고 다시 STX부터
	                }
	                calculate_cout = 0;
	            }
	        }
	    }
	    HAL_UART_Receive_IT(&huart1, &rx_data1, 1);
	}

    if (huart->Instance == USART2)
    {
    	if((rx_data2=='\r')||(rx_data2=='\n'))
    	{
        	if(rx_index2 > 0)
        	{
				rx_buffer2[rx_index2] = '\0'; //rx_index을 늘릴 필요x , 문자열의 끝 널을 abc'0'로 만들었음
				rx_index2 = 0;	//rx_index 이제 또 받아야하니 초기화
				RX_flag2 = 1;
        	}
    	}
    	else
    	{
    		rx_buffer2[rx_index2++] = (char)rx_data2; //rx_index++ = index++
    	}
    	HAL_UART_Receive_IT(&huart2, &rx_data2, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
		case ENCODER_SW_Pin:
			E_count =0;
			break;
		case A_SIG_Pin:
			if(!HAL_GPIO_ReadPin(B_SIG_GPIO_Port, B_SIG_Pin))
			E_count++;
			break;
		case B_SIG_Pin:
			if(!HAL_GPIO_ReadPin(A_SIG_GPIO_Port, A_SIG_Pin))
			E_count--;
			break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TimerCount++;

	if(TimerCount == 50)
	{
		TimerCount = 0;
		act_mode = 1;
	}
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
