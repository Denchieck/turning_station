#include "main.h"
#include "stdbool.h"
#include "stdlib.h"

#define MT6701CT_Addr 0x0C
#define MAX_COORDINATE 49151
#define MAX_ENCODER_DATA 16383
#define BUFFER_SIZE 5
#define MIN_PULSEWIDTH_OFFSET 63

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

HAL_StatusTypeDef sig_r;

uint8_t Byte_Data_B = 0x03;
uint8_t Byte_Data_L = 0x04;
uint8_t Read_Data_B;
uint8_t Read_Data_L;

uint16_t Data_of_Encoder = 0;
uint16_t buffer = -1;

int coordinate = 0;
int target = 0;
int offset;
int error;

uint8_t data_to_send [5] = {0xff,3,3,7,9};
uint8_t data_received [5];

uint16_t pwm_error;

uint8_t speed_pelco_d;
uint8_t direction_pelco_d;

bool servo_stop = 1;
bool data_ready = 0;
bool need_rorate_servo_pelco_d = 0;
bool need_restart_i2c = false;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);


void Reset_I2C(I2C_HandleTypeDef* rev_i2c)
{
	HAL_I2C_DeInit(rev_i2c);
	HAL_I2C_Init(rev_i2c);
}

void Set_Coordinate(int x)
{
	if (abs(x) > 15000) {
		x = x < 0 ? -1 * (MAX_ENCODER_DATA % x) : MAX_ENCODER_DATA % x;
	}
	coordinate += x;
	if (coordinate > MAX_COORDINATE) {
		coordinate = coordinate % MAX_COORDINATE;
	}
	if (coordinate < 0) {
		coordinate = MAX_COORDINATE + coordinate;
	}
}

void Set_Servo_Pwm (uint16_t pwm)
{
	TIM1->CCR1=pwm;
}

void Rotate_Servo_Pelco_d(uint8_t speed, uint8_t direction)
{
	if (direction == 0x00) {
		Set_Servo_Pwm(1500);
		target = coordinate;
		if (servo_stop) {
			servo_stop = 0;
		}
		HAL_TIM_Base_Start_IT(&htim4);
	}
	else {
		HAL_TIM_Base_Stop_IT(&htim4);
		uint16_t ans;
		ans = min((speed * (700 - MIN_PULSEWIDTH_OFFSET) / 100 + MIN_PULSEWIDTH_OFFSET), 700);
		if (direction == 0x01) {
			Set_Servo_Pwm(1500 + ans);
		}
		if (direction == 0x02) {
			Set_Servo_Pwm(1500 - ans);
		}
		if (!servo_stop) {
			servo_stop = 1;
		}
	}
}


void Uart_Parser(uint8_t *data)
{
	if (data[0] == 0xff && (data[4] == ((data[1] + data[2] + data[3]) & 0xff))) {
		switch (data[1]) {
			case 0x07:
				speed_pelco_d = data[2];
				direction_pelco_d = data[3];
				need_rorate_servo_pelco_d = 1;
				break;
			case 0x12:
				target = data[2] << 8 | data[3];
				break;
			case 0x14:
				data_to_send[0] = 0xff;
				data_to_send[1] = 0x24;
				data_to_send[2] = (target >> 8) & 0xff;
				data_to_send[3] = target & 0xff;
				data_to_send[4] = (data_to_send[1] + data_to_send[2] + data_to_send[3]) & 0xff;
				data_ready = 1;
				break;
			default:
				break;
		}
	}
	else {
		data_to_send[0] = 0xFF;
	}
}


int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  HAL_I2C_Mem_Read(&hi2c1, MT6701CT_Addr, Byte_Data_B, 1, &Read_Data_B, 1, 1);
  HAL_I2C_Mem_Read(&hi2c1, MT6701CT_Addr, Byte_Data_L, 1, &Read_Data_L, 1, 1);
  Data_of_Encoder = Read_Data_B << 6 | Read_Data_L >> 2;
  if (buffer == 65535) {
	  coordinate = Data_of_Encoder;
	  buffer = Data_of_Encoder;
  }
  offset = Data_of_Encoder - buffer;
  Set_Coordinate(offset);
  buffer = Data_of_Encoder;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_UART_Receive_DMA(&huart1, data_received, 5);

  while (1)
  {
	  if (data_ready & !servo_stop) {
		  HAL_UART_Transmit_DMA(&huart1, data_to_send, BUFFER_SIZE);
		  data_ready = 0;
	  }
	  if (need_rorate_servo_pelco_d) {
		  Rotate_Servo_Pelco_d(speed_pelco_d, direction_pelco_d);
		  need_rorate_servo_pelco_d = 0;
	  }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
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
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3030-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 510-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3030-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Uart_Parser(data_received);
	HAL_UART_Receive_DMA(&huart1, data_received, BUFFER_SIZE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		if ((sig_r = HAL_I2C_Mem_Read(&hi2c1, MT6701CT_Addr, Byte_Data_B, 1, &Read_Data_B, 1, 0xff)) == HAL_OK)
		{
			HAL_I2C_Mem_Read(&hi2c1, MT6701CT_Addr, Byte_Data_L, 1, &Read_Data_L, 1, 0xff);
			Data_of_Encoder = Read_Data_B << 6 | Read_Data_L >> 2;
			offset = Data_of_Encoder - buffer;
			Set_Coordinate(offset);
			buffer = Data_of_Encoder;
		}
		else
		{
			Reset_I2C(&hi2c1);
		}
	}


	if (htim->Instance == TIM4)
	{
		error = target - coordinate;
		pwm_error = max(min(abs((error % (MAX_COORDINATE / 2)) / 10), 100), MIN_PULSEWIDTH_OFFSET );
		if (abs(error) > 50 && abs(error) < 49122)
		{
			if (error > 0 )
			{
				Set_Servo_Pwm(error < MAX_COORDINATE / 2 ? 1500 - pwm_error : 1500 + pwm_error);
				if (!servo_stop)
					servo_stop = 1;
			}
			else
			{
				Set_Servo_Pwm(-error > MAX_COORDINATE / 2 ? 1500 - pwm_error : 1500 + pwm_error);
				if (!servo_stop)
					servo_stop = 1;
			}
		}
		else
		{
		  Set_Servo_Pwm(1500);
		  if (servo_stop)
			  servo_stop = 0;
		}
	}

  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
