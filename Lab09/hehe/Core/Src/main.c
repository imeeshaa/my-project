/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (LSM303DLHC I2C Accelerometer)
  ******************************************************************************
  * @attention
  * 
  * STM32F3Discovery + LSM303DLHC (onboard)
  * I2C1_SCL = PB6
  * I2C1_SDA = PB7
  * CS_I2C_SPI = PE3 (must be HIGH for I2C)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdarg.h"
#include "stdio.h"
#include "stm32f3xx_hal.h"
#include "string.h"
#include "math.h"

#define RAD_TO_DEG 57.2958f
#define device_address (0x19 << 1) // Accelerometer I2C address (0x19 << 1)

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
float set_x = 0.0, set_y = 0.0, set_z = 0.0;
float g_x = 0.0, g_y = 0.0, g_z = 0.0;
float roll = 0.0, pitch = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN 0 */
void print_lsm(const char *fmt, ...) {
    char buffer[135];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void init_lsm(void) {
    uint8_t reg1_addr = 0x20;
    uint8_t reg1_data = 0x67; // 100Hz, normal mode, XYZ enabled
    if (HAL_I2C_Mem_Write(&hi2c1, device_address, reg1_addr, I2C_MEMADD_SIZE_8BIT, &reg1_data, 1, 100) != HAL_OK) {
        print_lsm("ERR: init_lsm reg1 write failed\r\n");
    }
    HAL_Delay(10);

    uint8_t reg4_addr = 0x23;
    uint8_t reg4_data = 0x00; // ±2g, continuous update
    if (HAL_I2C_Mem_Write(&hi2c1, device_address, reg4_addr, I2C_MEMADD_SIZE_8BIT, &reg4_data, 1, 100) != HAL_OK) {
        print_lsm("ERR: init_lsm reg4 write failed\r\n");
    }
    HAL_Delay(10);
}

void read_lsm(){
    uint8_t xla, xha, yla, yha, zla, zha;
    uint16_t x, y, z;
    
    // X VALUES
    HAL_I2C_Mem_Read(&hi2c1, device_address, 0x28 ,I2C_MEMADD_SIZE_8BIT, &xla, 1, 100 ); // reads xla
    HAL_I2C_Mem_Read(&hi2c1, device_address, 0x29 ,I2C_MEMADD_SIZE_8BIT, &xha, 1, 100 ); // reads xha
    
   // Y VALUES
    HAL_I2C_Mem_Read(&hi2c1, device_address, 0x2A ,I2C_MEMADD_SIZE_8BIT, &yla, 1, 100 ); // reads yla
    HAL_I2C_Mem_Read(&hi2c1, device_address, 0x2B ,I2C_MEMADD_SIZE_8BIT, &yha, 1, 100 ); // reads yha
   
    // Z VALUES 
    HAL_I2C_Mem_Read(&hi2c1, device_address, 0x2C ,I2C_MEMADD_SIZE_8BIT, &zla, 1, 100 ); // reads zla
    HAL_I2C_Mem_Read(&hi2c1, device_address, 0x2D ,I2C_MEMADD_SIZE_8BIT, &zha, 1, 100 ); // reads zha
    

    x = (xha << 8) | xla;  
    y = (yha << 8) | yla; 
    z = (zha << 8) | zla; 

    float g = 0.0039f;
    g_x = (x * g)-set_x;
    g_y = (y * g)-set_y;
    g_z = (z * g)-set_z;
    
    roll = (atan2(g_x,g_z)) * RAD_TO_DEG; //sideway rotation
    pitch = (atan2(g_y,g_z)) * RAD_TO_DEG; //forward/backward rotation
 
}

void offset(void) {
    float off_x = 0, off_y = 0, off_z = 0;
    const int samples = 50;
    HAL_Delay(50);
    for (int i = 0; i < samples; i++) {
        read_lsm();
        off_x += g_x;
        off_y += g_y;
        off_z += g_z;
        HAL_Delay(10);
    }
    set_x = off_x / samples;
    set_y = off_y / samples;
    set_z = (off_z / samples) - 1.0f;

}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();

  /* === Force CS High BEFORE I2C init === */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  /* Check CS pin state */
  GPIO_PinState cs_state = HAL_GPIO_ReadPin(GPIOE, CS_I2C_SPI_Pin);

  /* Continue with system init */
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();

  print_lsm("\r\n--- STM32F3Discovery LSM303DLHC I2C Test ---\r\n");
  print_lsm("CS pin state: %d (1=HIGH, 0=LOW)\r\n", cs_state);
  HAL_Delay(100);

  /* Try reading WHO_AM_I */
  uint8_t whoami = 0;
  if (HAL_I2C_Mem_Read(&hi2c1, device_address, 0x0F, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 100) == HAL_OK)
      print_lsm("WHO_AM_I (0x19): 0x%02X\r\n", whoami);
  else
      print_lsm("WHO_AM_I read failed\r\n");

  /* Initialize accelerometer and calibrate */
  init_lsm();
  offset();

  while (1)
  {
      read_lsm();
      //print_lsm("Pitch: %.2f  Roll: %.2f\r\n", pitch, roll);
      print_lsm("%f\n %f\n %f\n", g_x, g_y, g_z);
      HAL_Delay(200);
  }
}
/* USER CODE END 1 */

/* --- System Configuration and Peripheral Init functions --- */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi1);
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

static void MX_USB_PCD_Init(void) {
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  HAL_PCD_Init(&hpcd_USB_FS);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Keep CS pin high initially */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET);

  /* Configure LEDs */
  HAL_GPIO_WritePin(GPIOE, LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin|
                           LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /* Configure CS and LED pins */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin|
                        LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* Configure Button */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE END */