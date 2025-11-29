/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h" 
#include "stdarg.h" 
#include "stdio.h" 
#include "string.h" 
#include "stdbool.h" 
#include "math.h"

#define GYRO_CS_PORT GPIOE 
#define GYRO_CS_PIN CS_I2C_SPI_Pin 
#define GYRO_CTRL_REG1 0x20U 
#define GYRO_OUT_TEMP 0x26U 
#define GYRO_OUT_X_L 0x28U 
#define GYRO_OUT_X_H 0x29U 
#define GYRO_OUT_Y_L 0x2AU 
#define GYRO_OUT_Y_H 0x2BU 
#define GYRO_OUT_Z_L 0x2CU 
#define GYRO_OUT_Z_H 0x2DU 
#define GYRO_SPI_READ 0x80U 
#define GYRO_SPI_AUTOINC 0x40U 
#define GYRO_SENS_245DPS 0.00875f // dps per LSB for 245 dps 
#define CAL_ROOM_C 25 // accelerometer (I2C - LSM) 
#define LSM_A_ADDR_8 (0x19 << 1) // Accelerometer 7-bit address shifted for HAL 
#define LSM_CTRL1_A 0x20 
#define LSM_CTRL4_A 0x23 
#define LSM_OUT_X_L 0x28 
#define LSM_AUTO_INC 0x80 
#define LSM_G_PER_LSB 0.0039f // 3.9 mg per LSB #define PRINT_RATE_DIV 10 // 
#define PRINT_RATE_DIV 10
#define LOOP_DT 0.01f   // TIM2 ISR = 100 Hz

/* Deadzone for small PID output */
#define MOTOR_DEADZONE 20
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Sensor / calibration state */
/* Sensor / calibration state */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* Sensor / calibration state */
static int8_t  g_t0 = 0;
static int     g_temp_slope = -1;
static uint8_t txb[8], rxb[8];

/* Accelerometer (in g) */
volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;

/* Accelerometer offsets (g) computed during calibration */
static float accel_off_x = 0.0f;
static float accel_off_y = 0.0f;
static float accel_off_z = 0.0f;

/* Gyroscope (deg/s) */
volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;


float angle = 0.0;
/* Gyro temperature / misc */
static int temp_c = 0;

/* Display flag (set from ISR to schedule UART printing in main loop) */
volatile bool print_flag = false;

/* Internal tick counter used to reduce UART rate */
static volatile uint32_t display_counter = 0;

typedef struct { float set_point; //should be zero 
float kp; float ki; float kd; float CoT; float prev_e_t; float integral; }PID;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* --- Helper: formatted UART printing using huart2 --- */
void print_lsm(const char *fmt, ...)
{
    char buffer[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, (uint16_t)strlen(buffer), HAL_MAX_DELAY);
}

/* --- I2C convenience wrappers --- */
HAL_StatusTypeDef i2c_write8(uint8_t dev, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

HAL_StatusTypeDef i2c_readn(uint8_t dev, uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 100);
}

/* --- Gyro SPI helpers --- */
static uint8_t gyro_read_u8(uint8_t reg)
{
  uint8_t cmd = reg | GYRO_SPI_READ;
  uint8_t val = 0;
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &val, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
  return val;
}

static void gyro_write_reg(uint8_t reg, uint8_t val)
{
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
  txb[0] = reg & 0x7F; // write
  txb[1] = val;
  HAL_SPI_Transmit(&hspi1, txb, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

static void gyro_read_regs(uint8_t start_reg, uint8_t *dst, uint16_t len)
{
  uint8_t cmd = start_reg | GYRO_SPI_READ | (len > 1 ? GYRO_SPI_AUTOINC : 0);
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, dst, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

/* --- Initialise gyro (minimal) --- */
static void gyro_init(void)
{
  // Power on, enable X,Y,Z (same as before)
  gyro_write_reg(GYRO_CTRL_REG1, 0x0F);
}

/* --- Gyro temperature read & rough calibration of register baseline --- */
static void gyro_temp_calibrate(void)
{
  uint8_t who = gyro_read_u8(0x0F);
  if (who == 0xD4 || who == 0xD7) g_temp_slope = +1;
  else                            g_temp_slope = -1;

  int32_t acc = 0;
  for (int i = 0; i < 32; i++) {
    acc += (int8_t)gyro_read_u8(GYRO_OUT_TEMP);
    HAL_Delay(1); // tiny delay during calibration in setup (not in ISR)
  }
  g_t0 = (int8_t)(acc / 32);
}

/* --- Accelerometer init and read --- */
void LSM_Accel_Init(void)
{
    // CTRL1: 0x57 -> ODR = 100Hz, all axes enabled (same as your original)
    i2c_write8(LSM_A_ADDR_8, LSM_CTRL1_A, 0x57);
    // CTRL4: ±2g
    i2c_write8(LSM_A_ADDR_8, LSM_CTRL4_A, 0x00);
}

/* Calibrate accelerometer offsets (samples taken at rest) */
void LSM_Accel_Calibrate(int samples)
{
    float sx = 0, sy = 0, sz = 0;
    uint8_t d[6];

    for (int i = 0; i < samples; i++)
    {
        if (i2c_readn(LSM_A_ADDR_8, (LSM_OUT_X_L | LSM_AUTO_INC), d, 6) != HAL_OK) {
            HAL_Delay(2);
            continue;
        }

        int16_t rx = (int16_t)((d[1] << 8) | d[0]);
        int16_t ry = (int16_t)((d[3] << 8) | d[2]);
        int16_t rz = (int16_t)((d[5] << 8) | d[4]);

        sx += ((float)rx) * LSM_G_PER_LSB;
        sy += ((float)ry) * LSM_G_PER_LSB;
        sz += ((float)rz) * LSM_G_PER_LSB;

        HAL_Delay(5);
    }

    accel_off_x = sx / (float)samples;
    accel_off_y = sy / (float)samples;
    // subtract 1g from Z because gravity should read ~ +1.0 g on the axis pointing up
    accel_off_z = (sz / (float)samples) - 1.0f;
}

/* Read accelerometer and apply offsets -> output in g */
void LSM_Accel_Read(void)
{
    uint8_t d[6];

    if (i2c_readn(LSM_A_ADDR_8, (LSM_OUT_X_L | LSM_AUTO_INC), d, 6) != HAL_OK)
        return;

    int16_t rx = (int16_t)((d[1] << 8) | d[0]);
    int16_t ry = (int16_t)((d[3] << 8) | d[2]);
    int16_t rz = (int16_t)((d[5] << 8) | d[4]);

    /* convert to g and apply stored offsets */
    accel_x_g = ((float)rx) * LSM_G_PER_LSB - accel_off_x;
    accel_y_g = ((float)ry) * LSM_G_PER_LSB - accel_off_y;
    accel_z_g = ((float)rz) * LSM_G_PER_LSB - accel_off_z;
}



float pid_controller (PID *pid, float dt, float PV){ //process variable   
    float e_t = pid->set_point - PV; //error
    pid->integral += e_t*dt;

    float d = 0;
    if(dt>0){
    d = pid->kd*((e_t - pid->prev_e_t )/dt);}
    float p = pid->kp * e_t;
    pid->prev_e_t = e_t;
    pid->CoT = p + d + (pid->integral*pid->ki);

    

    if(pid->CoT > 999)
        pid->CoT = 999;
    else if (pid->CoT < -999)
        pid->CoT = -999;

    return pid->CoT;
}

// void speed(uint16_t s){
//     if (s>999){
//       s=999;
//       __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 999);
//     }
//     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, s);
//   }

// void set_direction(uint8_t a){
//     if(a==0){
//       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
//       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//     }
//     else{
//       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
//     }

//   }


// void speed2(uint16_t s){
//     if (s > 999) s = 999;
//     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, s);
// }

// void set_direction2(uint8_t a){
//     if(a==0){
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
//     } else {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
//     }
// }

void speed_motor(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t s) {
    if (s > 999) s = 999;
    __HAL_TIM_SET_COMPARE(htim, channel, s);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* TIM ISR: read sensors fast and set flag for printing at a slower rate */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    /* --- Read gyro registers (non-blocking SPI calls used synchronously) --- */
    uint8_t raw_g[6];
    gyro_read_regs(GYRO_OUT_X_L, raw_g, 6);

    int16_t gx_raw = (int16_t)((raw_g[1] << 8) | raw_g[0]);
    int16_t gy_raw = (int16_t)((raw_g[3] << 8) | raw_g[2]);
    int16_t gz_raw = (int16_t)((raw_g[5] << 8) | raw_g[4]);

    /* convert to deg/s */
    gyro_x_dps = (float)gx_raw * GYRO_SENS_245DPS;
    gyro_y_dps = (float)gy_raw * GYRO_SENS_245DPS;
    gyro_z_dps = (float)gz_raw * GYRO_SENS_245DPS;

    /* temperature (for info only) */
    uint8_t t_raw_u8 = 0;
    gyro_read_regs(GYRO_OUT_TEMP, &t_raw_u8, 1);
    int8_t t_raw = (int8_t)t_raw_u8;
    temp_c = CAL_ROOM_C + g_temp_slope * (t_raw - g_t0);

    /* --- Read accelerometer (I2C) --- */
    LSM_Accel_Read();

    /* --- Manage print rate (do not print from ISR) --- */
/* --- Complementary filter for pitch angle --- */
        float accel_angle = atan2f(accel_x_g, accel_z_g) * 57.2958f; // rad -> deg
        angle = 0.98f * (angle + gyro_y_dps * LOOP_DT) + 0.02f * accel_angle;
    //angle = 0.98*(angle + gyro_x_dps*0.01) + 0.02*accel_y_g;
    display_counter++;
    if (display_counter >= PRINT_RATE_DIV) {
        display_counter = 0;
        print_flag = true;  // main loop will handle UART print
    }
  }
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* Ensure gyro CS high before init */
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

  /* Init sensors */
  gyro_init();
  gyro_temp_calibrate();

  LSM_Accel_Init();
  /* small delay to let sensor stabilise on startup */
  
  /* calibrate accel offsets (put board flat and still) */
  LSM_Accel_Calibrate(40);

  /* Start TIM2 interrupt (main sensor loop) */
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 

  // speed(0); 
  // set_direction(0);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); 

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); 
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
  PID pid = { 
    .set_point = 0.0f, //should be zero 
    .kp= 25.0f, 
    .ki = 0.5f, 
    .kd = 2.0f, 
    .CoT =0.0f, 
    .prev_e_t = 0.0f, 
    .integral = 0.0f }; 
  /* Infinite loop: do printing and any background tasks here */ 
 while (1) { 
  if (print_flag) 
  { 
  print_lsm("%f\r\n", angle); 
   //float dt = 0.01; 
   float pid_out = pid_controller(&pid, LOOP_DT, angle); 

  //  if(pid_out > 0) set_direction(0); else set_direction(1);
  //   speed((uint16_t)fabs(pid_out)); 
     /* other non-time-critical background tasks can go here */ 
    //HAL_Delay(5); // keep main loop light; this delay doesn't affect ISR timing 
uint16_t output = (uint16_t)fminf(fabs(pid_out), 999.0f);

if (fabs(pid_out) < MOTOR_DEADZONE)
{
    speed_motor(&htim3, TIM_CHANNEL_1, 0);
    speed_motor(&htim3, TIM_CHANNEL_2, 0);
}
else
{
    // Set direction pins for both motors
    if(pid_out > 0)
    {
        // Forward
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // Motor 1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);   // Motor 2
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    }
    else
    {
        // Backward
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // Motor 1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // Motor 2
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    }

    // Set PWM for both motors
    uint16_t pwm_val = (uint16_t)fminf(fabs(pid_out), 999.0f);
    speed_motor(&htim3, TIM_CHANNEL_1, pwm_val);
    speed_motor(&htim3, TIM_CHANNEL_2, pwm_val);
}
    print_flag = false; 
    } }}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 47999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
    
  /* USER CODE END MX_GPIO_Init_2 */
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
