/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - STM32F411 + MPU6050
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"

#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  int16_t x, y, z;
} accel_raw_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* MPU-6050 (I2C) */
#define MPU_ADDR_68           (0x68u << 1)   /* HAL expects 8-bit addr (7-bit <<1) */
#define MPU_ADDR_69           (0x69u << 1)

#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_WHO_AM_I      0x75          /* expected 0x68 */
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint16_t mpu_addr = MPU_ADDR_68;   /* set to MPU_ADDR_69 if AD0 tied high */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef mpu_write_u8(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef mpu_read_u8(uint8_t reg, uint8_t *val);
static HAL_StatusTypeDef mpu_read_bytes(uint8_t start_reg, uint8_t *buf, uint16_t len);
static void i2c_scan(void);
static int  mpu_init(void);
static int  mpu_read_accel(accel_raw_t *r);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Retarget printf to USART2 */
int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

/* Basic MPU I2C functions */
static HAL_StatusTypeDef mpu_write_u8(uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c1, mpu_addr, reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef mpu_read_u8(uint8_t reg, uint8_t *val) {
  return HAL_I2C_Mem_Read(&hi2c1, mpu_addr, reg, I2C_MEMADD_SIZE_8BIT,
                          val, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef mpu_read_bytes(uint8_t start_reg, uint8_t *buf, uint16_t len) {
  return HAL_I2C_Mem_Read(&hi2c1, mpu_addr, start_reg, I2C_MEMADD_SIZE_8BIT,
                          buf, len, HAL_MAX_DELAY);
}

/* I2C bus scanner for diagnostics */
static void i2c_scan(void) {
  printf("\r\n=== I2C Bus Scan ===\r\n");
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 128; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
      printf("  Device found at 0x%02X\r\n", addr);
      found++;
    }
  }

  if (found == 0) {
    printf("  No devices found!\r\n");
    printf("  Check:\r\n");
    printf("    - SDA/SCL connections (PB9/PB8)\r\n");
    printf("    - Pull-up resistors (4.7k or 10k)\r\n");
    printf("    - Power supply (3.3V)\r\n");
  }
  printf("===================\r\n\r\n");
}

/* Initialize MPU6050 with full diagnostics */
static int mpu_init(void) {
  HAL_StatusTypeDef ret;

  printf("MPU6050 initialization...\r\n");

  /* Check if device responds at default address */
  printf("Checking address 0x68...\r\n");
  ret = HAL_I2C_IsDeviceReady(&hi2c1, mpu_addr, 3, 100);

  if (ret != HAL_OK) {
    printf("  No response at 0x68 (HAL status=%d)\r\n", ret);
    printf("Trying alternate address 0x69...\r\n");

    mpu_addr = MPU_ADDR_69;
    ret = HAL_I2C_IsDeviceReady(&hi2c1, mpu_addr, 3, 100);

    if (ret != HAL_OK) {
      printf("  ERROR: MPU not found at either address!\r\n");
      printf("  Check hardware connections.\r\n");
      return -10;
    }
    printf("  Found at 0x69!\r\n");
  } else {
    printf("  Found at 0x68!\r\n");
  }

  /* Reset device */
  printf("Resetting MPU6050...\r\n");
  if (mpu_write_u8(MPU_REG_PWR_MGMT_1, 0x80) != HAL_OK) {
    printf("  ERROR: Reset command failed\r\n");
    return -1;
  }
  HAL_Delay(100);  /* Wait for reset to complete */

  /* Wake up from sleep mode */
  printf("Waking up MPU6050...\r\n");
  if (mpu_write_u8(MPU_REG_PWR_MGMT_1, 0x00) != HAL_OK) {
    printf("  ERROR: Wake up command failed\r\n");
    return -2;
  }
  HAL_Delay(100);  /* CRITICAL: Wait for sensor to stabilize */

  /* Configure accelerometer: ±2g range */
  printf("Configuring accelerometer (±2g)...\r\n");
  if (mpu_write_u8(MPU_REG_ACCEL_CONFIG, 0x00) != HAL_OK) {
    printf("  ERROR: Accel config failed\r\n");
    return -3;
  }
  HAL_Delay(10);

  /* Verify WHO_AM_I register */
  uint8_t who = 0;
  printf("Reading WHO_AM_I...\r\n");
  if (mpu_read_u8(MPU_REG_WHO_AM_I, &who) != HAL_OK) {
    printf("  ERROR: WHO_AM_I read failed\r\n");
    return -4;
  }

  printf("  WHO_AM_I = 0x%02X ", who);
  if (who == 0x68) {
    printf("(OK)\r\n");
  } else {
    printf("(WARNING: expected 0x68, but sensor may still work)\r\n");
  }

  /* Test accelerometer reading */
  printf("Testing accelerometer read...\r\n");
  accel_raw_t test;
  if (mpu_read_accel(&test) == 0) {
    printf("  Test read successful: X=%d Y=%d Z=%d\r\n", test.x, test.y, test.z);

    /* Sanity check: Z-axis should show gravity (~16384 for ±2g when flat) */
    if (test.z < -20000 || test.z > 20000) {
      printf("  WARNING: Z-axis value seems unusual\r\n");
    }

    printf("\r\n*** MPU6050 initialization SUCCESS! ***\r\n\r\n");
  } else {
    printf("  ERROR: Test read failed\r\n");
    return -5;
  }

  return 0;
}

/* Read raw accelerometer data */
static int mpu_read_accel(accel_raw_t *r) {
  uint8_t b[6];

  /* Read 6 bytes starting from ACCEL_XOUT_H */
  if (mpu_read_bytes(MPU_REG_ACCEL_XOUT_H, b, 6) != HAL_OK) {
    return -1;
  }

  /* Combine high and low bytes (big-endian) */
  r->x = (int16_t)((b[0] << 8) | b[1]);
  r->y = (int16_t)((b[2] << 8) | b[3]);
  r->z = (int16_t)((b[4] << 8) | b[5]);

  return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();

  /* Startup banner */
  printf("\r\n");
  printf("========================================\r\n");
  printf("  STM32F411 + MPU6050 Accelerometer\r\n");
  printf("  I2C1: PB8(SCL), PB9(SDA)\r\n");
  printf("  UART2: PA2(TX), PA3(RX)\r\n");
  printf("========================================\r\n");

  /* Scan I2C bus for devices */
  i2c_scan();

  /* Initialize MPU6050 */
  int rc = mpu_init();
  if (rc != 0) {
    printf("\r\n!!! MPU6050 initialization FAILED (code=%d) !!!\r\n", rc);
    printf("System will continue but data may be invalid.\r\n\r\n");
  }

  /* Start data streaming */
  printf("Starting accelerometer data stream...\r\n");
  printf("Format: timestamp,accel_x,accel_y,accel_z (raw values)\r\n");
  printf("Sample rate: ~20 Hz\r\n\r\n");

  uint32_t t0 = HAL_GetTick();
  uint32_t sample_count = 0;

  /* Infinite loop */
  while (1)
  {
    MX_USB_HOST_Process();

    /* Read accelerometer at ~20 Hz */
    if (HAL_GetTick() - t0 >= 50) {
      t0 += 50;

      accel_raw_t r;
      if (mpu_read_accel(&r) == 0) {
        /* CSV format: tick,ax,ay,az (raw) */
        printf("%lu,%d,%d,%d\r\n",
               (unsigned long)HAL_GetTick(), r.x, r.y, r.z);

        sample_count++;

        /* Toggle LED every second (alive indicator) */
        if (sample_count % 20 == 0) {
          HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        }
      } else {
        printf("%lu,ERROR,ERROR,ERROR\r\n", (unsigned long)HAL_GetTick());
      }
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
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
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
