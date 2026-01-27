/* USER CODE BEGIN Header */
/**
  * FINAL INTEGRATION: STM32 + ENC28J60 + LwIP
  * CS Pin: PA5 (Green LED) - Hardwired Override
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* LwIP Includes */
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "ethernetif.h"
#include "enc28j60.h"

/* Global Variables */
struct netif gnetif;

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
ENC_HandleTypeDef henc;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  // --- 1. HARDWARE FIX: Force PA5 (LED) to be our Chip Select ---
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Deselect
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // 2. UART Debug Message
  char msg[100];
  sprintf(msg, "\r\n--- STM32 + LwIP + ENC28J60 START ---\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
  HAL_Delay(100);

  // 3. Configure Driver
  henc.Init.MACAddr[0] = 0x54;
  henc.Init.MACAddr[1] = 0x55;
  henc.Init.MACAddr[2] = 0x58;
  henc.Init.MACAddr[3] = 0x10;
  henc.Init.MACAddr[4] = 0x00;
  henc.Init.MACAddr[5] = 0x24;

  henc.Init.DuplexMode = ETH_MODE_HALFDUPLEX; // Forced Half-Duplex
  henc.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  henc.Init.InterruptEnableBits = 0;

  // 4. Start Driver
  sprintf(msg, "Initializing Hardware Driver...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  if (enc_start(&henc)) {
      sprintf(msg, "SUCCESS! ENC28J60 found and initialized.\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
      for(int i=0; i<6; i++) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); HAL_Delay(50); }
  } else {
      sprintf(msg, "FAILURE! Could not detect ENC28J60.\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
      Error_Handler();
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)"Forcing MAC update...\r\n", 23, 100);
    enc_force_mac_hardware(&henc);
    HAL_UART_Transmit(&huart2, (uint8_t*)"MAC Updated.\r\n", 14, 100);

  // 5. LwIP Init
  lwip_init();

  // 6. IP Settings
  ip4_addr_t ipaddr, netmask, gw;
  IP4_ADDR(&ipaddr, 192, 168, 0, 200);
  IP4_ADDR(&netmask, 255, 255, 255, 0);
  IP4_ADDR(&gw, 192, 168, 0, 1);

  // 7. Add Interface
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif)) {
      netif_set_up(&gnetif);
      sprintf(msg, "LwIP UP! IP: 192.168.0.200\r\n");
  } else {
      netif_set_up(&gnetif);
      sprintf(msg, "LwIP UP (Forced). IP: 192.168.0.200\r\n");
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  uint32_t last_arp_time = 0;

  while (1)
  {
    ethernetif_input(&gnetif);
    sys_check_timeouts();

    if (HAL_GetTick() - last_arp_time > 5000)
    {
       last_arp_time = HAL_GetTick();
       HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Heartbeat
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
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

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

uint32_t sys_now(void)
{
  return HAL_GetTick();
}
