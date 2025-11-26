/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "packets.h"
#include "tft.h"
#include <string.h>

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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */


		#define PKT_SYNC_SIZE    2   // 0xA5, 0x5A
		#define PKT_HEADER_SIZE  (sizeof(pkt_header_t))  // 7 bytes (packed)
		#define PKT_PREFIX_SIZE  (PKT_SYNC_SIZE + PKT_HEADER_SIZE) // 9 bytes before payload
		#define PKT_CRC_SIZE     2

		#define UART_ACK_HEADER  'H'
		#define UART_ACK_OK      'S'
		#define UART_ACK_ERR     'E'
		#define UART_ACK_NEXT_AUDIO_CHUNK 'A'

		// Packet size/overhead (must match Python script)
		#define PKT_OVERHEAD     (PKT_SYNC_SIZE + PKT_HEADER_SIZE + PKT_CRC_SIZE) // 11
		#define PKT_MAX_SIZE     0xFFFFu  // total packet: sync+header+payload+CRC

		#define IMG_PKT_MAX_SIZE     PKT_MAX_SIZE
		#define AUD_PKT_MAX_SIZE     PKT_MAX_SIZE

		#define IMG_MAX_PAYLOAD_BYTES  (IMG_PKT_MAX_SIZE - PKT_OVERHEAD)
		#define AUD_MAX_PAYLOAD_BYTES  (AUD_PKT_MAX_SIZE - PKT_OVERHEAD)

		// DAC circular buffer: 2 halves, each half holds one max audio payload worth of samples
		// 2 bytes per 16-bit PCM sample => samples = bytes/2
		#define DAC_HALF_SAMPLES   (AUD_MAX_PAYLOAD_BYTES / 2u)
		#define DAC_BUF_SAMPLES    (2u * DAC_HALF_SAMPLES)

		// UART RX context
	typedef enum {
		RX_STATE_WAIT_PREFIX,   // waiting for 2 sync + 7 header bytes
		RX_STATE_WAIT_PAYLOAD,  // waiting for payload (image or audio)
		RX_STATE_WAIT_CRC       // waiting for audio CRC (2 bytes)
	} uart_rx_state_t;


		typedef struct {
			uart_rx_state_t state;
			uint8_t  prefix_buf[PKT_PREFIX_SIZE];  // 9 bytes: sync + header
			pkt_header_t header;
			uint16_t payload_len;
			uint8_t *payload_dst;                  // destination buffer for payload+CRC
		} uart_rx_ctx_t;

		uart_rx_ctx_t g_uart_rx;

		// Buffers
		uint16_t dac_buf[DAC_BUF_SAMPLES];  // DAC circular buffer

		uint8_t image_pkt_buf[IMG_MAX_PAYLOAD_BYTES + PKT_CRC_SIZE];

		// Audio RX views onto dac_buf halves
		uint8_t *audio_pkt_buf0 = (uint8_t *)dac_buf;                      // first half
		uint8_t *audio_pkt_buf1 = (uint8_t *)&dac_buf[DAC_HALF_SAMPLES];   // second half

		// NEW: CRC storage for audio packets
		uint8_t audio_crc_buf[PKT_CRC_SIZE];

		// Flags
		volatile uint8_t  image_pkt_ready = 0;
		volatile uint16_t image_pkt_len   = 0;

		volatile uint8_t  audio_pkt_ready = 0;
		volatile uint8_t  audio_write_idx = 0;   // 0 or 1
		volatile uint16_t audio_pkt_len   = 0;

		// Which halves of dac_buf are free to write into
		volatile uint8_t half0_free = 1;  // dac_buf[0 .. DAC_HALF_SAMPLES-1]
		volatile uint8_t half1_free = 1;  // dac_buf[DAC_HALF_SAMPLES .. end]

		volatile uint8_t audio_stream_active = 0;

		// Stream state for DAC priming
		volatile uint8_t dac_running       = 0;  // 0 = DAC DMA stopped, 1 = running
		volatile uint8_t audio_prime_count = 0;  // how many good packets we've accepted this stream



		volatile uint16_t ctrl_pkt_len   = 0;
		volatile uint8_t ctrl_pkt_ready = 0;
		uint8_t ctrl_buf[16];




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void uart_start_header_rx(void);
static void protocol_soft_reset(void);

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_LPUART1_UART_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  tft_init();
  tft_fill_rect(0, 0, 480, 320, 0xAAAA);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // LED ON

  uart_start_header_rx();

  /* Start TIM2 (48 kHz trigger) */
  HAL_TIM_Base_Start(&htim2);

  /* Init DAC circular buffer to mid-scale (silence) */
  for (uint32_t i = 0; i < DAC_BUF_SAMPLES; ++i) {
      dac_buf[i] = 2048; // mid-scale for 12-bit
  }

  /* Do NOT start DAC DMA here.
   * We'll start it after we've "primed" the buffer with at least
   * two good audio packets of a new stream.
   */
  dac_running       = 0;
  audio_prime_count = 0;
  audio_stream_active = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

	  if (ctrl_pkt_ready) {
	          ctrl_pkt_ready = 0;
	          uint8_t cmd = ctrl_buf[0];

	          if (cmd == CTRL_CMD_AUDIO_STOP) {
	              __disable_irq();
	              audio_stream_active = 0;

	              // Drop any pending audio packet in-flight
	              audio_pkt_ready = 0;
	              audio_pkt_len   = 0;

	              // Release both halves so future streams start clean
	              half0_free = 1;
	              half1_free = 1;

	              // Reset priming state
	              audio_prime_count = 0;

	              __enable_irq();

	              // Stop DAC DMA if it was running
	              if (dac_running) {
	                  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	                  dac_running = 0;
	              }

	              // Fill DAC buffer with silence
	              for (uint32_t i = 0; i < DAC_BUF_SAMPLES; ++i) {
	                  dac_buf[i] = 2048;
	              }

	              uint8_t ack = UART_ACK_OK; // 'S'
	              HAL_UART_Transmit(&hlpuart1, &ack, 1, 10);

	              // Re-arm header RX
	              uart_start_header_rx();
	          }

	          // Skip directly to next loop iteration so we don't process audio
	          // in the same pass after a STOP.
	          continue;
	      }


	  // 1) Handle image packets
	  if (image_pkt_ready) {
	      uint16_t payload_len;
	      uint8_t  hdr_bytes[CRC_HEADER_LEN];

	      __disable_irq();
	      payload_len      = image_pkt_len;   // bytes of image payload
	      image_pkt_ready  = 0;

	      // Snapshot the header fields for this packet into hdr_bytes[0..6]
	      hdr_bytes[0] = g_uart_rx.header.version;
	      hdr_bytes[1] = g_uart_rx.header.data_type;
	      hdr_bytes[2] = g_uart_rx.header.flags;
	      hdr_bytes[3] = (uint8_t)(g_uart_rx.header.seq >> 8);
	      hdr_bytes[4] = (uint8_t)(g_uart_rx.header.seq & 0xFF);
	      hdr_bytes[5] = (uint8_t)(g_uart_rx.header.len >> 8);
	      hdr_bytes[6] = (uint8_t)(g_uart_rx.header.len & 0xFF);
	      __enable_irq();

	      if (payload_len <= sizeof(image_pkt_buf)) {

	          uint16_t recv_crc =
	              ((uint16_t)image_pkt_buf[payload_len] << 8) |
	               (uint16_t)image_pkt_buf[payload_len + 1];

	          // Use the *snapshotted* header bytes, not prefix_buf
	          uint16_t calc_crc = crc16_ccitt(hdr_bytes, image_pkt_buf, payload_len);

	          if (calc_crc != recv_crc) {
	              uint8_t nack = UART_ACK_ERR;
	              HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
	          } else {
	              parse_and_apply_image_packet(image_pkt_buf, payload_len);

	              uint8_t ack = UART_ACK_OK;
	              HAL_UART_Transmit(&hlpuart1, &ack, 1, 10);
	          }
	      } else {
	          uint8_t nack = UART_ACK_ERR;
	          HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
	      }
	  }



		  // 2) Handle audio packets (fill halves of dac_buf, DAC runs continuously)
      // 2) Handle audio packets (convert in-place in dac_buf, DAC runs continuously)
	  if (audio_pkt_ready) {
	      uint8_t  idx;
	      uint16_t payload_len;
	      uint8_t  hdr_bytes[CRC_HEADER_LEN];
	      uint8_t  send_manual_A = 0;  // whether to send 'A' after ACK (priming)

	      __disable_irq();
	      idx             = audio_write_idx;   // 0 or 1
	      payload_len     = audio_pkt_len;     // bytes of audio payload
	      audio_pkt_ready = 0;

	      // Snapshot header into hdr_bytes
	      hdr_bytes[0] = g_uart_rx.header.version;
	      hdr_bytes[1] = g_uart_rx.header.data_type;
	      hdr_bytes[2] = g_uart_rx.header.flags;
	      hdr_bytes[3] = (uint8_t)(g_uart_rx.header.seq >> 8);
	      hdr_bytes[4] = (uint8_t)(g_uart_rx.header.seq & 0xFF);
	      hdr_bytes[5] = (uint8_t)(g_uart_rx.header.len >> 8);
	      hdr_bytes[6] = (uint8_t)(g_uart_rx.header.len & 0xFF);
	      __enable_irq();

	      // Sanity-check payload length vs max audio payload
	      if (payload_len > AUD_MAX_PAYLOAD_BYTES) {
	          // free the reserved half so we don't leak it
	          if (idx == 0) half0_free = 1;
	          else          half1_free = 1;

	          uint8_t nack = UART_ACK_ERR;
	          HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);

	          // Now ready to receive the next header
	          uart_start_header_rx();
	          continue;
	      }

	      // CRC from separate buffer (big-endian: [hi][lo])
	      uint16_t recv_crc =
	          ((uint16_t)audio_crc_buf[0] << 8) |
	           (uint16_t)audio_crc_buf[1];

	      // Payload lives in selected half of dac_buf
	      uint8_t *payload_bytes = (uint8_t *)(
	          (idx == 0)
	          ? &dac_buf[0]
	          : &dac_buf[DAC_HALF_SAMPLES]
	      );

	      // Use snapshotted header, not prefix_buf
	      uint16_t calc_crc = crc16_ccitt(hdr_bytes,
	                                      payload_bytes,
	                                      payload_len);

	      if (calc_crc != recv_crc) {
	          // free the half so we can reuse it
	          if (idx == 0) half0_free = 1;
	          else          half1_free = 1;

	          uint8_t nack = UART_ACK_ERR;
	          HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);

	          // Re-arm header RX
	          uart_start_header_rx();
	          continue;
	      }

	      // Good packet. Convert in-place in chosen half of dac_buf.
	      uint16_t num_samples = payload_len / 2;  // 2 bytes/sample
	      if (num_samples > DAC_HALF_SAMPLES) {
	          num_samples = DAC_HALF_SAMPLES;
	      }

	      int16_t  *src = (int16_t *)payload_bytes;  // signed PCM
	      uint16_t *dst = (uint16_t *)payload_bytes; // same locations for DAC

	      for (uint16_t i = 0; i < num_samples; ++i) {
	          int32_t s = src[i];      // -32768..32767
	          s += 32768;              // 0..65535
	          if (s < 0)      s = 0;
	          if (s > 65535)  s = 65535;
	          dst[i] = (uint16_t)(s >> 4);   // 12-bit right aligned
	      }

	      // Pad rest of half with mid-scale if packet smaller than half
	      for (uint16_t i = num_samples; i < DAC_HALF_SAMPLES; ++i) {
	          dst[i] = 2048;
	      }

	      // ---- Priming / DAC-start logic ----
	      // If DAC is not running yet, we are in "priming" mode.
	      if (!dac_running) {
	          audio_prime_count++;

	          if (audio_prime_count == 1) {
	              // First good packet of this stream:
	              // we want the PC to send the 2nd packet immediately,
	              // even though the DAC isn't running yet.
	              send_manual_A = 1;
	          }
	          else if (audio_prime_count == 2) {
	              // Second good packet: we now have two halves filled,
	              // so it's safe to start DAC DMA over the full buffer.
	              if (HAL_DAC_Start_DMA(&hdac1,
	                                    DAC_CHANNEL_1,
	                                    (uint32_t *)dac_buf,
	                                    DAC_BUF_SAMPLES,
	                                    DAC_ALIGN_12B_R) != HAL_OK) {
	                  Error_Handler();
	              }
	              dac_running         = 1;
	              audio_stream_active = 1;
	          }
	      } else {
	          // Already running: just keep stream_active unless this is LAST
	          if (!(hdr_bytes[2] & PKT_FLAG_LAST)) {
	              audio_stream_active = 1;
	          }
	      }

	      // If this packet is marked LAST, stop requesting future audio.
	      if (hdr_bytes[2] & PKT_FLAG_LAST) {
	          audio_stream_active = 0;
	      }

	      // Tell PC this packet was accepted and written into that half
	      uint8_t ack = UART_ACK_OK;
	      HAL_UART_Transmit(&hlpuart1, &ack, 1, 10);

	      // If we are in priming and just finished the first packet,
	      // trigger the PC to send the next chunk immediately.
	      if (send_manual_A) {
	          uint8_t req = UART_ACK_NEXT_AUDIO_CHUNK; // 'A'
	          HAL_UART_Transmit(&hlpuart1, &req, 1, 10);
	      }

	      // NOW re-arm UART to receive the next header
	      uart_start_header_rx();
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 15;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 1500000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 119;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3749;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /*Configure GPIO pins : PD8 PD9 (USART3 TX/RX) */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



	static void protocol_soft_reset(void)
	{
		__disable_irq();
		HAL_UART_AbortReceive(&hlpuart1);
		g_uart_rx.state       = RX_STATE_WAIT_PREFIX;
		g_uart_rx.payload_len = 0;
		g_uart_rx.payload_dst = NULL;

		image_pkt_ready = 0;
		image_pkt_len   = 0;
		audio_pkt_ready = 0;
		audio_pkt_len   = 0;

		half0_free = 1;
		half1_free = 1;
		audio_stream_active = 0;
		audio_prime_count   = 0;  // NEW: reset priming
		__enable_irq();

		uart_start_header_rx();
	}




	static void uart_start_header_rx(void)
	{
		g_uart_rx.state = RX_STATE_WAIT_PREFIX;
		HAL_UART_Receive_DMA(&hlpuart1,
							g_uart_rx.prefix_buf,
							PKT_PREFIX_SIZE);   // 9 bytes: sync + header


	}


	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	    if (huart->Instance != LPUART1)
	        return;

	    if (g_uart_rx.state == RX_STATE_WAIT_PREFIX) {
	        // We just received sync + header (9 bytes)
	        uint8_t *buf = g_uart_rx.prefix_buf;

	        // 1) Check sync bytes
	        if (buf[0] != 0xA5 || buf[1] != 0x5A) {
	            uart_start_header_rx();
	            return;
	        }

	        // 2) Extract payload length
	        uint16_t payload_len = ((uint16_t)buf[7] << 8) | buf[8];

	        // 3) Parse header fields
	        g_uart_rx.header.version   = buf[2];
	        g_uart_rx.header.data_type = buf[3];
	        g_uart_rx.header.flags     = buf[4];
	        g_uart_rx.header.seq       = ((uint16_t)buf[5] << 8) | buf[6];
	        g_uart_rx.header.len       = payload_len;
	        g_uart_rx.payload_len      = payload_len;

	        // 4) Choose destination buffer based on packet type
	        switch (g_uart_rx.header.data_type) {
	        case PKT_DATA_IMAGE:
	            if (payload_len + PKT_CRC_SIZE > sizeof(image_pkt_buf)) {
	                uart_start_header_rx();
	                return;
	            }
	            g_uart_rx.payload_dst = image_pkt_buf;
	            break;

	        case PKT_DATA_AUDIO:
	            // Sanity-check payload length in BYTES (just payload, *not* CRC)
	            if (payload_len > AUD_MAX_PAYLOAD_BYTES) {
	                uart_start_header_rx();
	                return;
	            }

	            // Choose a free half of dac_buf as DMA destination for the *payload*
	            if (half0_free) {
	                g_uart_rx.payload_dst = audio_pkt_buf0; // first half of dac_buf
	                audio_write_idx = 0;
	                half0_free = 0;  // now reserved
	            } else if (half1_free) {
	                g_uart_rx.payload_dst = audio_pkt_buf1; // second half of dac_buf
	                audio_write_idx = 1;
	                half1_free = 0;  // now reserved
	            } else {
	                // No free half; can't accept this packet safely
	                uart_start_header_rx();
	                return;
	            }
	            break;

	        case PKT_DATA_CMD:
	            if (payload_len > sizeof(ctrl_buf)) {
	                uart_start_header_rx();
	                return;
	            }
	            g_uart_rx.payload_dst = ctrl_buf;
	            break;



	        default:
	            // Unknown type -> ignore this frame
	            uart_start_header_rx();
	            return;
	        }

	        // 5) Set state and start DMA for payload + CRC
	        g_uart_rx.state = RX_STATE_WAIT_PAYLOAD;

	        if (g_uart_rx.header.data_type == PKT_DATA_IMAGE) {
	            // Image: payload + CRC in one shot
	            HAL_UART_Receive_DMA(&hlpuart1,
	                                 g_uart_rx.payload_dst,
	                                 payload_len + PKT_CRC_SIZE);
	        }
	        else if (g_uart_rx.header.data_type == PKT_DATA_AUDIO) {
	            // Audio: ONLY the payload goes into dac_buf; CRC comes next in a tiny transfer
	            HAL_UART_Receive_DMA(&hlpuart1,
	                                 g_uart_rx.payload_dst,
	                                 payload_len);
	        }
	        else if (g_uart_rx.header.data_type == PKT_DATA_CMD) {
	            // Control: simplest is payload+CRC in one shot like image
	            HAL_UART_Receive_DMA(&hlpuart1,
	                                 g_uart_rx.payload_dst,
	                                 payload_len + PKT_CRC_SIZE);
	        }


	        // 6) Tell PC "header OK, I'm ready for payload"
	        uint8_t ack_hdr = UART_ACK_HEADER;  // 'H'
	        HAL_UART_Transmit(&hlpuart1, &ack_hdr, 1, 10);
	    }
	    else if (g_uart_rx.state == RX_STATE_WAIT_PAYLOAD) {
	        if (g_uart_rx.header.data_type == PKT_DATA_IMAGE) {
	            // IMAGE: we already DMA'd payload+CRC into image_pkt_buf
	            image_pkt_len   = g_uart_rx.payload_len;  // payload length (no CRC)
	            image_pkt_ready = 1;

	            // Back to waiting for next header
	            uart_start_header_rx();
	        }
	        else if (g_uart_rx.header.data_type == PKT_DATA_AUDIO) {
	            // AUDIO: we have JUST the payload in dac_buf now.
	            // Next, we need to grab the 2 CRC bytes.

	            g_uart_rx.state = RX_STATE_WAIT_CRC;

	            // Start a tiny 2-byte DMA into audio_crc_buf
	            HAL_UART_Receive_DMA(&hlpuart1,
	                                 audio_crc_buf,
	                                 PKT_CRC_SIZE);
	            // Do NOT call uart_start_header_rx() yet; we still need CRC.
	        }
	        else if (g_uart_rx.header.data_type == PKT_DATA_CMD) {
	            // CTRL: payload + CRC already in ctrl_buf
	            ctrl_pkt_len   = g_uart_rx.payload_len;  // payload length (no CRC)
	            ctrl_pkt_ready = 1;

	            // Go back to waiting for next header
	            uart_start_header_rx();
	        }

	    }
	    else if (g_uart_rx.state == RX_STATE_WAIT_CRC) {
	        // We just finished receiving the 2 CRC bytes for an audio packet
	        if (g_uart_rx.header.data_type == PKT_DATA_AUDIO) {
	            audio_pkt_len   = g_uart_rx.payload_len;  // just payload length
	            audio_pkt_ready = 1;
	        }

	        // Do NOT start the next header RX here.
	        // Just mark that we're logically back in WAIT_PREFIX;
	        // the main loop will call uart_start_header_rx() once it
	        // has validated this packet and sent 'S' or 'E'.
	        g_uart_rx.state = RX_STATE_WAIT_PREFIX;
	    }



	}

	void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
	{
	    if (hdac->Instance != DAC1) return;

	    half0_free = 1;

	    if (audio_stream_active) {
	        uint8_t req = UART_ACK_NEXT_AUDIO_CHUNK; // 'A'
	        HAL_UART_Transmit(&hlpuart1, &req, 1, 10);
	    }
	}

	void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
	{
	    if (hdac->Instance != DAC1) return;

	    half1_free = 1;

	    if (audio_stream_active) {
	        uint8_t req = UART_ACK_NEXT_AUDIO_CHUNK; // 'A'
	        HAL_UART_Transmit(&hlpuart1, &req, 1, 10);
	    }
	}



	void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
	{
	    if (huart->Instance == LPUART1) {
	        // Log something if you have a debug LED/UART
	        protocol_soft_reset();
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
