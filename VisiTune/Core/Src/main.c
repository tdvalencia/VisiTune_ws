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
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

// ADC declarations
uint16_t adc_buf[4];
uint8_t rewind_flag = 0;
uint8_t pause_state = 0;
uint8_t forward_flag = 0;

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




		//CODE FOR FILTERING
		//LOW PASS
		//#define FIR_TAPS 74
		//arm_fir_instance_q15 fir_low;
		//q15_t firState_low[FIR_TAPS + DAC_HALF_SAMPLES - 1];
//		q15_t firCoeffs_low[74] = {
//		      -6,   -9,  -12,  -15,
//		     -19,  -22,  -25,  -26,
//		     -26,  -24,  -20,  -12,
//		       0,   16,   38,   65,
//		      98,  137,  183,  235,
//		     293,  357,  427,  501,
//		     578,  658,  739,  820,
//		     899,  975, 1047, 1112,
//		    1170, 1219, 1258, 1287,
//		    1305, 1311, 1305, 1287,
//		    1258, 1219, 1170, 1112,
//		    1047,  975,  899,  820,
//		     739,  658,  578,  501,
//		     427,  357,  293,  235,
//		     183,  137,   98,   65,
//		      38,   16,    0,  -12,
//		     -20,  -24,  -26,  -26,
//		     -25,  -22,  -19,  -15,
//		     -12,   -9
//		};
//
//		q15_t filter_buf_low[DAC_HALF_SAMPLES];
		//END LOW PASS

		//arm_fir_instance_q15 fir_high;
		//q15_t firState_high[FIR_TAPS + DAC_HALF_SAMPLES - 1];
		//q15_t firCoeffs_high[74] = {
//		       0,   -4,  -10,  -16,
//		     -22,  -23,  -18,   -4,
//		      17,   45,   73,   94,
//		     100,   85,   43,  -24,
//		    -108, -194, -263, -295,
//		    -271, -179,  -22,  186,
//		     414,  619,  751,  760,
//		     608,  270, -252, -929,
//		    -1707, -2513, -3262, -3869,
//		    -4266, 28365, -4266, -3869,
//		    -3262, -2513, -1707, -929,
//		    -252,  270,  608,  760,
//		     751,  619,  414,  186,
//		     -22, -179, -271, -295,
//		    -263, -194, -108,  -24,
//		      43,   85,  100,   94,
//		      73,   45,   17,   -4,
//		     -18,  -23,  -22,  -16,
//		     -10,   -4
//		};
//
		q15_t filter_buf[DAC_HALF_SAMPLES];
		#define FFT_SIZE 1024
		#define FFT_INCREMENT (FFT_SIZE / 2)

		float32_t fft_in[FFT_SIZE];
		float32_t fft_out[FFT_SIZE];
		float32_t overlap[FFT_INCREMENT];
		static float32_t window[FFT_SIZE];
		q15_t debug_out_buffer[DAC_HALF_SAMPLES];

		arm_rfft_fast_instance_f32 rfft;


		//END FILTERING
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
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void uart_start_header_rx(void);
void processFFT(const q15_t* in, q15_t* out, uint16_t num_samples);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void processFFT(const q15_t* in, q15_t* out, uint16_t num_samples) {
	uint32_t pos = 0;
	while ((pos + FFT_INCREMENT) <= num_samples) {
		for (int i = 0; i < FFT_SIZE; i++) {
			float32_t sample = 0.0f;
		    if (pos + i < num_samples) {
		    	sample = (float32_t)in[pos + i] / 32768.0f;  // q15 → float
		    }
		    fft_in[i] = sample * window[i];
		}

		arm_rfft_fast_f32(&rfft, fft_in, fft_out, 0);

		int low_bin = 15;
		int high_bin = 450;
		float32_t lowShelfGain = 3.5f;
		//for (int b = cutoff_bin; b < FFT_INCREMENT; b++) {
		//	fft_out[b*2]   = 0.0f;
		//    fft_out[b*2+1] = 0.0f;
		//}
		for (int b = 0; b < low_bin; b++) {
			float32_t weight = lowShelfGain +
			                       (1.0f - lowShelfGain) * ((float)b / low_bin);
			fft_out[b*2] *= weight;
			fft_out[b*2+1] *= weight;
		}


		for (int b = low_bin; b < high_bin; b++) {
			fft_out[b*2] *= 0.0f;
			fft_out[b*2] *= 0.0f;
		}

		arm_rfft_fast_f32(&rfft, fft_out, fft_in, 1);

		//don't get this
		for (int i = 0; i < FFT_INCREMENT; i++) {
			fft_in[i] += overlap[i];
		}
		//end don't get this

		for (int i = 0; i < FFT_INCREMENT && pos + i < num_samples; i++) {
			float32_t x = fft_in[i];
			if (x > 1.0f) x = 1.0f;
			if (x < -1.0f) x = -1.0f;
			out[pos + i] = (int16_t)(x * 32767.0f);
		}

		for (int i = 0; i < FFT_INCREMENT; ++i) {
			overlap[i] = fft_in[i+FFT_INCREMENT];
		}

		pos += FFT_INCREMENT;
	}
	//for (uint32_t i = 0; i < num_samples; i++) {
	//    debug_out_buffer[i] = out[i];
	//}
	//int bruhhhh = 0;
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_LPUART1_UART_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//  tft_init();
//  tft_fill_rect(0, 0, 480, 320, 0xAAAA);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // LED ON

  // Start the ADC reads for the potentiometers and sliders
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, adc_buf, 4);
  HAL_TIM_Base_Start(&htim3); // Start the adc conversions

  // Receieve the header from UART
  uart_start_header_rx();

  /* Start TIM2 (48 kHz trigger) */
  HAL_TIM_Base_Start(&htim2);

  /* Init DAC circular buffer to mid-scale (silence) */
  for (uint32_t i = 0; i < DAC_BUF_SAMPLES; ++i) {
      dac_buf[i] = 2048; // mid-scale for 12-bit
  }



  /* Start DAC in circular DMA mode over dac_buf */
  if (HAL_DAC_Start_DMA(&hdac1,
                        DAC_CHANNEL_1,
                        (uint32_t *)dac_buf,
                        DAC_BUF_SAMPLES,
                        DAC_ALIGN_12B_R) != HAL_OK) {
      Error_Handler();
  }

  //INIT FIR
  arm_rfft_fast_init_f32(&rfft, FFT_SIZE);
  for (int i = 0; i < FFT_SIZE; i++) {
      window[i] = 0.5f - 0.5f * arm_cos_f32((2 * PI * i) / (FFT_SIZE - 1));
  }
  memset(overlap, 0, sizeof(overlap));
  //arm_status lowPassStatus = arm_fir_init_q15(&fir_low, FIR_TAPS, firCoeffs_low, firState_low, DAC_HALF_SAMPLES);
  //if (lowPassStatus != ARM_MATH_SUCCESS) {
	//  Error_Handler();
  //}
//  arm_status highPassStatus = arm_fir_init_q15(&fir_high, FIR_TAPS, firCoeffs_high, firState_high, DAC_HALF_SAMPLES);
//    if (highPassStatus != ARM_MATH_SUCCESS) {
//  	  Error_Handler();
//    }
  //END INIT FIR


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  // 1) Handle image packets
	  if (image_pkt_ready) {
	      __disable_irq();
	      uint16_t payload_len = image_pkt_len;   // bytes of image payload
	      image_pkt_ready = 0;
	      __enable_irq();

	      if (payload_len  <= sizeof(image_pkt_buf)) {

	          uint16_t recv_crc =
	              ((uint16_t)image_pkt_buf[payload_len] << 8) |
	               (uint16_t)image_pkt_buf[payload_len + 1];

	          uint16_t calc_crc = crc16_ccitt(&g_uart_rx.prefix_buf[2], image_pkt_buf, payload_len);

	          if (calc_crc != recv_crc) {
	              uint8_t nack = UART_ACK_ERR;
	              HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
	          } else {
	              // Example: full-frame 480x320 RGB565
	        	  parse_and_apply_image_packet(image_pkt_buf, payload_len);
	              // else: decode according to your actual format.

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
	      __disable_irq();
	      uint8_t  idx         = audio_write_idx;  // 0 or 1
	      uint16_t payload_len = audio_pkt_len;    // bytes of audio payload
	      audio_pkt_ready = 0;
	      __enable_irq();

	      // Sanity-check payload length vs half-buffer size
	      if (payload_len > AUD_MAX_PAYLOAD_BYTES) {
	          uint8_t nack = UART_ACK_ERR;
	          HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
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

	      uint16_t calc_crc = crc16_ccitt(&g_uart_rx.prefix_buf[2],
	                                      payload_bytes,
	                                      payload_len);

	      if (calc_crc != recv_crc) {
	          uint8_t nack = UART_ACK_ERR;
	          HAL_UART_Transmit(&hlpuart1, &nack, 1, 10);
	          continue;
	      }

	      // Good packet. Convert in-place in chosen half of dac_buf.
	      uint16_t num_samples = payload_len / 2;  // 2 bytes/sample
	      if (num_samples > DAC_HALF_SAMPLES) {
	          num_samples = DAC_HALF_SAMPLES;
	      }

	      int16_t  *src = (int16_t *)payload_bytes;        // signed PCM
	      uint16_t *dst = (uint16_t *)payload_bytes;       // same locations for DAC

	      //arm_fir_fast_q15(&fir_low, src, filter_buf_low, DAC_HALF_SAMPLES);
	      //arm_fir_fast_q15(&fir_high, src, filter_buf_high, DAC_HALF_SAMPLES);

	      processFFT(src, filter_buf, num_samples);

	      //Below translates src into the dst. Our goal is to turn src into the output of the FFT
	      for (uint16_t i = 0; i < num_samples; ++i) {
	          int32_t s = filter_buf[i];      // -32768..32767
	          s += 32768;              // 0..65535
	          if (s < 0)      s = 0;
	          if (s > 65535)  s = 65535;
	          dst[i] = (uint16_t)(s >> 4);   // 12-bit right aligned
	      }

	      // Pad rest of half with mid-scale if packet smaller than half
	      for (uint16_t i = num_samples; i < DAC_HALF_SAMPLES; ++i) {
	          dst[i] = 2048;
	      }

	      //dst[i] should be what we're outputting, here is where we'll mess with it


	      // Tell PC this packet was accepted and written into that half
	      uint8_t ack = UART_ACK_OK;
	      HAL_UART_Transmit(&hlpuart1, &ack, 1, 10);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hlpuart1.Init.BaudRate = 2000000;
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
  htim2.Init.Period = 2720;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
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

  /*Configure GPIO pins : PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : REWIND_Pin */
  GPIO_InitStruct.Pin = REWIND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(REWIND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAUSE_Pin FORWARD_Pin */
  GPIO_InitStruct.Pin = PAUSE_Pin|FORWARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	 static uint32_t last = 0;
	uint32_t now = HAL_GetTick();

	if (now - last < 30) return;  // debounce
	last = now;

    if (GPIO_Pin == REWIND_Pin)      // Rewind button
        rewind_flag = 1;

    else if (GPIO_Pin == PAUSE_Pin) // Pause/unpause toggle
        pause_state ^= 1;

    else if (GPIO_Pin == FORWARD_Pin) // Forward button
        forward_flag = 1;
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


		default:
			// Unknown type -> ignore this frame
			uart_start_header_rx();
			return;
		}

		// 5) Set state and start DMA for payload + CRC
		g_uart_rx.state = RX_STATE_WAIT_PAYLOAD;

		if (g_uart_rx.header.data_type == PKT_DATA_IMAGE) {
			// Image: payload + CRC in one shot, as before
			HAL_UART_Receive_DMA(&hlpuart1,
								 g_uart_rx.payload_dst,
								 payload_len + PKT_CRC_SIZE);
		} else {
			// Audio: ONLY the payload goes into dac_buf
			HAL_UART_Receive_DMA(&hlpuart1,
								 g_uart_rx.payload_dst,
								 payload_len);
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
	}
	else if (g_uart_rx.state == RX_STATE_WAIT_CRC) {
		// We just finished receiving the 2 CRC bytes for an audio packet
		if (g_uart_rx.header.data_type == PKT_DATA_AUDIO) {
			audio_pkt_len   = g_uart_rx.payload_len;  // just payload length
			audio_pkt_ready = 1;
		}

		// Now we can go back to waiting for a new header
		uart_start_header_rx();
	}


}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	if (hdac->Instance != DAC1) return;

	// Just finished playing first half [0 .. DAC_HALF_SAMPLES-1],
	// now DMA is reading second half.
	half0_free = 1;

	// Request next audio chunk from PC
	uint8_t req = UART_ACK_NEXT_AUDIO_CHUNK; // 'A'
	HAL_UART_Transmit(&hlpuart1, &req, 1, 10);


}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	if (hdac->Instance != DAC1) return;

	// Just finished second half [DAC_HALF_SAMPLES .. end],
	// now DMA wrapped back to first half.
	half1_free = 1;

	uint8_t req = UART_ACK_NEXT_AUDIO_CHUNK; // 'A'
	HAL_UART_Transmit(&hlpuart1, &req, 1, 10);

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
