
#define TFT_CS_GPIO_Port   GPIOC
#define TFT_CS_Pin         GPIO_PIN_8

#define TFT_DC_GPIO_Port   GPIOC
#define TFT_DC_Pin         GPIO_PIN_9

#define TFT_RST_GPIO_Port  GPIOC
#define TFT_RST_Pin        GPIO_PIN_6

// Panel logical size for rotation=0 (we'll track width/height variables)
#define TFT_W  480
#define TFT_H  320

static inline void tft_select(void)   { HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET); }
static inline void tft_deselect(void) { HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);   }

static inline void tft_dc_cmd(void)  { HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET); }
static inline void tft_dc_data(void) { HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);   }

static inline void tft_write_cmd(uint8_t cmd)
{
    tft_dc_cmd();
    tft_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_deselect();
}

static inline void tft_write_data(const uint8_t *buf, size_t n)
{
    tft_dc_data();
    tft_select();
    HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, n, HAL_MAX_DELAY);
    tft_deselect();
}

static inline void tft_write_data8(uint8_t v) { tft_write_data(&v, 1); }

static void tft_hw_reset(void)
		{
		    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
		    HAL_Delay(10);
		    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
		    HAL_Delay(120);
		}
		// Globals that track rotation and current width/height
		static uint16_t tft_width  = TFT_W;
		static uint16_t tft_height = TFT_H;
		static void tft_set_rotation(uint8_t r)
		{
		    // MADCTL bits: MY MX MV ML BGR MH 0 0
		    // We'll use BGR=1 typical; tweak as you like
		    static const uint8_t madctl_tbl[4] = {
		        0x48, // 0: 320x480 (X left->right, Y top->bottom)
		        0x28, // 1: 480x320 (rot 90)
		        0x88, // 2: 320x480 (rot 180)
		        0xE8  // 3: 480x320 (rot 270)
		    };
		    r &= 3;
		    tft_write_cmd(0x36);               // MADCTL
		    tft_write_data8(madctl_tbl[r]);
		    if (r & 1) { tft_width = 480; tft_height = 320; }
		    else       { tft_width = 320; tft_height = 480; }
		}
		void tft_init(void)
		{
		    tft_hw_reset();
		    tft_write_cmd(0x11);               // SLPOUT
		    HAL_Delay(120);
		    tft_write_cmd(0x3A);               // COLMOD (Pixel Format)
		    tft_write_data8(0x55);             // 16-bit (RGB565)
		    tft_set_rotation(3);               // choose 0..3
		    tft_write_cmd(0x29);               // DISPON
		    HAL_Delay(20);
		}

        static inline void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t b[4];

    tft_write_cmd(0x2A);   // CASET
    b[0] = x0 >> 8; b[1] = x0 & 0xFF; b[2] = x1 >> 8; b[3] = x1 & 0xFF;
    tft_write_data(b, 4);

    tft_write_cmd(0x2B);   // PASET
    b[0] = y0 >> 8; b[1] = y0 & 0xFF; b[2] = y1 >> 8; b[3] = y1 & 0xFF;
    tft_write_data(b, 4);
}

static inline uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


static inline int in_bounds(int x, int y)
{
    return (x >= 0) && (y >= 0) && (x < (int)tft_width) && (y < (int)tft_height);
}

void tft_draw_pixel(int x, int y, uint16_t rgb565)
{
    if (!in_bounds(x, y)) return;

    tft_set_addr_window((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)y);

    uint8_t cmd = 0x2C;                 // RAMWR
    tft_dc_cmd(); tft_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);

    tft_dc_data();
    uint8_t two[2] = { rgb565 >> 8, rgb565 & 0xFF };
    HAL_SPI_Transmit(&hspi1, two, 2, HAL_MAX_DELAY);
    tft_deselect();
}
void tft_fill_rect(int x, int y, int w, int h, uint16_t rgb565)
{
    if (w <= 0 || h <= 0) return;
    // Clip to bounds
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > tft_width)  w = tft_width  - x;
    if (y + h > tft_height) h = tft_height - y;
    if (w <= 0 || h <= 0) return;

    tft_set_addr_window((uint16_t)x, (uint16_t)y, (uint16_t)(x+w-1), (uint16_t)(y+h-1));

    // Issue RAMWR once, then stream all pixels
    tft_dc_cmd(); tft_select();
    uint8_t cmd = 0x2C;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_dc_data();

    // Build a small line buffer with the color repeated
    uint8_t buf[128];
    uint8_t hb = rgb565 >> 8, lb = rgb565 & 0xFF;
    for (size_t i = 0; i < sizeof(buf); i += 2) { buf[i] = hb; buf[i+1] = lb; }

    uint32_t px = (uint32_t)w * (uint32_t)h; // total pixels
    while (px) {
        size_t chunk_px = px > (sizeof(buf)/2) ? (sizeof(buf)/2) : px;
        HAL_SPI_Transmit(&hspi1, buf, chunk_px*2, HAL_MAX_DELAY);
        px -= chunk_px;
    }
    tft_deselect();
}

// img points to w*h*2 bytes in RGB565 (big-endian per pixel)
// If your buffer is little-endian words in RAM, byte-swap into a temp line buffer before sending.
void tft_blit565(int x, int y, int w, int h, const uint8_t *img)
{
    if (w <= 0 || h <= 0) return;
    // clip (left/top)
    int x0 = x, y0 = y;
    int x1 = x + w - 1, y1 = y + h - 1;
    if (x0 >= (int)tft_width || y0 >= (int)tft_height || x1 < 0 || y1 < 0) return;
    int sx = 0, sy = 0;
    if (x0 < 0) { sx = -x0; x0 = 0; }
    if (y0 < 0) { sy = -y0; y0 = 0; }
    if (x1 >= tft_width)  x1 = tft_width  - 1;
    if (y1 >= tft_height) y1 = tft_height - 1;
    int cw = x1 - x0 + 1;
    int ch = y1 - y0 + 1;

    tft_set_addr_window(x0, y0, x1, y1);
    tft_dc_cmd(); tft_select();
    uint8_t cmd = 0x2C; // RAMWR
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_dc_data();

    // Send line by line (handles left/top clipping via src offset)
    const uint8_t *src = img + ( (size_t)sy * w + sx ) * 2;
    size_t stride_bytes = (size_t)w * 2;

    for (int row = 0; row < ch; ++row) {
        // contiguous run of cw pixels starting at src + sx*2
        HAL_SPI_Transmit(&hspi1, (uint8_t*)src, (uint16_t)(cw*2), HAL_MAX_DELAY);
        src += stride_bytes;
    }
    tft_deselect();
}
// Global semaphore/flag if you want to block until done
static volatile uint8_t spi_tx_done = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        tft_deselect();        // finish the CS frame here
        spi_tx_done = 1;
    }
}

// Stream 'nbytes' already-prepared data (e.g., a line buffer) via DMA
void tft_stream_dma(const uint8_t *data, size_t nbytes)
{
    tft_dc_data();
    tft_select();
    spi_tx_done = 0;
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)data, (uint16_t)nbytes);
    // Option A: spin-wait (simple)
    while (!spi_tx_done) { /* optionally low-power wait */ }
    // Option B: return immediately and chain next steps in the callback
}

// Works with PSC=79, ARR=49 example above
void TFT_SetBacklightPercent(uint8_t pct){
    if (pct > 100) pct = 100;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);   // e.g., 49
    uint32_t ccr = (pct * (arr + 1)) / 100;            // scale 0..arr+1
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
}

void TFT_SetBacklight255(uint8_t val){
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t ccr = (val * (arr + 1)) / 255;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
}
