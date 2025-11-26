#include "tft.h"

// --- private state ---
static uint16_t tft_width  = TFT_W;
static uint16_t tft_height = TFT_H;

static void tft_hw_reset(void);
static int  in_bounds(int x, int y);

// --- private helpers ---
static void tft_hw_reset(void)
{
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120);
}

static int in_bounds(int x, int y)
{
    return (x >= 0) && (y >= 0) && (x < (int)tft_width) && (y < (int)tft_height);
}

// --- low-level write helpers (internal) ---
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
    HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
    tft_deselect();
}

static inline void tft_write_data8(uint8_t v) { tft_write_data(&v, 1); }

// --- API impl ---
void tft_set_rotation(uint8_t r)
{
    static const uint8_t madctl_tbl[4] = {
        0x48, 0x28, 0x88, 0xE8
    };
    r &= 3;
    tft_write_cmd(0x36);
    tft_write_data8(madctl_tbl[r]);

    if (r & 1) { tft_width = 480; tft_height = 320; }
    else       { tft_width = 320; tft_height = 480; }
}

void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t b[4];
    tft_write_cmd(0x2A);
    b[0]=x0>>8; b[1]=x0&0xFF; b[2]=x1>>8; b[3]=x1&0xFF;
    tft_write_data(b,4);

    tft_write_cmd(0x2B);
    b[0]=y0>>8; b[1]=y0&0xFF; b[2]=y1>>8; b[3]=y1&0xFF;
    tft_write_data(b,4);
}

void tft_init(void)
{
    tft_hw_reset();
    tft_write_cmd(0x11);        // SLPOUT
    HAL_Delay(120);
    tft_write_cmd(0x3A);        // COLMOD
    tft_write_data8(0x55);      // RGB565
    tft_set_rotation(3);
    tft_write_cmd(0x29);        // DISPON
    HAL_Delay(20);
}

void tft_draw_pixel(int x, int y, uint16_t rgb565)
{
    if (!in_bounds(x,y)) return;

    tft_set_addr_window((uint16_t)x,(uint16_t)y,(uint16_t)x,(uint16_t)y);

    uint8_t cmd = 0x2C;
    tft_dc_cmd(); tft_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_dc_data();
    uint8_t two[2] = { (uint8_t)(rgb565>>8), (uint8_t)(rgb565 & 0xFF) };
    HAL_SPI_Transmit(&hspi1, two, 2, HAL_MAX_DELAY);
    tft_deselect();
}

void tft_fill_rect(int x, int y, int w, int h, uint16_t rgb565)
{
    if (w<=0 || h<=0) return;
    if (x<0){ w+=x; x=0; } if (y<0){ h+=y; y=0; }
    if (x+w>tft_width)  w = tft_width  - x;
    if (y+h>tft_height) h = tft_height - y;
    if (w<=0 || h<=0) return;

    tft_set_addr_window((uint16_t)x,(uint16_t)y,(uint16_t)(x+w-1),(uint16_t)(y+h-1));

    tft_dc_cmd(); tft_select();
    uint8_t cmd = 0x2C;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_dc_data();

    uint8_t buf[128];
    uint8_t hb = (uint8_t)(rgb565>>8), lb = (uint8_t)(rgb565 & 0xFF);
    for (size_t i=0;i<sizeof(buf);i+=2){ buf[i]=hb; buf[i+1]=lb; }

    uint32_t px = (uint32_t)w*(uint32_t)h;
    while (px){
        size_t chunk_px = px > (sizeof(buf)/2) ? (sizeof(buf)/2) : px;
        HAL_SPI_Transmit(&hspi1, buf, (uint16_t)(chunk_px*2), HAL_MAX_DELAY);
        px -= chunk_px;
    }
    tft_deselect();
}

void tft_blit565(int x, int y, int w, int h, const uint8_t *img)
{
    if (w<=0 || h<=0) return;

    int x0=x, y0=y, x1=x+w-1, y1=y+h-1;
    if (x0>=tft_width || y0>=tft_height || x1<0 || y1<0) return;

    int sx=0, sy=0;
    if (x0<0){ sx=-x0; x0=0; }
    if (y0<0){ sy=-y0; y0=0; }
    if (x1>=tft_width)  x1=tft_width-1;
    if (y1>=tft_height) y1=tft_height-1;
    int cw = x1-x0+1;
    int ch = y1-y0+1;

    tft_set_addr_window((uint16_t)x0,(uint16_t)y0,(uint16_t)x1,(uint16_t)y1);

    tft_dc_cmd(); tft_select();
    uint8_t cmd = 0x2C;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    tft_dc_data();

    const uint8_t *src = img + ((size_t)sy * w + sx) * 2;
    size_t stride_bytes = (size_t)w * 2;

    for (int row=0; row<ch; ++row) {
        HAL_SPI_Transmit(&hspi1, (uint8_t*)src, (uint16_t)(cw*2), HAL_MAX_DELAY);
        src += stride_bytes;
    }
    tft_deselect();
}

static volatile uint8_t spi_tx_done = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        tft_deselect();
        spi_tx_done = 1;
    }
}

void tft_stream_dma(const uint8_t *data, size_t nbytes)
{
    tft_dc_data();
    tft_select();
    spi_tx_done = 0;
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)data, (uint16_t)nbytes);
    while (!spi_tx_done) { /* wait */ }
}

void TFT_SetBacklightPercent(uint8_t pct){
    if (pct > 100) pct = 100;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t ccr = (pct * (arr + 1)) / 100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
}

void TFT_SetBacklight255(uint8_t val){
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t ccr = (val * (arr + 1)) / 255;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
}
