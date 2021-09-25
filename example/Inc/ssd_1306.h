/* Define to prevent recursive inclusion */
#ifndef __SSD_1306_H
#define __SSD_1306_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <stdbool.h>
#include "ssd_1306_font.h"
#include "stm32f4xx_hal.h"

#define SSD1306_WIDTH        128
#define SSD1306_HEIGHT       64
#define SSD1306_BUFFER_SZ    (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

/* Values for the user */
#define SSD1306_CONTRAST_DEFAULT_NOVCC      0xCF    /* Default contrast value - After reset */
#define SSD1306_CONTRAST_DEFAULT_VCC        0x9F    /* Default contrast value - After reset */
#define SSD1306_VCOMDETECT_DEFAULT          0x20    /* [Set VCOMH Deselect Level] default - medium level */
#define SSD1306_VCOMDETECT_LOW              0x00    /* [Set VCOMH Deselect Level] at lowest */
#define SSD1306_VCOMDETECT_HIGH             0x30    /* [Set VCOMH Deselect Level] at the highest */
#define SSD1306_EXTERNALVCC                 0x01    /* External display voltage source */
#define SSD1306_SWITCHCAPVCC                0x02    /* Generate display voltage from 3.3V pin */

/* Extra options */
#define SSD1306_DEBUG               /* Activate screen debug mode - Thorough printing in the terminal */
#define SSD1306_DMA_ACTIVE          /* Enable SPI transmissions via DMA */
#define SSD1306_TIMEOUT     10      /* Timeout for polling SPI - 10ms is enough */

/* Structure used for the GPIO definitions */
typedef struct ssd_1306_base_struct
{
    /* SPI handle and draw buffer */
    SPI_HandleTypeDef *h_spi;
    uint8_t *buffer;

    /* Port and pin pairs for the GPIOs */
    uint32_t rst_pin, ce_pin, dc_pin;
    GPIO_TypeDef *rst_port, *ce_port, *dc_port;

    /* Contrast/Bias */
    uint8_t contast, vcs;

    /* Extras - Cursor position */
    uint8_t x_pos, y_pos;

#ifdef SSD1306_DMA_ACTIVE
    /* Flag for DMA transfer status - User must not write this field during operation !! */
    volatile bool dma_transfer;

    /* We need to have a constant buffer for DMA transfers (commands at least) */
    uint8_t command_buffer[10];
#endif
}ssd_1306_t;

/* Initializers */
bool SSD1306_init(ssd_1306_t *init);
ssd_1306_t *SSD1306_handle_swap(ssd_1306_t *new);

/* Utilities */
void SSD1306_fill(bool black);
bool SSD1306_sleep_mode(bool sleep);
bool SSD1306_refresh(void);
bool SSD1306_invert(bool invert);
bool SSD1306_contrast(uint8_t contrast);
bool SSD1306_vcomh(uint8_t vcomh);
bool SSD1306_timings(uint8_t freq, uint8_t div_ratio);
bool SSD1306_precharge(uint8_t period);

/* Scrolling */
bool SSD1306_hscroll(uint8_t timing, bool dir);
bool SSD1306_hvscroll(uint8_t hspeed, uint8_t vspeed, bool dir);
bool SSD1306_scroll_disable(void);

/* Lines and pixels */
void SSD1306_set_pixel(uint8_t x, uint8_t y, bool color);
uint8_t SSD1306_get_pixel(uint8_t x, uint8_t y);
void SSD1306_draw_line(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1, bool color);
void SSD1306_draw_hline(uint8_t x, uint8_t y, uint8_t len, bool color);
void SSD1306_draw_vline(uint8_t x, uint8_t y, uint8_t len, bool color);

/* Shape drawing */
void SSD1306_draw_rectangle(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1, bool color, bool fill);
void SSD1306_draw_triangle(uint8_t x0, uint8_t x1, uint8_t x2, uint8_t y0, uint8_t y1, uint8_t y2, bool color);
void SSD1306_draw_fill_triangle(uint8_t x0, uint8_t x1, uint8_t x2, uint8_t y0, uint8_t y1, uint8_t y2, bool color);
void SSD1306_draw_circle(uint8_t x, uint8_t y, uint8_t r, bool color);
void SSD1306_draw_fill_circle(uint8_t x0, uint8_t y0, uint8_t r, bool color);
void SSD1306_draw_round_rect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, bool color, bool fill);

/* Bitmaps */
void SSD1306_draw_bitmap(const uint8_t *bitmap, uint8_t x0, uint8_t y0, uint8_t len_x, uint8_t len_y, uint8_t scale);
void SSD1306_draw_bitmap_opt8(const uint8_t *bitmap, uint8_t x0, uint8_t y0, uint8_t len_x, uint8_t len_y);

/* Text */
void SSD1306_coord(uint8_t x, uint8_t p);
void SSD1306_print_str(const char *str, uint8_t option, bool invert);
void SSD1306_print_fstr(const char *str, uint8_t option, uint8_t x, uint8_t y, uint8_t scale, bool invert);

#ifdef __cplusplus
}
#endif

#endif /* __SSD_1306_H */
