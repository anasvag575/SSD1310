#include <ssd_1306.h> /* External header */
#include <string.h> /* For memcpy */
#include <stdio.h>  /* TODO - For debug printf */
#include <stdlib.h>  /* TODO - For debug printf */

/* Screen size and parameters */
#define LCDWIDTH            SSD1306_WIDTH
#define LCDHEIGHT           SSD1306_HEIGHT
#define LCDBUFFER_SZ        SSD1306_BUFFER_SZ
#define LCDBANK_SZ          8

/***** Fundamental commands *****/
#define SSD1306_SETCONTRAST         0x81 /* [Set Contrast Control] - 2byte command */
#define SSD1306_DISPLAYALLON_RESUME 0xA4 /* [Entire Display ON] - 1byte command */
#define SSD1306_DISPLAYALLON        0xA5 /* [Entire Display ON] - 1byte command */
#define SSD1306_NORMALDISPLAY       0xA6 /* [Set Normal/Inverse Display] - 1byte command */
#define SSD1306_INVERTDISPLAY       0xA7 /* [Set Normal/Inverse Display] - 1byte command */
#define SSD1306_DISPLAYON           0xAF /* [Set Display ON/OFF] - 1byte command */
#define SSD1306_DISPLAYOFF          0xAE /* [Set Display ON/OFF] - 1byte command */

/***** Scrolling commands *****/
#define SSD1306_RIGHT_HORIZONTAL_SCROLL                 0x26 /* [Continuous Horizontal Scroll Setup] - 7byte command */
#define SSD1306_LEFT_HORIZONTAL_SCROLL                  0x27 /* [Continuous Horizontal Scroll Setup] - 7byte command */
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL    0x29 /* [Continuous Vertical and Horizontal Scroll Setup] - 6byte command */
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL     0x2A /* [Continuous Vertical and Horizontal Scroll Setup] - 6byte command */
#define SSD1306_DEACTIVATE_SCROLL                       0x2E /* [Activate scroll] - 1byte command */
#define SSD1306_ACTIVATE_SCROLL                         0x2F /* [Deactivate scroll] - 1byte command */
#define SSD1306_SET_VERTICAL_SCROLL_AREA                0xA3 /* [Set Vertical Scroll Area] - 3byte command */

/***** Addresing setting commands *****/
#define SSD1306_MEMORYMODE      0x20            /* [Set Memory Addressing Mode] - 2byte command */
#define SSD1306_COLUMNADDR      0x21            /* [Set Column Address] - 3byte command */
#define SSD1306_PAGEADDR        0x22            /* [Set Page Address] - 3byte command */

/* Page addressing mode stuff */
#define SSD1306_SETLOWCOLUMN    0x00    /* [Set Lower Column Start Address for Page Addressing Mode] - 1byte command */
#define SSD1306_SETHIGHCOLUMN   0x10    /* [Set Higher Column Start Address for Page Addressing Mode] - 1byte command */
#define SSD1306_SETPAGESTART    0xB0    /* [Set Page Start Address for Page Addressing Mode] - 1byte command */

/***** Hardware Configuration commands *****/
#define SSD1306_SETSTARTLINE        0x40       /* [Set Display Start Line] - 1byte command */
#define SSD1306_SEGREMAP            0xA0       /* [Set Segment Re-map] - 1byte command */
#define SSD1306_SETMULTIPLEX        0xA8       /* [Set Multiplex Ratio] - 2byte command */
#define SSD1306_COMSCANINC          0xC0       /* [Set COM Output Scan Direction] - 1byte command */
#define SSD1306_COMSCANDEC          0xC8       /* [Set COM Output Scan Direction] - 1byte command */
#define SSD1306_SETDISPLAYOFFSET    0xD3       /* [Set Display Offset] - 2byte command */
#define SSD1306_SETCOMPINS          0xDA       /* [Set COM Pins Hardware Configuration] - 2byte command */

/***** Timing & Driving Scheme Setting commands *****/
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5       /* [Set Display Clock Divide Ratio/Oscillator Frequency] - 2byte command */
#define SSD1306_SETPRECHARGE        0xD9       /* [Set Pre-charge Period] - 2byte command */
#define SSD1306_SETVCOMDETECT       0xDB       /* [Set VCOMH Deselect Level] - 2byte command */
#define SSD1306_NOP                 0X72       /* [No operation] - 1byte command */

/***** Charge Bump Setting commands *****/
#define SSD1306_SETCHARGEPUMP 0x8D             /* [Charge Pump Setting] - 2byte command */

/************* DEFAULT VALUES *************/
#define SSD1306_CHARGEPUMP_OFF              0x10    /* [Charge Pump Setting] option */
#define SSD1306_CHARGEPUMP_ON               0x14    /* [Charge Pump Setting] option */
#define SSD1306_PRECHARGE_DEFAULT_VCC       0x22    /* [Set Pre-charge Period] default1 */
#define SSD1306_PRECHARGE_DEFAULT_NOVCC     0xF1    /* [Set Pre-charge Period] default2 */
#define SSD1306_MEMORYMODE_HORIZONTAL       0x00    /* [Set Memory Addressing Mode] option */
#define SSD1306_DISPLAYCLOCKDIV_DEFAULT     0x80    /* Default ratio - suggested value */
#define SSD1306_COMPINS_DEFAULT             0x12    /* Default for 128x64 */

/* Macros to set and reset pins */
#define SET_GPIO(port, pin)     (HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET))
#define RESET_GPIO(port, pin)   (HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET))

/* TODO - Remove these when done */
#ifdef SSD1306_DEBUG
    #define ASSERT_DEBUG(cond, ...) do{ if((cond)) printf(__VA_ARGS__);}while(0)
#else
    #define ASSERT_DEBUG(cond, ...)
#endif

/* Swap macro for variables */
#define SWAP_VAR(a, b)                  \
    do                                       \
    {                                       \
        typeof((a)) __loc_var__ = (a);    \
        (a) = (b);                          \
        (b) = __loc_var__;                        \
    }while(0);

/* Assisting MACROs in common transformations and manipulations */
#define MSB2LSB_MASK(num)               (~(0xff >> (num)))
#define LSB2MSB_MASK(num)               ((1 << (num)) - 1)
#define COORDS2BUFF_POS(x, y)           ((((uint16_t)(y))>>3) * LCDWIDTH + (x))
#define COORDS2BIT_POS(x, y, width)     ((((uint16_t)(y))>>3) * (width) + (x))

/* Handle to be used for the screen */
static ssd_1306_t *_screen_h = NULL;

#ifdef SSD1306_DMA_ACTIVE
    /*!
        @brief    The internal ISR callback when a DMA transfer is complete.
        This unfortunately might be need to be defined somewhere else, in case
        multiple SPIs with DMA are used. For now it is left here as an example.
        @param    hspi      SPI handle, given by the external ISR
    */
    void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
    {
        /* Chip deselect - Active low */
        if(hspi->Instance == _screen_h->h_spi->Instance)
        {
            _screen_h->dma_transfer = false;
            SET_GPIO(_screen_h->ce_port, _screen_h->ce_pin);
        }
    }
#endif

/**********************************************************/
/************************ OPERATIONS **********************/
/**********************************************************/

/*!
    @brief    SPI transmission internal routine.
    @param    data      The SPI packet buffer to be sent
    @param    nb_data   The number of packets(bytes) to be sent
    @param    type      Type of transmission, true for data else command.
    @return             Success(True) or Failure(False) of the SPI transmission.
*/
static bool _send_packet(uint8_t *data, uint16_t nb_data , bool type)
{
    HAL_StatusTypeDef ret;

    /* Data needs DC high - Command needs DC low */
    type ? SET_GPIO(_screen_h->dc_port, _screen_h->dc_pin) \
         : RESET_GPIO(_screen_h->dc_port, _screen_h->dc_pin);

    /* Chip enable - Active Low */
    RESET_GPIO(_screen_h->ce_port, _screen_h->ce_pin);

    #ifdef SSD1306_DMA_ACTIVE
        /* Set handler flag */
        _screen_h->dma_transfer = true;

        /* Transmit through SPI using DMA */
        ret = HAL_SPI_Transmit_DMA(_screen_h->h_spi, data, nb_data);
    #else
        /* Transmit through SPI */
        ret = HAL_SPI_Transmit(_screen_h->h_spi, data, nb_data, SSD1306_TIMEOUT);

        /* Chip disable - Active Low */
        SET_GPIO(_screen_h->ce_port, _screen_h->ce_pin);
    #endif

    return ret == HAL_OK;
}

/*!
    @brief    Initializes the display and the library with a new handle.
    @return   Success(True) or Failure(False) of the procedure.
*/
bool SSD1306_init(ssd_1306_t *init)
{
    /* Initialize the screen handle */
    _screen_h = init;

    /* 1) Chip enable initialization - Active low */
    SET_GPIO(_screen_h->ce_port, _screen_h->ce_pin);

    /* 2) We reset for 10ms - Active low */
    RESET_GPIO(_screen_h->rst_port, _screen_h->rst_pin);
    HAL_Delay(10);
    SET_GPIO(_screen_h->rst_port, _screen_h->rst_pin);
    HAL_Delay(10);

    /* 3) Initialize handle fields and check inputs */
    bool vcs_flag = _screen_h->vcs == SSD1306_EXTERNALVCC;
    _screen_h->x_pos = _screen_h->y_pos = 0;

    /* 4a) Send base commands to set the screen up */
#ifdef SSD1306_DMA_ACTIVE
    _screen_h->dma_transfer = false;
    uint8_t *payload = _screen_h->command_buffer;
#else
    uint8_t payload[10];
#endif

    payload[0] = SSD1306_DISPLAYOFF;                         /* Display off to apply settings */
    payload[1] = SSD1306_SETDISPLAYCLOCKDIV;                 /* Set clockdiv ratio - Command code */
    payload[2] = SSD1306_DISPLAYCLOCKDIV_DEFAULT;            /* Set clockdiv ratio - Value */
    payload[3] = SSD1306_SETMULTIPLEX;                       /* Set multiplexing - Command code */
    payload[4] = LCDHEIGHT - 1;                              /* Set multiplexing - This is also the default after reset */
    payload[5] = SSD1306_SETDISPLAYOFFSET;                   /* Set display offset - Command code */
    payload[6] = SSD1306_MEMORYMODE_HORIZONTAL;              /* Set display offset - Value (0 offset) */
    payload[7] = SSD1306_SETSTARTLINE | 0x00;                /* Set starting line - 0 default */
    payload[8] = SSD1306_NORMALDISPLAY;                      /* Uninverted */
    payload[9] = SSD1306_DEACTIVATE_SCROLL;                  /* Deactivate scroll */

    /* Quick return in case of failure */
    if(!_send_packet(payload, 10, false)) return false;

    #ifdef SSD1306_DMA_ACTIVE
        while(_screen_h->dma_transfer); // Wait for DMA to finish //
    #endif

    /* 4a) Second round of commands */
    payload[0] = SSD1306_MEMORYMODE;                        /* Set memory mode - Command code */
    payload[1] = 0x00;                                      /* Set memory mode - Value (Vertical mode) */
    payload[2] = SSD1306_SEGREMAP | 0x01;                   /* Set Segment Re-map - Map col addr to 127 */
    payload[3] = SSD1306_COMSCANDEC;                        /* Set COM Output Scan Direction - From [N-1] to [0] */
    payload[4] = SSD1306_SETCOMPINS;                        /* Set COM Pins Hardware Configuration - Command code */
    payload[5] = SSD1306_COMPINS_DEFAULT;                   /* Set COM Pins Hardware Configuration - Value */
    payload[6] = SSD1306_SETCONTRAST;                       /* Set contrast - Command code */
    payload[7] = _screen_h->contast;                        /* Set contrast - Value */
    payload[8] = SSD1306_SETPRECHARGE;                      /* Set precharge - Command code */
    payload[9] = vcs_flag ? SSD1306_PRECHARGE_DEFAULT_VCC:  /* Set precharge - Value */
                            SSD1306_PRECHARGE_DEFAULT_NOVCC;

    /* Quick return in case of failure */
    if(!_send_packet(payload, 10, false)) return false;

    #ifdef SSD1306_DMA_ACTIVE
        while(_screen_h->dma_transfer); // Wait for DMA to finish //
    #endif

    /* 4a) Third and final round of commands */
    payload[0] = SSD1306_SETVCOMDETECT;                     /* Set VCOMH Deselect Level - Command code */
    payload[1] = SSD1306_VCOMDETECT_DEFAULT;                /* Set VCOMH Deselect Level - Value */
    payload[2] = SSD1306_DISPLAYALLON_RESUME;               /* Set display all on for a sec */
    payload[3] = SSD1306_SETCHARGEPUMP,                     /* Set charge pump - Command code */
    payload[4] = vcs_flag ? SSD1306_CHARGEPUMP_OFF:         /* On */
                            SSD1306_CHARGEPUMP_ON;
    payload[5] = SSD1306_DISPLAYON;                         /* Finally set display on */

    return _send_packet(payload, 6, false);
}

/*!
    @brief    Swaps the current screen handle.
    This is used to change the current screen that the library sends commands and updates
    graphics into. Initialization for the new screen is the user's responsibility.
    @param    new  The new screen handle
    @return        The old display's handle
*/
ssd_1306_t *PCD8544_handle_swap(ssd_1306_t *new)
{
    ASSERT_DEBUG(new != NULL, "Null pointer - PCD8544_handle_swap()\n");

    ssd_1306_t *old = _screen_h;
    _screen_h = new;

    return old;
}

/*!
    @brief    Draws the contents of the buffer on the display.
    @return   Success(True) or Failure(False) in sending the data.
*/
bool SSD1306_refresh(void)
{
    #ifdef SSD1306_DMA_ACTIVE
        /* Check for active transmissions */
        if(_screen_h->dma_transfer) return false;
    #endif

    /* Draw and return */
    return _send_packet(_screen_h->buffer, LCDBUFFER_SZ, true);
}

/*!
    @brief    Fills the display buffer with the specified color.
    @param    color  Fill with black(true) or with white(false).
*/
void SSD1306_fill(bool black)
{
    /* Fill the buffer with it */
    memset(_screen_h->buffer, black ? 0xff : 0, LCDBUFFER_SZ * sizeof(*_screen_h->buffer));
}

/*!
    @brief    Inverts or uninverts the display.
    @param    invert  True(Invert) and False(Uninvert).
    @return           Success(True) or Failure(False) in sending the command.
*/
bool SSD1306_invert(bool invert)
{
#ifdef SSD1306_DMA_ACTIVE
    /* Check for active transmissions */
    if(_screen_h->dma_transfer) return false;

    /* Command code */
    _screen_h->command_buffer[0] = invert ? SSD1306_INVERTDISPLAY: SSD1306_NORMALDISPLAY;

    return _send_packet(_screen_h->command_buffer, 1, false);
#else
    /* Allocate the buffer on the stack */
    uint8_t rx_data = invert ? SSD1306_INVERTDISPLAY: SSD1306_NORMALDISPLAY;

    return _send_packet(&rx_data, 1, false);
#endif
}

/*!
    @brief    Enables or disables sleep mode.
    @param    enable  Enable(true) sleep mode or disable(false).
    @return           Success(True) or Failure(False) in sending the command.
*/
bool SSD1306_sleep_mode(bool sleep)
{
#ifdef SSD1306_DMA_ACTIVE
    /* Check for active transmissions */
    if(_screen_h->dma_transfer) return false;

    /* Command code */
    _screen_h->command_buffer[0] = sleep ? SSD1306_DISPLAYOFF: SSD1306_DISPLAYON;

    return _send_packet(_screen_h->command_buffer, 1, false);
#else
    /* Allocate the buffer on the stack */
    uint8_t rx_data = sleep ? SSD1306_DISPLAYOFF: SSD1306_DISPLAYON;

    return _send_packet(&rx_data, 1, false);
#endif
}

/*!
    @brief    Set display's contrast value.
    @param    contrast  The contrast value.
    @return             Success(True) or Failure(False) in sending the command.
*/
bool SSD1306_contrast(uint8_t contrast)
{
#ifdef SSD1306_DMA_ACTIVE
    /* Check for active transmissions */
    if(_screen_h->dma_transfer) return false;

    /* Command */
    _screen_h->command_buffer[0] = SSD1306_SETCONTRAST;
    _screen_h->command_buffer[1] = contrast;

    return _send_packet(_screen_h->command_buffer, 2, false);
#else
    /* Allocate the buffer on the stack */
    uint8_t rx_data[2] = {SSD1306_SETCONTRAST, contrast};

    return _send_packet(rx_data, 2, false);
#endif
}

/*!
    @brief    Set vcomh value (dim or brighten the screen).
    @param    vcomh     The vcomh value.
    @return             Success(True) or Failure(False) in sending the command.
*/
bool SSD1306_vcomh(uint8_t vcomh)
{
    /* Check that it a valid input */
    switch(vcomh)
    {
        case SSD1306_VCOMDETECT_DEFAULT:
        case SSD1306_VCOMDETECT_LOW:
        case SSD1306_VCOMDETECT_HIGH: break;
        default: return false;
    }

#ifdef SSD1306_DMA_ACTIVE
    /* Check for active transmissions */
    if(_screen_h->dma_transfer) return false;

    /* Command */
    _screen_h->command_buffer[0] = SSD1306_SETVCOMDETECT;
    _screen_h->command_buffer[1] = vcomh;

    return _send_packet(_screen_h->command_buffer, 2, false);
#else
    /* Allocate the buffer on the stack */
    uint8_t rx_data[2] = {SSD1306_SETVCOMDETECT, vcomh};

    return _send_packet(rx_data, 2, false);
#endif
}

/*!
    @brief    Activate horizontal scrolling.
    @param    speed     The speed of the scrolling, can be from 0 to 7, with
    higher values meaning more speed.
    @param    dir       The direction, right (true) or left (false)
    @return             Success(True) or Failure(False) in sending the command.
*/
bool SSD1306_hscroll(uint8_t speed, bool dir)
{
    /* Speeds in frames: 2, 3, 4, 5, 25, 64, 128, 256 */
    const uint8_t timing_table[8] = {0x07, 0x04, 0x05, 0x00, 0x06, 0x01, 0x02, 0x03};

#ifdef SSD1306_DMA_ACTIVE
    /* Check for active transmissions */
    if(_screen_h->dma_transfer) return false;

    /* Max speed supported */
    if(speed > 7) speed = 7;

    /* Commands */
    _screen_h->command_buffer[0] = dir ? SSD1306_RIGHT_HORIZONTAL_SCROLL:
                                         SSD1306_LEFT_HORIZONTAL_SCROLL;
    _screen_h->command_buffer[1] = 0x00;                    /* Dummy */
    _screen_h->command_buffer[2] = 0x00;                    /* Start page address */
    _screen_h->command_buffer[3] = timing_table[speed];     /* Scroll speed */
    _screen_h->command_buffer[4] = 0xFF;                    /* End page address */
    _screen_h->command_buffer[5] = 0x00;                    /* Dummy */
    _screen_h->command_buffer[6] = 0xFF;                    /* Dummy */
    _screen_h->command_buffer[7] = SSD1306_ACTIVATE_SCROLL;

    return _send_packet(_screen_h->command_buffer, 8, false);
#else
    /* Allocate the buffer on the stack */
    uint8_t rx_data[8];

    /* Max speed supported */
    if(speed > 7) speed = 7;

    /* Commands */
    rx_data[0] = dir ? SSD1306_RIGHT_HORIZONTAL_SCROLL:
                       SSD1306_LEFT_HORIZONTAL_SCROLL;
    rx_data[1] = 0x00;                      /* Dummy */
    rx_data[2] = 0x00;                      /* Start page address */
    rx_data[3] = timing_table[speed];       /* Scroll speed */
    rx_data[4] = 0xFF;                      /* End page address */
    rx_data[5] = 0x00;                      /* Dummy */
    rx_data[6] = 0xFF;                      /* Dummy */
    rx_data[7] = SSD1306_ACTIVATE_SCROLL;

    return _send_packet(rx_data, 8, false);
#endif
}

/*!
    @brief    Activate horizontal and vertical scrolling.
    @param    hspeed     The speed of horizontal scrolling, can be from 0 to 7, with
    higher values meaning more speed.
    @param    vspeed    The vertical speed (or scrolling offset), can be from 0 to 0x3f
    @param    dir       The direction, right (true) or left (false)
    @return             Success(True) or Failure(False) in sending the command.
*/
bool SSD1306_hvscroll(uint8_t hspeed, uint8_t vspeed, bool dir)
{
    /* Speeds in frames: 2, 3, 4, 5, 25, 64, 128, 256 */
    const uint8_t timing_table[8] = {0x07, 0x04, 0x05, 0x00, 0x06, 0x01, 0x02, 0x03};

#ifdef SSD1306_DMA_ACTIVE
    /* Check for active transmissions */
    if(_screen_h->dma_transfer) return false;

    /* Max speed supported */
    if(hspeed > 7) hspeed = 7;

    /* Commands */
    _screen_h->command_buffer[0] = dir ? SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL:
                                         SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL;
    _screen_h->command_buffer[1] = 0x00;                    /* Dummy */
    _screen_h->command_buffer[2] = 0x00;                    /* Start page address */
    _screen_h->command_buffer[3] = timing_table[hspeed];    /* Scroll speed */
    _screen_h->command_buffer[4] = 0xFF;                    /* End page address */
    _screen_h->command_buffer[5] = vspeed;                  /* Vertical scrolling offset */
    _screen_h->command_buffer[6] = SSD1306_ACTIVATE_SCROLL;

    return _send_packet(_screen_h->command_buffer, 7, false);
#else
    /* Allocate the buffer on the stack */
    uint8_t rx_data[8];

    /* Max speed supported */
    if(hspeed > 7) hspeed = 7;

    /* Commands */
    rx_data[0] = dir ? SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL:
                       SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL;
    rx_data[1] = 0x00;                      /* Dummy */
    rx_data[2] = 0x00;                      /* Start page address */
    rx_data[3] = timing_table[hspeed];       /* Scroll speed */
    rx_data[4] = 0xFF;                      /* End page address */
    rx_data[5] = vspeed;                    /* Vertical scrolling offset */
    rx_data[6] = SSD1306_ACTIVATE_SCROLL;

    return _send_packet(rx_data, 7, false);
#endif
}

/*!
    @brief    Deactivate any scrolling currently on the screen.
    @return   Success(True) or Failure(False) in sending the command.
*/
bool SSD1306_scroll_disable(void)
{
#ifdef SSD1306_DMA_ACTIVE
    /* Check for active transmissions */
    if(_screen_h->dma_transfer) return false;

    /* Command */
    _screen_h->command_buffer[0] = SSD1306_DEACTIVATE_SCROLL;

    return _send_packet(_screen_h->command_buffer, 1, false);
#else
    /* Allocate the buffer on the stack */
    uint8_t rx_data = SSD1306_DEACTIVATE_SCROLL;

    return _send_packet(&rx_data, 1, false);
#endif
}

/*!
    @brief    Set display's oscillator frequency and display clock divide ratio.
    @param    freq       The oscillator frequency value (values of 0 to 15).
    @param    div_ratio  The clock division ration (values of 0 to 15).
    @return              Success(True) or Failure(False) in sending the command.
*/
bool SSD1306_timings(uint8_t freq, uint8_t div_ratio)
{
#ifdef SSD1306_DMA_ACTIVE
    /* Check for active transmissions */
    if(_screen_h->dma_transfer) return false;

    /* Clip in case the values are larger */
    if(freq > 15) freq = 15;
    if(div_ratio > 15) div_ratio = 15;

    /* Command */
    _screen_h->command_buffer[0] = SSD1306_SETDISPLAYCLOCKDIV;
    _screen_h->command_buffer[1] = (freq << 4) | div_ratio;

    return _send_packet(_screen_h->command_buffer, 2, false);
#else
    /* Clip in case the values are larger */
    if(freq > 15) freq = 15;
    if(div_ratio > 15) div_ratio = 15;

    /* Allocate the buffer on the stack */
    uint8_t rx_data[2] = {SSD1306_SETDISPLAYCLOCKDIV, (freq << 4) | div_ratio};

    return _send_packet(rx_data, 2, false);
#endif
}

/*!
    @brief    Set display's precharge period value.
    @param    period     The period value (values of 1 to 15).
    @return              Success(True) or Failure(False) in sending the command.
*/
bool SSD1306_precharge(uint8_t period)
{
#ifdef SSD1306_DMA_ACTIVE
    /* Check for active transmissions */
    if(_screen_h->dma_transfer) return false;

    /* In case input is 0 - Invalid */
    period += !period;

    /* Also check that is not above 15DCLK */
    if(period > 15) period = 15;

    /* Command */
    _screen_h->command_buffer[0] = SSD1306_SETPRECHARGE;
    _screen_h->command_buffer[1] = period;

    return _send_packet(_screen_h->command_buffer, 2, false);
#else
    /* In case input is 0 - Invalid */
    period += !period;

    /* Also check that is not above 15DCLK */
    if(period > 15) period = 15;

    /* Allocate the buffer on the stack */
    uint8_t rx_data[2] = {SSD1306_SETPRECHARGE, period};

    return _send_packet(rx_data, 2, false);
#endif
}

/**********************************************************/
/************************ GRAPHICS ************************/
/**********************************************************/

/*!
    @brief    Set a pixel's value. Internal routine, no error checking performed.
    @param    x         x-coordinate
    @param    y         y-coordinate
    @param    color     Black(True) or White(False).
*/
static void _set_single_pixel(uint8_t x, uint8_t y, bool color)
{
    uint16_t pos = COORDS2BUFF_POS(x, y);
    ASSERT_DEBUG(pos >= LCDBUFFER_SZ, "Error at _set_single_pixel %d\n", pos);

    if(color)
        _screen_h->buffer[pos] |= 1 << (y & 0x07);
    else
        _screen_h->buffer[pos] &= ~(1 << (y & 0x07));
}

/*!
    @brief    Set a pixel's value. Internal routine used for loops, no error checking performed.
    @param    pos       Position in the buffer
    @param    mask      The mask to apply
    @param    color     Black(True) or White(False).
*/
static void _set_single_pixel_opt(uint16_t pos, uint8_t mask, bool color)
{
    ASSERT_DEBUG(pos >= LCDBUFFER_SZ, "Error at _set_single_pixel_opt %d\n", pos);

    if(color)
        _screen_h->buffer[pos] |= mask;
    else
        _screen_h->buffer[pos] &= ~mask;
}

/*!
    @brief    Get a pixel's value. Internal routine, no error checking performed.
    @param    x     x-coordinate
    @param    y     y-coordinate
    @return         Black(True) or White(False).
*/
static uint8_t _get_single_pixel(uint8_t x, uint8_t y)
{
    /* First find the exact position */
    uint16_t pos = COORDS2BUFF_POS(x, y);
    ASSERT_DEBUG(pos >= LCDBUFFER_SZ, "Error at _get_single_pixel %d\n", pos);

    return (_screen_h->buffer[pos] >> (y & 0x07)) & 0x01;
}

/*!
    @brief    Set multiple pixel values. Internal routine, no error checking performed.
    The routine sets pixels of a single bank(check documentation for the display's pixel layout).
    The color goes from the LSB to MSB in the buffer, or from the bottom to the top in the actual
    display.
    @param    pos     The bank position in the buffer
    @param    num     Number of pixels to color, must be less than 8(bank size).
    @param    color   Either set pixels to black(true) or white(false).
*/
static void _set_pixels_lsb2msb(uint16_t pos, uint8_t num, bool color)
{
    ASSERT_DEBUG(pos >= LCDBUFFER_SZ, "Error at _set_pixels_lsb2msb\n");
    ASSERT_DEBUG(num >= 8, "Error at _set_pixels_lsb2msb\n");
    uint8_t mask = LSB2MSB_MASK(num);

    if(color)
        _screen_h->buffer[pos] |= mask;
    else
        _screen_h->buffer[pos] &= ~mask;
}

/*!
    @brief    Set multiple pixel values. Internal routine, no error checking performed.
    The routine sets pixels of a single bank(check documentation for the display's pixel layout).
    The color goes from the MSB to LSB in the buffer, or from the top to the bottom in the actual
    display.
    @param    pos     The bank position in the buffer
    @param    num     Number of pixels to color, must be less than 8(bank size).
    @param    color   Either set pixels to black(true) or white(false).
*/
static void _set_pixels_msb2lsb(uint16_t pos, uint8_t num, bool color)
{
    ASSERT_DEBUG(pos >= LCDBUFFER_SZ, "Error at _set_pixels_invert_msb2lsb\n");
    ASSERT_DEBUG(num >= 8, "Error at _set_pixels_invert_msb2lsb\n");
    uint8_t mask = MSB2LSB_MASK(num);

    if(color)
        _screen_h->buffer[pos] |= mask;
    else
        _screen_h->buffer[pos] &= ~mask;
}

/*!
    @brief    Draws a generic line. Internal routine, it uses Bresenhm's algorithm and is based on the implementation
    by the Adafruit GFX library.
    @param    x0     Starting x-coordinate
    @param    x1     Ending x-coordinate
    @param    y0     Starting y-coordinate
    @param    y1     Ending y-coordinate
    @param    color  Black(true)/white(false)
*/
static void _draw_generic_line(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1, bool color)
{
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);

    if(steep)
    {
        SWAP_VAR(x0, y0);
        SWAP_VAR(x1, y1);
    }

    if(x0 > x1)
    {
        SWAP_VAR(x0, x1);
        SWAP_VAR(y0, y1);
    }

    int16_t dx = x1 - x0;
    int16_t dy = abs(y1 - y0);
    int16_t err = dx >> 1;
    int16_t ystep = (y0 < y1) ? 1 : -1;

    for (; x0 <= x1; x0++)
    {
        if(steep)
        {
            SSD1306_set_pixel(y0, x0, color);
        }
        else
        {
            SSD1306_set_pixel(x0, y0, color);
        }

        err -= dy;
        if(err < 0)
        {
            y0 += ystep;
            err += dx;
        }
    }
}

/*!
    @brief    Set a pixel's value.
    @param    x       x-coordinate
    @param    y       y-coordinate
    @param    color   Black(true) or white(false)
*/
void SSD1306_set_pixel(uint8_t x, uint8_t y, bool color)
{
    /* Sanity check */
    if((x >= LCDWIDTH) || (y >= LCDHEIGHT)) return;

    /* Call the internal routine */
    _set_single_pixel(x, y, color);
}

/*!
    @brief    Returns a pixel's value.
    @param    x   x-coordinate
    @param    y   y-coordinate
    @return       In case of error, 0xFF is returned, otherwise true or false.
*/
uint8_t SSD1306_get_pixel(uint8_t x, uint8_t y)
{
    /* Return max value in case of failure */
    return ((x >= LCDWIDTH) || (y >= LCDHEIGHT)) ? 0xff : _get_single_pixel(x, y);
}

/*!
    @brief    Draw a horizontal line.
    @param    x      Left-most x-coordinate
    @param    y      Left-most y-coordinate
    @param    len    The length of the line including the starting pixel
    @param    color  Black(true)/white(false)
*/
void SSD1306_draw_hline(uint8_t x, uint8_t y, uint8_t len, bool color)
{
    /* Sanity check - x value is taken care of by the loop conditions */
    if(y >= LCDHEIGHT || x >= LCDWIDTH) return;

    uint16_t pos = COORDS2BUFF_POS(x, y);
    if(((uint16_t)x + len) > LCDWIDTH) len = LCDWIDTH - x;
    uint8_t mask = 1 << (y & 0x07);

    if(color)
    {
        for(uint8_t i = 0; i < len; i++)
        {
            ASSERT_DEBUG((pos + i) >= LCDBUFFER_SZ, "Error at SSD1306_draw_hline\n");
            _screen_h->buffer[pos + i] |= mask;
        }
    }
    else
    {
        for(uint8_t i = 0; i < len; i++)
        {
            ASSERT_DEBUG((pos + i) >= LCDBUFFER_SZ, "Error at SSD1306_draw_hline\n");
            _screen_h->buffer[pos + i] &= ~mask;
        }
    }
}

/*!
    @brief    Draw a vertical line.
    @param    x      Left-most x-coordinate
    @param    y      Left-most y-coordinate
    @param    len    The length of the line including the starting pixel
    @param    color  Black(true)/white(false)
*/
void SSD1306_draw_vline(uint8_t x, uint8_t y, uint8_t len, bool color)
{
    /* Sanity check - y value is taken care of by the loop conditions */
    if(x >= LCDWIDTH || y >= LCDHEIGHT) return;

    /* Limit in case we exceed maximum height */
    if(((uint16_t)y + len) >= LCDHEIGHT) len = LCDHEIGHT - y;

    const uint8_t color_fill = color ? 0xff : 0;
    uint16_t pos = COORDS2BUFF_POS(x, y);
    uint8_t temp = y & 0x07;

    /* Partial bank fill */
    if(temp)
    {
        ASSERT_DEBUG(pos >= LCDBUFFER_SZ, "Error at SSD1306_draw_vline\n");
        uint8_t pixel_num = 8 - temp;

        if(len <= pixel_num) /* Sub-case that needs to be handled */
        {
            pixel_num = len;
            _set_pixels_msb2lsb(pos, pixel_num, color);
            return;
        }

        _set_pixels_msb2lsb(pos, pixel_num, color);
        pos += LCDWIDTH;
        len -= pixel_num;
    }

    /* Number of complete banks to fill */
    while(len >= 8)
    {
        ASSERT_DEBUG(pos >= LCDBUFFER_SZ, "Error at SSD1306_draw_vline\n");
        _screen_h->buffer[pos] = color_fill;
        pos += LCDWIDTH;
        len -= 8;
    }

    /* Draw leftovers */
    if(len) _set_pixels_lsb2msb(pos, len, color);
}

/*!
    @brief    Draw a generic line.
    @param    x0     Starting x-coordinate
    @param    x1     Ending x-coordinate
    @param    y0     Starting y-coordinate
    @param    y1     Ending y-coordinate
    @param    color  Black(true)/white(false)
*/
void SSD1306_draw_line(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1, bool color)
{
    if(x0 == x1) /* Horizontal line -> Call optimized version */
    {
        if(y0 > y1) SWAP_VAR(y0, y1);

        SSD1306_draw_vline(x0, y0, y1 - y0 + 1, color);
    }
    else if(y0 == y1) /* Vertical line -> Call optimized version */
    {
        if(x0 > x1) SWAP_VAR(x0, x1);

        SSD1306_draw_hline(x0, y0, x1 - x0 + 1, color);
    }
    else /* General case */
    {
        _draw_generic_line(x0, x1, y0, y1, color);
    }
}

/*!
    @brief    Draw a rectangle.
    @param    x0     Upper left x-coordinate
    @param    x1     Lower right x-coordinate
    @param    y0     Upper left y-coordinate
    @param    y1     Lower right y-coordinate
    @param    color  Black(true)/white(false)
    @param    fill   If true also fill the rectangle with the specified color
*/
void SSD1306_draw_rectangle(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1, bool color, bool fill)
{
    /* Sanity check */
    if(x0 >= LCDWIDTH || y0 >= LCDHEIGHT) return;

    /* Just in case mistakes were made */
    if(x0 > x1) SWAP_VAR(x0, x1);
    if(y0 > y1) SWAP_VAR(y0, y1);

    /* Set the line lengths */
    uint8_t len_x = x1 - x0 + 1;
    uint8_t len_y = y1 - y0 + 1;

    if(!fill)
    {
        /* Connect 4 lines together */
        SSD1306_draw_hline(x0, y0, len_x, color);
        SSD1306_draw_hline(x0, y1, len_x, color);
        SSD1306_draw_vline(x0, y0, len_y, color);
        SSD1306_draw_vline(x1, y0, len_y, color);
        return;
    }

    /* It's more efficient to use vertical lines to draw for filling.
     * That's the case since fewer memory accesses and instructions happen (on average) */
    if(((uint16_t)y0 + len_y) >= LCDHEIGHT) len_y = LCDHEIGHT - y0;
    if(((uint16_t)x0 + len_x) >= LCDWIDTH) len_x = LCDWIDTH - x0;

    const uint8_t color_fill = color ? 0xff : 0;
    uint16_t pos = COORDS2BUFF_POS(x0, y0);
    uint8_t temp = y0 & 0x07;

    /* Partial bank fill */
    if(temp)
    {
        ASSERT_DEBUG(pos >= LCDBUFFER_SZ, "Error at SSD1306_draw_rectangle\n");
        uint8_t pixel_num = 8 - temp;

        if(len_y <= pixel_num) /* Sub-case that needs to be handled */
        {
            pixel_num = len_y;
            for(uint8_t i = 0; i < len_x; i++) _set_pixels_msb2lsb(pos + i, pixel_num, color);
            return;
        }

        for(uint8_t i = 0; i < len_x; i++) _set_pixels_msb2lsb(pos + i, pixel_num, color);
        pos += LCDWIDTH;
        len_y -= pixel_num;
    }

    /* Number of complete banks to fill */
    while(len_y >= 8)
    {
        ASSERT_DEBUG(pos >= LCDBUFFER_SZ, "Error at SSD1306_draw_rectangle\n");
        memset(_screen_h->buffer + pos, color_fill, len_x * sizeof(uint8_t));
        pos += LCDWIDTH;
        len_y -= 8;
    }

    /* Draw leftovers */
    if(len_y)
    {
        for(uint8_t i = 0; i < len_x; i++) _set_pixels_lsb2msb(pos + i, len_y, color);
    }
}

/*!
    @brief    Draws a triangle. Also taken by the Adafruit GFX library.
    @param    x0     First x-coordinate
    @param    x1     Second x-coordinate
    @param    x2     Third x-coordinate
    @param    y0     First y-coordinate
    @param    y1     Second y-coordinate
    @param    y2     Third y-coordinate
    @param    color  Black(true)/white(false)
*/
void SSD1306_draw_triangle(uint8_t x0, uint8_t x1, uint8_t x2, uint8_t y0, uint8_t y1, uint8_t y2, bool color)
{
    SSD1306_draw_line(x0, x1, y0, y1, color);
    SSD1306_draw_line(x1, x2, y1, y2, color);
    SSD1306_draw_line(x0, x2, y0, y2, color);
}

/*!
    @brief    Draws a filled triangle. Also taken by the Adafruit GFX library.
    @param    x0     First x-coordinate
    @param    x1     Second x-coordinate
    @param    x2     Third x-coordinate
    @param    y0     First y-coordinate
    @param    y1     Second y-coordinate
    @param    y2     Third y-coordinate
    @param    color  Triangle color, Black(true)/white(false)
*/
void SSD1306_draw_fill_triangle(uint8_t x0, uint8_t x1, uint8_t x2, uint8_t y0, uint8_t y1, uint8_t y2, bool color)
{
    uint8_t a, b, y, last;

    /* Sort coordinates by Y order (y2 >= y1 >= y0) */
    if (y0 > y1)
    {
        SWAP_VAR(y0, y1);
        SWAP_VAR(x0, x1);
    }

    if (y1 > y2)
    {
        SWAP_VAR(y2, y1);
        SWAP_VAR(x2, x1);
    }

    if (y0 > y1)
    {
        SWAP_VAR(y0, y1);
        SWAP_VAR(x0, x1);
    }

    /* Handle awkward all-on-same-line case as its own thing */
    if(y0 == y2)
    {
        a = b = x0;
        if (x1 < a)       a = x1;
        else if (x1 > b)  b = x1;

        if (x2 < a)       a = x2;
        else if (x2 > b)  b = x2;

        SSD1306_draw_hline(a, y0, b - a + 1, color);
        return;
    }

    uint8_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0,
            dx12 = x2 - x1, dy12 = y2 - y1;
    uint16_t sa = 0, sb = 0;

    /* For upper part of triangle, find scanline crossings for segments 0-1 and 0-2.
     * If y1=y2 (flat-bottomed triangle), the scanline y1
     * is included here (and second loop will be skipped, avoiding a /0
     * error there), otherwise scanline y1 is skipped here and handled
     * in the second loop...which also avoids a /0 error here if y0=y1
     * (flat-topped triangle).
     */

    /* Include y1 scanline(?) or skip it(:) */
    last = (y1 == y2) ? y1 : (y1 - 1);

    for (y = y0; y <= last; y++)
    {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if (a > b) SWAP_VAR(a, b);
        SSD1306_draw_hline(a, y, b - a + 1, color);
    }

    /* For lower part of triangle, find scanline crossings for segments
     * 0-2 and 1-2.  This loop is skipped if y1=y2.
     */
    sa = (uint16_t)dx12 * (y - y1);
    sb = (uint16_t)dx02 * (y - y0);
    for (; y <= y2; y++)
    {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
        a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if (a > b) SWAP_VAR(a, b);
        SSD1306_draw_hline(a, y, b - a + 1, color);
    }
}

/*!
    @brief    Draws a circle - Uses the Midpoint circle algorithm.
    @param    x0   Center x-coordinate
    @param    y0   Center y-coordinate
    @param    r    Circle radius
    @param    color - black(true)/white(false)
*/
void SSD1306_draw_circle(uint8_t x, uint8_t y, uint8_t r, bool color)
{
    int8_t a = 0;
    int8_t b = r;
    int8_t p = 1 - r;

    do
    {
        SSD1306_set_pixel(x+a, y+b, color);
        SSD1306_set_pixel(x+b, y+a, color);
        SSD1306_set_pixel(x+a, y-b, color);
        SSD1306_set_pixel(x+b, y-a, color);
        SSD1306_set_pixel(x-a, y+b, color);
        SSD1306_set_pixel(x-b, y+a, color);
        SSD1306_set_pixel(x-a, y-b, color);
        SSD1306_set_pixel(x-b, y-a, color);

        if(p < 0)
        {
            p += (3 + 2*a);
            a++;
        }
        else
        {
            p += (5 + 2*(a-b));
            a++;
            b--;
        }
    }while(a <= b);
}

/*!
    @brief    Draws a filled circle - Uses the Midpoint circle algorithm.
    @param    x0   Center x-coordinate
    @param    y0   Center y-coordinate
    @param    r    Circle radius
    @param    color - black(true)/white(false)
*/
void SSD1306_draw_fill_circle(uint8_t x0, uint8_t y0, uint8_t r, bool color)
{
    /* Write out the middle line - we use the pixel setters since lines might be out of bounds */
    for(uint8_t i = 0; i < (2 * r + 1); i++) SSD1306_set_pixel(x0, y0 - r + i, color);

    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    int16_t px = x;
    int16_t py = y;

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }

        x++;
        ddF_x += 2;
        f += ddF_x;

        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if (x < (y + 1))
        {
            for(uint i = 0; i < 2 * y; i++) /* Same as initial vline drawing */
            {
                SSD1306_set_pixel(x0 + x, y0 - y + i, color);
                SSD1306_set_pixel(x0 - x, y0 - y + i, color);
            }
        }

        if (y != py)
        {
            for(uint i = 0; i < 2 * px; i++) /* Same as initial vline drawing */
            {
                SSD1306_set_pixel(x0 + py, y0 - px + i, color);
                SSD1306_set_pixel(x0 - py, y0 - px + i, color);
            }

            py = y;
        }

        px = x;
    }

}

/*!
    @brief    Draw a rounded rectangle.
    @param    x0     Upper left x-coordinate
    @param    x1     Lower right x-coordinate
    @param    y0     Upper left y-coordinate
    @param    y1     Lower right y-coordinate
    @param    color  Black(true)/white(false)
    @param    fill   If true also fill the rectangle with the specified color
*/
void SSD1306_draw_round_rect(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1, bool color, bool fill)
{
    /* Just in case mistakes were made */
    if(x0 > x1) SWAP_VAR(x0, x1);
    if(y0 > y1) SWAP_VAR(y0, y1);

    /* As long as it is a valid rectangle */
    if((x1 - x0) > 4 && (y1 - y0) > 4)
    {
        if(fill)
        {
            /* Draw a normal filed rectangle and opposite color for the other bits */
            SSD1306_draw_rectangle(x0, x1, y0, y1, color, true);

            /* Upper left corner */
            SSD1306_set_pixel(x0, y0, !color);
            SSD1306_set_pixel(x0 + 1, y0, !color);
            SSD1306_set_pixel(x0, y0 + 1, !color);

            /* Upper right corner */
            SSD1306_set_pixel(x1, y0, !color);
            SSD1306_set_pixel(x1 - 1, y0, !color);
            SSD1306_set_pixel(x1, y0 + 1, !color);

            /* Lower left corner */
            SSD1306_set_pixel(x0, y1, !color);
            SSD1306_set_pixel(x0 + 1, y1, !color);
            SSD1306_set_pixel(x0, y1 - 1, !color);

            /* Lower right corner */
            SSD1306_set_pixel(x1, y1, !color);
            SSD1306_set_pixel(x1 - 1, y1, !color);
            SSD1306_set_pixel(x1, y1 - 1, !color);
        }
        else
        {
            SSD1306_set_pixel(x0 + 1, y0 + 1, color);
            SSD1306_set_pixel(x1 - 1, y0 + 1, color);
            SSD1306_set_pixel(x0 + 1, y1 - 1, color);
            SSD1306_set_pixel(x1 - 1, y1 - 1, color);
            SSD1306_draw_hline(x0 + 2, y0, x1 - x0 - 3, color);
            SSD1306_draw_hline(x0 + 2, y1, x1 - x0 - 3, color);
            SSD1306_draw_vline(x0, y0 + 2, y1 - y0 - 3, color);
            SSD1306_draw_vline(x1, y0 + 2, y1 - y0 - 3, color);
        }
    }
}

/**********************************************************/
/************************ BITMAPS *************************/
/**********************************************************/

/*!
    @brief    Get a pixel's value from a bitmap array.
    @param    bitmap    The bitmap array
    @param    x         x-coordinate
    @param    y         y-coordinate
    @param    bit_w     The bitmap width
    @return             Black(True) or White(False).
*/
static bool _get_bmp_pixel(const uint8_t *bitmap, uint8_t x, uint8_t y, uint8_t bit_w)
{
    /* First find the exact position */
    uint16_t pos = COORDS2BIT_POS(x, y, bit_w);//((uint16_t)y>>3) * bit_w + x;

    return (bitmap[pos] >> (y & 0x07)) & 0x01;
}

/*!
    @brief    Get a pixel's value from a bitmap array.
    Internal routine and different variant than the normal version.
    @param    bitmap    The bitmap array
    @param    pos       The position in the array
    @param    shift     The shift value
    @return             Black(True) or White(False).
*/
static bool _get_bmp_pixel_opt(const uint8_t *bitmap, uint16_t pos, uint8_t shift)
{
    return (bitmap[pos] >> shift) & 0x01;
}

/*!
    @brief    Draws a bitmap on the screen.
    @param    bitmap    The bitmap array
    @param    x0        Leftmost x-coordinate
    @param    y0        Leftmost y-coordinate
    @param    len_x     The width of the bitmap
    @param    len_y     The height of the bitmap
*/
static void _draw_bitmap(const uint8_t *bitmap, uint8_t x0, uint8_t y0, uint8_t draw_x, uint8_t draw_y, uint8_t len_x)
{
    for(uint8_t j = 0; j < draw_y; j++)
    {
        /* Since we use the opt version of set, we calculate it ourselves */
        uint8_t mask = 1 << ((y0 + j) & 0x07);
        uint8_t bmp_shift = (j & 0x07);
        uint16_t pos = COORDS2BUFF_POS(x0, y0 + j);
        uint16_t pos_src = COORDS2BIT_POS(0, j, len_x);

        for(uint8_t i = 0; i < draw_x; i++)
        {
            bool bmp_color = _get_bmp_pixel_opt(bitmap, pos_src + i, bmp_shift);
            _set_single_pixel_opt(pos + i, mask, bmp_color);
        }
    }
}

/*!
    @brief    The optimized scaler kernel for nearest neighbor - x2 scale.
    @param    bitmap    The bitmap array
    @param    x0        Leftmost x-coordinate
    @param    y0        Leftmost y-coordinate
    @param    draw_x    Draw length on the x-axis
    @param    draw_y    Draw length on the y-axis
    @param    len_x     The width of the bitmap
*/
static void _scale_bitmap_nb_x2(const uint8_t *bitmap, uint8_t x0, uint8_t y0, uint8_t draw_x, uint8_t draw_y, uint8_t len_x)
{
    const uint8_t scale = 2;
    const uint8_t high_mask = 0x80;
    const uint8_t low_mask = 0x01;

    for(uint8_t j = 0; j < draw_y; j++, y0 += scale)
     {
         uint16_t pos = COORDS2BUFF_POS(x0, y0);
         uint16_t src_pos = COORDS2BIT_POS(0, j, len_x);
         uint8_t bmp_shift = (j & 0x07);

         if((y0 & 0x07) == 0x07)
         {
             for(uint8_t i = 0; i < draw_x; i++)
             {
                 bool color = _get_bmp_pixel_opt(bitmap, src_pos + i, bmp_shift);

                 if(color)
                 {
                     _screen_h->buffer[pos] |= high_mask;
                     _screen_h->buffer[pos + 1] |= high_mask;
                     _screen_h->buffer[pos + LCDWIDTH] |= low_mask;
                     _screen_h->buffer[pos + LCDWIDTH + 1] |= low_mask;
                 }
                 else
                 {
                     _screen_h->buffer[pos] &= ~high_mask;
                     _screen_h->buffer[pos + 1] &= ~high_mask;
                     _screen_h->buffer[pos + LCDWIDTH] &= ~low_mask;
                     _screen_h->buffer[pos + LCDWIDTH + 1] &= ~low_mask;
                 }

                 pos += scale;
             }
         }
         else
         {
             uint8_t mask = 3 << (y0 & 0x07);

             for(uint8_t i = 0; i < draw_x; i++)
             {
                 bool color = _get_bmp_pixel_opt(bitmap, src_pos + i, bmp_shift);

                 if(color)
                 {
                     _screen_h->buffer[pos] |= mask;
                     _screen_h->buffer[pos + 1] |= mask;
                 }
                 else
                 {
                     _screen_h->buffer[pos] &= ~mask;
                     _screen_h->buffer[pos + 1] &= ~mask;
                 }

                 pos += scale;
             }
         }
     }
}

/*!
    @brief    The optimized scaler kernel for nearest neighbor - x3 scale.
    @param    bitmap    The bitmap array
    @param    x0        Leftmost x-coordinate
    @param    y0        Leftmost y-coordinate
    @param    draw_x    Draw length on the x-axis
    @param    draw_y    Draw length on the y-axis
    @param    len_x     The width of the bitmap
*/
static void _scale_bitmap_nb_x3(const uint8_t *bitmap, uint8_t x0, uint8_t y0, uint8_t draw_x, uint8_t draw_y, uint8_t len_x)
{
    const uint8_t scale = 3;

    for(uint8_t j = 0; j < draw_y; j++, y0 += scale)
    {
        uint16_t pos = COORDS2BUFF_POS(x0, y0);
        uint16_t src_pos = COORDS2BIT_POS(0, j, len_x);
        uint8_t bmp_shift = (j & 0x07);

        if((y0 & 0x07) >= 0x06)
        {
            uint8_t butes_num = 8 - (y0 & 0x07);
            uint8_t up_mask = MSB2LSB_MASK(butes_num);
            uint8_t low_mask = LSB2MSB_MASK(scale - butes_num);

            for(uint8_t i = 0; i < draw_x; i++)
            {
                bool color = _get_bmp_pixel_opt(bitmap, src_pos + i, bmp_shift);

                if(color)
                {
                    _screen_h->buffer[pos] |= up_mask;
                    _screen_h->buffer[pos + 1] |= up_mask;
                    _screen_h->buffer[pos + 2] |= up_mask;
                    _screen_h->buffer[pos + LCDWIDTH] |= low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 1] |= low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 2] |= low_mask;
                }
                else
                {
                    _screen_h->buffer[pos] &= ~up_mask;
                    _screen_h->buffer[pos + 1] &= ~up_mask;
                    _screen_h->buffer[pos + 2] &= ~up_mask;
                    _screen_h->buffer[pos + LCDWIDTH] &= ~low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 1] &= ~low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 2] &= ~low_mask;
                }

                pos += scale;
            }
        }
        else
        {
            uint8_t mask = 7 << (y0 & 0x07);

            for(uint8_t i = 0; i < draw_x; i++)
            {
                bool color = _get_bmp_pixel_opt(bitmap, src_pos + i, bmp_shift);

                if(color)
                {
                    _screen_h->buffer[pos] |= mask;
                    _screen_h->buffer[pos + 1] |= mask;
                    _screen_h->buffer[pos + 2] |= mask;
                }
                else
                {
                    _screen_h->buffer[pos] &= ~mask;
                    _screen_h->buffer[pos + 1] &= ~mask;
                    _screen_h->buffer[pos + 2] &= ~mask;
                }

                pos += scale;
            }
        }
    }

}

/*!
    @brief    The optimized scaler kernel for nearest neighbor - x4 scale.
    @param    bitmap    The bitmap array
    @param    x0        Leftmost x-coordinate
    @param    y0        Leftmost y-coordinate
    @param    draw_x    Draw length on the x-axis
    @param    draw_y    Draw length on the y-axis
    @param    len_x     The width of the bitmap
*/
static void _scale_bitmap_nb_x4(const uint8_t *bitmap, uint8_t x0, uint8_t y0, uint8_t draw_x, uint8_t draw_y, uint8_t len_x)
{
    const uint8_t scale = 4;

    for(uint8_t j = 0; j < draw_y; j++, y0 += scale)
    {
        uint16_t pos = COORDS2BUFF_POS(x0, y0);
        uint16_t src_pos = COORDS2BIT_POS(0, j, len_x);
        uint8_t bmp_shift = (j & 0x07);

        if((y0 & 0x07) >= 0x05)
        {
            uint8_t butes_num = 8 - (y0 & 0x07);
            uint8_t up_mask = MSB2LSB_MASK(butes_num);
            uint8_t low_mask = LSB2MSB_MASK(scale - butes_num);

            for(uint8_t i = 0; i < draw_x; i++)
            {
                bool color = _get_bmp_pixel_opt(bitmap, src_pos + i, bmp_shift);

                if(color)
                {
                    _screen_h->buffer[pos] |= up_mask;
                    _screen_h->buffer[pos + 1] |= up_mask;
                    _screen_h->buffer[pos + 2] |= up_mask;
                    _screen_h->buffer[pos + 3] |= up_mask;
                    _screen_h->buffer[pos + LCDWIDTH] |= low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 1] |= low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 2] |= low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 3] |= low_mask;
                }
                else
                {
                    _screen_h->buffer[pos] &= ~up_mask;
                    _screen_h->buffer[pos + 1] &= ~up_mask;
                    _screen_h->buffer[pos + 2] &= ~up_mask;
                    _screen_h->buffer[pos + 3] &= ~up_mask;
                    _screen_h->buffer[pos + LCDWIDTH] &= ~low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 1] &= ~low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 2] &= ~low_mask;
                    _screen_h->buffer[pos + LCDWIDTH + 3] &= ~low_mask;
                }

                pos += scale;
            }
        }
        else
        {
            uint8_t mask = 15 << (y0 & 0x07);

            for(uint8_t i = 0; i < draw_x; i++)
            {
                bool color = _get_bmp_pixel_opt(bitmap, src_pos + i, bmp_shift);

                if(color)
                {
                    _screen_h->buffer[pos] |= mask;
                    _screen_h->buffer[pos + 1] |= mask;
                    _screen_h->buffer[pos + 2] |= mask;
                    _screen_h->buffer[pos + 3] |= mask;
                }
                else
                {
                    _screen_h->buffer[pos] &= ~mask;
                    _screen_h->buffer[pos + 1] &= ~mask;
                    _screen_h->buffer[pos + 2] &= ~mask;
                    _screen_h->buffer[pos + 3] &= ~mask;
                }

                pos += scale;
            }
        }
    }

}

/*!
    @brief    Draws a bitmap on the screen and scale upwards by the argument scale.
    The scaling is performed by using the nearest neighbor interpolation method.
    If scale is set to 1, then simply draw the bitmap as is.
    @param    bitmap    The bitmap array
    @param    x0        Leftmost x-coordinate
    @param    y0        Leftmost y-coordinate
    @param    len_x     The width of the bitmap
    @param    len_y     The height of the bitmap
    @param    scale     The scale factor, can be 1, 2, 3 or 4
*/
void SSD1306_draw_bitmap(const uint8_t *bitmap, uint8_t x0, uint8_t y0, uint8_t len_x, uint8_t len_y, uint8_t scale)
{
    /* Illegal format of the bitmap or initial position */
    if(x0 >= LCDWIDTH || y0 >= LCDHEIGHT) return;

    /* Draw lengths */
    uint8_t draw_y = len_y, draw_x = len_x;

    /* Adjust traverse length of bitmap if need be */
    if(((uint16_t)y0 + scale * len_y) > LCDHEIGHT) draw_y = (LCDHEIGHT - y0)/scale;
    if(((uint16_t)x0 + scale * len_x) > LCDWIDTH) draw_x = (LCDWIDTH - x0)/scale;

    /* Factor has to be x2, x3, x4 */
    switch(scale)
    {
        case 1: /* No scaling */
        {
            _draw_bitmap(bitmap, x0, y0, draw_x, draw_y, len_x);
            break;
        }
        case 2:
        {
            _scale_bitmap_nb_x2(bitmap, x0, y0, draw_x, draw_y, len_x);
            break;
        }
        case 3:
        {
            _scale_bitmap_nb_x3(bitmap, x0, y0, draw_x, draw_y, len_x);
            break;
        }
        case 4:
        {
            _scale_bitmap_nb_x4(bitmap, x0, y0, draw_x, draw_y, len_x);
            break;
        }
        default: return;
    }
}

/*!
    @brief    Draws a bitmap on the screen.
    Optimized version that draws bitmaps with height a multiple of 8
    and that start from a multiple of 8 y-coordinate.
    Basically we draw from the start of a bank continuously.

    @param    bitmap    The bitmap array
    @param    x0        Leftmost x-coordinate
    @param    y0        Leftmost y-coordinate
    @param    len_x     The width of the bitmap
    @param    len_y     The height of the bitmap
*/
void SSD1306_draw_bitmap_opt8(const uint8_t *bitmap, uint8_t x0, uint8_t y0, uint8_t len_x, uint8_t len_y)
{
    /* Illegal format of the bitmap or initial position or height */
    if(x0 >= LCDWIDTH || y0 >= LCDHEIGHT) return;
    if((y0 & 0x07) || (len_y & 0x07)) return;

    /* Fix drawing length */
    if(((uint16_t)y0 + len_y) >= LCDHEIGHT) len_y = LCDHEIGHT - y0;
    if(((uint16_t)x0 + len_x) >= LCDWIDTH) len_x = LCDWIDTH - x0;

    uint8_t full_banks = len_y >> 3;
    uint16_t pos = COORDS2BUFF_POS(x0, y0);
    uint16_t pos_src = 0;

    for(uint8_t j = 0; j < full_banks; j++)
    {
        ASSERT_DEBUG((pos) >= LCDBUFFER_SZ, "Error at SSD1306_draw_bitmap_opt8 -> %d\n", pos);
        ASSERT_DEBUG((pos_src) >= (len_x*len_y), "Error at SSD1306_draw_bitmap_opt8 -> %d\n", pos_src);

        memcpy(_screen_h->buffer + pos, bitmap + pos_src, len_x * sizeof(uint8_t));
        pos += LCDWIDTH;
        pos_src += len_x;
    }
}

/**********************************************************/
/************************* TEXT ***************************/
/**********************************************************/

/*!
    @brief    Set the cursor position for the default printer.
    @param    x  x-coordinate
    @param    y  y-coordinate
*/
void SSD1306_coord(uint8_t x, uint8_t y)
{
    if(x < LCDWIDTH) _screen_h->x_pos = x;
    if(y < LCDHEIGHT) _screen_h->y_pos = y >> 3;
}

/*!
    @brief    Draws a string on the screen.
    This is the default printer that draws on the preassigned coordinates with the
    preassigned font (small, medium, large).
    The printing is done on the bank borders, so text can be aligned UP, CENTER and LOW.

    CENTER:    Text aligned exactly at the center
    BOTTOM:    Text aligned exactly at the lowest part
    TOP:       Text aligned exactly at the highest possible spot

    In the case of LARGE text, no option is available.
    In the case of MEDIUM text, only TOP and BOTTOM options available.
    In the case of SMALL text, all options are available.

    @param    str       The string to print
    @param    option    The options (font and potential centering)
    @param    invert    Flag to invert the text, if true inverts (black bg with white character)
    otherwise left as is
*/
void SSD1306_print_str(const char *str, uint8_t option, bool invert)
{
    /* Sanity check */
    if(!str) return;

    uint8_t width, shift, byte_num, *font;
    const char offset = 0x20; /* For now this is constant - TODO No big number fonts */

    /* Get the parameters of the text */
    switch(option & FONT_MASK)
    {
        case LARGE_FONT:
        {
            shift = 0;
            width = 6;
            byte_num = 6;
            font = (uint8_t *)large_font; // For Compiler warnings //
            break;
        }
        case MEDIUM_FONT:
        {
            shift = (option & ALIGMENT_MASK) >> 1; /* Only up or bottom alignment for medium */
            width = 5;
            byte_num = 5;
            font = (uint8_t *)medium_font; // For Compiler warnings //
            break;
        }
        case SMALL_FONT:
        {
            shift = option & ALIGMENT_MASK;
            width = 4;
            byte_num = 3;
            font = (uint8_t *)small_font; // For Compiler warnings //
            break;
        }
        default: return; /* Illegal option */
    }

    /* Print buffer in case we need to edit a character */
    uint8_t buffer[6];

    for(; *str; str++)
    {
        /* Screen bounds exceeded or newline found */
        if((_screen_h->x_pos + width) >= LCDWIDTH || *str == '\n')
        {
            _screen_h->x_pos = 0;
            _screen_h->y_pos++;
        }

        /* Screen bounds exceeded, reset back to start */
        if(_screen_h->y_pos >= LCDHEIGHT/8) _screen_h->y_pos = 0;

        if(*str >= offset)
        {
            uint16_t dest_pos = ((uint16_t)_screen_h->y_pos) * LCDWIDTH + _screen_h->x_pos;
            uint16_t src_pos = (*str - offset) * byte_num;

            /* Copy to the print buffer */
            memcpy(buffer, font + src_pos, byte_num * sizeof(uint8_t));

            /* Small font has to be decoded since the bytes are packed */
            if((option & FONT_MASK) == SMALL_FONT)
            {
                /* Unpack the 3 bytes into 4 */
                buffer[3] = (buffer[0] & 0xfc) >> 2;
                buffer[4] = ((buffer[0] & 0x03) << 4) | ((buffer[1] & 0xf0) >> 4);
                buffer[5] = ((buffer[1] & 0x0f) << 2) | ((buffer[2] & 0xc0) >> 6);
                uint8_t temp = buffer[2] & 0x3f;

                /* Correct position */
                buffer[0] = buffer[3];
                buffer[1] = buffer[4];
                buffer[2] = buffer[5];
                buffer[3] = temp;
            }

            /* Shifting */
            if(shift)
            {
                for(uint8_t i = 0; i < width; i++) buffer[i] = buffer[i] << shift;
            }

            /* Invert option */
            if(invert)
            {
                for(uint8_t i = 0; i < width; i++) buffer[i] = ~buffer[i];
            }

            memcpy(_screen_h->buffer + dest_pos, buffer, width * sizeof(uint8_t));

            _screen_h->x_pos += width;
        }
    }
}

/*!
    @brief    Draws a string on the screen.
    This variant prints a string on any xy coordinate in the screen (starting positions) freely, hence
    the 'f' in method name. The starting position is the uppermost left point of where a character should be.
    This variant can also scale the letters upwards if need be.
    @param    str       The string to print
    @param    option    Font type (Alignment is not needed here)
    @param    x         Starting x-coordinate
    @param    y         Starting y-coordinate
    @param    scale     How much to scale the existing font (x1, x2, x3, x4 only)
    @param    invert    Flag to invert the text, if true inverts (black bg with white character)
    otherwise left as is.
*/
void SSD1306_print_fstr(const char *str, uint8_t option, uint8_t x, uint8_t y, uint8_t scale, bool invert)
{
    /* Sanity check */
    if(!str) return;

    uint8_t width, height, byte_num, *font;
    uint8_t real_width, real_height;
    const char offset = 0x20; /* For now this is constant - TODO No big number fonts */

    /* Get the parameters of the text */
    switch(option & FONT_MASK)
    {
        case LARGE_FONT:
        {
            height = 8;
            width = 6;
            byte_num = 6;
            font = (uint8_t *)large_font; // For Compiler warnings //
            break;
        }
        case MEDIUM_FONT:
        {
            height = 7;
            width = 5;
            byte_num = 5;
            font = (uint8_t *)medium_font; // For Compiler warnings //
            break;
        }
        case SMALL_FONT:
        {
            height = 6;
            width = 4;
            byte_num = 3;
            font = (uint8_t *)small_font; // For Compiler warnings //
            break;
        }
        default: return; /* Illegal option */
    }

    /* Set parameters for the scaling */
    real_width = width * scale;
    real_height = height * scale;

    /* Print buffer in case we need to edit a character */
    uint8_t buffer[6];

    for(; *str; str++)
    {
        /* Screen bounds exceeded or newline found */
        if((x + real_width) >= LCDWIDTH || *str == '\n')
        {
            x = 0;
            y += real_height;
        }

        /* Screen bounds exceeded, reset back to start */
        if(y >= LCDHEIGHT) y = 0;

        if(*str >= offset)
        {
            uint16_t src_pos = (*str - offset) * byte_num;

            /* Copy to the print buffer */
            memcpy(buffer, font + src_pos, byte_num * sizeof(uint8_t));

            /* Small font has to be decoded since the bytes are packed */
            if((option & FONT_MASK) == SMALL_FONT)
            {
                /* Unpack the 3 bytes into 4 */
                buffer[3] = (buffer[0] & 0xfc) >> 2;
                buffer[4] = ((buffer[0] & 0x03) << 4) | ((buffer[1] & 0xf0) >> 4);
                buffer[5] = ((buffer[1] & 0x0f) << 2) | ((buffer[2] & 0xc0) >> 6);
                uint8_t temp = buffer[2] & 0x3f;

                /* Correct position */
                buffer[0] = buffer[3];
                buffer[1] = buffer[4];
                buffer[2] = buffer[5];
                buffer[3] = temp;
            }

            /* Invert option */
            if(invert)
            {
                for(uint8_t i = 0; i < width; i++) buffer[i] = ~buffer[i];
            }

            /* Draw the bitmap */
            SSD1306_draw_bitmap(buffer, x, y, width, height * sizeof(uint8_t), scale);

            x += real_width;
        }
    }
}
