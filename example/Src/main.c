#include <ssd_1306.h>
#include "main.h"
#include <stdio.h>

/* Private variables */
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_spi2_tx;

/** the memory buffer for the LCD */
uint8_t SSD1306_buffer[SSD1306_BUFFER_SZ] = {0xff};

/* Timer - Using the DWT core one, it's enough for our purposes */
#define START_TIMER()   (DWT->CYCCNT = 0)
#define GET_TIMER()     (DWT->CYCCNT)

/* Macro used for screen testing*/
#define SCREEN_DELAY_FILL(delay, color) do{HAL_Delay((delay));SSD1306_fill((color));}while(0)


/* Private function prototypes */
static void MX_Core_Counter_Init(void);
static void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);

/* Testing */
static void init_example();
static void draw_examples();
static void test_lcd_basic();
static void test_lcd_simple_patterns();
static void test_lcd_intermediate_patterns();
static void test_lcd_bitmaps();
static void test_lcd_text();

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all peripherals */
    MX_DMA_Init();  // DMA should be first - SPI Initialization will access DMA registers //
    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_USART2_UART_Init();
    MX_Core_Counter_Init();

    while(1)
    {
        /* How to initialize the screen - Mostly documentation */
        init_example();
        HAL_Delay(5000);

        /* Drawing and setting the screen */
        draw_examples();
    }
}

/**********************************/
/*********** TEST CODE ************/
/**********************************/

/* Showcases initialization and how to set the screen up */
static void init_example()
{
    ssd_1306_t SSD1306_handle;
    printf("\n\n************DUMMY TEST************\n");

    /* Extra GPIOs to be used besides MOSI and CLK for the SPI Alternate function pins
     * Simply use the HAL definitions of the pins/ports and set them on the screen handle.
     * Make sure that they are initialized beforehand and they are ready to go {MX_GPIO_Init()}
     * */
    SSD1306_handle.rst_pin = GPIO_PIN_0;
    SSD1306_handle.ce_pin = GPIO_PIN_1;
    SSD1306_handle.dc_pin = GPIO_PIN_2;
    SSD1306_handle.rst_port = GPIOB;
    SSD1306_handle.ce_port = GPIOB;
    SSD1306_handle.dc_port = GPIOB;

    /* The HAL handle for the SPI peripheral to use. Same as before make sure to initialize SPI before
     * initializing the screen. SPI initialization (DMA or not) takes place in the following routine:
     * - MX_DMA_Init<main.c> (if DMA is used it should be initialized beforehand)
     * - MX_SPI2_Init <main.c> (Initializes SPI according to the options in the handle)
     * - HAL_SPI_MspInit()<stm32f4xx_hal_msp.c> (Called by the HAL_SPI_Init() and initializes
     * clocks, GPIOs and DMA if it is needed)
     *
     * Most of these will be taken care of by CUBEMX during code generation for your board.
     * */
    SSD1306_handle.h_spi = &hspi2;

    /* The buffer to be used by this screen handle. This is were drawing is taken place
     * and then user with a call to PCD8544_refresh() updates the contents of the actual display.
     * This is separate in case multiple screens (which share a buffer) are used or
     * memory is shared and the buffer is also used for something else afterwards.
     * */
    SSD1306_handle.buffer = SSD1306_buffer;

    /* These are the base screen settings. In general the contrast value varies wildly from screen to screen
     * so it might be the first thing needed to modify and experiment with. */
    SSD1306_handle.contast = SSD1306_CONTRAST_DEFAULT_NOVCC;   // In case of problem adjust up/down by steps of 5
    SSD1306_handle.vcs = SSD1306_SWITCHCAPVCC;

    /* In case the user wants to use DMA, they should uncomment the SSD1306_DMA_ACTIVE definition on
     * the library header file. This means that the library will use DMA for SPI transmissions.
     * User can check if a transfer is underway by the {dma_transfer} flag on the screen handle.
     *
     * Finaly, the DMA has to be set up beforehand and be ready to go for the library to function
     * correctly.
     * */

    /* Initialize PCD_8544 */
    if(!SSD1306_init(&SSD1306_handle))
    {
        printf("\tInitialization failed, entering infinite loop...\n");
        while(1);
    }
    else
    {
        printf("\tDummy initialization complete\n");
    }
}

/* The settings and drawing testbench */
void draw_examples(void)
{
    ssd_1306_t SSD1306_handle =
                            {
                                .rst_pin = GPIO_PIN_0,
                                .ce_pin = GPIO_PIN_1,
                                .dc_pin = GPIO_PIN_2,

                                .rst_port = GPIOB,
                                .ce_port = GPIOB,
                                .dc_port = GPIOB,

                                .h_spi = &hspi2,
                                .buffer = SSD1306_buffer,

                                .contast = SSD1306_CONTRAST_DEFAULT_NOVCC,
                                .vcs = SSD1306_SWITCHCAPVCC
                            };

    /* Initialize SSD1306 */
    if(!SSD1306_init(&SSD1306_handle))
    {
        printf("Initialization failed, entering infinite loop...\n");
        while(1);
    }

    printf("\n\n************BASIC TESTS************\n");
    test_lcd_basic();

    printf("\n\n************PATTERN TESTS************\n");
    test_lcd_simple_patterns();

    printf("\n\n************PATTERN TESTS (INTERMEDIATE)************\n");
    test_lcd_intermediate_patterns();

    printf("\n\n************BITMAP TESTS************\n");
    test_lcd_bitmaps();

    printf("\n\n************TEXT TESTS************\n");
    test_lcd_text();

#ifdef SSD1306_DMA_ACTIVE
    /* Wait for any potential DMA transfers to finish */
    while(SSD1306_handle.dma_transfer);
#endif
}

/* Tests basic functionalities */
static void test_lcd_basic()
{
    /* Empty the screen */
    SSD1306_fill(false);
    if(SSD1306_refresh()) printf("\t[0]Emptying screen:OK\n");
    HAL_Delay(3000);


    /* Fill the screen completely */
    SSD1306_fill(true);
    if(SSD1306_refresh()) printf("\t[1]Filling screen:OK\n");
    HAL_Delay(3000);


    /* Test inversion */
    if(SSD1306_invert(true)) printf("\t[2]Inverting screen:OK\n");
    HAL_Delay(3000);


    /* Test uninversion */
    if(SSD1306_invert(false)) printf("\t[3]Uninverting screen:OK\n");
    HAL_Delay(3000);


    /* Fill the screen using horizontal lines */
    for(uint8_t i = 0; i < SSD1306_HEIGHT; i++) SSD1306_draw_hline(0, i, SSD1306_WIDTH, true);
    if(SSD1306_refresh()) printf("\t[4]Filling screen with hlines:OK\n");
    SCREEN_DELAY_FILL(3000, false);


    /* Fill the screen using horizontal lines */
    for(uint8_t i = 0; i < SSD1306_WIDTH; i++) SSD1306_draw_vline(i, 0, SSD1306_HEIGHT, true);
    if(SSD1306_refresh()) printf("\t[5]Filling screen with vlines:OK\n");
    SCREEN_DELAY_FILL(3000, false);


    /* Fill the screen using individual sets */
    for(uint8_t i = 0; i < SSD1306_WIDTH; i++)
        for(uint8_t j = 0; j < SSD1306_HEIGHT; j++)
            SSD1306_set_pixel(i, j, true);

    if(SSD1306_refresh()) printf("\t[6]Filling screen with setpixel:OK\n");
    SCREEN_DELAY_FILL(3000, false);


    /* Power OFF */
    if(SSD1306_sleep_mode(true)) printf("\t[7]Powering off display:OK\n");
    HAL_Delay(6000);


    /* Power ON */
    if(SSD1306_sleep_mode(false)) printf("\t[8]Powering on display:OK\n");
    HAL_Delay(6000);


    /* Vcomm low */
    if(SSD1306_vcomh(SSD1306_VCOMDETECT_LOW)) printf("\t[9]Changing to low vcomh:OK\n");
    HAL_Delay(6000);


    /* Vcomm high */
    if(SSD1306_vcomh(SSD1306_VCOMDETECT_HIGH)) printf("\t[10]Changing to high vcomh:OK\n");
    HAL_Delay(6000);


    /* Vcomm medium */
    if(SSD1306_vcomh(SSD1306_VCOMDETECT_DEFAULT)) printf("\t[11]Changing to default vcomh:OK\n");
    SCREEN_DELAY_FILL(3000, false);
}

/* Draw basic shapes with lines/rectangles and pixel settings */
static void test_lcd_simple_patterns()
{
    bool color = true;
    for(uint8_t reps = 0; reps < 2; reps++)
    {
        /* Fill the screen with the specified color */
        SSD1306_fill(!color);

        /* Draw a chessboard */
        for(uint8_t j = 0; j < SSD1306_HEIGHT; j++)
        {
            bool toggle = j & 0x01;
            for(uint8_t i = 0; i < SSD1306_WIDTH; i++)
            {
                SSD1306_set_pixel(i, j, toggle);
                toggle = !toggle;
            }
        }
        if(SSD1306_refresh()) printf("\t[%s]:Chessboard pattern:OK\n", color ? "Black" : "White");
        SCREEN_DELAY_FILL(3000, !color);


        /* Draw a grid */
        for(uint8_t i = 0; i < SSD1306_HEIGHT; i++) if((i & 0x01)) SSD1306_draw_hline(0, i, SSD1306_WIDTH, color);
        for(uint8_t i = 0; i < SSD1306_WIDTH; i++) if((i & 0x01)) SSD1306_draw_vline(i, 0, SSD1306_HEIGHT, color);
        if(SSD1306_refresh()) printf("\t[%s]:Grid pattern:OK\n", color ? "Black" : "White");
        SCREEN_DELAY_FILL(3000, !color);


        /* Draw parallel horizontal lines */
        for(uint8_t i = 0; i < SSD1306_HEIGHT; i++) if((i & 0x01)) SSD1306_draw_hline(0, i, SSD1306_WIDTH, color);
        if(SSD1306_refresh()) printf("\t[%s]:Parallel horizontals:OK\n", color ? "Black" : "White");
        SCREEN_DELAY_FILL(3000, !color);


        /* Draw parallel vertical lines */
        for(uint8_t i = 0; i < SSD1306_WIDTH; i++) if((i & 0x01)) SSD1306_draw_vline(i, 0, SSD1306_HEIGHT, color);
        if(SSD1306_refresh()) printf("\t[%s]:Parallel verticals:OK\n", color ? "Black" : "White");
        SCREEN_DELAY_FILL(3000, !color);


        /* Draw non overlapping rectangles */
        for(uint8_t i = 0; i < (SSD1306_HEIGHT - 1 - i); i+=2) SSD1306_draw_rectangle(i, SSD1306_WIDTH - 1 - i, i, SSD1306_HEIGHT - 1 - i, color, false);
        if(SSD1306_refresh()) printf("\t[%s]:Rectangles non-overlapping:OK\n", color ? "Black" : "White");
        SCREEN_DELAY_FILL(3000, !color);


        /* Draw vertical lines that scale should scale with height */
        for(uint8_t i = 1; i < SSD1306_WIDTH/2; i++) SSD1306_draw_vline(i, 0, i, color);
        for(uint8_t i = SSD1306_WIDTH/2; i < SSD1306_WIDTH; i++) SSD1306_draw_vline(i, 0, SSD1306_WIDTH - i, color);


        if(SSD1306_refresh()) printf("\t[%s]:Vertical line triangle:OK\n", color ? "Black" : "White");
        SCREEN_DELAY_FILL(3000, !color);

        /* Toggle foreground */
        color = !color;
    }

    /* Draw some flag patterns */
    SSD1306_fill(false);


    /* Flag pattern 1 - Cross */
    SSD1306_draw_rectangle(0, SSD1306_WIDTH - 1, SSD1306_HEIGHT/2 - 4, SSD1306_HEIGHT/2 + 3, true, true);
    SSD1306_draw_rectangle(SSD1306_WIDTH/2 - 4, SSD1306_WIDTH/2 + 3, 0, SSD1306_HEIGHT - 1, true, true);
    if(SSD1306_refresh()) printf("\tFlag variant 1:OK\n");
    SCREEN_DELAY_FILL(3000, false);


    /* Flag pattern 2 */
    for(uint8_t dist = 0, i = 0; i < 5; i++, dist += 10) SSD1306_draw_rectangle(0, SSD1306_WIDTH - 1, dist + 0, dist + 4, true, true);
    SSD1306_draw_rectangle(0, 29, 0, 24, true, true);
    SSD1306_draw_rectangle(0, 29, 10, 14, false, true);
    SSD1306_draw_rectangle(13, 17, 0, 24, false, true);
    if(SSD1306_refresh()) printf("\tFlag variant 2:OK\n");
    SCREEN_DELAY_FILL(3000, false);


    /* Flag pattern 3 */
    SSD1306_draw_rectangle(0, SSD1306_WIDTH/3 - 1, 0, SSD1306_HEIGHT - 1, true, true);
    SSD1306_draw_rectangle(2*(SSD1306_WIDTH/3), SSD1306_WIDTH - 1 , 0, SSD1306_HEIGHT - 1, true, true);
    if(SSD1306_refresh()) printf("\tFlag pattern 3:OK\n");
    SCREEN_DELAY_FILL(3000, false);
}

/* Draw intermediate 'approximate' shapes/patterns.
 * Generic lines (angle)
 * Triangles
 * Circles
 */
static void test_lcd_intermediate_patterns()
{
    /* Clear screen */
    SCREEN_DELAY_FILL(3000, false);

    /* Draw some generic lines - Should see something like a symmetric curtain */
    for(uint8_t i = 0; i < 80; i += 5) SSD1306_draw_line(0, i , 0, 70, true);
    for(uint8_t i = 0; i < 80; i += 5) SSD1306_draw_line(SSD1306_WIDTH - 1, SSD1306_WIDTH - 1 - i , 0, 70, true);
    if(SSD1306_refresh()) printf("\t[0]Generic line - (Curtains off):OK\n");
    SCREEN_DELAY_FILL(3000, false);


    /* Draw orthogonal triangles */
    for(uint8_t i = 0; i < 15; i += 2)
    {
        bool toggle = true;
        const uint8_t x0 = 0, x1 = 0, x2 = 5;
        const uint8_t y0 = 0, y1 = 5, y2 = 5;
        const uint8_t dist = 5;

        for(uint8_t j = 0; j < 10; j += 2)
        {
            if(toggle)
            {
                SSD1306_draw_triangle(x0 + i * dist, x1 + i * dist, x2 + i * dist,
                                      y0 + j * dist, y1 + j * dist, y2 + j * dist,
                                      toggle);
            }
            else
            {
                SSD1306_draw_fill_triangle(x0 + i * dist, x1 + i * dist, x2 + i * dist,
                                           y0 + j * dist, y1 + j * dist, y2 + j * dist,
                                           !toggle);
            }

            toggle = !toggle;
        }
    }
    if(SSD1306_refresh()) printf("\t[1]Drawing triangles:OK\n");
    SCREEN_DELAY_FILL(3000, false);


    /* Draw circles - Not filled */
    SSD1306_draw_circle(0, 0, 10, true);
    SSD1306_draw_circle(0, 0, 20, true);
    SSD1306_draw_circle(0, 0, 30, true);
    SSD1306_draw_circle(40, 20, 10, true);
    SSD1306_draw_circle(40, 20, 20, true);
    SSD1306_draw_circle(40, 20, 30, true);
    if(SSD1306_refresh()) printf("\t[2]Drawing circles:OK\n");
    SCREEN_DELAY_FILL(3000, false);


    /* Draw filled circles */
    SSD1306_draw_fill_circle(20, 20, 10, true);
    SSD1306_draw_fill_circle(0, 0, 5, true);
    SSD1306_draw_fill_circle(40, 40, 5, true);
    if(SSD1306_refresh()) printf("\t[3]Drawing filled circles:OK\n");
    SCREEN_DELAY_FILL(3000, false);


    /* Draw rounded rectangles */
    SSD1306_draw_round_rect(10, 40, 10, 40, true, false);
    SSD1306_draw_round_rect(20, 30, 20, 30, true, false);
    SSD1306_draw_round_rect(55, 65, 25, 35, true, true);
    if(SSD1306_refresh()) printf("\t[4]Drawing rounded rectangles:OK\n");
    SCREEN_DELAY_FILL(3000, false);

}

/* Draw and testes bitmap functionality */
static void test_lcd_bitmaps()
{
    uint32_t time;
    bool ret;

    /* Chessboard bitmap */
    const uint8_t bitmap1[4 * 8] = {    0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa,
                                        0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa,
                                        0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa,
                                        0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa
                                   };

    /* Elegant bitmap */
    const uint8_t elegant_bitmap[] =
    {
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f,
            0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f,
            0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x0f, 0x0f, 0x0f, 0x8f, 0x8f, 0x0f, 0x0f,
            0x0f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0x01, 0x00, 0xe0, 0xf0, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xf0, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x3f,
            0x3f, 0x3f, 0x1f, 0x0e, 0x00, 0x80, 0xe0, 0x7f, 0x0f, 0x07, 0xc3, 0xe3, 0xe1, 0xe3, 0x03, 0x07,
            0x1f, 0xff, 0xff, 0x03, 0x03, 0x83, 0xe3, 0xe1, 0xf3, 0xff, 0x03, 0x03, 0x03, 0xc3, 0xe3, 0xe1,
            0x83, 0x03, 0x07, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x3c, 0x3c, 0xfc, 0xfc,
            0xf8, 0xe0, 0xfc, 0xfc, 0xfc, 0xfc, 0x00, 0x00, 0xfc, 0xfc, 0xfc, 0xf8, 0xff, 0xff, 0xff, 0xfd,
            0x3c, 0x3c, 0x7c, 0xfc, 0xf8, 0xe0, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0x00, 0x00, 0x00, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xf0, 0x80, 0x00, 0x1f, 0x3f,
            0x3f, 0x3f, 0x07, 0x00, 0xc0, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
            0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
            0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0xc0, 0xc0, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xc0, 0xc0, 0xe0, 0xff, 0xff, 0x7f, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xfe, 0xfe, 0xfc, 0xfc, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfe, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xfe, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0x80, 0x00,
            0x01, 0x03, 0x03, 0x03, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03,
            0x01, 0x03, 0x03, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe,
            0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe,
            0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };

    const uint8_t bitmap2[] =
    {
        0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xF7, 0xF7, 0xFF, 0xFF, 0xFF, 0xFF,
        0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB, 0xFF,
        0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x1F, 0x1F, 0x5F, 0x1F,
        0x1F, 0x3F, 0xFF, 0x1F, 0x1F, 0x1F, 0xDF, 0x1F, 0x1F, 0x3F, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF,
        0xFF, 0xFF, 0x1F, 0x1F, 0xDF, 0xDF, 0x1F, 0x1F, 0xFF, 0xFF, 0x1F, 0x1F, 0xDF, 0xDF, 0xFF, 0xEF,
        0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB, 0xFB, 0xFF, 0xFF, 0xF3,
        0xF2, 0xE2, 0xE6, 0xE4, 0xE0, 0xF1, 0xFF, 0xE0, 0xE0, 0xE0, 0xFC, 0xFC, 0xFC, 0xFE, 0xE7, 0xE0,
        0xF2, 0xF2, 0xE0, 0xE4, 0xFF, 0xFF, 0xF0, 0xE0, 0xE7, 0xE7, 0xE1, 0xF1, 0xFF, 0xFF, 0xE0, 0xE0,
        0xE6, 0xE6, 0xEF, 0xFF, 0xFF, 0xF7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x01, 0x01, 0xFF, 0xFF, 0x01,
        0x01, 0xC3, 0x01, 0x01, 0xFF, 0xF9, 0x01, 0x01, 0x7F, 0x01, 0x01, 0xF9, 0x7F, 0x01, 0x21, 0x21,
        0x01, 0x0F, 0xFF, 0xFF, 0x01, 0x01, 0x7D, 0x7D, 0x01, 0x01, 0xFF, 0xFF, 0x01, 0x01, 0x6D, 0x6D,
        0xFF, 0xFF, 0x01, 0x01, 0xED, 0x01, 0x01, 0xFF, 0xFF, 0x31, 0x21, 0x65, 0x4D, 0x41, 0x11, 0x1F,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3E, 0x0E,
        0x0E, 0x1F, 0x3F, 0xFE, 0xFE, 0xFF, 0xFE, 0xFE, 0x7E, 0x1F, 0x3F, 0x3E, 0x1E, 0x1E, 0x7F, 0xFF,
        0xFE, 0xFE, 0xFF, 0x7F, 0xBE, 0x1E, 0x9F, 0x7F, 0xFE, 0xFE, 0xFE, 0xFE, 0x7E, 0x1E, 0x3F, 0x3F,
        0x1E, 0x7E, 0xFE, 0xFE, 0xFE, 0xFF, 0x3E, 0x0E, 0x0F, 0x1E, 0x3E, 0xFF, 0xFF, 0xFF, 0xFE, 0x7E,
        0x9E, 0x1E, 0xBE, 0x7F, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xF8, 0xF8, 0xFC, 0xF8, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFE, 0xF8, 0xBC, 0xFE,
        0xF8, 0xFA, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xFC, 0xFC, 0xFC, 0xFC, 0xFF, 0xFF, 0xFF, 0xFC,
        0xFA, 0xF8, 0xFE, 0xFE, 0xF8, 0xFE, 0xFC, 0xFF, 0xFF, 0xFF, 0xF8, 0xF8, 0xFC, 0xF8, 0xF8, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC, 0xFC, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    };

    /* Bitmap 1 test */
    START_TIMER();
    SSD1306_draw_bitmap_opt8(bitmap1, 0, 0, 8, 32);
    SSD1306_draw_bitmap(bitmap1, 17, 0, 32, 8, 1);
    time = GET_TIMER();
    ret = SSD1306_refresh();
    SCREEN_DELAY_FILL(3000, false);
    if(ret) printf("\t[1]Drawing simple bitmap twice - Time:%ld\n", time);

    /* Bitmap 1 test */
    START_TIMER();
    SSD1306_draw_bitmap(bitmap1, 17, 1, 32, 8, 2);
    time = GET_TIMER();
    ret = SSD1306_refresh();
    SCREEN_DELAY_FILL(3000, false);
    if(ret) printf("\t[1.2]Drawing simple bitmap scaled x2- Time:%ld\n", time);

    /* Bitmap 1 test */
    START_TIMER();
    SSD1306_draw_bitmap(bitmap1, 17, 1, 32, 8, 3);
    time = GET_TIMER();
    ret = SSD1306_refresh();
    SCREEN_DELAY_FILL(3000, false);
    if(ret) printf("\t[1.3]Drawing simple bitmap scaled x3- Time:%ld\n", time);

    /* Bitmap 1 test */
    START_TIMER();
    SSD1306_draw_bitmap(bitmap1, 17, 1, 32, 8, 4);
    time = GET_TIMER();
    ret = SSD1306_refresh();
    SCREEN_DELAY_FILL(3000, false);
    if(ret) printf("\t[1.4]Drawing simple bitmap scaled x4- Time:%ld\n", time);

    /* Bitmap 2 test */
    START_TIMER();
    SSD1306_draw_bitmap(elegant_bitmap, 0, 0, 84, 48, 1);
    time = GET_TIMER();
    ret = SSD1306_refresh();
    SCREEN_DELAY_FILL(3000, false);
    if(ret) printf("\t[2]Drawing elegant bitmap ;) - Time:%ld\n", time);


    /* Bitmap 2 test with opt routine */
    START_TIMER();
    SSD1306_draw_bitmap_opt8(elegant_bitmap, 0, 0, 84, 48);
    time = GET_TIMER();
    ret = SSD1306_refresh();
    SCREEN_DELAY_FILL(3000, false);
    if(ret) printf("\t[3]Drawing elegant bitmap ;) with opt routine - Time:%ld\n", time);


    /* Enable scrolling for a bit and then disable */
    bool dir = true;
    for(uint j = 0; j < 2; j++)
    {
        for(uint8_t i = 0; i < 8; i++)
        {
            ret &= SSD1306_hscroll(i, dir);
            HAL_Delay(500);
        }
        ret &= SSD1306_scroll_disable();
        dir = !dir;
    }
    if(ret) printf("\t[4]Horizontal scrolling utility test - OK\n");


    /* Enable scrolling for a bit and then disable */
    dir = true;
    for(uint j = 0; j < 2; j++)
    {
        for(uint8_t i = 0; i < 8; i++)
        {
            ret &= SSD1306_hvscroll(i, 1, dir);
            HAL_Delay(500);
        }
        ret &= SSD1306_scroll_disable();
        dir = !dir;
    }
    if(ret) printf("\t[5]Horizontal and vertical scrolling utility test - OK\n");


    /* Bitmap 3 test with opt routine */
    START_TIMER();
    SSD1306_draw_bitmap_opt8(bitmap2, 0, 0, 84, 48);
    time = GET_TIMER();
    ret = SSD1306_refresh();
    SCREEN_DELAY_FILL(3000, false);
    if(ret) printf("\t[6]Drawing space invaders bitmap with opt routine - Time:%ld\n", time);
}

/* Draw and testes printing text functionality */
static void test_lcd_text()
{
    uint32_t time, reps = 7;

    /* Empty the screen initially */
    SSD1306_fill(false);

    /* Test 1 - Scroll test */
    SSD1306_coord(0, 0);
    for(uint8_t i = 0; i < reps; i++)
    {
        START_TIMER();
        SSD1306_print_str("Scroll large text!", LARGE_FONT, false);
        time = GET_TIMER();
        SSD1306_refresh();
        SCREEN_DELAY_FILL(500, false);
    }
    printf("\t[1]Printing scroll text - Time:%ld\n", time);


    /* Test 2 - Newline test */
    SSD1306_coord(0, 0);
    for(uint8_t i = 0; i < reps; i++)
    {
        START_TIMER();
        SSD1306_print_str("Medium newline\n", MEDIUM_FONT | ALIGN_BOTTOM, false);
        time = GET_TIMER();
        SSD1306_refresh();
        SCREEN_DELAY_FILL(500, false);
    }
    printf("\t[2]Printing newline - Time:%ld\n", time);


    /* Test 3 - Inverted test */
    SSD1306_coord(0, 0);
    for(uint8_t i = 0; i < reps; i++)
    {
        START_TIMER();
        SSD1306_print_str("Inverted top centering", MEDIUM_FONT | ALIGN_UP, true);
        time = GET_TIMER();
        SSD1306_refresh();
        SCREEN_DELAY_FILL(500, false);
    }
    printf("\t[3]Printing inverted - Time:%ld\n", time);


    /* Test 4 - Small center font */
    SSD1306_coord(0, 0);
    for(uint8_t i = 0; i < reps; i++)
    {
        START_TIMER();
        SSD1306_print_str("Small center\n", SMALL_FONT | ALIGN_CENTER, false);
        time = GET_TIMER();
        SSD1306_refresh();
        SCREEN_DELAY_FILL(500, false);
    }
    printf("\t[4]Small center - Time:%ld\n", time);


    /* Test 5 - Small top font */
    SSD1306_coord(0, 0);
    for(uint8_t i = 0; i < reps; i++)
    {
        START_TIMER();
        SSD1306_print_str("Small top\n", SMALL_FONT | ALIGN_UP, false);
        time = GET_TIMER();
        SSD1306_refresh();
        SCREEN_DELAY_FILL(500, false);
    }
    printf("\t[5]Small top - Time:%ld\n", time);


    /* Test 6 - Small bottom font */
    SSD1306_coord(0, 0);
    for(uint8_t i = 0; i < reps; i++)
    {
        START_TIMER();
        SSD1306_print_str("SMALL BOTTOM\n", SMALL_FONT | ALIGN_BOTTOM, false);
        time = GET_TIMER();
        SSD1306_refresh();
        SCREEN_DELAY_FILL(500, false);
    }
    printf("\t[6]Small bottom - Time:%ld\n", time);


    /* Test 7 - Small grammar */
    SSD1306_coord(0, 0);
    START_TIMER();
    char str [2] = "1";
    for(char i = 0x20; i != 0x7f; i++)
    {
        str[0] = i;
        SSD1306_print_str(str, SMALL_FONT | ALIGN_BOTTOM, false);
    }
    time = GET_TIMER();
    SSD1306_refresh();
    SCREEN_DELAY_FILL(5000, false);
    printf("\t[7]Printing small grammar - Time:%ld\n", time);


    /* Test 8 - Medium grammar */
    SSD1306_coord(0, 0);
    START_TIMER();
    for(char i = 0x20; i != 0x7f; i++)
    {
        str[0] = i;
        SSD1306_print_str(str, MEDIUM_FONT | ALIGN_UP, false);
    }
    time = GET_TIMER();
    SSD1306_refresh();
    SCREEN_DELAY_FILL(5000, false);
    printf("\t[8]Printing medium grammar - Time:%ld\n", time);


    /* Test 9 - Large grammar */
    SSD1306_coord(0, 0);
    START_TIMER();
    for(char i = 0x20; i != 0x7f; i++)
    {
        str[0] = i;
        SSD1306_print_str(str, LARGE_FONT, false);
    }
    time = GET_TIMER();
    SSD1306_refresh();
    SCREEN_DELAY_FILL(5000, false);
    printf("\t[9]Printing large grammar - Time:%ld\n", time);


    /* Test 10 - Free print test 1 */
    START_TIMER();
    SSD1306_print_fstr("Hello", LARGE_FONT, 0, 0, 1, false);
    SSD1306_print_fstr("Hello", LARGE_FONT, 4, 13, 1, false);
    time = GET_TIMER();
    SSD1306_refresh();
    SCREEN_DELAY_FILL(5000, false);
    printf("\t[10]Printing free text - Time:%ld\n", time);


    /* Test 11 - Free print test 2 */
    START_TIMER();
    SSD1306_print_fstr("Hello", SMALL_FONT | ALIGN_BOTTOM, 0, 0, 1, false);
    SSD1306_print_fstr("Hello", SMALL_FONT | ALIGN_BOTTOM, 4, 13, 1, false);
    time = GET_TIMER();
    SSD1306_refresh();
    SCREEN_DELAY_FILL(5000, false);
    printf("\t[11]Printing free text 2 - Time:%ld\n", time);


    /* Test 12 - Free print test 3 */
    START_TIMER();
    SSD1306_print_fstr("Hello", MEDIUM_FONT | ALIGN_BOTTOM, 0, 0, 1, false);
    SSD1306_print_fstr("Hello", MEDIUM_FONT | ALIGN_BOTTOM, 4, 13, 1, false);
    SSD1306_print_fstr("Hello", MEDIUM_FONT | ALIGN_BOTTOM, 70, 20, 1, false);
    time = GET_TIMER();
    SSD1306_refresh();
    SCREEN_DELAY_FILL(5000, false);
    printf("\t[12]Printing free text 3 - Time:%ld\n", time);

    /* Test 13 - Free print test 4 */
    START_TIMER();
    SSD1306_print_fstr("Hello", MEDIUM_FONT | ALIGN_BOTTOM, 0, 0, 2, false);
    SSD1306_print_fstr("Hello", MEDIUM_FONT | ALIGN_BOTTOM, 4, 13, 3,false);
    SSD1306_print_fstr("Hello", MEDIUM_FONT | ALIGN_BOTTOM, 70, 20, 2, false);
    time = GET_TIMER();
    SSD1306_refresh();
    SCREEN_DELAY_FILL(5000, false);
    printf("\t[12]Printing free text 4 - Time:%ld\n", time);
}

/**********************************/
/*********** INIT CODE ************/
/**********************************/

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
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

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 230400; //115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief Enable DMA controller clock
  * @param None
  * @retval None
  */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

/**
  * @brief Enable CPU core clock counter
  * @param None
  * @retval None
  */
static void MX_Core_Counter_Init(void)
{
  unsigned int *DWT_LAR      = (unsigned int *) 0xE0001FB0; //address of the register
  unsigned int *SCB_DEMCR    = (unsigned int *) 0xE000EDFC; //address of the register

  // ??? Helps though //
  *DWT_LAR = 0xC5ACCE55; // unlock (CM7)

  // Enable access //
  *SCB_DEMCR |= 0x01000000;

  DWT->CYCCNT = 0;      // Reset the counter //
  DWT->CTRL |= 1 ;      // Enable the counter //
}

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

#ifdef  USE_FULL_ASSERT
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
