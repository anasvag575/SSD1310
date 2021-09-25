# STM32 SSD1306 OLED Library

A library for the SSD1306 OLED display for STM32 devices (Still in progress).

### Description

This library is implemented using the HAL API, in order to have portability and ease of use across all STM32 MCUs. Most of the library is based on other implementations from these repositories (check them out):

- https://github.com/adafruit/Adafruit_SSD1306 (SSD1306 library for Arduino)
- https://github.com/adafruit/Adafruit-GFX-Library (Graphics library for Arduino)

### Hardware & screen initialization

The user has the option of using SPI in either DMA or polling mode for the transmission of the commands (or image data). The library supports all available commands for:

- Function set (Power ON, Sleep mode)
- Display control (Fill, Invert, Blank)
- Setting XY address coordinates
- Contrast and Dimming
- Temperature coefficient
- Scrolling control
- Timing control

For the initialization user has to define a screen handle, initialize the fields and make a call to the appropriate initialization routine like below:

```c
uint8_t ssd_1306_buffer[SSD1306_BUFFER_SZ];
ssd_1306_t SSD1306_handle ={ // Which GPIOs to use (Pin-port combinations) //
                            .rst_pin = RST_PIN,	
                            .rst_port = RST_PORT,
                            .ce_pin = CE_PIN,
                            .ce_port = CE_PORT,
                            .dc_pin = DC_PIN,
                            .dc_port = DC_PORT,
                            
                            // SPI HAL handle and the buffer to use //
                            .h_spi = &hspi2,
                            .buffer = ssd_1306_buffer,

                            // Set contrast and supply voltage choice //
                            .contast = SSD1306_CONTRAST_DEFAULT_NOVCC,
                            .vcs = SSD1306_SWITCHCAPVCC
                           };
                            
// Initialization routine //
bool status = SSD1306_init(&SSD1306_handle);
```

Common things to look out for (issues/tips):

- The correct GPIOs on the MCU correspond to the correct pins on the actual display
- The needed peripherals are properly initialized
  - For the GPIOs (RST, CE, DC), clock/settings should be enabled/set
  - For the SPI GPIOs(MOSI, CLK), alternate function should be set too
  - For SPI, peripheral clock/settings should be enabled/configured
  - For DMA SPI transfers, DMA should be enabled/configured before SPI
- Check if the display does not move and the pins are soldered properly

## Graphics & Text

The library supports multiple shapes, bitmap drawing and character printing. User has to call a drawing routine followed by a screen refresh, just like below:

```c
// Draw shapes on the screen //
SSD1306_draw_line(0, 70 , 0, 10, true);
SSD1306_draw_circle(0, 20 , 10, true);
SSD1306_draw_rectangle(0, 0, 20, 20, true, false);

// Bitmap drawing //
SSD1306_draw_bitmap(bitmap, 0, 0, 25, 25, 1);

// Set XY initial text coordinates and then print string //
SSD1306_coord(0, 0);
SSD1306_print_str("Hello World", LARGE_FONT, false);

// Update the display //
SSD1306_refresh();
```

For the character printing, 3 fonts are supported with different centering options when calling the printing routines.

### Using the library

Inside the **example** folder, is a small app that testes most of the functionalities of the library and provides some insight into how to enable and use the display. All of the peripheral initialization code is automatically generated by CUBEMX, so it is easy enough to reproduce for a different board.

Inside the **src** folder, is the core of the library and all the necessary header and source files. Before adding it to your own project, make sure that the correct HAL header is included (around line **12** of file **ssd_1306.h**):

```c
<ssd_1306.h> 12: #include "stm32f4xx_hal.h"	// Set your own series (F0, F1, ..) HAL header //
```

### In progress

- Doxygen documentation
- Clang formatting
- Add pins and connectivity

Feel free to inform me, in case any issue is found. Testing was performed on a NUCLEO-F401RE board.