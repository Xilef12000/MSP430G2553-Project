/*
 * ssd1306_spi_lib.h
 *
 *  Created on: 29.11.2024
 *      Author: bboeck
 */

#include <msp430.h>
#include <stdint.h>     // so we can use uint8_t, uint16_t, ...
#include <stdio.h>

#ifndef DRIVERS_SSD1306_SPI_LIB_H_
#define DRIVERS_SSD1306_SPI_LIB_H_


#define SPI 0
#define I2C 1


// SSD1306 SPI Commands
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64

// Column shift
#define START_COLUMN    0x00        // default: 0x00




// Select hardware connection here
#define INTERFACE_TYPE   SPI




#define ssd1306_clear()     (ssd1306_fillDisplay(0x00))


#if (INTERFACE_TYPE == SPI)

    // GPIO Pins
    #define CS_PIN   BIT3  // Chip Select
    #define DC_PIN   BIT4  // Data/Command
    #define RST_PIN  BIT0  // Reset



    #define CS_LOW()        (P1OUT &= ~CS_PIN)      // Select Display
    #define CS_HIGH()       (P1OUT |= CS_PIN)       // Deselect Display

    #define RESET_LOW()     (P1OUT &= ~RST_PIN)     // Reset the display
    #define RESET_HIGH()    (P1OUT |= RST_PIN)      // Reset the display

    #define MODE_COMMAND()  (P1OUT &= ~DC_PIN)      // Select Command mode
    #define MODE_DATA()     (P1OUT |= DC_PIN)       // Select Data mode



#elif (INTERFACE_TYPE == I2C)
    // i2c not yet working or has to be tested
    #error ("I2C interface not yet implemented!")
#else
    #error ("invalid interface type!")
#endif





// SSD1306 Commands
#define SSD1306_SET_CONTRAST           0x81
#define SSD1306_ENTIRE_DISPLAY_RESUME  0xA4
#define SSD1306_ENTIRE_DISPLAY_ON      0xA5
#define SSD1306_NORMAL_DISPLAY         0xA6
#define SSD1306_INVERSE_DISPLAY        0xA7
#define SSD1306_DISPLAY_OFF            0xAE
#define SSD1306_DISPLAY_ON             0xAF
#define SSD1306_SET_LCOL_START_ADDRESS 0x00
#define SSD1306_SET_HCOL_START_ADDRESS 0x10
#define SSD1306_MEMORY_ADDRESS_MODE    0x20
#define SSD1306_SET_COLUMN_ADDRESS     0x21
#define SSD1306_SET_PAGE_ADDRESS       0x22
#define SSD1306_SET_PAGE_START_ADDRESS 0xB0
#define SSD1306_SET_START_LINE         0x40
#define SSD1306_SEGMENT_REMAP          0xA0
#define SSD1306_SET_MULTIPLEX_RATIO    0xA8
#define SSD1306_COM_SCAN_NORMAL        0xC0
#define SSD1306_COM_SCAN_INVERSE       0xC8
#define SSD1306_SET_DISPLAY_OFFSET     0xD3
#define SSD1306_SET_COM_PINS_CONFIG    0xDA
#define SSD1306_SET_DISPLAY_CLOCK_DIV  0xD5
#define SSD1306_SET_PRECHARGE_PERIOD   0xD9
#define SSD1306_SET_VCOM_DESELECT_LVL  0xDB
#define SSD1306_NOP                    0xE3
#define SSD1306_SET_CHARGE_PUMP        0x8D

// Function Prototypes
void spi_init(void);


void ssd1306_init(void);
void ssd1306_send_command(uint8_t command);
void ssd1306_send_data(uint8_t *data, uint16_t size);


void ssd1306_setCursor (unsigned char x, unsigned char p);
void ssd1306_drawPixel (unsigned char x, unsigned char y, unsigned char clear);
void ssd1306_fillDisplay(unsigned char param);
void ssd1306_drawImage(unsigned char x, unsigned char y, unsigned char sx,
                       unsigned char sy, const unsigned char img[],
                       unsigned char invert);
void ssd1306_draw6x8Str(unsigned char x, unsigned char p, const char str[],
                        unsigned char invert, unsigned char underline);
void ssd1306_draw12x16Str(unsigned char x, unsigned char y, const char str[],
                          unsigned char invert);


#endif /* DRIVERS_SSD1306_SPI_LIB_H_ */
