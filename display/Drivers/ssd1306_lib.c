/*
 * ssd1306_spi_lib.c
 *
 *  Created on: 29.11.2024
 *      Author: bboeck
 */

// i2c not yet working or has to be tested


#include "ssd1306_lib.h"
#include "font6x8.h"
#include "font12x16.h"



// Small buffer for one page (128 bytes)
static uint8_t page_buffer[SSD1306_WIDTH];



void ssd1306_init(void) {
    RESET_LOW(); // Reset the display
    __delay_cycles(1000);
    RESET_HIGH();

    CS_LOW(); // Select the display
    ssd1306_send_command(SSD1306_DISPLAY_OFF);  // Turn display off
    ssd1306_send_command(SSD1306_SET_DISPLAY_CLOCK_DIV);                 // Set display clock
    ssd1306_send_command(0x80);                 // Suggested clock ratio
    ssd1306_send_command(0xA8);                 // Set multiplex
    ssd1306_send_command(SSD1306_HEIGHT - 1);
    ssd1306_send_command(SSD1306_SET_DISPLAY_OFFSET);                 // Set display offset
    ssd1306_send_command(0x00);                 // No offset
    ssd1306_send_command(SSD1306_SET_START_LINE);                 // Set start line
    ssd1306_send_command(0x8D);                 // Enable charge pump
    ssd1306_send_command(0x14);


    ssd1306_send_command(SSD1306_SET_COLUMN_ADDRESS);                 // Set column address
    ssd1306_send_command(START_COLUMN);                 // Start at column START_COLUMN
    ssd1306_send_command(SSD1306_WIDTH - 1);    // End at column 127

    ssd1306_send_command(SSD1306_SET_DISPLAY_OFFSET);                 // Set display offset
    ssd1306_send_command(0x00);                 // No vertical offset (default)

    ssd1306_send_command(0x2E);                 // Deactivate scrolling

    //ssd1306_send_command(0xA1);                   // Set segment remap (reverse column order)
    ssd1306_send_command(SSD1306_SEGMENT_REMAP);                 // Default segment remap



    ssd1306_send_command(SSD1306_MEMORY_ADDRESS_MODE);    // Set memory addressing mode
    ssd1306_send_command(0x00);    // Horizontal addressing mode
    //ssd1306_send_command(0xA1);    // Set segment remap (reverse column order)
    //ssd1306_send_command(SSD1306_COM_SCAN_INVERSE);    // Set COM output scan direction (reverse row order)
    ssd1306_send_command(SSD1306_SET_COM_PINS_CONFIG);    // Set COM pins hardware configuration
    ssd1306_send_command(0x12);    // Default value
    ssd1306_send_command(SSD1306_SET_CONTRAST);    // Set contrast control
    ssd1306_send_command(0x8F);    // Default contrast
    ssd1306_send_command(SSD1306_SET_PRECHARGE_PERIOD);    // Set pre-charge period
    ssd1306_send_command(0xF1);    // Default value
    ssd1306_send_command(SSD1306_SET_VCOM_DESELECT_LVL);    // Set VCOMH deselect level
    ssd1306_send_command(0x40);    // Default value
    ssd1306_send_command(SSD1306_ENTIRE_DISPLAY_RESUME);    // Entire display ON (resume RAM content)

    ssd1306_send_command(SSD1306_NORMAL_DISPLAY);      // Normal display
    ssd1306_send_command(SSD1306_DISPLAY_ON);          // Turn display on
    CS_HIGH();  // Deselect the display
}



#if (INTERFACE_TYPE == SPI)


    void ssd1306_send_command(uint8_t command) {
        MODE_COMMAND();     // Command mode
        CS_LOW();           // Select the display

        while (!(IFG2 & UCB0TXIFG)); // Wait for TX buffer ready

        UCB0TXBUF = command;         // Send command

        while (UCB0STAT & UCBUSY);   // Wait for transmission to complete

        CS_HIGH();          // Deselect the display
    }



    void ssd1306_send_data(uint8_t *data, uint16_t size) {
        MODE_DATA();    // Data mode
        CS_LOW();       // Select the display

        uint16_t i = 0;
        while(i < size) {
        //for (uint16_t i = 0; i < size; i++) {
            while (!(IFG2 & UCB0TXIFG)); // Wait for TX buffer ready

            UCB0TXBUF = data[i];         // Send data byte

            while (UCB0STAT & UCBUSY);   // Wait for transmission to complete
            i++;
        }

        CS_HIGH();      // Deselect the display
    }




    void ssd1306_drawPixel(uint8_t x, uint8_t y, uint8_t color) {
        if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
            return; // Out of bounds
        }

        uint8_t bit_position = y % 8; // Calculate the bit position within the page


        // Read-modify-write to update the pixel
        uint8_t current_byte = 0; // No read-back support in SSD1306, maintain state externally if needed
        if (color) {
            current_byte |= (1 << bit_position); // Set the bit
        } else {
            current_byte &= ~(1 << bit_position); // Clear the bit
        }


        ssd1306_setCursor(x, y >> 3);
        ssd1306_send_data(&current_byte, 1);  // Send the updated byte
    }




    void ssd1306_fillDisplay(unsigned char param) {
      unsigned char page, x;


      for (x = 0; x < 128; x++) {
          page_buffer[x] = param;
      }

      for (page = 0; page < (SSD1306_HEIGHT / 8); page++) {
            ssd1306_setCursor(0, page);
            ssd1306_send_data(page_buffer, SSD1306_WIDTH);
      }

    }


    void ssd1306_drawImage(unsigned char x, unsigned char y, unsigned char sx,
                           unsigned char sy, const unsigned char img[],
                           unsigned char invert) {
      unsigned int j, t;
      unsigned char i, p, p0, p1, n, n1, b;

      if (((x + sx) > SSD1306_WIDTH) || ((y + sy) > SSD1306_HEIGHT) ||
          (sx == 0) || (sy == 0)) return;

      // Total bytes of the image array
      if (sy % 8)
        t = (sy / 8 + 1) * sx;
      else
        t = (sy / 8) * sx;
      p0 = y / 8;                 // first page index
      p1 = (y + sy - 1) / 8;      // last page index
      n = y % 8;                  // offset form begin of page

      n1 = (y + sy) % 8;
      if (n1) n1 = 8 - n1;

      j = 0;                      // bytes counter [0..t], or [0..(t+sx)]

      for (p=p0; p<(p1+1); p++) {
          ssd1306_setCursor(x, p);
        for (i=x; i<(x+sx); i++) {
          if (p == p0) {
            b = (img[j] << n) & 0xFF;
          } else if ((p == p1) && (j >= t)) {
            b = (img[j - sx] >> n1) & 0xFF;
          } else {
            b = ((img[j - sx] >> (8 - n)) & 0xFF) | ((img[j] << n) & 0xFF);
          };
          if (invert)
              page_buffer[i - x] = ~b;
          else
              page_buffer[i - x] = b;
          j++;
        }
        ssd1306_send_data(page_buffer, sx); // send the buf to display
      }

    }


    void ssd1306_draw6x8Str(unsigned char x, unsigned char p, const char str[],
                            unsigned char invert, unsigned char underline) {
      unsigned char i, j, b, buf[FONT6X8_WIDTH];
      unsigned int c;

      i = 0;

      while (str[i] != '\0') {
        if (str[i] > 191)
          c = (str[i] - 64) * FONT6X8_WIDTH;
        else
          c = str[i] * FONT6X8_WIDTH;
        if (x > (SSD1306_WIDTH - FONT6X8_WIDTH - 1))
        {
          x = 0;
          p++;
        };
        if (p > 7) p = 0;
        ssd1306_setCursor(x, p);
        for (j = 0; j < FONT6X8_WIDTH; j++)
        {
          if (underline)
            b = font6x8[(unsigned int)(c + j)] | 0x80;
          else
            b = font6x8[(unsigned int)(c + j)];
          if (invert)
            buf[j] = b;
          else
            buf[j] = ~b;
        };
        ssd1306_send_data(buf, FONT6X8_WIDTH); // send the buf to display
        x += FONT6X8_WIDTH;
        i++;
      };
    }



#elif (INTERFACE_TYPE == I2C)




    #define SSD1306_COMMAND_MODE           0x00
    #define SSD1306_DATA_MODE              0x40



    void sendCommand (unsigned char command) {
        data[0] = 0x00;
        data[1] = command;

        sendData(data, 2);
    }
    //******************************************************************************************************************************************
    void ssd1306_sendData (unsigned char *params, unsigned char flag) {
        TI_transmit_field = params;
        i2c_init ();
        i2c_transmit (TI_transmit_field,flag);
    }


    //******************************************************************************************************************************************

    void ssd1306_drawPixel(unsigned char x, unsigned char y, unsigned char clear) {

        if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT)) return;
        ssd1306_setCursor(x, y >> 3);
        data[0] = SSD1306_DATA_MODE;
        if (clear)
            data[1] = (1 << (y & 7));
        else
            data[1] = ~(1 << (y & 7));

        ssd1306_send_data(data, 2);
    }




    // ******************************************************************************************************************************************

    void ssd1306_fillDisplay(unsigned char param) {
      unsigned char page, x;

      dataBuffer = malloc(129);
      dataBuffer[0] = SSD1306_DATA_MODE;
      for (page = 0; page < 8; page++) {
          ssd1306_setCursor(0, page);
        for (x = 0; x < 128; x++) {
            dataBuffer[x + 1] = param;
        }
        ssd1306_send_data(dataBuffer, 129);
      }
      free(dataBuffer);
    }



    void ssd1306_drawImage(unsigned char x, unsigned char y, unsigned char sx,
                           unsigned char sy, const unsigned char img[],
                           unsigned char invert) {
      unsigned int j, t;
      unsigned char i, p, p0, p1, n, n1, b;

      if (((x + sx) > SSD1306_WIDTH) || ((y + sy) > SSD1306_HEIGHT) ||
          (sx == 0) || (sy == 0)) return;

      // Total bytes of the image array
      if (sy % 8)
        t = (sy / 8 + 1) * sx;
      else
        t = (sy / 8) * sx;
      p0 = y / 8;                 // first page index
      p1 = (y + sy - 1) / 8;      // last page index
      n = y % 8;                  // offset form begin of page

      n1 = (y + sy) % 8;
      if (n1) n1 = 8 - n1;

      j = 0;                      // bytes counter [0..t], or [0..(t+sx)]
      dataBuffer = malloc(sx + 1);       // allocate memory for the buf
      dataBuffer[0] = SSD1306_DATA_MODE; // fist item "send data mode"
      for (p=p0; p<(p1+1); p++) {
          ssd1306_setCursor(x, p);
        for (i=x; i<(x+sx); i++) {
          if (p == p0) {
            b = (img[j] << n) & 0xFF;
          } else if ((p == p1) && (j >= t)) {
            b = (img[j - sx] >> n1) & 0xFF;
          } else {
            b = ((img[j - sx] >> (8 - n)) & 0xFF) | ((img[j] << n) & 0xFF);
          };
          if (invert)
              dataBuffer[i - x + 1] = ~b;
          else
              dataBuffer[i - x + 1] = b;
          j++;
        }
        ssd1306_send_data(dataBuffer, sx + 1); // send the buf to display
      }
      free(dataBuffer);
    }



    // ******************************************************************************************************************************************
    void ssd1306_draw6x8Str(unsigned char x, unsigned char p, const char str[],
                            unsigned char invert, unsigned char underline) {
      unsigned char i, j, b, buf[FONT6X8_WIDTH + 1];
      unsigned int c;

      i = 0;
      buf[0] = SSD1306_DATA_MODE; // fist item "send data mode"
      while (str[i] != '\0') {
        if (str[i] > 191)
          c = (str[i] - 64) * FONT6X8_WIDTH;
        else
          c = str[i] * FONT6X8_WIDTH;
        if (x > (SSD1306_WIDTH - FONT6X8_WIDTH - 1))
        {
          x = 0;
          p++;
        };
        if (p > 7) p = 0;
        ssd1306_setCursor(x, p);
        for (j = 0; j < FONT6X8_WIDTH; j++)
        {
          if (underline)
            b = font6x8[(unsigned int)(c + j)] | 0x80;
          else
            b = font6x8[(unsigned int)(c + j)];
          if (invert)
            buf[j + 1] = b;
          else
            buf[j + 1] = ~b;
        };
        ssd1306_send_data(buf, FONT6X8_WIDTH + 1); // send the buf to display
        x += FONT6X8_WIDTH;
        i++;
      };
    }



#endif


void ssd1306_setCursor (unsigned char x, unsigned char p) {

    x = x + 2;

    ssd1306_send_command(SSD1306_SET_LCOL_START_ADDRESS | (x & 0x0F));
    ssd1306_send_command(SSD1306_SET_HCOL_START_ADDRESS | (x >> 4));
    ssd1306_send_command(SSD1306_SET_PAGE_START_ADDRESS | p);
}


void ssd1306_draw12x16Str(unsigned char x, unsigned char y, const char str[],
                          unsigned char invert) {
  unsigned char i;
  unsigned int c;

  i = 0;
  while (str[i] != '\0') {
    if (str[i] > 191)
      c = (str[i] - 64) * FONT12X16_WIDTH * 2;
    else
      c = str[i] * FONT12X16_WIDTH * 2;
    ssd1306_drawImage(x, y, 12, 16, (unsigned char *) &font12x16[c], invert);
    i++;
    x += 12;
  };
}

void spi_init(void) {
    P1SEL  |= BIT5 + BIT7;   // P1.5 = SCLK, P1.7 = SIMO (MOSI)
    P1SEL2 |= BIT5 + BIT7;
    P1DIR  |= BIT0 + BIT3 + BIT4;   // Set CS, DC, RST as outputs

    UCB0CTL1 |= UCSWRST;    // Put USCI in reset
    UCB0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC; // SPI: 3-pin, master, MSB first
    UCB0CTL1 = UCSSEL_2 + UCSWRST; // SMCLK as clock source
    UCB0BR0 = 2;            // Set clock divider (SMCLK / 2)
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST;   // Release USCI for operation

    CS_HIGH();              // Set CS high (inactive)
    RESET_HIGH();           // Set RST high
}




