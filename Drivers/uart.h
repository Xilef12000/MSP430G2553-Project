
/*
 * uart.h
 *
 *  Created on:
 *      Author:
 */
#ifndef UART_H_
#define UART_H_

#include <stdio.h>
#include <stdint.h>
// ANSI/VT100 Terminal Control Escape Sequences  (<ESC> = \033 octal = 0x1b)
// <ESC>c   : Reset Device: Reset all terminal settings to default.
// <ESC>[2J : Erase Screen: Erases the screen with the background colour and moves the cursor to home.
// <ESC>[H  : cursor will move to the home position, at the upper left of the screen
//puts("\033c");                                // clear uart terminal
//puts("\033[2J\033[H");                          // clear uart terminal
#define uart_clearTerminal()    puts("\033c")
#define uart_cursorHome()       puts("\033[H")
int uart_init(uint32_t dataRate, uint32_t SMCLK_FrequencyHz);
void uart_puts(const char *pcBuf);
int uart_peek(unsigned char c);
int uart_gets(char *pcBuf, uint32_t len);

void encode(uint16_t num, char *str);
uint16_t decode(char *str);

#define puts(str)                   uart_puts(str)
#define gets(pstr)                  uart_gets(pstr)
#define fgets(pstr, value, pstream) uart_gets(pstr, value)

#endif /* UART_H_ */

