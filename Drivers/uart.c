/*
 * uart.c
 *
 *  Created on:
 *      Author: ChatGPT
 */
#include <msp430.h>
#include <stdint.h>     // so we can use uint8_t, uint16_t, ...
#include <stdbool.h>    // so we can use true, false
#include "uart.h"
#include "cs.h"

/*
*
*   Format:     8N1
*   P1.2/UCA0TXD:   TXD
*   P1.1/UCA0RXD:   RXD
*   IMPORTANT:  At the MSP430G2ET evaluation board:
*               For using the back-channel UART to the PC via USB:
*               Connect the jumpers for RXD and TXD perpendicular to the other jumpers
*               (HW-UART)!!!!
*
*   Note:
*   The stdlib printf, putchar, puts functions require too much space and are not
*   available at the MSP430G2553
*   Therefore simple replacements for putchar and puts are created
*
*/
#include <msp430.h>
#include <stdint.h>
#include <string.h>
// Code from ChatGPT
#define UART_TX_BUFFER_SIZE 32
#define UART_RX_BUFFER_SIZE 32
static volatile char uart_tx_buffer[UART_TX_BUFFER_SIZE];
static volatile char uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t tx_write_index = 0, tx_read_index = 0;
static volatile uint16_t rx_write_index = 0, rx_read_index = 0;
int uart_init(uint32_t dataRate, uint32_t SMCLK_FrequencyHz) {
    uint16_t brw;
    // Validiere SMCLK und Baudrate
    if ((SMCLK_FrequencyHz != 1000000 && SMCLK_FrequencyHz != 4000000 &&
         SMCLK_FrequencyHz != 8000000 && SMCLK_FrequencyHz != 16000000) ||
        (dataRate != 9600 && dataRate != 115200 )) {  //&& dataRate != 460800
        return 0; // NOK
    }
    // UART-Modul deaktivieren für Konfiguration
    UCA0CTL1 |= UCSWRST;
    // SMCLK als Taktquelle setzen
    UCA0CTL1 |= UCSSEL_2;
    // Berechnung des Baudraten-Werts
    brw = SMCLK_FrequencyHz / dataRate;
    UCA0BR0 = brw & 0xFF;    //hier ändern zu 2
    UCA0BR1 = (brw >> 8) & 0xFF; //3
    UCA0MCTL = UCBRS_0 | UCBRF_0; // Keine Modulation UCBRS_3 | UCBRF_2
    // Pins für UART konfigurieren
    P1SEL |= BIT1 | BIT2;  // P1.1 = RXD, P1.2 = TXD
    P1SEL2 |= BIT1 | BIT2;
    // UART-Modul aktivieren
    UCA0CTL1 &= ~UCSWRST;
    // RX-Interrupt aktivieren
    IE2 |= UCA0RXIE;
    return 1; // OK
}
void uart_puts(const char *pcBuf) {
    while (*pcBuf) {
        // Warten, falls Puffer voll ist
        // Zum funktionsaufruf ist buff leer, sting darf nicht mehr als 127 Zeichen haben:
        //while (next_index == tx_read_index);
        register uint16_t tx_write_index_reg = tx_write_index;
        uart_tx_buffer[tx_write_index_reg] = *pcBuf++;
        tx_write_index = (tx_write_index_reg + 1) % UART_TX_BUFFER_SIZE;
        // TX-Interrupt aktivieren
        IE2 |= UCA0TXIE;
    }
}
int uart_peek(unsigned char c) {
    uint16_t i = rx_read_index;
    while (i != rx_write_index) {
        if (uart_rx_buffer[i] == c) {
            return (i - rx_read_index + UART_RX_BUFFER_SIZE) % UART_RX_BUFFER_SIZE;
        }
        i = (i + 1) % UART_RX_BUFFER_SIZE;
    }
    return -1; // Zeichen nicht gefunden
}
int uart_gets(char *pcBuf, uint32_t len) {
    uint32_t count = 0;
    while (count < len - 1) {
        // Warten, bis Daten im RX-Puffer verfügbar sind
        //while (rx_read_index == rx_write_index);
        char c = uart_rx_buffer[rx_read_index];
        rx_read_index = (rx_read_index + 1) % UART_RX_BUFFER_SIZE;
        if (c == '\r') { // Zeilenende-Zeichen
            break;
        }
        pcBuf[count++] = c;
    }
    pcBuf[count] = '\0'; // Null-Terminator hinzufügen
    return count;
}
// RX-Interrupt Service-Routine
__attribute__((interrupt(USCIAB0RX_VECTOR))) void USCI0RX_ISR(void) {
    register uint16_t rx_write_index_reg = rx_write_index;
    register uint16_t rx_read_index_reg = rx_read_index;
    register uint16_t next_index = (rx_write_index_reg + 1) % UART_RX_BUFFER_SIZE;
    if (next_index != rx_read_index_reg) { // Pufferüberlauf verhindern
        uart_rx_buffer[rx_write_index_reg] = UCA0RXBUF;
        rx_write_index = next_index;
    }
}
// TX-Interrupt Service-Routine
__attribute__((interrupt(USCIAB0TX_VECTOR))) void USCI0TX_ISR(void) {
    register uint16_t tx_write_index_reg = tx_write_index;
    register uint16_t tx_read_index_reg = tx_read_index;
    if (tx_read_index_reg != tx_write_index_reg) {
        UCA0TXBUF = uart_tx_buffer[tx_read_index_reg];
        tx_read_index = (tx_read_index_reg + 1) % UART_TX_BUFFER_SIZE;
    } else {
        IE2 &= ~UCA0TXIE; // TX-Interrupt deaktivieren, wenn Puffer leer ist
    }
}

void encode(uint16_t num, char *str) {
    //Buffer needs to be of char[8] and contains "X00000#"
    // convert int to char array in reversed order
    // use offset to accommodate for message start and end character
    uint8_t i = 5;

    while (num > 0) {
        str[i--] = (num % 10) + '0';  // Convert last digit to a character
        num /= 10;  // Remove the last digit
    }
}
uint16_t decode(char *str) {
    // simple atoi
    uint16_t num = 0;
    uint8_t i = 0;
    while (str[i] >= '0' && str[i] <= '9') {
        num = num * 10 + (str[i] - '0');
        i++;
    }
    return num;
}
