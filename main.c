// Motor
#include <msp430.h> 

#include "Drivers/uart.h"
#include "Drivers/cs.h"

#define STRING_SIZE 8

#define DATARATE 9600     // UART baud rate

uint16_t motorSpeed = 65535;
uint16_t targedSpeed = 0;

void sendSpeed(uint16_t speed, char c);

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;       // stop watchdog timer

    // Init clock (setup DCO,  MCLK and SMCLK)
    CS_setDCOFrequency(1000000);                                           // => DCOCLK = 1 MHz
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);      // => MCLK = DCOCLK
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);     // => SMCLK = DCOCLK

    // Init UART interface
    uart_init(DATARATE, CS_getSMCLK());     // Set baud rate 9600 for a given SMCLK frequency

    TA0CCR0 = (49999);                  // 400ms
    TA0CTL = TASSEL_2 + MC_1 + ID_3;    // SMCLK, upmode, div 8
    TA0CCTL0 = CCIE;                    // enable interrupt

    __enable_interrupt();       // Global interrupt enable

    sendSpeed(motorSpeed, 'A');

    while(1){
        int i = uart_peek('B');
        if(i != -1){
            char ioStr[STRING_SIZE+1];
            fgets(ioStr,STRING_SIZE,stdin);
            if (i == 6) {
                targedSpeed = decode(ioStr+1);
                sendSpeed(targedSpeed, 'C');
                motorSpeed = targedSpeed; // WIP: just for testing
            }
        }
    }
}

void sendSpeed(uint16_t speed, char c){
    char buff[8] = "#00000Z";
    buff[6] = c;
    // A Motorspeed from Motor
    // B TargedSpeed from Display
    // C TargedSpeed from Motor
    // D Motorspeed from Display
    // Z placeholder
    encode(speed, buff);
    puts(buff);
}

// Timer0 A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void){
    sendSpeed(motorSpeed, 'A');
}
