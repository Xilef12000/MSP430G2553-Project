#include <msp430.h> 

#include "Drivers/uart.h"
#include "Drivers/cs.h"

#define STRING_SIZE 8

#define DATARATE 9600     // UART baud rate

uint16_t motorSpeed = 65535;
uint16_t targedSpeed = 0;

void sendMotorSpeed();

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;       // stop watchdog timer
	
	// Init clock (setup DCO,  MCLK and SMCLK)
    CS_setDCOFrequency(1000000);                                           // => DCOCLK = 1 MHz
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);      // => MCLK = DCOCLK
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);     // => SMCLK = DCOCLK

    // Init UART interface
    uart_init(DATARATE, CS_getSMCLK());     // Set baud rate 9600 for a given SMCLK frequency

    __enable_interrupt();       // Global interrupt enable

    sendMotorSpeed();

    while(1){
        int i = uart_peek('B');
        if(i != -1){
            char ioStr[STRING_SIZE+1];
            fgets(ioStr,STRING_SIZE,stdin);
            if (i == 6) {
                targedSpeed = decode(ioStr+1);
                motorSpeed = targedSpeed;
                sendMotorSpeed();
            }
        }
    }
}

void sendMotorSpeed(){
    puts("\r\n");
    char buff[8] = "#00000A";
    encode(motorSpeed, buff);
    puts(buff);
    puts("\r\n");
}
