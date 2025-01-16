// Display
#include <msp430.h> 

#include "Drivers/uart.h"
#include "Drivers/cs.h"
#include <Drivers/ssd1306_lib.h>

#define STRING_SIZE 8

#define DATARATE 9600     // UART baud rate

uint16_t motorSpeed = 65535;
uint16_t targedSpeed = 0;

void sendSpeed(uint16_t speed, char c);
void updateDisplay();
void itoa(uint16_t num, char *str);

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;       // stop watchdog timer
	
	// Init clock (setup DCO,  MCLK and SMCLK)
    CS_setDCOFrequency(16000000);                                           // => DCOCLK = 1 MHz
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);      // => MCLK = DCOCLK
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);     // => SMCLK = DCOCLK

    // Init UART interface
    uart_init(DATARATE, CS_getSMCLK());     // Set baud rate 9600 for a given SMCLK frequency

    spi_init();               // Initialize SPI
    ssd1306_init();           // Initialize SSD1306
    ssd1306_clear();          // Clear the screen
    //ssd1306_draw6x8Str(0,  0, "Test4", 1, 0);
    //ssd1306_draw12x16Str(60,  40, "Test3", 0);


    TA1CCR0 = (49999);                  // 400ms
    TA1CTL = TASSEL_2 + MC_1 + ID_3;    // SMCLK, upmode, div 8
    TA1CCTL0 = CCIE;                    // enable interrupt

    P2DIR &= ~(BIT1 + BIT2);    // input
    P2REN |= BIT1 + BIT2;       // enable internal pull
    P2OUT |= BIT1 + BIT2;       // pullup
    P2IE |= BIT1 + BIT2;        // enable interrupt
    P2IES |= BIT1 + BIT2;       // falling edge
    P2IFG = 0x00;               // clear interrupt

    __enable_interrupt();       // Global interrupt enable

    sendSpeed(targedSpeed, 'B');
    updateDisplay();

    while(1){
        int i = uart_peek('A');
        if(i != -1){
            char ioStr[STRING_SIZE+1];
            fgets(ioStr,STRING_SIZE,stdin);
            if (i == 6) {
                motorSpeed = decode(ioStr+1);
                sendSpeed(motorSpeed, 'D');
                targedSpeed = motorSpeed; // WIP: just for testing
                updateDisplay();
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

void updateDisplay() {
    char buffd[6] = "00000";
    itoa(targedSpeed, buffd);
    ssd1306_draw6x8Str(0, 0, "Soll:", 1, 0);
    ssd1306_draw12x16Str(12,  12, buffd, 0);
    char buffe[6] = "00000";
    itoa(motorSpeed, buffe);
    ssd1306_draw6x8Str(0, 4, "Ist:", 1, 0);
    ssd1306_draw12x16Str(12,  44, buffe, 0);
}

void itoa(uint16_t num, char *str) {
    //Buffer needs to be of type char[6]
    uint8_t i = 4;

    while (num > 0) {
        str[i--] = (num % 10) + '0';  // Convert last digit to a character
        num /= 10;  // Remove the last digit
    }
}

// Timer1 A0 interrupt service routine
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A (void){
    sendSpeed(targedSpeed, 'B');
}

// Port2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port2 (void){
    if(P2IFG & BIT1 && targedSpeed > 0){
        targedSpeed--;
    }
    if(P2IFG & BIT2 && targedSpeed < UINT16_MAX){
        targedSpeed++;
    }

   P2IFG = 0x00;    // clear interrupt
   updateDisplay();
}
