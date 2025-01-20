// Display
#include <msp430.h> 

#include "Drivers/uart.h"
#include "Drivers/cs.h"
#include <Drivers/ssd1306_lib.h>

#define STRING_SIZE 8 // max length of one Message

#define DATARATE 9600     // UART baud rate

uint16_t motorSpeed = 65535;
uint16_t targedSpeed = 0;
#define INC 4096 // increment/decrement step width

// BIT for UP and DOWN Button in Port2
#define UP BIT3
#define DOWN BIT4

void sendSpeed(uint16_t speed, char c);
void updateDisplay();
void itoa(uint16_t num, char *str);

uint16_t timerDivCounter = 0;
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;       // stop watchdog timer
	
	// Init clock (setup DCO,  MCLK and SMCLK)
    CS_setDCOFrequency(16000000);                                           // => DCOCLK = 16 MHz
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);      // => MCLK = DCOCLK
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);     // => SMCLK = DCOCLK

    // Init UART interface
    uart_init(DATARATE, CS_getSMCLK());     // Set baud rate 9600 for a given SMCLK frequency

    spi_init();               // Initialize SPI
    ssd1306_init();           // Initialize SSD1306
    ssd1306_clear();          // Clear the screen

    // Timer for sending targedSpeed every x ms
    TA1CCR0 = (UINT16_MAX);             // x ms
    TA1CTL = TASSEL_2 + MC_1 + ID_3;    // SMCLK, upmode, div 8
    TA1CCTL0 = CCIE;                    // enable interrupt

    // P2.1 and P2.2 for incement and decrement
    P2DIR &= ~(DOWN + UP);    // input
    P2REN |= DOWN + UP;       // enable internal pull
    P2OUT |= DOWN + UP;       // pullup
    P2IE |= DOWN + UP;        // enable interrupt
    P2IES |= DOWN + UP;       // falling edge
    P2IFG = 0x00;               // clear interrupt

    __enable_interrupt();       // Global interrupt enable

    sendSpeed(targedSpeed, 'B');
    updateDisplay();

    while(1){
        int i = uart_peek('A'); // get Position of 'A' in rxBuffer
        if(i != -1){ // if 'A' in rxBuffer
            char ioStr[STRING_SIZE+1]; // create local Buffer
            fgets(ioStr,STRING_SIZE,stdin); // copy string of size STRING_SIZE from rxBuffer to local Buffer
            if (i == 6) { // make shure message is of correct length
                motorSpeed = decode(ioStr+1); // convert value in Message to uint16
                sendSpeed(motorSpeed, 'D'); // reply with received value for debugging
                //targedSpeed = motorSpeed; // WIP: just for testing
                updateDisplay();
            }
        }
    }
}

void sendSpeed(uint16_t speed, char c){
    char buff[8] = "#00000Z";
    buff[6] = c; // message Buffer
    // A Motorspeed from Motor
    // B TargedSpeed from Display
    // C TargedSpeed from Motor
    // D Motorspeed from Display
    // Z placeholder
    encode(speed, buff); // character offseted itoa
    puts(buff);
}

void updateDisplay() {
    char buffd[6] = "00000"; // itoa for targedSpeed
    itoa(targedSpeed, buffd);
    ssd1306_draw6x8Str(0, 0, "Soll:", 1, 0);
    ssd1306_draw12x16Str(12,  12, buffd, 0);
    char buffe[6] = "00000"; // itoa for motorSpeed
    itoa(motorSpeed, buffe);
    ssd1306_draw6x8Str(0, 4, "Ist:", 1, 0);
    ssd1306_draw12x16Str(12,  44, buffe, 0);
}

void itoa(uint16_t num, char *str) { // itoa for displaying values
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
    // Software Divider to reduce sending Frequency
    if (timerDivCounter >= 16){
        sendSpeed(targedSpeed, 'B');
        timerDivCounter = 0;
    }
    timerDivCounter++;
}

// Port2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port2 (void){
    if(P2IFG & DOWN){ // if button DOWN
        // if INC can't be subtracted from targedSpeed (without overflow), set to zero, otherwise subtract
        targedSpeed = targedSpeed < INC ? 0 : targedSpeed - INC;
    }
    if(P2IFG & UP){ // if button UP
        // if INC can't be added to targedSpeed (without overflow), set to UINT16_MAX, otherwise add
        targedSpeed = targedSpeed > UINT16_MAX - INC ? UINT16_MAX : targedSpeed + INC;
    }

   P2IFG = 0x00;    // clear interrupt
   updateDisplay();
}
