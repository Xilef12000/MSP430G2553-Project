// Motor
#include <msp430.h> 
#include <stdint.h>

#include "Drivers/delay.h"
#include "Drivers/uart.h"
#include "Drivers/cs.h"

#define STRING_SIZE 8 // UART message size

#define DATARATE 9600     // UART baud rate

uint16_t motorSpeed = 65535;
uint16_t targedSpeed = 0;

#define TIME_PERIODE    65535
#define ON_TIME_0           0
#define ON_TIME_40          20000
#define ON_TIME_70          35000
#define ON_TIME_100         50000

#define SMOOTHING_FACTOR 16  // Strength of the digital low-pass filter

void        portInit(void); // initialize counter input pin

volatile uint16_t        capturedCount;
volatile uint16_t        lastCount;
volatile uint16_t        interval;
volatile uint32_t        smoothed_interval_x128;

void sendSpeed(uint16_t speed, char c);

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;       // stop watchdog timer

    // Init clock (setup DCO,  MCLK and SMCLK)
    CS_setDCOFrequency(1000000);                                           // => DCOCLK = 1 MHz
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);      // => MCLK = DCOCLK
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);     // => SMCLK = DCOCLK

    // Init ports
    portInit();

    // Init UART interface
    uart_init(DATARATE, CS_getSMCLK());     // Set baud rate 9600 for a given SMCLK frequency

    // Interval timer configuration
    TA0CCR0 = (49999);                  // 400ms
    TA0CTL = TASSEL_2 + MC_1 + ID_3;    // SMCLK, upmode, div 8
    TA0CCTL0 = CCIE;                    // enable interrupt

    // Configure the TA1 CCR0 to do input capture
    //   (look for "TACCTLx" (Capture/Compare Control Register) in the Family Guide)
    TA1CCTL2 = CAP | CM_1 | CCIE | SCS | CCIS_1;// Capture Mode:                            CAP
                                                // Capture Input: CCI0A:                    CCIS_0
                                                // Capture on positive (rising) Edge:       CM_1
                                                // Capture/compare interrupt enable:        CCIE
                                                // Synchronize capture source:              SCS

    // Setup and start Timer TA1
    //TA1CTL |= TASSEL_2 | MC_1 | ID_0;   // PWM Testing
    //TA1CCR0  = TIME_PERIODE-1;              // PWM Period

    TA1CTL |= TASSEL_2 | MC_2 | TACLR | ID_0;   // TASSEL_2:    SMCLK
                                                // Cont Mode:   MC_2
                                                // TimerA clear: TACLR
                                                // Input divider: ID_0: /1

    // Timer A0 CaptureCompareUnit 1
    TA1CCTL1 = OUTMOD_7;                    // CCR1 reset/set
    TA1CCR1  = ON_TIME_40;                 // CCR1 PWM duty cycle


    __enable_interrupt();       // Global interrupt enable

    sendSpeed(motorSpeed, 'A');

    while(1){
        int msg_complete = uart_peek('B');
        if(msg_complete != -1){
            char ioStr[STRING_SIZE+1];
            fgets(ioStr,STRING_SIZE,stdin);
            if (msg_complete == 6) {
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

// Setup I/O ports
void portInit(void){

    // Configure Port Pins

    P1DIR  = 0x00;                  // Set all P1 pins as inputs
    P2DIR  = 0x00;                  // Set all P2 pins as inputs

    // P2.0/TA1.0 Input Capture
    P2SEL  |= BIT5;                 // TA1.0 option select
    P2SEL2 &= ~BIT5;                // TA1.0 option select

    // MOTOR PWM
    P2DIR |= BIT1;                          // P1.6 output
    P2SEL |= BIT1;                          // P1.6 for TA0.1 output
    P2SEL2 &= ~BIT1;                             // Select default function for P1.2

    // TESTPIN_INIT();

    return;
}

// Timer0 A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void){
    sendSpeed(motorSpeed, 'A');
}

// Timer A0 interrupt service routine
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer_A1 (void)
{

    //TESTPIN1_HIGH();                // Use Testpin to measure the run time of the ISR with an oscilloscope

    capturedCount   = TA1CCR2;      // Readout the captured timer value

    interval        = capturedCount - lastCount;


    // We are calculating with integer values. Therefore in a division by SMOOTHING_FACTOR the decimal places are removed
    // Idea for a solution: Calculate always with value*128
    // (The use of floats is much more resource-intensive)
    smoothed_interval_x128 = (smoothed_interval_x128 * (SMOOTHING_FACTOR - 1) + (((uint32_t)interval)*128)) / SMOOTHING_FACTOR;

    motorSpeed = (uint16_t)(smoothed_interval_x128/128);


    lastCount       = capturedCount;

    TA1CCTL2 &= ~CCIFG;   // reset IFG
    //TESTPIN1_LOW();                 // Use Testpin to measure the run time of the ISR with an oscilloscope

}
