//******************************
// MPS Project - EIB3 - WiSe 24/25
//
// Niklas Bachman / 284301
// Manuel König / 310284
//
// Motor Controller
//******************************
#include <msp430.h> 
#include <stdint.h>

#include "Drivers/uart.h"
#include "Drivers/cs.h"

#define STRING_SIZE 8// UART message size

#define DATARATE 9600     // UART baud rate

volatile uint16_t motorSpeed = 65535;
volatile uint16_t targedSpeed = 1000;

#define TIME_PERIODE    65535

// PID variables
#define Kp 48
#define Ki 2
#define Kd 1
#define input_div 24 //map to achievable motor speeds

volatile uint16_t e = 0; // difference target to actual speed
volatile int32_t e_ld = 0; // used for last and diff e
volatile uint16_t e_sum = 0; // sum of differences target to actual speed
volatile uint32_t y = 0; // calculated PWM value

#define unit16_max 65535
#define e_max (unit16_max / Kp)
#define e_sum_max (unit16_max / Ki)
#define e_ld_max (unit16_max / Kd)

volatile char timer_div_counter = 0;

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
    CS_setDCOFrequency(8000000);                                           // => DCOCLK = 1 MHz
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

    TA1CTL |= TASSEL_2 | MC_1 | TACLR | ID_0;   // TASSEL_2:    SMCLK
                                                // Up Mode:   MC_2
                                                // TimerA clear: TACLR
                                                // Input divider: ID_0: /1
    TA1CCR0  = unit16_max-1;                 // CCR1 PWM duty cycle


    // Timer A CaptureCompareUnit 1
    TA1CCTL1 = OUTMOD_7;                    // CCR1 reset/set
    TA1CCR1  = 20000;                 // initial CCR1 PWM duty cycle


    __enable_interrupt();       // Global interrupt enable

    sendSpeed(motorSpeed, 'A');

    while(1){
        int msg_complete = uart_peek('B');
        if(msg_complete != -1){
            char ioStr[STRING_SIZE+1];
            fgets(ioStr,STRING_SIZE,stdin);
            if (msg_complete == 6) {
                targedSpeed = decode(ioStr+1) / input_div;
                sendSpeed(targedSpeed, 'C');
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
    if(timer_div_counter == 7){
        sendSpeed(motorSpeed, 'A'); // TA1CCR1, motorSpeed, e
        timer_div_counter = 0;
    }
    timer_div_counter++;
}

// Timer A1 interrupt service routine
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer_A1 (void)
{
    if(TA1CCTL2 & CCIFG){

        capturedCount   = TA1CCR2;      // Readout the captured timer value

        // Timer overflow handling
        if(capturedCount > lastCount){
            interval        = capturedCount - lastCount;
        }
        else{
            interval = (unit16_max - lastCount) + capturedCount;
        }
        lastCount       = capturedCount;



        // reversed relation with fraction
        motorSpeed = 8e6 / interval; // pulses per second

        // regulator code
        // limit variables
        if(targedSpeed > motorSpeed){       // e > 0
            e = targedSpeed - motorSpeed;
            e_sum = e_sum + e;
            e_ld = e - e_ld;
        }
        else{                               // e < 0
            e = motorSpeed - targedSpeed;
            e_ld = -e - e_ld;

            if(e_sum > e){
                e_sum = e_sum - e;
            }
            else{
                e_sum = 0;
            }
            e = 0;
        }

        if(e > e_max) {e = e_max;}                          // prevent overflow on multiplication with Kd
        if(e_sum > e_sum_max) {e_sum = e_sum_max;}          // prevent overflow on multiplication with Ki
        if(e_ld > e_ld_max) {e_ld = e_ld_max;}              // prevent overflow on multiplication with Kd
        if(e_ld < -e_ld_max) {e_ld = -e_ld_max;}

        // calculate output
        y = (uint32_t)(Kp * e) + (uint32_t)(Ki * e_sum);

        /*
        // no stable output
        e_ld = (Kd * e_ld);
        if(y < -e_ld) {y = 0;}
        else {y = y + e_ld;}
        */


        if(y > unit16_max) {y = unit16_max;}

        e_ld = e;

        // write PWM value
        TA1CCR1 = (uint16_t)y;

    }

    TA1CCTL2 &= ~CCIFG;   // reset IFG

}

