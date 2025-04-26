//*****************************************************************************
//
// cs.c - Driver for the cs Module.
//
//*****************************************************************************


#include "cs.h"

#include <stdint.h>
#include <msp430.h>


static uint32_t clock_dco_frequency = CLOCK_DCO_1MHZ;   // dco clock in [Hz]
static uint32_t clock_mclk_frequency;                   // mclk clock in [Hz]
static uint32_t clock_smclk_frequency;                  // smclk clock in [Hz]




//******************************************************************************
//
//! Sets the DCO to the given frequency (Hz).
//! Valid values: 1000000, 8000000, 12000000, 16000000
//! In case of an invalid parameter select 1000000Hz
//!
//! \return None
//
//******************************************************************************
extern void CS_setDCOFrequency(uint32_t dcoFrequency)
{
    // Setup clock (internal osc, 1MHz)

    switch(dcoFrequency){

        default:
        case CLOCK_DCO_1MHZ:
            // setup UART using 1MHz internal oscillator
            if (CALBC1_1MHZ==0xFF)                    // If calibration constant erased
            {
              while(1);                               // do not load, trap CPU!!
            }

            DCOCTL = 0;                               // Select lowest DCOx and MODx settings
            BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
            DCOCTL = CALDCO_1MHZ;

            dcoFrequency = CLOCK_DCO_1MHZ;
            break;

        case CLOCK_DCO_8MHZ:
            // setup UART using 1MHz internal oscillator
            if (CALBC1_8MHZ==0xFF)                    // If calibration constant erased
            {
              while(1);                               // do not load, trap CPU!!
            }

            DCOCTL = 0;                               // Select lowest DCOx and MODx settings
            BCSCTL1 = CALBC1_8MHZ;                    // Set DCO
            DCOCTL = CALDCO_8MHZ;

            break;

        case CLOCK_DCO_12MHZ:
            // setup UART using 1MHz internal oscillator
            if (CALBC1_12MHZ==0xFF)                    // If calibration constant erased
            {
              while(1);                               // do not load, trap CPU!!
            }

            DCOCTL = 0;                               // Select lowest DCOx and MODx settings
            BCSCTL1 = CALBC1_12MHZ;                    // Set DCO
            DCOCTL = CALDCO_12MHZ;

            break;

        case CLOCK_DCO_16MHZ:
            // setup UART using 1MHz internal oscillator
            if (CALBC1_16MHZ==0xFF)                    // If calibration constant erased
            {
              while(1);                               // do not load, trap CPU!!
            }

            DCOCTL = 0;                               // Select lowest DCOx and MODx settings
            BCSCTL1 = CALBC1_16MHZ;                    // Set DCO
            DCOCTL = CALDCO_16MHZ;

            break;


    }

    clock_dco_frequency = dcoFrequency;
}







void CS_initClockSignal(uint8_t selectedClockSignal,
    uint16_t clockSource,
    uint16_t clockSourceDivider
    )
{

    if(clockSource != CS_DCOCLK_SELECT){
        // Invalid clock source
        return;
    }

    switch (selectedClockSignal) {

        case CS_SMCLK:

            BCSCTL2 &= 0xF1;   // Clear bits 3,2,1
            BCSCTL2 |=  (clockSourceDivider<<1);  // Set bits 2,1 to correct value

            clock_smclk_frequency  = clock_dco_frequency/(1<<clockSourceDivider);

            break;

        case CS_MCLK:

            BCSCTL2 &= 0x0f;   // Clear bits 7,6,5,4
            BCSCTL2 |=  (clockSourceDivider<<4);  // Set bits 5,4 to correct value

            clock_mclk_frequency   = clock_dco_frequency/(1<<clockSourceDivider);

            break;

        default:
            // not yet implemented

            break;
    }
}



uint32_t CS_getSMCLK(void)
{
    return(clock_smclk_frequency);
}


uint32_t CS_getMCLK(void)
{
    return(clock_mclk_frequency);
}

