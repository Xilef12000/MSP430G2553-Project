//*****************************************************************************
//
// cs.h - Driver for the CS Module.
//
//*****************************************************************************

#ifndef __CS_H__
#define __CS_H__

#include <stdint.h>


//*****************************************************************************
//
// The following are values that can be passed to the clockSource parameter for
// functions: CS_initClockSignal().
//
//*****************************************************************************
#define CS_DCOCLK_SELECT        1


//*****************************************************************************
//
// The following are values that can be passed to the clockSourceDivider
// parameter for functions: CS_initClockSignal().
//
//*****************************************************************************
#define CS_CLOCK_DIVIDER_1       0       // DCO/1
#define CS_CLOCK_DIVIDER_2       1       // DCO/2
#define CS_CLOCK_DIVIDER_4       2       // DCO/4
#define CS_CLOCK_DIVIDER_8       3       // DCO/8


//*****************************************************************************
//
// The following are values that can be passed to the selectClock parameter for
// functions: CS_enableClockRequest(), and CS_disableClockRequest(); the
// selectedClockSignal parameter for functions: CS_initClockSignal().
//
//*****************************************************************************
#define CS_MCLK                  0x02
#define CS_SMCLK                 0x04


// Clock frequencies for DCO
#define CLOCK_DCO_1MHZ      (1000000)
#define CLOCK_DCO_8MHZ      (8000000)
#define CLOCK_DCO_12MHZ     (12000000)
#define CLOCK_DCO_16MHZ     (16000000)




//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************


//******************************************************************************
//
//! Sets the DCO to the given frequency (Hz).
//! Valid values: 1000000, 8000000, 12000000, 16000000
//!
//! \return None
//
//******************************************************************************
extern void CS_setDCOFrequency(uint32_t dcoFrequency);



//*****************************************************************************
//
//! \brief Initializes a clock signal
//!
//! This function initializes each of the clock signals. The user must ensure
//! that this function is called for each clock signal.
//!
//! \param selectedClockSignal selected clock signal
//!        Valid values are:
//!        - \b CS_MCLK
//!        - \b CS_SMCLK
//! \param clockSource is clock source for the selectedClockSignal
//!        - \b CS_DCOCLKDIV_SELECT
//! \param clockSourceDivider selected the clock divider to calculate
//!        clocksignal from clock source.
//!        Valid values are:
//!        - \b CS_CLOCK_DIVIDER_1
//!        - \b CS_CLOCK_DIVIDER_2
//!        - \b CS_CLOCK_DIVIDER_4
//!        - \b CS_CLOCK_DIVIDER_8
//!
//! Modified bits of \b CSCTL3 register, bits of \b CSCTL5 register and bits of
//! \b CSCTL4 register.
//!
//! \return None
//
//*****************************************************************************
extern void CS_initClockSignal(uint8_t selectedClockSignal,
                               uint16_t clockSource,
                               uint16_t clockSourceDivider);



//*****************************************************************************
//
//! \brief Get the current SMCLK frequency
//!
//! Get the current SMCLK frequency. The user of this API must ensure that
//! CS_setExternalClockSource API was invoked before in case XT1 is being used.
//!
//!
//! \return Current SMCLK frequency in Hz
//
//*****************************************************************************
extern uint32_t CS_getSMCLK(void);

//*****************************************************************************
//
//! \brief Get the current MCLK frequency
//!
//! Get the current MCLK frequency. The user of this API must ensure that
//! CS_setExternalClockSource API was invoked before in case XT1 is being used.
//!
//!
//! \return Current MCLK frequency in Hz
//
//*****************************************************************************
extern uint32_t CS_getMCLK(void);


#endif // __MSP430WARE_CS_H__
