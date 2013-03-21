/* 
 * File:   TIMER.h
 * Author: team8
 *
 * Created on March 10, 2013, 3:20 PM
 *
 * This code was built upon and modified using the base code from : http://code.google.com/p/the-incredible-hud/source/browse/Microcontroller_Code/Final_Code/src/HelmetTimers.h
 */

#ifndef TIMER_H
#define  TIMER_H

#include <p32xxxx.h>
#include <plib.h>
#include "LCD.h"

unsigned short int SecondParts;
unsigned short int TenthSecond = 0;
unsigned short int SecondCount = 0;

#define T1_FREQ 100
#define TENTH_SEC       10 // count var for tenth of a second

// Macro to instruct micro that a 'new second'
#define startNewSecond() SecondParts=T1_FREQ;

// One 'second part' has gone by, decrement the count.
// When the count reaches zero (i.e. - has no parts left),
// the second is old and should be decremented no more.
#define secondPartElapsed() if(SecondParts>0) SecondParts--;

// Macro to test whether second is 'old'
// i.e. - that it has not parts left
#define isOldSecond() ( SecondParts<2 ? 1 : 0 )

// Note on second parts:
//       The first new character sent by the GPS
//       received by the PIC begins a 'new second'.
//       The timer sets the second as 'old' just
//       before the new second is expected to start
//       long after the GPS will have sent its data.
//       See HelmetUART.h's GPShandler routine for
//       an example.

// actions to take every second
#define EverySecondDo(x)                if( SecondCount > 0 ) { SecondCount--; } else { SecondCount = T1_FREQ-1; x }

/**************************************************************************************
        Function Prototypes
**************************************************************************************/
void ConfigTimer1( void );

/**************************************************************************************
        Functions
**************************************************************************************/
void ConfigTimer1() {
        #define T1_TICK (( GetPeripheralClock() / 256 / T1_FREQ ))
        OpenTimer1( T1_ON | T1_SOURCE_INT | T1_PS_1_256, T1_TICK );
    ConfigIntTimer1( T1_INT_ON | T1_INT_PRIOR_3 );
}

/**************************************************************************************
        Interrupt handling routines
**************************************************************************************/
// Note: Priority 1 (lowest) to 7 (highest)

/**************************************
                Timer 1
                        Priority 3
**************************************/
void __ISR(_TIMER_1_VECTOR, ipl3) Timer1Handler(void)
{
    // clear the interrupt flag
        INTClearFlag( INT_T1 );

        EverySecondDo({
            TurnOffLexmarkLED();
            SampleLexmarkLEDVoltage();
            //printToLCD("1");
            TurnOnLexmarkLED();
        })

    // one second part has elapsed
    secondPartElapsed();

    // Make sure the flag is cleared
    INTClearFlag( INT_T1 );
}

#endif	/* TIMER_H */

