/* 
 * File:   CONFIG.h
 * Author: team8
 *
 * Created on March 10, 2013, 3:10 PM
 */

#ifndef CONFIG_H
#define  CONFIG_H

#include <p32xxxx.h>
#include <plib.h>

// Config settings
// POSCMOD = HS, FNOSC = PRIPLL, FWDTEN = OFF
// PLLIDIV = DIV_2, PLLMUL = MUL_16
// PBDIV = 8 (default)
// Main clock = 8MHz /2 * 16    = 80MHz
// Peripheral clock = 80MHz /8  =  10MHz

// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 10 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
//

#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1

#define GetSystemClock()                        (80000000ul)
#define GetPeripheralClock()            (GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define GetInstructionClock()           (GetSystemClock())

// 1. define timing constant
#define SHORT_DELAY (50*8)
#define LONG_DELAY      (400*8)

#define LED_MASK BIT_0|BIT_1|BIT_2|BIT_3|BIT_4|BIT_5|BIT_6|BIT_7 // LED mask for Port A
#define LED1 BIT_0 // LED masks for LEDs 1 ..,
#define LED2 BIT_1 // .
#define LED3 BIT_2 // .
#define LED4 BIT_3 // .
#define LED5 BIT_4 // .
#define LED6 BIT_5 // .
#define LED7 BIT_6 // .
#define LED8 BIT_7 // ... through 7

#define SetAllLEDs() mPORTASetBits( LED_MASK )
#define ClearAllLEDs() mPORTAClearBits( LED_MASK )

#define SetLED1() mPORTASetBits( LED1 )
#define SetLED2() mPORTASetBits( LED2 )
#define SetLED3() mPORTASetBits( LED3 )
#define SetLED4() mPORTASetBits( LED4 )
#define SetLED5() mPORTASetBits( LED5 )
#define SetLED6() mPORTASetBits( LED6 )
#define SetLED7() mPORTASetBits( LED7 )
#define SetLED8() mPORTASetBits( LED8 )

#define ClearLED1() mPORTAClearBits( LED1 )
#define ClearLED2() mPORTAClearBits( LED2 )
#define ClearLED3() mPORTAClearBits( LED3 )
#define ClearLED4() mPORTAClearBits( LED4 )
#define ClearLED5() mPORTAClearBits( LED5 )
#define ClearLED6() mPORTAClearBits( LED6 )
#define ClearLED7() mPORTAClearBits( LED7 )
#define ClearLED8() mPORTAClearBits( LED8 )

#define ToggleLED1() mPORTAToggleBits( LED1 )
#define ToggleLED2() mPORTAToggleBits( LED2 )
#define ToggleLED3() mPORTAToggleBits( LED3 )
#define ToggleLED4() mPORTAToggleBits( LED4 )
#define ToggleLED5() mPORTAToggleBits( LED5 )
#define ToggleLED6() mPORTAToggleBits( LED6 )
#define ToggleLED7() mPORTAToggleBits( LED7 )
#define ToggleLED8() mPORTAToggleBits( LED8 )


#define PB_MASK_D BIT_6|BIT_7|BIT_13 // Pushbutton mask for Port D
#define PB1 BIT_6  // PORT D
#define PB2 BIT_7  // PORT D
#define PB4 BIT_13 // PORT D

#define PB1_Pressed() !mPORTDReadBits( PB1 )
#define PB2_Pressed() !mPORTDReadBits( PB2 )
#define PB4_Pressed() !mPORTDReadBits( PB4 )

#endif	/* CONFIG_H */

