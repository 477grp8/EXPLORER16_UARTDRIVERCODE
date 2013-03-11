#include <p32xxxx.h>
#include <plib.h>
#include <math.h>
#include "ADC.h"
#include "LCD.h"
#include "UART.h"
#include "CONFIG.h"
#include "MISCELLANEOUS.h"
#include "TIMER.h"
main()
{
    // Disable JTAG (on RA0 and RA1 )
    mJTAGPortEnable( DEBUG_JTAGPORT_OFF );

        // Configure the device for maximum performance but do not change the PBDIV
        // Given the options, this function will change the flash wait states, RAM
        // wait state and enable prefetch cache but will not change the PBDIV.
        // The PBDIV value is already set via the pragma FPBDIV option above..
        SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

        initializeUART();
        initializeADC();
        initializeLCD();
        ConfigTimer1(); // Enable Timer1 for second counts
        configureInterrupts();

    T1CON = 0x8030; // TMR1 on, prescale 1:256 PB

    mPORTASetPinsDigitalOut( LED_MASK ); // LEDs = output
    mPORTDSetPinsDigitalIn( PB_MASK_D ); // PBs on D = input

    // enable interrupts
    INTEnableInterrupts();

    int i = 0;
    while( 1 )
    {
        //WriteString("Testt");
        convertAndPrintIntegerToString("i => ", i++);
        convertAndPrintIntegerToString(" 4 => ", getChannel4Value());
        convertAndPrintIntegerToString(" 5 => ", getChannel5Value());
        long j = 1024*1024;
        while(j--) {};
        mPORTAToggleBits(LED_MASK);
        TMR1 =  0;
        while ( TMR1 < SHORT_DELAY ){} // delay

    } // main (while) loop

        return 0;

} // main
