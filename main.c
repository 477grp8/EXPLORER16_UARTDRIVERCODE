#include <p32xxxx.h>
#include <plib.h>
#include <math.h>

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

void WriteString(const char *string);

void initializeUART(void) {
           // Configure UART2
        // This initialization assumes 36MHz Fpb clock. If it changes,
        // you will have to modify baud rate initializer.
        UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, GetPeripheralClock(), 38400 );
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART1
    // Same notes apply from above
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART1, GetPeripheralClock(), 38400 );
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

 
}

void initializeADC(void) {
        CloseADC10(); // ensure ADC is off before adjusting settings

    PORTSetPinsAnalogIn( IOPORT_B, BIT_5 | BIT_4 ); // AtD inputs for Pot and Thermometer unit
    //ConfigIntADC10( ADC_INT_DISABLE ); // disable AtD interrupts

    #define ADC_CONFIG1 ADC_MODULE_ON | ADC_FORMAT_INTG16 | ADC_CLK_MANUAL | ADC_AUTO_SAMPLING_ON | ADC_SAMP_OFF
    #define ADC_CONFIG2 ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON
    #define ADC_CONFIG3 ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15
    #define ADC_CONFIG4 ENABLE_AN4_ANA | ENABLE_AN5_ANA
    #define ADC_CONFIG5 SKIP_SCAN_ALL

    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN4 | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN5 );
    OpenADC10( ADC_CONFIG1, ADC_CONFIG2, ADC_CONFIG3, ADC_CONFIG4, ADC_CONFIG5 );
    EnableADC10();
}

void configureInterrupts(void) {
    // Configure UART2 RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);

    // Config UART1 Rx Interrupt
    // Higher priority than UART2 interrupt
    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_3);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);

    // configure for multi-vectored mode
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
}

/*
 * @author - Vineeth
 *
 * @params - valueToBePrinted => Is the integer value that needs to be converted
 *                              into a string and printed out on UART
 *
 * This function accepts a integer value, and does the necessary conversions
 * required to send the value out on UART using printf
 */
void convertAndPrintIntegerToString(char * stringToBePrinted, int valueToBePrinted) {
    int temp = valueToBePrinted;
    int lengthOfInteger = 0;

    WriteString(stringToBePrinted);
    /*
     *  Loop to count number of digits in the integer to be printed.
     */
    while(temp != 0) {
        temp = temp / 10;
        lengthOfInteger++;
    }

    int modDivideValue = 0;
    int digitToBePrinted = 0;

    /*
     *  Loop to actually start printing out each digit in the integer from left
     *  to right.
     */
    while(lengthOfInteger != 0) {
        modDivideValue = pow(10, --lengthOfInteger);
        digitToBePrinted = valueToBePrinted / modDivideValue;
        valueToBePrinted = valueToBePrinted % modDivideValue;

        switch(digitToBePrinted) {
            case 0 : WriteString("0");
                     break;
            case 1 : WriteString("1");
                     break;
            case 2 : WriteString("2");
                     break;
            case 3 : WriteString("3");
                     break;
            case 4 : WriteString("4");
                     break;
            case 5 : WriteString("5");
                     break;
            case 6 : WriteString("6");
                     break;
            case 7 : WriteString("7");
                     break;
            case 8 : WriteString("8");
                     break;
            case 9 : WriteString("9");
                     break;
            default : WriteString("");
                     break;
        }
    }
    WriteString(" ");
}

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
        long j = 1024*1024;
        while(j--) {};
        mPORTAToggleBits(LED_MASK);
        TMR1 =  0;
        while ( TMR1 < SHORT_DELAY ){} // delay

    } // main (while) loop

        return 0;

} // main


// helper functions
void WriteString(const char *string)
{
    while(*string != '\0')
    {
        while(!UARTTransmitterIsReady(UART2))
            ;

        UARTSendDataByte(UART2, *string);

        string++;

        while(!UARTTransmissionHasCompleted(UART2))
            ;
    }
}
void PutCharacter(const char character)
{
        while(!UARTTransmitterIsReady(UART2))
            ;

        UARTSendDataByte(UART2, character);


        while(!UARTTransmissionHasCompleted(UART2))
            ;
}

// UART 2 interrupt handler
// it is set at priority level 2
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void)
{
        // Is this an RX interrupt?
        if(INTGetFlag(INT_SOURCE_UART_RX(UART2)))
        {
                // Clear the RX interrupt Flag
            INTClearFlag( INT_SOURCE_UART_RX(UART2) );

                // Echo what we just received.
                while( !UARTReceivedDataIsAvailable(UART2) );
                UARTSendDataByte( UART1, UARTGetDataByte(UART2) );

                // Toggle LED to indicate UART activity
                ToggleLED8();

        }

        // We don't care about TX interrupt
        if ( INTGetFlag(INT_SOURCE_UART_TX(UART2)) )
        {
                INTClearFlag(INT_SOURCE_UART_TX(UART2));
        }
}

void __ISR(_UART1_VECTOR, ipl3) IntUart1Handler(void)
{
        if(( INTGetFlag( INT_SOURCE_UART_RX(UART1) ) )) {
                // Clear Rx Interrupt Flag
                INTClearFlag( INT_SOURCE_UART_RX(UART1) );

                // Echo UART1 intput to UART2 output
                UARTSendDataByte(UART2, UARTGetDataByte(UART1) );

                ToggleLED1();

        }

        if(( INTGetFlag( INT_SOURCE_UART_TX(UART1) ) )) {
                INTClearFlag( INT_SOURCE_UART_TX(UART1) );
        }
}  
