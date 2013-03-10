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

unsigned int adcSampledInputChannel4 = 0; // ADC sampled input for channel 4 should be stored here
unsigned int adcSampledInputChannel5 = 0; // ADC sampled input for channel 5 should be stored here

void WriteString(const char *string);

/*
 * Initializes the UART with the config
 */
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

/*
 * Initializes the ADC config
 */
void initializeADC(void) {
    CloseADC10(); // ensure ADC is off before adjusting settings

    PORTSetPinsAnalogIn( IOPORT_B, BIT_5 | BIT_4 ); // AtD inputs for sampling LED
    //ConfigIntADC10( ADC_INT_DISABLE ); // disable AtD interrupts

    // all these PARAMS are defined in adc10.h
    #define ADC_CONFIG1 ADC_MODULE_ON | ADC_FORMAT_INTG16 | ADC_CLK_MANUAL | ADC_AUTO_SAMPLING_ON | ADC_SAMP_OFF
    #define ADC_CONFIG2 ADC_VREF_EXT_EXT | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON
    #define ADC_CONFIG3 ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15
    #define ADC_CONFIG4 ENABLE_AN4_ANA | ENABLE_AN5_ANA
    #define ADC_CONFIG5 SKIP_SCAN_ALL

    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN4 | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN5 );
    OpenADC10( ADC_CONFIG1, ADC_CONFIG2, ADC_CONFIG3, ADC_CONFIG4, ADC_CONFIG5 );
    EnableADC10();
}

/*
 * Configures the UART interrupts
 */
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

/*
 * @author - Vineeth
 *
 * @params - int delayTime in milliseconds -- causes delay based on delayTime
 *
 * Can be used anywhere a delay is needed to accodomate processing time or to wait on something
 */
void delay(int delayTime) {
    long count = ((long)delayTime * 0.001) * 80000000ul;
    while(count != 0) {
        count--;
    }
}

/*
 * @author - Vineeth
 *
 * @params - int LCDByte -- byte to be sent
 *         - int dataOrInstruction -- 0 if instruction, 1 if data
 *         - int RW -- 0 if write, 1 if read
 *
 * Used to send a specific data byte to the LCD to display
 */
void sendByteToLCD(char LCDByte, int dataOrInstruction, int RW) {
    
    PORTSetPinsDigitalOut(IOPORT_E, BIT_4); //RS
    PORTSetPinsDigitalOut(IOPORT_E, BIT_3); //R/W
    PORTSetPinsDigitalOut(IOPORT_E, BIT_2); //E
    PORTSetPinsDigitalOut(IOPORT_G, BIT_13); //DB0
    PORTSetPinsDigitalOut(IOPORT_G, BIT_12); //DB1
    PORTSetPinsDigitalOut(IOPORT_G, BIT_14); //DB2
    PORTSetPinsDigitalOut(IOPORT_E, BIT_1); //DB3
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0); //DB4
    PORTSetPinsDigitalOut(IOPORT_A, BIT_7); //DB5
    PORTSetPinsDigitalOut(IOPORT_A, BIT_6); //DB6
    PORTSetPinsDigitalOut(IOPORT_G, BIT_0); //DB7
    

    (dataOrInstruction == 0) ? PORTClearBits(IOPORT_E, BIT_4) : PORTSetBits(IOPORT_E, BIT_4);
    (RW == 0) ? PORTClearBits(IOPORT_E, BIT_3) : PORTSetBits(IOPORT_E, BIT_3);
    ((LCDByte & 0x01) == 0x01) ? PORTSetBits(IOPORT_G, BIT_13) : PORTClearBits(IOPORT_G, BIT_13);
    (((LCDByte >> 1) & 0x01) == 0x01) ? PORTSetBits(IOPORT_G, BIT_12) : PORTClearBits(IOPORT_G, BIT_12);
    (((LCDByte >> 2) & 0x01) == 0x01) ? PORTSetBits(IOPORT_G, BIT_14) : PORTClearBits(IOPORT_G, BIT_14);
    (((LCDByte >> 3) & 0x01) == 0x01) ? PORTSetBits(IOPORT_E, BIT_1) : PORTClearBits(IOPORT_E, BIT_1);
    (((LCDByte >> 4) & 0x01) == 0x01) ? PORTSetBits(IOPORT_E, BIT_0) : PORTClearBits(IOPORT_E, BIT_0);
    (((LCDByte >> 5) & 0x01) == 0x01) ? PORTSetBits(IOPORT_A, BIT_7) : PORTClearBits(IOPORT_A, BIT_7);
    (((LCDByte >> 6) & 0x01) == 0x01) ? PORTSetBits(IOPORT_A, BIT_6) : PORTClearBits(IOPORT_A, BIT_6);
    (((LCDByte >> 7) & 0x01) == 0x01) ? PORTSetBits(IOPORT_G, BIT_0) : PORTClearBits(IOPORT_G, BIT_0);

    PORTSetBits(IOPORT_E, BIT_2);
    delay(1);
    PORTClearBits(IOPORT_E, BIT_2);

}

/*
 * @author - Vineeth & Fabian
 *
 * @params - void
 *
 * Intializes the LCD by sending the various initializations
 */
void initializeLCD() {
    /*
     * LCD PIN # || Symbol || PIC32MX795F512H pin assignment
     *     1          Vss                GND
     *     2          Vdd                3.3V
     *     3          Vo                 0.1V(contrast)
     *     4          RS                 100
     *     5          R/W                99
     *     6          E                  98
     *     7          DB0                97
     *     8          DB1                96
     *     9          DB2                95
     *     10         DB3                94
     *     11         DB4                93
     *     12         DB5                92
     *     13         DB6                91
     *     14         DB7                90
     *     15         LED+               3.3V(Backlight Power)
     *     16         LED-               GND (Backlight GND)
     */

    PORTSetPinsDigitalOut(IOPORT_E, BIT_4); //RS
    PORTSetPinsDigitalOut(IOPORT_E, BIT_3); //R/W
    PORTSetPinsDigitalOut(IOPORT_E, BIT_2); //E
    PORTSetPinsDigitalOut(IOPORT_G, BIT_13); //DB0
    PORTSetPinsDigitalOut(IOPORT_G, BIT_12); //DB1
    PORTSetPinsDigitalOut(IOPORT_G, BIT_14); //DB2
    PORTSetPinsDigitalOut(IOPORT_E, BIT_1); //DB3
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0); //DB4
    PORTSetPinsDigitalOut(IOPORT_A, BIT_7); //DB5
    PORTSetPinsDigitalOut(IOPORT_A, BIT_6); //DB6
    PORTSetPinsDigitalOut(IOPORT_G, BIT_0); //DB7

    PORTClearBits(IOPORT_E, BIT_2); // E = 0
    delay(100);
    sendByteToLCD(0x30, 0, 0); // Wake up
    delay(30);
    sendByteToLCD(0x30, 0, 0); // WAKE UP!
    delay(10);
    sendByteToLCD(0x30, 0, 0); // I WILL BEAT YOU TO DEATH IF YOU DONT WAKE UP NOW
    delay(10);
    sendByteToLCD(0x38, 0, 0); // 8-bit and 2 line
    sendByteToLCD(0x10, 0, 0); // set cursor
    sendByteToLCD(0x0F, 0, 0); // display on; cursor on; blinking cursor on
    sendByteToLCD(0x06, 0, 0); // entry mode on
    sendByteToLCD(0x01, 0, 0); // clear the display

    printToLCD(" LCD Initialization       Complete");
}



/*
 * @author - Vineeth
 *
 * @params - char charToLCD -- character to be displayed
 *
 * Used for writing a character onto the LCD
 */
void displayCharacterOnLCD(char charToLCD) {
    sendByteToLCD(charToLCD, 1, 0);
}



void sampleADCInputs() {
     ConvertADC10(); // start a conversion
     while( BusyADC10() ) {} // while busy, wait

     unsigned int offset;
     offset = 8 * ((~ReadActiveBufferADC10() & 0x01));

     adcSampledInputChannel4 = ReadADC10( offset );
     adcSampledInputChannel5 = ReadADC10( offset + 1 );
}

/*
 * @author - Fabian
 *
 * @params - char* str -- string to be displayed
 *
 * Used for writing string onto the LCD
 */
void printToLCD(char* str){
    char* x = str;
    while (*x != '\0'){ //loop through the string and display it per character
        displayCharacterOnLCD(*x);
        x++; //increment pointer
    }
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
        initializeLCD();
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
        sampleADCInputs();
        convertAndPrintIntegerToString("i => ", i++);
        convertAndPrintIntegerToString(" 4 => ", adcSampledInputChannel4);
        convertAndPrintIntegerToString(" 5 => ", adcSampledInputChannel5);
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
