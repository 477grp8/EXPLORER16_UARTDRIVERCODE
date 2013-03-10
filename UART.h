/* 
 * File:   UART.h
 * Author: team8
 *
 * Created on March 10, 2013, 3:09 PM
 */

#ifndef UART_H
#define  UART_H

#include <p32xxxx.h>
#include <plib.h>
#include "CONFIG.h"

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
#endif	/* UART_H */

