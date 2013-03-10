/* 
 * File:   ADC.h
 * Author: team8
 *
 * Created on March 10, 2013, 2:55 PM
 */

#ifndef ADC_H
#define  ADC_H

#include <p32xxxx.h>
#include <plib.h>

unsigned int adcSampledInputChannel4 = 0; // ADC sampled input for channel 4 should be stored here
unsigned int adcSampledInputChannel5 = 0; // ADC sampled input for channel 5 should be stored here

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


void sampleADCInputs() {
     ConvertADC10(); // start a conversion
     while( BusyADC10() ) {} // while busy, wait

     unsigned int offset;
     offset = 8 * ((~ReadActiveBufferADC10() & 0x01));

     adcSampledInputChannel4 = ReadADC10( offset );
     adcSampledInputChannel5 = ReadADC10( offset + 1 );
}

int getChannel4Value() {
    return adcSampledInputChannel4;
}

int getChannel5Value() {
    return adcSampledInputChannel5;
}

#endif	/* ADC_H */

