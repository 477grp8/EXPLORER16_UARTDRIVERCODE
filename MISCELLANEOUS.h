/* 
 * File:   MISCELLANEOUS.h
 * Author: team8
 *
 * Created on March 10, 2013, 3:12 PM
 */

#ifndef MISCELLANEOUS_H
#define  MISCELLANEOUS_H


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

#endif	/* MISCELLANEOUS_H */

