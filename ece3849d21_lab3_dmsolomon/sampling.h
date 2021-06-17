/*
 * sampling.h
 *
 *  Created on: Apr 3, 2021
 *      Author: Drew Solomon
 */

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <stdint.h>

#define ADC_SAMPLING_RATE 1000000   // [samples/sec] desired ADC sampling rate
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates

#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro

#define ADC_OFFSET 2048

#define VIN_RANGE 3.3
#define PIXELS_PER_DIV 20
#define ADC_BITS 12


//Initializes ADC and ADC sampling sequence
void ADCInit(void);
void ADC_ISR(void);
int RisingTrigger(void);
int FallingTrigger(void);
int32_t getADCBufferIndex(void);

#endif /* SAMPLING_H_ */
