/**
 * main.c
 *
 * ECE 3849 Lab 1
 * Gene Bogdanov    10/18/2017
 * Drew Solomon     4/3/2021
 * This version is using the new hardware for B2017: the EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sampling.h"

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz

uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 8345; // time in hundredths of a second

extern volatile uint32_t gButtons; // from buttons.c

//From sampling.c
extern volatile int32_t gADCBufferIndex;                // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];   // circular buffer
extern volatile uint32_t gADCErrors;

bool edge = true; //true = rising, false = falling
char buttons; //Button state
float load_count, cpu_unload_count, cpu_load;

char intToChar(int digit);
void signal_init();
uint32_t cpu_load_count(void);

int main(void)
{
    IntMasterDisable(); //globally disables interrupts so we can safely initialize the hardware

    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                                      120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    uint16_t sample;

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    // full-screen rectangle
    tRectangle rectFullScreen = { 0, 0, GrContextDpyWidthGet(&sContext) - 1,
    GrContextDpyHeightGet(&sContext) - 1 };

    //Local Variables
    float fVoltsPerDiv = 1;
    char str[50];   // string buffer
    int y, lastY, trigger;

    //Peripheral Initialization
    ButtonInit(); //This calls the timer initialization code in buttons.c
    ADCInit(); //Calls ADC initialization from sampling.c
    signal_init(); //initialize PWM signal

    cpu_unload_count = cpu_load_count();

    IntMasterEnable(); //then globally enables interrupts

    while (true)
    {
//ORDER OF EVENTS:
//Check for button presses (aka rising trig or falling)
//Draw grid
//Draw the wave (gather data as part)
//Draw scaling-related text + cpu data

        // Get CPU load
        load_count = cpu_load_count();
        cpu_load = 1.0 - (float) load_count / cpu_unload_count;

        if (fifo_get(&buttons) && buttons == 'a')
        {
            edge = !edge;
        }

        // Fill screen with black
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        // Draw grid
        GrContextForegroundSet(&sContext, ClrBlue);
        int i;
        for (i = -3; i < 4; i++)
        {
            GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1,
            LCD_VERTICAL_MAX / 2 + i * PIXELS_PER_DIV);
            GrLineDrawV(&sContext, LCD_VERTICAL_MAX / 2 + i * PIXELS_PER_DIV, 0,
            LCD_HORIZONTAL_MAX - 1);

        }

        float fScale = (VIN_RANGE * PIXELS_PER_DIV)
                / ((1 << ADC_BITS) * fVoltsPerDiv);

        //Drawing the wave w/ ADC samples
        GrContextForegroundSet(&sContext, ClrYellow);

        if (edge)
        {
            trigger = RisingTrigger();
        }
        else
        {
            trigger = FallingTrigger();
        }

        for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++)
        {
            sample = gADCBuffer[ADC_BUFFER_WRAP(
                    trigger - LCD_HORIZONTAL_MAX / 2 + i)];

            y = LCD_VERTICAL_MAX / 2
                    - (int) roundf(fScale * ((int) sample - ADC_OFFSET));
            GrLineDraw(&sContext, i, lastY, i + 1, y);
            lastY = y;
        }

        //Time scale of 20 us across any voltage scale
        GrContextForegroundSet(&sContext, ClrWhite);
        GrStringDraw(&sContext, "20 us", -1, 4, 0, false);

        //Voltage scale of 1 V/div
        GrStringDraw(&sContext, "1 V", -1, 55, 0, false);

        if (edge)
        {
            // Draw rising trigger
            GrLineDraw(&sContext, 105, 10, 115, 10);
            GrLineDraw(&sContext, 115, 10, 115, 0);
            GrLineDraw(&sContext, 115, 0, 125, 0);
            GrLineDraw(&sContext, 112, 6, 115, 2);
            GrLineDraw(&sContext, 115, 2, 118, 6);
        }
        else
        {
            // Draw falling trigger
            GrLineDraw(&sContext, 105, 10, 115, 10);
            GrLineDraw(&sContext, 115, 10, 115, 0);
            GrLineDraw(&sContext, 115, 0, 125, 0);
            GrLineDraw(&sContext, 112, 3, 115, 7);
            GrLineDraw(&sContext, 115, 7, 118, 3);
        }

        // CPU load
        snprintf(str, sizeof(str), "CPU load = %.1f%%", cpu_load * 100);

        GrStringDraw(&sContext, str, -1, 0, 120, false);

        GrFlush(&sContext); //Flush to display

    }
}

char intToChar(int digit)
{
    switch (digit)
    {
    case 0:
        return '0';
    case 1:
        return '1';
    case 2:
        return '2';
    case 3:
        return '3';
    case 4:
        return '4';
    case 5:
        return '5';
    case 6:
        return '6';
    case 7:
        return '7';
    case 8:
        return '8';
    case 9:
        return '9';
    }
    return '?';
}

void signal_init()
{
    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    // configure M0PWM3, at GPIO PF3, BoosterPack 1 header C1 pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,
                    roundf((float) gSystemClock / PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                     roundf((float) gSystemClock / PWM_FREQUENCY * 0.4f));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,
                     roundf((float) gSystemClock / PWM_FREQUENCY * 0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

