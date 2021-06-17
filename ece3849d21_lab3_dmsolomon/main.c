/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 * Drew Solomon     4/26/2021
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

//#include files from Lab 1
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
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"

//#include files from Lab 2
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

//#include files from Lab 3
#include "driverlib/udma.h"
#include "audio_waveform.h"

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz

//From sampling.c
//extern volatile int32_t gADCBufferIndex;                // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];   // circular buffer
extern volatile uint32_t gADCErrors;

bool display_mode = true; //true = voltage, false = spectrum
bool edge = true; //true = rising, false = falling
volatile uint16_t sample[LCD_HORIZONTAL_MAX]; //Waveform buffer
volatile int displayBuffer[LCD_HORIZONTAL_MAX]; //Display buffer
float fVoltsPerDiv = 1;
float fScale; //Scale for display calculated using fVoltsperDiv

//Kiss FFT Variables
#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
size_t buffer_size = KISS_FFT_CFG_SIZE;
kiss_fft_cfg cfg; // Kiss FFT config
static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
volatile uint16_t fft_sample[NFFT];
float out_db[LCD_HORIZONTAL_MAX];

//For CPU load calculation
float cpu_load;
uint32_t load_count, cpu_unload_count;

char str[50];   // string buffer for CPU utlization

//Lab 3 Globals
#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64]; // uDMA control table (global)
extern volatile uint32_t DMAbufferIndex;
volatile uint32_t pwmPeriod = 0, prev_timestamp = 0, current_timestamp = 0;

char freq_str[20]; // string buffer for PWM frequency
char period_str[20]; // string buffer for PWM period

int pwmAdjust = 0; //Can adjust the PWM period by +/- 2000 clock cycles
#define PWM_ADJUST_STEP 200
#define AUDIO_PWM_FREQUENCY 465116 // PWM period
//#define AUDIO_PWM_FREQUENCY 20000 // PWM period

uint32_t gPWMSample = 0; // PWM sample counter
uint32_t gSamplingRateDivider = 29; // sampling rate divider
/* gSamplingRateDivider = (gSystemClock) / (PWMPeriod * AUDIO_SAMPLING_RATE), where PWMPeriod = 258 clock cycles */

//Function prototypes
void signal_init();
uint32_t cpu_load_count();
void DMA_init();
void captureTimer_ISR(void);
void PWM_init();
void PWM_ISR(void);

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    // hardware initialization goes here
    ButtonInit(); //This calls the timer initialization code in buttons.c
    ADCInit(); //Calls ADC initialization from sampling.c
    signal_init(); //initialize PWM signal
    DMA_init(); //initialize a DMA channel to ADC1 Seq 0
    PWM_init(); //initialize audio PWM signal

    /* Start BIOS */
    BIOS_start();

    //Task priority order:
    //Waveform (highest) 5
    //Button (high) 4
    //User Input (medium) 3
    //Display(low) 2
    //Processing (lowest) 1

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
    PWMGenPeriodSet(
            PWM0_BASE,
            PWM_GEN_1,
            roundf((float) gSystemClock
                    / (PWM_FREQUENCY + pwmAdjust * PWM_ADJUST_STEP)));
    PWMPulseWidthSet(
            PWM0_BASE,
            PWM_OUT_2,
            roundf((float) gSystemClock
                    / (PWM_FREQUENCY + pwmAdjust * PWM_ADJUST_STEP) * 0.4f));
    PWMPulseWidthSet(
            PWM0_BASE,
            PWM_OUT_3,
            roundf((float) gSystemClock
                    / (PWM_FREQUENCY + pwmAdjust * PWM_ADJUST_STEP) * 0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

void PWM_init()
{
    // configure M0PWM5, at GPIO PG1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 2, output 5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2,
                    roundf((float) gSystemClock / AUDIO_PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,
                     roundf((float) gSystemClock / AUDIO_PWM_FREQUENCY * 0.5f));
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO); //PWM interrupt every time counter = 0
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
}

void DMA_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);

    uDMAChannelAssign(UDMA_CH24_ADC1_0); // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);

    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
    UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
    UDMA_MODE_PINGPONG,
                           (void*) &ADC1_SSFIFO0_R, (void*) &gADCBuffer[0],
                           ADC_BUFFER_SIZE / 2);

    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
    UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
    UDMA_MODE_PINGPONG,
                           (void*) &ADC1_SSFIFO0_R,
                           (void*) &gADCBuffer[ADC_BUFFER_SIZE / 2],
                           ADC_BUFFER_SIZE / 2);

    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);
}

void waveform_task(UArg arg1, UArg arg2)
{
    cpu_unload_count = cpu_load_count();

    IntMasterEnable();

    int trigger;
    int i;

    while (1)
    {
        Semaphore_pend(semaphoreWaveform, BIOS_WAIT_FOREVER);
        if (edge)
        {
            trigger = RisingTrigger();
        }
        else
        {
            trigger = FallingTrigger();
        }

        if (display_mode)
        {
            //Take LCD_HORIZONTAL_MAX samples based on trigger position
            for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++)
            {
                sample[i] = gADCBuffer[ADC_BUFFER_WRAP(
                        trigger - LCD_HORIZONTAL_MAX / 2 + i)];
            }
        }
        else
        {
            //Take NFFT many of the most recent samples
            for (i = 0; i < NFFT - 1; i++)
            {
                fft_sample[i] = gADCBuffer[ADC_BUFFER_WRAP(DMAbufferIndex - i)];
            }
        }
        Semaphore_post(semaphoreProcessing);
    }
}

void processing_task(UArg arg1, UArg arg2)
{
    int i;
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    while (1)
    {
        Semaphore_pend(semaphoreProcessing, BIOS_WAIT_FOREVER);

        if (display_mode)
        {
            fScale = (VIN_RANGE * PIXELS_PER_DIV)
                    / ((1 << ADC_BITS) * fVoltsPerDiv);

            for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++)
            {
                displayBuffer[i] = LCD_VERTICAL_MAX / 2
                        - (int) roundf(fScale * ((int) sample[i] - ADC_OFFSET));
            }
        }
        else
        {
            for (i = 0; i < NFFT; i++)
            { // generate an input waveform
                in[i].r = fft_sample[i]; // real part of waveform
                in[i].i = 0; // imaginary part of waveform
            }

            kiss_fft(cfg, in, out); // compute FFT

            for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++)
            {
                out_db[i] = 160.0f
                        - 10.0f
                                * log10f(
                                        out[i].r * out[i].r
                                                + out[i].i * out[i].i);
            }
        }
        Semaphore_post(semaphoreDisplay);
        Semaphore_post(semaphoreWaveform);
    }

}

void display_task(UArg arg1, UArg arg2)
{

    int y, lastY, i;

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    // full-screen rectangle
    tRectangle rectFullScreen = { 0, 0, GrContextDpyWidthGet(&sContext) - 1,
    GrContextDpyHeightGet(&sContext) - 1 };

    while (1)
    {
        Semaphore_pend(semaphoreDisplay, BIOS_WAIT_FOREVER);

            // Get CPU load
            load_count = cpu_load_count();
            cpu_load = (100.0 - (float) load_count / cpu_unload_count * 100.0);

        // Fill screen with black
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        if (display_mode)
        {
            // Draw grid
            GrContextForegroundSet(&sContext, ClrBlue);

            for (i = -3; i < 4; i++)
            {
                GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1,
                LCD_VERTICAL_MAX / 2 + i * PIXELS_PER_DIV);
                GrLineDrawV(&sContext,
                LCD_VERTICAL_MAX / 2 + i * PIXELS_PER_DIV,
                            0,
                            LCD_HORIZONTAL_MAX - 1);
            }

            //Drawing the wave w/ ADC samples
            GrContextForegroundSet(&sContext, ClrYellow);

            for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++)
            {
                y = displayBuffer[i];
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

            // PWM Frequency (Lab 3)
            snprintf(freq_str, sizeof(freq_str), "f=%5d Hz",
                     gSystemClock / pwmPeriod);
            GrStringDraw(&sContext, freq_str, -1, 0, 112, false);

            snprintf(period_str, sizeof(period_str), "T=%4d", pwmPeriod);
            GrStringDraw(&sContext, period_str, -1, 80, 112, false);

        }
        else
        {

            // Draw grid
            GrContextForegroundSet(&sContext, ClrBlue);

            for (i = 0; i < 7; i++)
            {
                GrLineDrawV(&sContext, i * PIXELS_PER_DIV, 0,
                LCD_VERTICAL_MAX - 1);
            }
            for (i = -3; i < 4; i++)
            {
                GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1,
                LCD_VERTICAL_MAX / 2 + i * PIXELS_PER_DIV);
            }

            //Drawing the spectrum wave w/ ADC samples
            GrContextForegroundSet(&sContext, ClrYellow);

            for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++)
            {
                y = out_db[i];
                GrLineDraw(&sContext, i, lastY, i + 1, y);
                lastY = y;
            }

            //Draw frequency scale
            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "20 kHz", -1, 4, 0, false);

            //Draw decibel scale
            GrStringDraw(&sContext, "20 dB", -1, 55, 0, false);
        }

        // CPU load
        snprintf(str, sizeof(str), "CPU load = %.1f%%", cpu_load);
        GrStringDraw(&sContext, str, -1, 0, 120, false);

        GrFlush(&sContext); //Flush to display
    }
}

void clock0_task(UArg arg1)
{
    Semaphore_post(semaphoreButton);
}

void button_task(UArg arg1, UArg arg2)
{
    char mailboxInput;

    while (1)
    {
        Semaphore_pend(semaphoreButton, BIOS_WAIT_FOREVER);

        // read hardware button state
        uint32_t gpio_buttons = ~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) // EK-TM4C1294XL buttons in positions 0 and 1
        & (GPIO_PIN_1 | GPIO_PIN_0)
                | ((~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1)) << 1) //Add button S1 state to bitmap
                | ((~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6)) >> 3) //Add button S2 state to bitmap
                | (~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4)); //Add joystick select state to bitmap

        uint32_t old_buttons = gButtons;    // save previous button state
        ButtonDebounce(gpio_buttons); // Run the button debouncer. The result is in gButtons.
        ButtonReadJoystick(); // Convert joystick state to button presses. The result is in gButtons.
        uint32_t presses = ~old_buttons & gButtons; // detect button presses (transitions from not pressed to pressed)
        presses |= ButtonAutoRepeat(); // autorepeat presses if a button is held long enough

        //static bool tic = false;
        //static bool running = true;

        if (presses & 1)
        { // EK-TM4C1294XL button 1 pressed
          //running = !running;
        }

        if (presses & 2)
        { // EK-TM4C1294XL button 2 pressed
          //gTime = 0;
        }

        if (presses & 4)
        { // Boosterpack button SW1 is pressed
            mailboxInput = 'b'; //Controls display mode toggle
            Mailbox_post(mailboxButton, &mailboxInput, BIOS_WAIT_FOREVER);
        }

        if (presses & 8)
        { // Boosterpack button SW2 is pressed
            mailboxInput = 'a'; //Controls edge mode toggle
            Mailbox_post(mailboxButton, &mailboxInput, BIOS_WAIT_FOREVER);
        }

        if (presses & 16)
        { // Boosterpack joystick is pressed
            mailboxInput = 'e'; //Plays back audio
            Mailbox_post(mailboxButton, &mailboxInput, BIOS_WAIT_FOREVER);
        }

        if (presses & 32)
        { // Boosterpack joystick is tilted right
            mailboxInput = 'c'; //Increments PWM period
            Mailbox_post(mailboxButton, &mailboxInput, BIOS_WAIT_FOREVER);
        }

        if (presses & 64)
        { // Boosterpack joystick is tilted left
            mailboxInput = 'd'; //Decrements PWM period
            Mailbox_post(mailboxButton, &mailboxInput, BIOS_WAIT_FOREVER);
        }

    }
}

void userInput_task(UArg arg1, UArg arg2)
{

    char mailboxInput;
    IArg gateKey;

    while (1)
    {
        if (Mailbox_pend(mailboxButton, &mailboxInput, BIOS_WAIT_FOREVER))
        {
            switch (mailboxInput)
            {
            case 'a':
                edge = !edge;
                break;
            case 'b':
                display_mode = !display_mode;
                break;
            case 'c': //right on joystick
                if (pwmAdjust > -100)
                {
                    pwmAdjust--; //increase pwm period by decreasing freq
                }

                gateKey = GateTask_enter(gateTask0);
                signal_init();
                GateTask_leave(gateTask0, gateKey);
                break;

            case 'd': //left on joystick
                if (pwmAdjust < 100)
                {
                    pwmAdjust++; //decrease pwm period by increasing freq
                }

                gateKey = GateTask_enter(gateTask0);
                signal_init();
                GateTask_leave(gateTask0, gateKey);
                break;
            case 'e':
                PWMIntEnable(PWM0_BASE, PWM_INT_GEN_2);
                break;
            }
            Semaphore_post(semaphoreDisplay);
        }
    }
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

void captureTimer_ISR(void)
{
    TIMER0_ICR_R = TIMER_ICR_CAECINT; //clear TIMER0A interrupt flag

    current_timestamp = TimerValueGet(TIMER0_BASE, TIMER_A); //take current timestamp
    pwmPeriod = (current_timestamp - prev_timestamp) & 0xffffff; //period of signal in clock cycles
    prev_timestamp = current_timestamp; //copy current timestamp for next period calculation
}

void PWM_ISR(void)
{
    PWMGenIntClear(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO); // clear PWM interrupt flag
    int i = (gPWMSample++) / gSamplingRateDivider; // waveform sample index
    PWM0_2_CMPB_R = 1 + gWaveform[i]; // write directly to the PWM compare B register
    if (i == gWaveformSize - 1)
    { // if at the end of the waveform array
        PWMIntDisable(PWM0_BASE, PWM_INT_GEN_2); // disable these interrupts
        gPWMSample = 0; // reset sample index so the waveform starts from the beginning
    }
}
