#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"

#define M_TMR_INTERRUPT_PRIORITY    (3)
#define PWM_FREQUENCY               (800)
#define ADC_V_SCALE                 (1241)
#define DISPLAY_FPS                 (50000)

/*****************************************************************
 * Global Variables - Declared globally for observation in watch expressions list (most do not need to be in global)
 *****************************************************************/
volatile uint32_t curr = 0;  // Storage for TimerValueGet
volatile uint32_t rpm = 0;
volatile uint32_t duty;
volatile uint32_t rpmtarget = 0;
volatile int error = 0;

/*****************************************************************
 * Function Prototypes
 *****************************************************************/

void binary_decoder(uint8_t DecimalDigit);

/*****************************************************************
 * Main
 *****************************************************************/
int main(void)
{
    uint32_t ui32ADC0Value[1];
    volatile uint32_t ui32Load;
    volatile uint32_t ui32PWMClock;
    volatile uint16_t ui16Adjust;
    ui16Adjust = 519;
    volatile uint32_t dutyadjust;

    uint8_t rpmdigit1;
    uint8_t rpmdigit2;
    uint8_t rpmdigit3;
    uint16_t rpmdigits;

    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |  SYSCTL_XTAL_16MHZ);

    /* GPIO Interrupt */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_3);
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
    IntEnable(INT_GPIOA);

    /* Down Timer */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
    TimerEnable(TIMER0_BASE, TIMER_A);

    /* PWM */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui16Adjust * ui32Load / 1000);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);

    /* ADC */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);

    /* BCD Output */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //enable port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //enable port C
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4); //set port B pin 1-4 as output GPIO
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7|GPIO_PIN_5|GPIO_PIN_6); //set port c pin 1-3 as output GPIO


    while(1)
    {
        ADCIntClear(ADC0_BASE, 3);
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false))
        {
        }
        ADCSequenceDataGet(ADC0_BASE, 3, ui32ADC0Value);

        /* Convert ADCValue to rpmtarget and duty */
        rpmtarget = ui32ADC0Value[0]*0.586;  // 0.586 = 2400/4095
        duty = 100 - rpmtarget*100/2400;
        dutyadjust = duty*1038;  // Scale for placement into PWMPulseWidthSet()
        rpm = 40000000*60/(curr*2);

        /* P Control */
        error = rpmtarget - rpm;
        dutyadjust -= error*40;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (dutyadjust * ui32Load / 100000));

        /* Append digits of rpm for 7-seg display */
        rpmdigit1 = rpm/1000; //the thousands digit for rpm, eg. the 2 in 2340
        rpmdigits = rpm - rpmdigit1*1000; //removes the thousands from rpm
        rpmdigit2 = rpmdigits/100; //the hundreds digit for rpm, eg. the 3 in 2340
        rpmdigits = rpmdigits - rpmdigit2*100; //removes the hundreds digit from rpm
        rpmdigit3 = rpmdigits/10; //the tens digit for rpm, eg. the 4 in 2340


        //display the digits using the 7-segments
        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7, 0);
        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_5, GPIO_PIN_5);
        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_6, GPIO_PIN_6);
        binary_decoder(rpmdigit1);

        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7, GPIO_PIN_7);
        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_5, 0);
        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_6, GPIO_PIN_6);
        binary_decoder(rpmdigit2);

        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_7, GPIO_PIN_7);
        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_5, GPIO_PIN_5);
        GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_6, 0);
        binary_decoder(rpmdigit3);

    }
}


/*****************************************************************
 * Interrupt Functions
 *****************************************************************/

void PortAIntHandler(void)
{
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
    curr = TimerValueGet(TIMER0_BASE, TIMER_A);
    if (rpm != 0) {
        HWREG(TIMER0_BASE+0x50) = 0;  // Write to timer register to reset back to zero
    }
}

/*****************************************************************
 * Private Functions
 *****************************************************************/

//function which configures the GPIO ports to send binary value to BCD
void binary_decoder(uint8_t DecimalDigit)
{
    switch (DecimalDigit) {
    case (0):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, 0);
        SysCtlDelay(DISPLAY_FPS);
        break;

    case (1):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, GPIO_PIN_4);
        SysCtlDelay(DISPLAY_FPS);
        break;

    case (2):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, 0);
        SysCtlDelay(DISPLAY_FPS);
        break;

    case (3):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, GPIO_PIN_4);
        SysCtlDelay(DISPLAY_FPS);
        break;

    case (4):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, 0);
        SysCtlDelay(DISPLAY_FPS);
        break;

    case (5):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, GPIO_PIN_4);
        SysCtlDelay(DISPLAY_FPS);
        break;

    case (6):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, 0);
        SysCtlDelay(DISPLAY_FPS);
        break;

    case (7):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, GPIO_PIN_4);
        SysCtlDelay(DISPLAY_FPS);
        break;

    case (8):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, 0);
        SysCtlDelay(DISPLAY_FPS);
        break;

    case (9):
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_4, GPIO_PIN_4);
        SysCtlDelay(DISPLAY_FPS);
        break;

    }
}
