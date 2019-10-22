/*
 *  File: main.c
 *  Date: 2017-Feb-02
 *  Target: PIC16F628A
 *  IDE: MPLAB 8.92
 *  Compiler: XC8 v1.38
 *
 *  Description:
 *
 *                    PIC16F628A
 *             +----------:_:----------+
 *   SEGC <> 1 : RA2               RA1 : 18 <> SEGB
 *   SEGD <> 2 : RA3               RA0 : 17 <> SEGA
 *   SEGE <> 3 : RA4          OSC1/RA7 : 16 <> SEGG
 *    VPP -> 4 : RA5/VPP      OSC2/RA6 : 15 <> SEGF
 *    GND -> 5 : VSS               VDD : 14 <- PWR
 *   DS2n <> 6 : RB0/INT       PGD/RB7 : 13 <> PGD
 * DBG_RX <> 7 : RB1/RX/DT     PGC/RB6 : 12 <> PGC
 * DBG_TX <> 8 : RB2/RX/CK         RB5 : 11 <> DS1n
 * BEEPER <> 9 : RB3/CCP       PGM/RB4 : 10 <> BUTTON
 *             +-----------------------:
 *                      DIP-18
 *
 * When we push the button, it starts to count in seconds and tens of seconds
 * and emits a sound with a buzzer. When button pressed again, sounds the buzzer,
 * stops the timer and starts blinking the time measured. Pushing the button
 * a third time blanks the display and is ready for a button press to start again.
 *
 * WARNING: This code has never been tested in real hardware.
 *          It works with the MPLAB simulator but some things
 *          like the beeper and the 7-segment encoding are
 *          just too hard to test in simulation.
 *
 *          I expect the code to work in real hardware but YMMV.
 *
 */
#ifndef __16F628A__
#error Device selecttion invalid. Select PIC16F628A with the IDE.
#endif

#pragma config FOSC = INTOSCIO // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is digital input, MCLR internally tied to VDD)
#pragma config BOREN = OFF // Brown-out Detect Enable bit (BOD disabled)
#pragma config LVP = OFF // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)

#include <stdint.h>
#include <stdbool.h>
#include <xc.h>

#define _XTAL_FREQ              (4000000UL)
#define CLOCKS_PER_INSTRUCTION  (4UL)
#define BEEPER_FREQUENCY        (4000UL)
#define TIMER2_TICKS_PER_SECOND (BEEPER_FREQUENCY/4UL)
#define DISPLAY_BLANK_MASK      (0x02)
#define BUTTON_MASK             (SW1_MASK | SW2_MASK)
#define BUTTON_CHANGED_MASK     (SW1_CHANGED_MASK | SW2_CHANGED_MASK)
#define BUTTON_DEBOUNCE_TICKS   (20)
#define SW1_PRESSED             (PORTBbits.RB4?1:0)
#define SW1_MASK                (1<<0)
#define SW1_CHANGED_MASK        (1<<4)
#define SW2_PRESSED             (PORTAbits.RA5?0:1)
#define SW2_MASK                (1<<1)
#define SW2_CHANGED_MASK        (1<<5)
#define DS1n                    PORTBbits.RB5
#define DS2n                    PORTBbits.RB0
#define LED_FLASH_TIME_MS       (200)
#define BEEP_TIME_MS            (100)

/*
 *  Global data
 */
volatile unsigned char gucSeconds;
volatile unsigned char gucMilliseconds;
volatile unsigned char gucButtonStatus;
volatile unsigned char gucLedSegments[2];
volatile unsigned char gucDiaplayStatus;
volatile unsigned char gucBeepOffTime;
/*
 *  Interrupt service routines
 */
void interrupt ISR_Handlers(void)
{
    static unsigned char InterruptLedIndex;
    static unsigned int  InterruptTimer2Ticks;
    static unsigned char InterruptButtonStatus;
    static unsigned char InterruptButtonDebounceTimer;
           unsigned char InterruptButtonSample;

    if (T0IE)
    {
        if (T0IF)
        {
            /* Clear TIMER0 interrupt request */
            T0IF = 0;

            /* refresh LED display */
            DS1n = 1;   /* turn off all digits */
            DS2n = 1;
            if ((gucDiaplayStatus & DISPLAY_BLANK_MASK) == 0)
            {
                if (InterruptLedIndex >= sizeof(gucLedSegments))
                {
                    InterruptLedIndex = 0;
                }
                PORTA = gucLedSegments[InterruptLedIndex];
                switch (InterruptLedIndex)
                {
                    case 0:
                        DS1n = 0;
                        break;
                    case 1:
                        DS2n = 0;
                        break;
                    default:
                        break;
                }
                InterruptLedIndex++;
            }
        }
    }

    if (PIE1bits.TMR2IE)
    {
        if (PIR1bits.TMR2IF)
        {
            /* Clear TIMER2 interrupt request */
            PIR1bits.TMR2IF = 0;
            gucMilliseconds++;
            InterruptTimer2Ticks++;

            /* check if time to turn off beeper */
            if (gucBeepOffTime)
            {
                gucBeepOffTime--;
                if (!gucBeepOffTime)
                {
                    /* beeper off, select 0% duty cycle */
                    CCPR1L  = 0;
                }
            }

            /* keep track of time in seconds */
            if (InterruptTimer2Ticks >= TIMER2_TICKS_PER_SECOND)
            {
                InterruptTimer2Ticks = 0;
                gucSeconds++;
            }

            /* Debounce button */
            InterruptButtonSample = 0;
            if (SW1_PRESSED)
            {
                InterruptButtonSample |=  SW1_MASK;
            }
            if (SW2_PRESSED)
            {
                InterruptButtonSample |=  SW2_MASK;
            }
            InterruptButtonSample = (InterruptButtonSample ^ InterruptButtonStatus) & BUTTON_MASK;
            if (InterruptButtonSample)
            {
                InterruptButtonStatus ^= InterruptButtonSample;
                InterruptButtonDebounceTimer = BUTTON_DEBOUNCE_TICKS;
            }
            else
            {
                if (InterruptButtonDebounceTimer)
                {
                    InterruptButtonDebounceTimer--;
                }
                else
                {
                    if ((gucButtonStatus & BUTTON_CHANGED_MASK) == 0 )
                    {
                        InterruptButtonStatus = (gucButtonStatus ^ InterruptButtonStatus) & BUTTON_MASK;
                        if (InterruptButtonStatus)
                        {
                            gucButtonStatus ^= InterruptButtonStatus;
                            gucButtonStatus |= InterruptButtonStatus << 4;
                        }
                    }
                }
            }
        }
    }
}

/*
 *  Initialize this PIC
 */
void PIC_Init (void)
{
    INTCON = 0;         /* Disable all interrupts */
    PIE1 = 0;
    CMCON = 7;          /* Disable comperators */
    TRISA = 0xFF;       /* Set all of PORTA as inputs */
    TRISB = 0xFF;       /* Set all of PORTB as inputs */
    OPTION_REG = OPTION_REG | 0x80;  /* Disable PORTB pull ups */
    PORTA = 0;
    PORTB = 0;
}

/*
 *  Initialize TIMER0 to interrupt every 16384 instruction cycles
 */
void TMR0_Init (void)
{
    T0IE = 0;           /* Disable TIMER0 interrupt */

    OPTION_REG = OPTION_REG & ~0b00101111; /* TMR0 clock source is FCYC */
    OPTION_REG = OPTION_REG |  0b00000101; /* TMR0 prescale (1:64) */

    TMR0 = 0;
    T0IF = 0;           /* Clear TIMER0 interrupt request */
    T0IE = 1;           /* Enable TIMER0 interrupt */
}

/*
 *  Initialize PWM for use as BEEPER on RB3/CCP1 output.
 */
void PWM_Init (void)
{
    TRISBbits.TRISB3 = 1;   /* disable CCP1 output */
    PIE1bits.TMR2IE  = 0;   /* disable TIMER2 interrupt */
    CCP1CON = 0;            /* turn off PWM */
    T2CON   = 0;            /* stop TIMER2 */
    gucBeepOffTime   = 0;   /* stop beep timer */
    PR2     = (_XTAL_FREQ/(BEEPER_FREQUENCY*CLOCKS_PER_INSTRUCTION))-1;
    CCPR1L  = 0;            /* select 0% duty cycle */
    CCP1CON = 0x0C;         /* select PWM function for CCP1 output */
    TRISBbits.TRISB3 = 0;   /* enable CCP1 output */
    T2CONbits.TOUTPS = 3;   /* assert interrupt at (BEEPER_FREQUENCY/4) Hz */
    PIR1bits.TMR2IF  = 0;   /* clear TIMER2 interrupt request */
    PIE1bits.TMR2IE  = 1;   /* enable TIMER2 interrupt */
    T2CONbits.TMR2ON = 1;   /* Turn on TIMER2 */
}
/*
 *  Test beeper global control.
 *  Returns ONE when beeper timer is running.
 *  Returns ZERO when beeper timer has finished.
 */
unsigned char BeepIsRunning (void)
{
    if (gucBeepOffTime) return 1;
    return 0;
}
/*
 *  Wait for beep timer to complete then
 *  start the beep tone for the number of
 *  milliseconds in the parameter. Range
 *  is 0 to 255 milliseconds.
 */
void BeeperOn (unsigned char Time)
{
    while(BeepIsRunning());
    /* select 50% duty cycle */
    CCPR1L  = (_XTAL_FREQ/(BEEPER_FREQUENCY*CLOCKS_PER_INSTRUCTION))/2;
    gucBeepOffTime = Time;
}
/*
 *  Wait for beep timer to complete then start
 *  a silent beep time for the number of
 *  milliseconds in the parameter. Range
 *  is 0 to 255 milliseconds.
 */
void BeeperOff (unsigned char Time)
{
    while(BeepIsRunning());
    /* select 0% duty cycle */
    CCPR1L  = 0;
    gucBeepOffTime = Time;
}
/*
 *  Initialize the LED display
 */
void LED_Init (void)
{
    DS1n = 1;                   /* turn off digit drivers */
    DS2n = 1;
    PORTA = 0xFF;               /* turn off all segment driver */
    TRISA = 0x20;               /* enable all segment driver outputs */
    gucLedSegments[0] = 0xFF;   /* set to blank LED refresh buffer */
    gucLedSegments[1] = 0xFF;
    gucDiaplayStatus  = 0;
}
/*
 * Segment drivers: A=RA0, B=RA1, C=RA2, D=RA3, E=RA4, F=RA6, G=RA7
 * Segment on when driver output bit is zero. Using "active low" is
 * convenient because RA4 is open drain only and cannot drive high.
 *
 *   -RA0-             -RA0-    -RA0-             -RA0-    -RA0-    -RA0-
 *  R     R        R        R        R  R     R  R        R              R
 *  A     A        A        A        A  A     A  A        A              A
 *  6     1        1        1        1  6     1  6        6              1
 *                     -RA7-    -RA7-    -RA7-    -RA7-    -RA7-
 *  R     R        R  R              R  R              R  R     R        R
 *  A     A        A  A              A  A              A  A     A        A
 *  4     2        2  4              2  4              2  4     2        2
 *   -RA3-             -RA3-    -RA3-             -RA3-    -RA3-
 *
 *   0xA0     0xF9     0x64     0x70     0x2D     0x32     0x22     0xF8
 *
 *
 *   -RA0-    -RA0-    -RA0-             -RA0-             -RA0-    -RA0-
 *  R     R  R     R  R     R  R        R              R  R        R
 *  A     A  A     A  A     A  A        A              A  A        A
 *  6     1  6     1  6     1  6        6              1  6        6
 *   -RA7-    -RA7-    -RA7-    -RA7-             -RA7-    -RA7-    -RA7-
 *  R     R        R  R     R  R     R  R        R     R  R        R
 *  A     A        A  A     A  A     A  A        A     A  A        A
 *  4     2        2  4     2  4     2  4        4     2  4        4
 *   -RA3-    -RA3-             -RA3-    -RA3-    -RA3-    -RA3-
 *
 *   0x20     0x30     0x24     0x23     0xA6     0x61     0x26     0x2E
 */
const unsigned char LedDigits[16] = {0xA0,0xF9,0x64,0x70,0x2D,0x32,0x22,0xF8,0x20,0x38,0x24,0x23,0xA6,0x61,0x26,0x2E};
/*
 * Show decimal value in LED display
 */
void LED_DisplayDec (unsigned char Value)
{
    unsigned char TensDigit;

    /* put Value in the 0 to 99 decimal range */
    if (Value >= 200) Value -= 200;
    if (Value >= 100) Value -= 100;

    /* find tens digit */
    TensDigit = 0;
    if (Value >= 80) {Value -= 80; TensDigit |= 8;}
    if (Value >= 40) {Value -= 40; TensDigit |= 4;}
    if (Value >= 20) {Value -= 20; TensDigit |= 2;}
    if (Value >= 10) {Value -= 10; TensDigit |= 1;}

    /* display tens digit */
    gucLedSegments[0] = LedDigits[TensDigit];

    /* display ones digit */
    gucLedSegments[1] = LedDigits[Value];
}
/*
 * Show hexadecimal value in LED display
 */
void LED_DisplayHex (unsigned char Value)
{
    /* display high 4-bits */
    gucLedSegments[0] = LedDigits[(Value >> 4) & 0x0F];

    /* display low 4-bits */
    gucLedSegments[1] = LedDigits[Value & 0x0F];
}
/*
 * Display test to turn on all segments of each display
 */
void LED_DisplayTest (unsigned char Value)
{
    switch (Value & 3)
    {
        case 0:
            /* turn off all segments */
            gucLedSegments[0] = 0xFF;
            gucLedSegments[1] = 0xFF;
            break;
        case 1:
            /* Turn on all segments of DS1 */
            gucLedSegments[0] = 0x20;
            gucLedSegments[1] = 0xFF;
            break;
        case 2:
            /* Turn on all segments of DS2 */
            gucLedSegments[0] = 0xFF;
            gucLedSegments[1] = 0x20;
            break;
        case 3:
            /* Turn on all segments of DS1 and DS2 */
            LED_DisplayHex(0x88);
            break;
        default:
            break;
    }
}
/*
 *  Main application
 */
void main (void)
{
    unsigned char Seconds;
    unsigned char Milliseconds;
    unsigned char Started;
    unsigned char LastTimeDisplayed;


    PIC_Init();         /* Initialize this PIC */
    TMR0_Init();        /* Initialize TIMER0 for interrupts */
    PWM_Init();         /* Initialize Buzzer drive */
    LED_Init();         /* Initialize LED display */
    PEIE = 1;           /* Enable peripheral interrupts */
    GIE = 1;            /* Global interrupt enable */

    /* Lamp test */
    Seconds = gucSeconds;
    LED_DisplayTest(0);
    BeeperOff(0);
    while ((unsigned char)(gucSeconds-Seconds) < 1);
    BeeperOn(BEEP_TIME_MS);
    LED_DisplayTest(1);
    while ((unsigned char)(gucSeconds-Seconds) < 2);
    BeeperOn(BEEP_TIME_MS);
    LED_DisplayTest(2);
    while ((unsigned char)(gucSeconds-Seconds) < 3);
    BeeperOn(BEEP_TIME_MS);
    LED_DisplayTest(3);
    while ((unsigned char)(gucSeconds-Seconds) < 4);
    LED_DisplayTest(0);

    Started = 0;
    LastTimeDisplayed = 0;
    for(;;)
    {
        /* check for button changes */
        if (gucButtonStatus & BUTTON_CHANGED_MASK)
        {
            /* check if the SW1 button changed */
            if (gucButtonStatus & SW1_CHANGED_MASK)
            {
                if (gucButtonStatus & SW1_MASK)
                {   /* if SW1 changed to pressed */
                    switch (Started)
                    {
                        case 0:
                            /* first press start counting seconds */
                            Seconds = gucSeconds;
                            Started = 1;
                            BeeperOn(BEEP_TIME_MS);
                            break;
                        case 1:
                            /* second press stop counting seconds */
                            Started = 2;
                            BeeperOn(BEEP_TIME_MS);
                            break;
                        default:
                            /* third press blank display and begin wait for next start */
                            LED_DisplayTest(0);
                            Started = 0;
                            break;
                    }
                }
                else
                {   /* if SW1 changed to released */
                }
            }
            /* check if the SW2 button changed */
            if (gucButtonStatus & SW2_CHANGED_MASK)
            {
                if (gucButtonStatus & SW2_MASK)
                {   /* if SW2 changed to pressed */
                }
                else
                {   /* if SW2 changed to released */
                }
            }
            gucButtonStatus &= ~BUTTON_CHANGED_MASK;
        }
        /* display results on LEDs */
        switch (Started)
        {
            case 1:
                if ((unsigned char)(gucSeconds - Seconds) != LastTimeDisplayed)
                {
                    LastTimeDisplayed = (unsigned char)(gucSeconds - Seconds);
                    LED_DisplayDec(LastTimeDisplayed);
                    gucDiaplayStatus &= ~DISPLAY_BLANK_MASK;
                }
                break;
            case 2:
                LastTimeDisplayed = (unsigned char)(gucSeconds - Seconds);
                LED_DisplayDec(LastTimeDisplayed);
                gucDiaplayStatus &= ~DISPLAY_BLANK_MASK;
                Started = 3;
                Milliseconds = gucMilliseconds;
                break;
            case 3:
                if ((unsigned char) (gucMilliseconds - Milliseconds) >= LED_FLASH_TIME_MS)
                {
                    Milliseconds += LED_FLASH_TIME_MS;
                    gucDiaplayStatus ^= DISPLAY_BLANK_MASK;
                }
                break;
            default:
                break;
        }
    }
}
