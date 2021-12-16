// PIC12F1572 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1
#pragma config FOSC = INTOSC    // (INTOSC oscillator; I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF    // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOREN = OFF    // Low Power Brown-out Reset enable bit (LPBOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


#include "main.h"

//----------------------------------------------------------------------------//
#define btn_click   PORTAbits.RA5
uint16_t result, brightness, pause, result_old;
unsigned int millis_count = 0;
unsigned int time1 = 0;
__bit btn_ok = 0;
__bit break_del = 0;
uint8_t btn_mode = 1, tim1_cnt = 0;
//----------------------------------------------------------------------------//

void delay_ms(uint16_t milliseconds) {
    while (milliseconds > 0) {
        if (break_del == 0) {
            __delay_ms(1);
            milliseconds--;
        } else milliseconds = 0;
    }
}
//----------------------------------------------------------------------------//

void osc_setup(void) {

    OSCCONbits.SPLLEN = 1;
    //Internal Oscillator Frequency Select bits
    OSCCONbits.IRCF0 = 0;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF3 = 1;
    //System Clock Select bits
    OSCCONbits.SCS0 = 0;
    OSCCONbits.SCS1 = 0;

}
//----------------------------------------------------------------------------//

/*Расчет таймера
  FCPU=16000000/4=4000000(Частота таймера)
 * 4000000/Prescaler=1000000/4=250000
 * 250000/1000=250 тиков за 1 мс
 * Поскольку таймер 8 бит считает только от 0...255, заносим знач в TMR0 5
 * чтобы до 255 осталось 250 
 * FCPU=32000000/4=8000000(Частота таймера)
 * 8000000/Prescaler=8000000/32=250000
 * 250000/1000=250 тиков за 1 мс
 * Поскольку таймер 8 бит считает только от 0...255, заносим знач в TMR0 5
 * чтобы до 255 осталось 250
 */
unsigned int millis(void) {
    return (millis_count); //возвращает значение счетчика
}

void tmr0_init(void) {
    TMR0CS = 0; //Clock Fosc/4
    PSA = 0; //Prescaler Tim0
    OPTION_REGbits.PS2 = 1; //Presc 8 010; Presc 4  001; Presc 32 100
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 0;
    TMR0 = 5;
    T0IE = 1; //Enable Tim0 interupt
    T0IF = 0; //Clear flag
    GIE = 1; //Global interrupt
    PEIE = 1; //peripheral interrupt
}

void tmr1_init(void) {
    //Fosc/4
    TMR1CS0 = 0;
    TMR1CS1 = 0;
    //Prescale: 8-11; 4-10; 2-01; 1-00
    T1CKPS0 = 1;
    T1CKPS1 = 1;
    TMR1H = 0;
    TMR1L = 0;
    //Timer ON
    TMR1ON = 1;
    //Flag Clear
    TMR1IF = 0;
    //Interrups
    TMR1IE = 1;
    GIE = 1; //Global interrupt
    PEIE = 1; //peripheral interrupt
}
//----------------------------------------------------------------------------//

void pwm2_init(void) {
    PWM2PH = 0; //Phase Register
    PWM2DC = 0; //Duty Register
    PWM2PR = 1000; //Period Register
    //Standart PWM Mode
    PWM2CONbits.MODE0 = 0;
    PWM2CONbits.MODE1 = 0;
    //Offset Mode Select bits(Independent Run mode)
    PWM2OFCONbits.OFM0 = 0;
    PWM2OFCONbits.OFM1 = 0;
    //CLOCK CONTROL REGISTER(FOSC)
    PWM2CLKCONbits.CS0 = 0;
    PWM2CLKCONbits.CS1 = 0;
    //Clock Source Prescaler Select bits
    PWM2CLKCONbits.PS0 = 1; //2
    PWM2CLKCONbits.PS1 = 0;
    PWM2CLKCONbits.PS2 = 0;
    //PWM CONTROL REGISTER
    PWM2CONbits.EN = 1; //PWM Enable
    PWM2CONbits.OE = 1; //PWM OUT Enable bit
    PWM2CONbits.POL = 0; //Polarity is LOW

}

void pwm1_init(void) {
    PWM1PH = 0; //Phase Register
    PWM1DC = 0; //Duty Register
    PWM1PR = 1000; //Period Register
    //Standart PWM Mode
    PWM1CONbits.MODE0 = 0;
    PWM1CONbits.MODE1 = 0;
    //Offset Mode Select bits(Independent Run mode)
    PWM1OFCONbits.OFM0 = 0;
    PWM1OFCONbits.OFM1 = 0;
    //CLOCK CONTROL REGISTER(FOSC)
    PWM1CLKCONbits.CS0 = 0;
    PWM1CLKCONbits.CS1 = 0;
    //Clock Source Prescaler Select bits
    PWM1CLKCONbits.PS0 = 1; //2
    PWM1CLKCONbits.PS1 = 0;
    PWM1CLKCONbits.PS2 = 0;
    //PWM CONTROL REGISTER
    PWM1CONbits.EN = 1; //PWM Enable
    PWM1CONbits.OE = 1; //PWM OUT Enable bit
    PWM1CONbits.POL = 0; //Polarity is LOW

}

void pwm3_init(void) {
    PWM3PH = 0; //Phase Register
    PWM3DC = 0; //Duty Register
    PWM3PR = 1000; //Period Register
    //Standart PWM Mode
    PWM3CONbits.MODE0 = 0;
    PWM3CONbits.MODE1 = 0;
    //Offset Mode Select bits(Independent Run mode)
    PWM3OFCONbits.OFM0 = 0;
    PWM3OFCONbits.OFM1 = 0;
    //CLOCK CONTROL REGISTER(FOSC)
    PWM3CLKCONbits.CS0 = 0;
    PWM3CLKCONbits.CS1 = 0;
    //Clock Source Prescaler Select bits
    PWM3CLKCONbits.PS0 = 1; //2
    PWM3CLKCONbits.PS1 = 0;
    PWM3CLKCONbits.PS2 = 0;
    //PWM CONTROL REGISTER
    PWM3CONbits.EN = 1; //PWM Enable
    PWM3CONbits.OE = 1; //PWM OUT Enable bit
    PWM3CONbits.POL = 0; //Polarity is LOW

}

void RED_LED(uint16_t duty) {
    PWM1CONbits.EN = 0;
    PWM1DC = duty * 10;
    //PWM1PH = 0; //Phase Register
    PWM1CONbits.EN = 1;
    __delay_ms(1);
}

void GREEN_LED(uint16_t duty) {
    PWM2CONbits.EN = 0;
    PWM2DC = duty * 10;
    //PWM2PH = 0; //Phase Register
    PWM2CONbits.EN = 1;
    __delay_ms(1);
}

void BLUE_LED(uint16_t duty) {
    PWM3CONbits.EN = 0;
    PWM3DC = duty * 10;
    //PWM3PH = 0; //Phase Register
    PWM3CONbits.EN = 1;
    __delay_ms(1);
}
//----------------------------------------------------------------------------//

void port_setup(void) {
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
    ANSELAbits.ANSA2 = 0;
    ANSELAbits.ANSA4 = 1;
    WPUA = 0x00;
    //PWM
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    //ADC
    TRISAbits.TRISA4 = 1;
    //Button
    TRISAbits.TRISA5 = 1;

    IOCANbits.IOCAN5 = 1;
    IOCAPbits.IOCAP5 = 0;
    IOCIE = 1; //Interrupt-On-Change Enable bit
    IOCIF = 0; // Interrupt-On-Change Interrupt Flag bit
    IOCAFbits.IOCAF5 = 0; //Flag pin RA5
    GIE = 1; //Global interrupt
    PEIE = 1; //peripheral interrupt
}
//----------------------------------------------------------------------------//

void adc_setup(void) {
    //Set ADC
    ADCON0 = 0b00001101; //AN3;GO/DONE=0;ADC=1;
    ADCON1 = 0b11100000; //ADFM=1(Right justified);FOSC/64;VDD
    //PIE1bits.ADIE = 1; //Enables the ADC interrupt
    //PIR1bits.ADIF = 0; //Clear flag
}
//----------------------------------------------------------------------------//

uint16_t adr_read(void) {
    uint16_t adc_result = 0;
    GO = 1;
    while (GO);
    adc_result = ADRESH << 8;
    adc_result = adc_result | ADRESL;
    return adc_result;
}
//----------------------------------------------------------------------------//

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//----------------------------------------------------------------------------//

/* del - delay*/
void miganie(uint16_t del) {
    GREEN_LED(100);
    RED_LED(100);
    BLUE_LED(100);
    delay_ms(del);
    GREEN_LED(0);
    RED_LED(0);
    BLUE_LED(0);
    delay_ms(del);
}
//----------------------------------------------------------------------------//
/*Мигание цветами по очереди*/

/* del - delay*/
void begogon1(uint16_t del) {
    GREEN_LED(100);
    RED_LED(0);
    BLUE_LED(0);
    delay_ms(del);
    GREEN_LED(0);
    RED_LED(100);
    BLUE_LED(0);
    delay_ms(del);
    GREEN_LED(0);
    RED_LED(0);
    BLUE_LED(100);
    delay_ms(del);
}
//----------------------------------------------------------------------------//
/*Плавное зажигание и угасание каждого цывета по отдельности*/

/* del - delay*/
void begogon2(uint16_t del) {
    uint16_t count1 = 0;
    for (count1 = 0; count1 < 100; count1 = count1 + 5) {
        GREEN_LED(count1);
        delay_ms(del);
    }
    for (count1 = 100; count1 > 0; count1 = count1 - 5) {
        GREEN_LED(count1);
        delay_ms(del);
    }
    GREEN_LED(0);
    //****************************************************************************//
    for (count1 = 0; count1 < 100; count1 = count1 + 5) {
        RED_LED(count1);
        delay_ms(del);
    }
    for (count1 = 100; count1 > 0; count1 = count1 - 5) {
        RED_LED(count1);
        delay_ms(del);
    }
    RED_LED(0);
    //****************************************************************************//
    for (count1 = 0; count1 < 100; count1 = count1 + 5) {
        BLUE_LED(count1);
        delay_ms(del);
    }
    for (count1 = 100; count1 > 0; count1 = count1 - 5) {
        BLUE_LED(count1);
        delay_ms(del);
    }
    BLUE_LED(0);
}
//----------------------------------------------------------------------------//

void begogon3(uint16_t del) {
    uint8_t count_up, count_down;
    count_down = 100;
    GREEN_LED(0);
    RED_LED(0);
    BLUE_LED(0);
    for (count_up = 0; count_up < 100; count_up = count_up + 5) {
        count_down = count_down - 5;
        RED_LED(count_down);
        BLUE_LED(count_up);
        delay_ms(del);
    }
    RED_LED(0);
    count_down = 100;
    for (count_up = 0; count_up < 100; count_up = count_up + 5) {
        count_down = count_down - 5;
        BLUE_LED(count_down);
        GREEN_LED(count_up);
        delay_ms(del);
    }
    BLUE_LED(0);
    count_down = 100;
    for (count_up = 0; count_up < 100; count_up = count_up + 5) {
        count_down = count_down - 5;
        GREEN_LED(count_down);
        RED_LED(count_up);
        delay_ms(del);
    }
    GREEN_LED(0);
    count_down = 100;
}

void begogonGB(uint16_t del) {
    GREEN_LED(100);
    RED_LED(100);
    BLUE_LED(0);
    delay_ms(del);
    GREEN_LED(0);
    RED_LED(100);
    BLUE_LED(100);
    delay_ms(del);
}

void begogonBR(uint16_t del) {
    GREEN_LED(100);
    RED_LED(100);
    BLUE_LED(0);
    delay_ms(del);
    GREEN_LED(100);
    RED_LED(0);
    BLUE_LED(100);
    delay_ms(del);
}

void begogonGR(uint16_t del) {
    GREEN_LED(100);
    RED_LED(0);
    BLUE_LED(100);
    delay_ms(del);
    GREEN_LED(0);
    RED_LED(100);
    BLUE_LED(100);
    delay_ms(del);
}
//----------------------------------------------------------------------------//

void RGB_ON(uint8_t yarc) {
    BLUE_LED(yarc);
    RED_LED(yarc);
    GREEN_LED(yarc);
}
//----------------------------------------------------------------------------//

void main(void) {
    osc_setup();
    tmr0_init();
    port_setup();
    pwm1_init();
    pwm2_init();
    pwm3_init();
    adc_setup();
    tmr1_init();
    while (1) {

        if (btn_ok == 1 & (millis() - time1) > 50) {
            if (btn_ok == 1 & btn_click == 1) {
                GREEN_LED(0);
                RED_LED(0);
                BLUE_LED(0);
                btn_ok = 0;
                break_del = 0;
                btn_mode++;
            }
            time1 = millis();
        }


        switch (btn_mode) {
            case 1: RGB_ON(brightness);
                break;
            case 2: BLUE_LED(brightness);
                break;
            case 3: RED_LED(brightness);
                break;
            case 4: GREEN_LED(brightness);
                break;
            case 5: BLUE_LED(brightness);
                GREEN_LED(brightness);
                break;
            case 6: BLUE_LED(brightness);
                RED_LED(brightness);
                break;
            case 7: RED_LED(brightness);
                GREEN_LED(brightness);
                break;
            case 8: miganie(pause); //pause1
                break;
            case 9: begogon1(pause); //pause1
                break;
            case 10: begogon2(pause);
                break;
            case 11: begogon3(pause);
                break;
            case 12: begogonGB(pause); //pause1
                break;
            case 13: begogonBR(pause); //pause1
                break;
            case 14: begogonGR(pause); //pause1
                break;
            case 15: btn_mode = 1;
                break;
        }
    }
}
//----------[ ISR  ]----------------------------------------------------------//

void __interrupt() ISR(void) {
    /*INT TIM0*/
    if (T0IF) {
        millis_count++;
        TMR0 = 5;
        T0IF = 0;
    }
    /*INT RA5*/
    if (IOCAFbits.IOCAF5) {
        btn_ok = 1;
        break_del = 1;
        //IOCIF=0;
        IOCAFbits.IOCAF5 = 0; // clear interrupt flag bit
    }
    /*INT TIM1*/
    if (TMR1IF) {
        if (tim1_cnt > 5) {
            result = adr_read();
            result = result/10;
            brightness = result;
            pause = result;
            brightness = map(brightness, 0, 102, 1, 100);
            pause = map(pause, 0, 102, 10, 500);
            if (result != result_old) {
                break_del = 1;
                result_old = result;
            } else break_del = 0;
            if (tim1_cnt > 5) {
                tim1_cnt = 0;
                //break_del = 0;
            }
        }
        tim1_cnt++;
        TMR1IF = 0;
    }
    /*INT ADC*/
    if (ADIF) {

        ADIF = 0;
    }
}
//----------[ END OF ISR ]----------------------------------------------------//