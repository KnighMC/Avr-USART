/* Microchip Technology Inc. and its subsidiaries.  You may use this software
 * and any derivatives exclusively with Microchip products.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
 * TERMS.
 */

 /*
  * File: _ATMEGA328PB_
  * Author: Mauricio Colli
  * Comments: Handling avr bits for Atmega328PB
  * Revision history: 29/12/2023
  */

  // this library is based on "iom328pb.h"

 // This is a guard condition so that contents of this file are not included
 // more than once.  
#ifndef _ATMEGA328PB_H_FILE
#define	_ATMEGA328PB_H_FILE

#if !defined(F_CPU)
#define F_CPU 16000000UL
#endif /* F_CPU clock system */

#include <avr/io.h>
#include <avr/interrupt.h>

/*
________________________________________________________________________________

--------------------------- SETTINGS FOR PINOUT OF PORT B ----------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned TRISB0 : 1;
        unsigned TRISB1 : 1;
        unsigned TRISB2 : 1;
        unsigned TRISB3 : 1;
        unsigned TRISB4 : 1;
        unsigned TRISB5 : 1;
        unsigned TRISB6 : 1;
        unsigned TRISB7 : 1;
    };
    struct {
        unsigned RB0 : 1;
        unsigned RB1 : 1;
        unsigned RB2 : 1;
        unsigned RB3 : 1;
        unsigned RB4 : 1;
        unsigned RB5 : 1;
        unsigned RB6 : 1;
        unsigned RB7 : 1;
    };
} DDRBbits_t;

#define TRISBbits (*(volatile DDRBbits_t *)&DDRB)

typedef union {
    struct {
        unsigned LATB0 : 1;
        unsigned LATB1 : 1;
        unsigned LATB2 : 1;
        unsigned LATB3 : 1;
        unsigned LATB4 : 1;
        unsigned LATB5 : 1;
        unsigned LATB6 : 1;
        unsigned LATB7 : 1;
    };
    struct {
        unsigned RB0 : 1;
        unsigned RB1 : 1;
        unsigned RB2 : 1;
        unsigned RB3 : 1;
        unsigned RB4 : 1;
        unsigned RB5 : 1;
        unsigned RB6 : 1;
        unsigned RB7 : 1;
    };
} PORTBbits_t;

#define LATBbits (*(volatile PORTBbits_t *)&PORTB)

typedef union {
    struct {
        unsigned RB0 : 1;
        unsigned RB1 : 1;
        unsigned RB2 : 1;
        unsigned RB3 : 1;
        unsigned RB4 : 1;
        unsigned RB5 : 1;
        unsigned RB6 : 1;
        unsigned RB7 : 1;
    };
} PINBbits_t;

#define PORTBbits (*(volatile PINBbits_t *)&PINB)

/*
________________________________________________________________________________

--------------------------- SETTINGS FOR PINOUT OF PORT C ----------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned TRISC0 : 1;
        unsigned TRISC1 : 1;
        unsigned TRISC2 : 1;
        unsigned TRISC3 : 1;
        unsigned TRISC4 : 1;
        unsigned TRISC5 : 1;
        unsigned TRISC6 : 1;
        unsigned TRISC7 : 1;
    };
    struct {
        unsigned RC0 : 1;
        unsigned RC1 : 1;
        unsigned RC2 : 1;
        unsigned RC3 : 1;
        unsigned RC4 : 1;
        unsigned RC5 : 1;
        unsigned RC6 : 1;
        unsigned RC7 : 1;
    };
} DDRCbits_t;

#define TRISCbits (*(volatile DDRCbits_t *)&DDRC)

typedef union {
    struct {
        unsigned LATC0 : 1;
        unsigned LATC1 : 1;
        unsigned LATC2 : 1;
        unsigned LATC3 : 1;
        unsigned LATC4 : 1;
        unsigned LATC5 : 1;
        unsigned LATC6 : 1;
        unsigned LATC7 : 1;
    };
    struct {
        unsigned RC0 : 1;
        unsigned RC1 : 1;
        unsigned RC2 : 1;
        unsigned RC3 : 1;
        unsigned RC4 : 1;
        unsigned RC5 : 1;
        unsigned RC6 : 1;
        unsigned RC7 : 1;
    };
} PORTCbits_t;

#define LATCbits (*(volatile PORTCbits_t *)&PORTC)

typedef union {
    struct {
        unsigned RC0 : 1;
        unsigned RC1 : 1;
        unsigned RC2 : 1;
        unsigned RC3 : 1;
        unsigned RC4 : 1;
        unsigned RC5 : 1;
        unsigned RC6 : 1;
        unsigned RC7 : 1;
    };
} PINCbits_t;

#define PORTCbits (*(volatile PINCbits_t *)&PINC)

/*
________________________________________________________________________________

--------------------------- SETTINGS FOR PINOUT OF PORT D ----------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned TRISD0 : 1;
        unsigned TRISD1 : 1;
        unsigned TRISD2 : 1;
        unsigned TRISD3 : 1;
        unsigned TRISD4 : 1;
        unsigned TRISD5 : 1;
        unsigned TRISD6 : 1;
        unsigned TRISD7 : 1;
    };
    struct {
        unsigned RD0 : 1;
        unsigned RD1 : 1;
        unsigned RD2 : 1;
        unsigned RD3 : 1;
        unsigned RD4 : 1;
        unsigned RD5 : 1;
        unsigned RD6 : 1;
        unsigned RD7 : 1;
    };
} DDRDbits_t;

#define TRISDbits (*(volatile DDRDbits_t *)&DDRD)

typedef union {
    struct {
        unsigned LATD0 : 1;
        unsigned LATD1 : 1;
        unsigned LATD2 : 1;
        unsigned LATD3 : 1;
        unsigned LATD4 : 1;
        unsigned LATD5 : 1;
        unsigned LATD6 : 1;
        unsigned LATD7 : 1;
    };
    struct {
        unsigned RD0 : 1;
        unsigned RD1 : 1;
        unsigned RD2 : 1;
        unsigned RD3 : 1;
        unsigned RD4 : 1;
        unsigned RD5 : 1;
        unsigned RD6 : 1;
        unsigned RD7 : 1;
    };
} PORTDbits_t;

#define LATDbits (*(volatile PORTDbits_t *)&PORTD)

typedef union {
    struct {
        unsigned RD0 : 1;
        unsigned RD1 : 1;
        unsigned RD2 : 1;
        unsigned RD3 : 1;
        unsigned RD4 : 1;
        unsigned RD5 : 1;
        unsigned RD6 : 1;
        unsigned RD7 : 1;
    };
} PINDbits_t;

#define PORTDbits (*(volatile PINDbits_t *)&PIND)

/*
________________________________________________________________________________

--------------------------- CONFIGURATION BIT FOR USART0  ----------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned MPCM : 1;
        unsigned U2X : 1;
        unsigned UPE : 1;
        unsigned DOR : 1;
        unsigned FE : 1;
        unsigned UDRE : 1;
        unsigned TXC : 1;
        unsigned RXC : 1;
    };
} UCSRnAbits_t;

#define UCSR0Abits (*(volatile UCSRnAbits_t *)&UCSR0A)
#define UCSR1Abits (*(volatile UCSRnAbits_t *)&UCSR1A)

typedef union {
    struct {
        unsigned TXB8 : 1;
        unsigned RXB8 : 1;
        unsigned UCSZ2 : 1;
        unsigned TXEN : 1;
        unsigned RXEN : 1;
        unsigned UDRIE : 1;
        unsigned TXCIE : 1;
        unsigned RXCIE : 1;
    };
} UCSRnBbits_t;

#define UCSR0Bbits (*(volatile UCSRnBbits_t *)&UCSR0B)
#define UCSR1Bbits (*(volatile UCSRnBbits_t *)&UCSR1B)

typedef union {
    struct {
        unsigned UCPOL : 1;
        unsigned UCSZ0 : 1;
        unsigned UCSZ1 : 1;
        unsigned USBS : 1;
        unsigned UPM0 : 1;
        unsigned UPM1 : 1;
        unsigned UMSEL : 2;                                         // UMSEL0 & UMSEL1 
    };
} UCSRnCbits_t;

#define UCSR0Cbits (*(volatile UCSRnCbits_t *)&UCSR0C)
#define UCSR1Cbits (*(volatile UCSRnCbits_t *)&UCSR1C)

/*
________________________________________________________________________________

--------------------------- CONFIGURATION BIT FOR TIMER0  ----------------------
--------------------------- CONFIGURATION BIT FOR TIMER2  ----------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned WGM0 : 1;
        unsigned WGM1 : 1;
        unsigned reserved : 2;                                      // bits 2 & 3
        unsigned COMB : 2;                                          // COMB0 & COMB1
        unsigned COMA : 2;                                          // COMA0 & COMA1
    };
} _8_bits_TCCRnAbits_t;

#define TCCR0Abits (*(volatile _8_bits_TCCRnAbits_t *)&TCCR0A)
#define TCCR2Abits (*(volatile _8_bits_TCCRnAbits_t *)&TCCR2A)

typedef union {
    struct {
        unsigned CS : 3;                                            // CS0, CS1, CS2
        unsigned WGM2 : 1;
        unsigned reserved : 2;                                      // bits 4 & 5
        unsigned FOC : 2;                                           // FOCB & FOCA
    };
} _8_bits_TCCRnBbits_t;

#define TCCR0Bbits (*(volatile _8_bits_TCCRnBbits_t *)&TCCR0B)
#define TCCR2Bbits (*(volatile _8_bits_TCCRnBbits_t *)&TCCR2B)

/*
________________________________________________________________________________

---------------------------   CONFIGURATION BIT FOR SPI0  ----------------------
---------------------------   CONFIGURATION BIT FOR SPI1  ----------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned _SPR : 2;                                          // SPR0 & SPR1
        unsigned _CPHA : 1;
        unsigned _CPOL : 1;
        unsigned _MSTR : 1;
        unsigned _DORD : 1;
        unsigned _SPE : 1;
        unsigned _SPIE : 1;
    };
} SPCRnbits_t;

#define SPCR0bits (*(volatile SPCRnbits_t *)&SPCR0)
#define SPCR1bits (*(volatile SPCRnbits_t *)&SPCR1)

typedef union {
    struct {
        unsigned _SPI2X : 1;
        unsigned reserved : 5;                                      // bits 1-5
        unsigned _WCOL : 1;
        unsigned _SPIF : 1;
    };
} SPSRnbits_t;

#define SPSR0bits (*(volatile SPSRnbits_t *)&SPSR0)
#define SPSR1bits (*(volatile SPSRnbits_t *)&SPSR1)

/*
________________________________________________________________________________

---------------  CONFIGURATION BIT FOR ANALOG COMPARATOR  ----------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned _ACIS : 2;                                         // ACIS0 & ACIS1
        unsigned _ACIC : 1;
        unsigned _ACIE : 1;
        unsigned _ACI : 1;
        unsigned _ACO : 1;
        unsigned _ACBG : 1;
        unsigned _ACD : 1;
    };
} ACSRbits_t;

#define ACSRbits (*(volatile ACSRbits_t *)&ACSR)

/*
________________________________________________________________________________

---------------------------   CONFIGURATION BIT FOR ADC   ----------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned _ADPS : 3;                                         // ADPS0, ADPS1, ADPS2
        unsigned _ADIE : 1;
        unsigned _ADIF : 1;
        unsigned _ADATE : 1;
        unsigned _ADSC : 1;
        unsigned _ADEN : 1;
    };
} ADCSRAbits_t;

#define ADCSRAbits (*(volatile ADCSRAbits_t *)&ADCSRA)

typedef union {
    struct {
        unsigned ADTS : 3;                                          // ADTS0, ADTS1, ADTS2
        unsigned reserved : 3;                                      // bits 3-5
        unsigned ACMEn : 1;
        unsigned _reserved : 1;                                     // bit 7
    };
} ADCSRBbits_t;

#define ADCSRBbits (*(volatile ADCSRBbits_t *)&ADCSRB)

typedef union {
    struct {
        unsigned _MUX : 4;                                          // MUX0, MUX1, MUX2, MUX3
        unsigned reserved : 1;                                      // bit 4
        unsigned _ADLAR : 1;
        unsigned _REFS : 2;                                         // REFS0 & REFS1
    };
} ADMUXbits_t;

#define ADMUXbits (*(volatile ADMUXbits_t *)&ADMUX)

/*
________________________________________________________________________________

-------------------------   CONFIGURATION BIT FOR TIMER1   ---------------------
-------------------------   CONFIGURATION BIT FOR TIMER3   ---------------------
-------------------------   CONFIGURATION BIT FOR TIMER4   ---------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned WGM0 : 1;
        unsigned WGM1 : 1;
        unsigned reserved : 2;                                      // bits 2 & 3
        unsigned COMB : 2;                                          //COMB0 & COMB1
        unsigned COMA : 2;                                          //COMA0 & COMA1
    };
} TCCRnAbits_t;

#define TCCR1Abits (*(volatile TCCRnAbits_t *)&TCCR1A)
#define TCCR3Abits (*(volatile TCCRnAbits_t *)&TCCR3A)
#define TCCR4Abits (*(volatile TCCRnAbits_t *)&TCCR4A)

typedef union {
    struct {
        unsigned CS : 3;                                            // CS0, CS1, CS2
        unsigned WGM2 : 1;
        unsigned WGM3 : 1;
        unsigned reserved : 1;                                      // bit 5
        unsigned ICES : 1;
        unsigned ICNC : 1;
    };
} TCCRnBbits_t;

#define TCCR1Bbits (*(volatile TCCRnBbits_t *)&TCCR1B)
#define TCCR3Bbits (*(volatile TCCRnBbits_t *)&TCCR3B)
#define TCCR4Bbits (*(volatile TCCRnBbits_t *)&TCCR4B)

typedef union {
    struct {
        unsigned reserved : 6;                                      // bits 0 - 5
        unsigned FOC : 2;                                           // FOCB & FOCA
    };
} TCCRnCbits_t;

#define TCCR1Cbits (*(volatile TCCRnCbits_t *)&TCCR1C)
#define TCCR3Cbits (*(volatile TCCRnCbits_t *)&TCCR3C)
#define TCCR4Cbits (*(volatile TCCRnCbits_t *)&TCCR4C)

/*
________________________________________________________________________________

--------------------------   CONFIGURATION BIT FOR I2C0   ----------------------
--------------------------   CONFIGURATION BIT FOR I2C1   ----------------------
________________________________________________________________________________
*/

typedef union {
    struct {
        unsigned _TWIE : 1;
        unsigned reserved : 1;
        unsigned _TWEN : 1;
        unsigned _TWWC : 1;
        unsigned _TWSTO : 1;
        unsigned _TWSTA : 1;
        unsigned _TWEA : 1;
        unsigned _TWINT : 1;
    };
} TWCRnbits_t;

#define TWCR0bits (*(volatile TWCRnbits_t *)&TWCR0)
#define TWCR1bits (*(volatile TWCRnbits_t *)&TWCR1)

#endif /*   _ATMEGA328PB_H_FILE */