#include "USART_RS232.h"

#ifdef HANDLING_AVR_BITS

#ifdef __AVR_ATmega328PB__
#include "Atmega328PB.h"
#endif /* __AVR_ATmega328PB__ */
#ifdef __AVR_ATmega328P__
#include "Atmega328P.h"
#endif /* __AVR_ATmega328P__ */
#ifdef __AVR_ATmega2560__
#include "Atmega2560.h"
#endif /* __AVR_ATmega328P__ */

void initialize_USART(uint16_t baudrate) {

    /* Set baud rate from equations Asynchonous normal mode (UBRRn = (fosc/16BAUD) -1) */
    UBRR0 = ((F_CPU / (16UL * baudrate)) - 1);
    /* Enable Transmitter and Receiver */
    UCSR0Bbits.RXEN = 1;
    UCSR0Bbits.TXEN = 1;
    /* sets number to data bits in a frame the RX and TX use.
     * 8 bits,     1 stop bit,     parity: none               */
    UCSR0Cbits.UCSZ0 = 1;
    UCSR0Cbits.UCSZ1 = 1;

}

void disable_USART(void) {

    /* Disable Transmitter and Receiver */
    UCSR0Bbits.RXEN = 0;
    UCSR0Bbits.TXEN = 0;

}

void USART_transmitChar(uint8_t character) {

    while (!UCSR0Abits.UDRE);                                                                   // Wait to empty TX buffer
    UDR0 = character;                                                                           // Transmit data into buffer
    while (!UCSR0Abits.TXC);                                                                    // Wait for the transmission to complete

    if (character == '\n')                                                                      // If data is '\n' call recursion to transmit 'CRLF'
        USART_transmitChar('\r');

}

uint8_t USART_receiveChar(void) {

    while (!UCSR0Abits.RXC);                                                                    // Wait for RX buffer to fill

    return UDR0;                                                                                // Get data from Buffer

}

void disable_UsartDataRegisterEmptyInterrupt(void) {
    UCSR0Bbits.UDRIE = 0;
}

void enable_UsartDataRegisterEmptyInterrupt(void) {
    UCSR0Bbits.UDRIE = 1;
}

void disable_UsartTransmitCompleteInterrupt(void) {
    UCSR0Bbits.TXCIE = 0;
}

void enable_UsartTransmitCompleteInterrupt(void) {
    UCSR0Bbits.TXCIE = 1;
}

void disable_UsartReceiveCompleteInterrupt(void) {
    UCSR0Bbits.RXCIE = 0;
}

void enable_UsartReceiveCompleteInterrupt(void) {
    UCSR0Bbits.RXCIE = 1;
}


#else
#include <avr/io.h>

void initialize_USART(uint16_t baudrate) {

    UBRR0 = ((F_CPU / (16UL * baudrate)) - 1);                                                      // Set baud rate from equations Asynchonous normal mode (UBRRn = (fosc/16BAUD) -1)
    /* Enable Transmitter and Receiver */
    UCSR0B |= _BV(3) | _BV(4);
    /*  sets number to data bits in a frame the RX and TX use.
        8 bits,     1 stop bit,     parity: none           */
    UCSR0C |= _BV(2) | _BV(1);

}

void disable_USART(void) {

    UCSR0B &= ~_BV(4);                                                                          // Receiver disable
    UCSR0B &= ~_BV(3);                                                                          // Transmitter disable

}

void USART_transmitChar(uint8_t character) {

    while (!(UCSR0A & _BV(5)));                                                                 // Wait to empty TX buffer
    UDR0 = character;                                                                           // Transmit data into buffer
    while (!(UCSR0A & _BV(6)));                                                                 // Wait for the transmission to complete

    if (character == '\n')                                                                      // Call recursion to transmit 'CRLF'
        USART_transmitChar('\r');

}

uint8_t USART_receiveChar(void) {

    while (!(UCSR0A & _BV(7)));                                                                 // Wait for RX buffer to fill

    return _SFR_MEM8(0xC6);                                                                     // Get data from Buffer

}


void disable_UsartDataRegisterEmptyInterrupt(void) {
    UCSR0B &= ~_BV(5);
}

void enable_UsartDataRegisterEmptyInterrupt(void) {
    UCSR0B |= _BV(5);
}

void disable_UsartTransmitCompleteInterrupt(void) {
    UCSR0B &= ~_BV(6);
}

void enable_UsartTransmitCompleteInterrupt(void) {
    UCSR0B |= _BV(6);
}

void disable_UsartReceiveCompleteInterrupt(void) {
    UCSR0B &= ~_BV(7);
}

void enable_UsartReceiveCompleteInterrupt(void) {
    UCSR0B |= _BV(7);
}

#endif /* HANDLING_AVR_BITS */

void fullInitialize_USART(uint16_t baudrate, uint8_t dataBits, uint8_t parity, uint8_t stopBits) {

    /* Config UBRR0 for transmission speed */
    _SFR_MEM16(0xC4) = ((F_CPU / (16UL * baudrate)) - 1);
    /* Config data length, parity and stop bits in UCSR0C */

    switch (dataBits) {
    case 6:
        UCSR0C |= (1 << 1);
        break;
    case 7:
        UCSR0C |= (1 << 2);
        break;
    case 8:
        UCSR0C |= (1 << 2) | (1 << 1);
        break;
    case 9:
        UCSR0C |= (1 << 2) | (1 << 1);
        UCSR0B |= (1 << 2);
        break;
    default:
        break;
    }

    if (stopBits == 2) {
        UCSR0C |= (1 << 3);
    }
    else {
        UCSR0C &= ~(1 << 3);
    }

    UCSR0C |= (parity << 4);
    /* Enable Transmitter and Receiver UART in UCSR0B*/
    UCSR0B = (1 << 3) | (1 << 4);
}