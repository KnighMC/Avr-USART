/*
 *                                          ATMEGA328PB
 *  24. USART - Universal Synchronous Asynchronous Receiver Transceiver  => datasheet page 262
 *
 *  Table 24-1. Equations for calculating Baud Rate register setting
 *
 *             Operating Mode                Equation for Calculating Baud Rate  Equation for Calculating UBRRn Value
 * __________________________________________________________________________________________________________________
 *  1. Asynchronous normal mode (U2Xn = 0)      BUAD = fosc/(16(UBRRn + 1))        UBRRn = (fosc/(16(BAUD))-1)
 *  2. Asynchronous double speed mode
 *     (U2Xn = 1)                               BUAD = fosc/(8(UBRRn + 1))         UBRRn = (fosc/(8(BAUD))-1)
 *  3. Synchronous master mode                  BUAD = fosc/(8(UBRRn + 1))         UBRRn = (fosc/(2(BAUD))-1)
 *
 *  BAUD: it is the Baud rate(in bits per second,bps).
 *  fosc: System Oscillator Clock Frecuenquency
 *  UBRRn: it is Contents of the UBRRnH and UBRRnL register (0-4095).
 *
 *  Table 24-9. Examples of UBRRn settings for commonly used oscillator frecuencies
 *
 *   fosc = 16.0000MHz    U2Xn = 0
 * ___________________________________
 *  Baud Rate(bps)    UBRRn    Error
 * ___________________________________
 *  9600              103      0.2%
 *  38400             25       0.2%
 *  115200            8       -3,5%
 *
 *   fosc = 18.4320MHz    U2Xn = 0
 * ___________________________________
 *  Baud Rate(bps)    UBRRn    Error
 * ___________________________________
 *  9600              119      0.0%
 *  38400             29       0.0%
 *  115200            9        0.0%
 *
 *
 *
 *
 *  24.12.2 UCSRnA - USART Control and Status Register n A
 *
 *   ___7______6______5_______4_____3______2______1_______0___
 *  | RXCn | TXCn | UDREn | FEn | DORn | UPEn | U2Xn | MPCMn |
 *  ----------------------------------------------------------
 *  (page 283)
 *
 *  24.12.3 UCSRnB - USART Control and Status Register n B
 *
 *   ____7________6________5_______4_______3________2_______1________0___
 *  | RXCIEn | TXCIEn | UDRIEn | RXENn | TXENn | UCSZn2 | RXB8n | TXB8n |
 *  ---------------------------------------------------------------------
 *  (page 285)
 *
 *  24.12.4 UCSRnC - USART Control and Status Register n C
 *
 *   _____7_________6_______5_______4_______3________2________1_________0___
 *  | UMSELn1 | UMSELn0 | UPMn1 | UPMn0 | USBSn | UCSZn1 | UCSZn0 | UCPOLn |
 *  ------------------------------------------------------------------------
 *  (page 287)
 *
 *
 *
 *  24.12.2 => UCSRnA
 *
 *  Pag. 284 Bit 1 - U2Xn: Double the USART Transmission Speed
 *
 *      This bit only has effect for the asynchronous operation. Write this bit to zero when
 *      using synchronous operation.Writing this bit to one will reduce the divisor of the
 *      baud rate divider from 16 to 8 effectively doubling the transfer rate for asynchronous communication
 *
 *  Pag. 284 Bit 0 - MPCMn: Multi-processor Communication Mode
 *
 *      This bit enables the multi-processor communication mode. When the MPCMn bit is written to one,
 *      all the incoming frames received by the USART receiver that do not contain address information
 *      will be ignored. The transmitter is unaffected by the MPCMn setting. For more detailed information
 *      see “Multi-processor Communication Mode”.
 *
 *
 *
 *  24.12.3 => UCSRnB
 *
 *  Pag. 285 Bit 7 - RXCIEn: RX Complete Interrupt Enable n
 *
 *      Writing this bit to one enables interrupt on the RXCn flag. A USART receive complete interrupt will
 *      be generated only if the RXCIEn bit is written to one, the global interrupt flag in SREG is written
 *      to one and the RXCn bit in UCSRnA is set
 *
 *  Pag. 285 Bit 6 - TXCIEn: TX Complete Interrupt Enable n
 *
 *      Writing this bit to one enables interrupt on the TXCn flag. A USART transmit complete interrupt will
 *      be generated only if the TXCIEn bit is written to one, the global interrupt flag in SREG is written
 *      to one and the TXCn bit in UCSRnA is set
 *
 *  Pag. 285 Bit 4 - RXENn: Receiver Enable n
 *
 *      Writing this bit to one enables the USART receiver. The receiver will override normal port operation
 *      for the RxDn pin when enabled. Disabling the receiver will flush the receive buffer invalidating the
 *      FEn, DORn, and UPEn flags
 *
 *  Pag. 285 Bit 3 - TXENn: Transmitter Enable n
 *
 *      Writing this bit to one enables the USART transmitter. The transmitter will override normal port
 *      operation for the TxDn pin when enabled. The disabling of the transmitter (writing TXENn to zero)
 *      will not become effective until ongoing and pending transmissions are completed, i.e., when the
 *      transmit shift register and transmit buffer register do not contain data to be transmitted. When
 *      disabled, the transmitter will no longer override the TxDn port
 *
 *
 *
 *  24.12.4 => UCSRnC
 *
 *  Pag. 287 Bit 7:6 - UMSELn1:0 : USART Mode Select
 *
 *      These bits select the mode of operation of the USARTn as shown in Table 19-4
 *
 *  Pag. 287 Bit 5:4 - UPMn1:0 : Parity Mode
 *
 *      These bits enable and set type of parity generation and check. If enabled, the transmitter will
 *      automatically generate and send the parity of the transmitted data bits within each frame. The
 *      receiver will generate a parity value for the incoming data and compare it to the UPMn setting.
 *      If a mismatch is detected, the UPEn flag in UCSRnA will be set
 *
 *  Pag. 288 Bit 3 - USBSn : Stop Bit Select
 *
 *      This bit selects the number of stop bits to be inserted by the transmitter. The receiver ignores
 *      this setting
 *
 *  Pag. 288 Bit 2:1 - UCSZn1:0 : Character Size
 *
 *      The UCSZn1:0 bits combined with the UCSZn2 bit in UCSRnB sets the number of data bits
 *      (character size) in a frame the receiver and transmitter use
 *
 *
 *  Table 24-10. UMSELn Bits Settings
 *
 *      UMSELn1  UMSELn0        Mode
 * __________________________________________
 *  1.    0        0      Asynchronous USART
 *  2.    0        1      Synchronous USART
 *  3.    1        0      (Reserved)
 *  4.    1        1      Master SPI (MSPIM)
 *
 *  Table 24-11. UPMn Bits Settings
 *
 *       UPMn1    UPMn0              Parity Mode
 * ___________________________________________________
 *  1.    0         0             Disabled
 *  2.    0         1             Reseved
 *  3.    1         0             Enable, even parity
 *  4.    1         1             Enable, odd parity
 *
 *  Table 24-12. USBS Bit Settings
 *
 *       USBSn       Stop Bit(s)
 * _____________________________
 *  1.    0            1-bit
 *  2.    1            2-bit
 *
 *  Table 24-13. UCSZn Bits Settings
 *
 *       UCSZn2  UCSZn1  UCSZn0  Character Size
 * ____________________________________________
 *  1.     0       0       0         5-bit
 *  2.     0       0       1         6-bit
 *  3.     0       1       0         7-bit
 *  4.     0       1       1         8-bit
 *  5.              Reserved
 *  6.              Reserved
 *  7.              Reserved
 *  8.     1       1       1         9-bit
 *
 */

#include "USART_RS232.h"

#ifdef _ATMEGA328PB_H_FILE

#include "Atmega328PB.h"

void USART_Init(unsigned int baud) {

    UBRR0 = ((F_CPU / (16UL * baud)) - 1);                                                      // Set baud rate from equations Asynchonous normal mode (UBRRn = (fosc/16BAUD) -1)
    /* Enable Transmitter and Receiver */
    UCSR0Bbits.RXEN = 1;                                                                        // Receiver enable
    UCSR0Bbits.TXEN = 1;                                                                        // Transmitter enable
    /*  sets number to data bits in a frame the RX and TX use.
        8 bits,     1 stop bit,     parity: none           */
    UCSR0Cbits.UCSZ0 = 1;
    UCSR0Cbits.UCSZ1 = 1;

}

void USART_Stop(void) {

    UCSR0Bbits.RXEN = 0;                                                                        // Receiver disable
    UCSR0Bbits.TXEN = 0;                                                                        // Transmitter disable

}

void USART_Putchar(unsigned char character) {

    while (!UCSR0Abits.UDRE);                                                                   // Wait to empty TX buffer
    UDR0 = character;                                                                           // Transmit data into buffer
    while (!UCSR0Abits.TXC);                                                                    // Wait for the transmission to complete

    if (character == '\n')                                                                      // Call recursion to transmit 'CRLF'
        USART_Putchar('\r');

}

char USART_Getchar(void) {

    while (!UCSR0Abits.RXC);                                                                    // Wait for RX buffer to fill

    return UDR0;                                                                                // Get data from Buffer

}

#else

#include <avr/io.h>

void USART_Init(unsigned int baud) {

    UBRR0 = ((F_CPU / (16UL * baud)) - 1);                                                      // Set baud rate from equations Asynchonous normal mode (UBRRn = (fosc/16BAUD) -1)
    /* Enable Transmitter and Receiver */
    UCSR0B |= _BV(TXEN0) | _BV(RXEN0);
    /*  sets number to data bits in a frame the RX and TX use.
        8 bits,     1 stop bit,     parity: none           */
    UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00);

}

void USART_Stop(void) {

    UCSR0B &= ~_BV(RXEN0);                                                                     // Receiver disable
    UCSR0B &= ~_BV(TXEN0);                                                                     // Transmitter disable                                                                     // Transmitter disable

}

void USART_Putchar(unsigned char character) {

    while (!(UCSR0A & _BV(UDRE0)));                                                             // Wait to empty TX buffer
    UDR0 = character;                                                                           // Transmit data into buffer
    while (!(UCSR0A & _BV(TXC0)));                                                              // Wait for the transmission to complete

    if (character == '\n')                                                                      // Call recursion to transmit 'CRLF'
        USART_Putchar('\r');

}

char USART_Getchar(void) {

    while (!(UCSR0A & _BV(RXC0)));                                                              // Wait for RX buffer to fill

    return UDR0;                                                                                // Get data from Buffer

}

#endif /* complement library */