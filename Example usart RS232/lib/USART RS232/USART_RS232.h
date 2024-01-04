/******************************************************************************
 * File: USART_RS232                 @file   USART_RS232.h
 * Author: KnightMC                  @author KnightMC
 * Comments: file of USART module.   @brief  Header file of AVR USART module.
 * Revision history: 02/01/2024
 ******************************************************************************
 * @brief        Interface declaration of USART module.
 * @attention    See implementation file for information about this module.
 *
 * Copyright (c) 2024 KnightMC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Sofware, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABLILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE
 * *****************************************************************************
 */

#ifndef USART_RS232_H_FILE
#define USART_RS232_H_FILE  "library usart V2.0\n"

#include <stdint.h>
#include <stdbool.h>

 /******************************************************************************
  * if you want to use the Handling AVR Bits library for USART programming
  * please uncomment the following #Define. Remember that using the Handling
  * AVR Bit Library does not increase the performance of the code, it is only a
  * tool to have better control of the configuration bits and will not have any
  * impact on RAM or Flash memory.
  *
  * Include Handling AVR Bits library for use with this project. Clone library
  * from page in GitHub: KnightMC/Handling-avr-bits
  ******************************************************************************/
  // #define HANDLING_AVR_BITS
  /*****************************************************************************/

#define USART_PARITY_NONE 0
#define USART_PARITY_EVEN 2
#define USART_PARITY_ODD  3

typedef struct {

  volatile uint8_t UCSR0A;      // Registro de control A USART0             Address offset: 0x00
  volatile uint8_t UCSR0B;      // Registro de control B USART0             Address offset: 0x01
  volatile uint8_t UCSR0C;      // Registro de control C USART0             Address offset: 0x02
  volatile uint8_t reserved[3]; // Espacio reservado.                       Address offset: 0x03
  volatile uint8_t UDR0;        // Registro de datos USART0                 Address offset: 0x06
  volatile uint16_t UBRR0;      // Registro de velocidad de baudios USART0  Address offset: 0x07

} USART_RegisterMap;

#define USART0_BASE   0xC0                                                // Address of USART0
#define USART0        ((USART_RegisterMap *)USART0_BASE)                  // Map Address USART0

/**********************************************************************************
 * Example about the use struct USART_RegisterMap;
 *
 * // Set BaudRate to 9600
 * USART0->UBRR0 = 51;
 * // Enable Transmiter and Receiver for Usart, RX Complete Interrupt Enable
 * USART0->UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
 * // Lenght to transmit: 8 bits,  Parity: NONE,  Stop bit: 1
 * USART0->UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
 **********************************************************************************/

 /*****************************************************************************************
  * @brief Initialize full USART configuration
  * @param baudrate  Data value (bits) to transmit per second
  * @param dataBits  Number of bits to transmit
  * @param parity    Additional bits to evaluate data and look for errors in the transmission.
  * @param stopBits  Bits that inform about the stop of the transmission
  *****************************************************************************************/
void fullInitialize_USART(uint16_t baudrate, uint8_t dataBits, uint8_t parity, uint8_t stopBits);

/*****************************************************************************************
 * @brief Initialize USART with default configuration
 * @param baudrate  Data value (bits) to transmit per second
 * @note  Default values: dataBits:8 parity: NONE stopBits: 1
 *****************************************************************************************/
void initialize_USART(uint16_t baudrate);

/******************************************************************************************
 * @brief Transmit data (char) with TX
 * @param character Data to transmit (unsigned char)
 *****************************************************************************************/
void USART_transmitChar(uint8_t character);

/******************************************************************************************
 * @brief   Receive data (char) with USART
 * @retval  Data received with RX (unsigned char)
 *****************************************************************************************/
uint8_t USART_receiveChar(void);

/******************************************************************************************
 * @brief Disable Communication USART
 * @note  Disable pins RX and TX for USART communication
 *****************************************************************************************/
void disable_USART(void);

/******************************************************************************************
 * @brief Disable Usart Data Register Empty Interrupt
 * @note  The register is UCSR0B => UDRIE
 *****************************************************************************************/
void disable_UsartDataRegisterEmptyInterrupt(void);

/******************************************************************************************
 * @brief Enable Usart Data Register Empty Interrupt
 * @note  The register is UCSR0B => UDRIE
 *****************************************************************************************/
void enable_UsartDataRegisterEmptyInterrupt(void);

/******************************************************************************************
 * @brief Disable Usart Transmit Complete Interrupt
 * @note  The register is UCSR0B => TXCIE
 *****************************************************************************************/
void disable_UsartTransmitCompleteInterrupt(void);

/******************************************************************************************
 * @brief Enable Usart Transmit Complete Interrupt
 * @note  The register is UCSR0B => TXCIE
 *****************************************************************************************/
void enable_UsartTransmitCompleteInterrupt(void);

/******************************************************************************************
 * @brief Disable Usart Receive Complete Interrupt
 * @note  The register is UCSR0B => TXCIE
 *****************************************************************************************/
void disable_UsartReceiveCompleteInterrupt(void);

/******************************************************************************************
 * @brief Enable Usart Receive Complete Interrupt
 * @note  The register is UCSR0B => RXCIE
 *****************************************************************************************/
void enable_UsartReceiveCompleteInterrupt(void);

#endif /* USART_RS232_H_FILE */