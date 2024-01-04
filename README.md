# Avr-USART
this library include basic functions for configuration and control of USART (normal funtionality) dedicate to avr microcontrollers

# AVAILABLE DEVICES

- ATMEGA328PB
- ATMEGA328P

# CONTENT

- INSTALLATION
- USE
- LICENSE

# INSTALLATION

clone the repository from KnightMC/Handling-avr-bits add the USART RS232 folder to your project

# USE

  * if you want to use the Handling AVR Bits library for USART programming
  * please uncomment the following #Define. Remember that using the Handling
  * AVR Bit Library does not increase the performance of the code, it is only a
  * tool to have better control of the configuration bits and will not have any
  * impact on RAM or Flash memory.
  *
  * Include Handling AVR Bits library for use with this project. Clone library
  * from page in GitHub: KnightMC/Handling-avr-bits

#define HANDLING_AVR_BITS

Example when HANDLING_BITS is enable:

  void USART_transmitChar(uint8_t character) {

      while (!UCSR0Abits.UDRE);
      UDR0 = character;
      while (!UCSR0Abits.TXC);
    
      if (character == '\n')
          USART_Putchar('\r');

  }

- Remember is Example, the function is available into library

Example when HANDLING_BITS is disable:

  void USART_Putchar(unsigned char character) {
  
      while (!(UCSR0A & _BV(UDRE0)));
      UDR0 = character;
  
  }

# LICENSE

 /******************************************************************************
 * File: USART_RS232
 * Author: KnightMC
 * Comments: Header file of AVR USART module.
 * Revision history: 03/01/2024
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
 * ******************************************************************************/