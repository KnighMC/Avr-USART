#include "USART_RS232.h"

int main(int argc, char const* argv[])
{

    fullInitialize_USART(9600, 8, 0, 1);               // Initialized Baud Rate in 9600

    /*  send characters of "HOLA\n" */
    USART_transmitChar('H');
    USART_transmitChar('O');
    USART_transmitChar('L');
    USART_transmitChar('A');
    USART_transmitChar('\n');

    for (;;) {

        char data = USART_receiveChar();  //Receive data

        if (data == '1') {

            /*  send characters of "Ok\n  "*/
            USART_transmitChar('O');
            USART_transmitChar('k');
            USART_transmitChar('\n');

        }

    }

    return 0;
}

/************************************************************
 * For transmit string the function is:
 *
 * void Puts(char* string) {
 *
 *    while (*string != 0x00) {
 *      USART_Putchar(*string);
 *      string++;
 *    }
 *
 * }
 **********************************************************/