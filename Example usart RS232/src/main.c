#include "USART_RS232.h"

int main(int argc, char const* argv[])
{

  USART_Init(9600);               // Initialized Baud Rate in 9600

  /*  send characters of "HOLA\n" */
  USART_Putchar('H');
  USART_Putchar('O');
  USART_Putchar('L');
  USART_Putchar('A');
  USART_Putchar('\n');

  for (;;) {

    char data = USART_Getchar();  //Receive data

    if (data == '1') {

      /*  send characters of "Ok\n  "*/
      USART_Putchar('O');
      USART_Putchar('k');
      USART_Putchar('\n');

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