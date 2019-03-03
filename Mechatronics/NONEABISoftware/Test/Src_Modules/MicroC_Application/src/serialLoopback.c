/*
A simple test of serial-port functionality.
Takes in a character at a time and sends it right back out,
 displaying the ASCII value on the LEDs.
*/

// ------- Preamble -------- //
#include <avr/io.h>
#include <util/delay.h>
#include "USART.h"

int main(void) {

  // -------- Inits --------- //
  initUSART();
  printString("Hello World!\n");                          /* to test */

  // ------ Event loop ------ //
  while (1) {


  }                                                  /* End event loop */
  return 0;
}
