#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "uart_intr.h"

UART uart;

int main() {
  set_sleep_mode(SLEEP_MODE_IDLE);

  uart.begin();
  _delay_ms(1);
  uart.print_P(PSTR("Attiny13 UART Echo Demo\r\n"));

  char str[16];
  uint8_t len = 0;

  for (;;) {
    while (uart.available()) {
      char c = uart.read();

      if (c == '\r') {
        str[len] = '\0';
        uart.print(str);
        uart.println();
        len = 0;
      } else {
        str[len++] = c;
        if (len >= sizeof(str) - 1) {
          str[sizeof(str) - 1] = '\0';
          uart.print(str);
//          uart.println();
          len = 0;
        }
      }
    }

    sleep_mode();
  }
}
