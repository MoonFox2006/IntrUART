#pragma once

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define SLEEP_IN_FLUSH

#define UART_DDR  DDRB
#define UART_PORT PORTB
#define UART_PIN  PINB

#define UART_TX 0
#define UART_RX 1 // May be commented

#define UART_SPEED  57600 // 9600
#define UART_PERIOD (F_CPU / UART_SPEED)

#if UART_PERIOD > 65535
#define UART_PRESCALER  ((1 << CS02) | (1 << CS00))
#undef UART_PERIOD
#define UART_PERIOD (F_CPU / UART_SPEED / 1024)
#elif UART_PERIOD > 16383
#define UART_PRESCALER  (1 << CS02)
#undef UART_PERIOD
#define UART_PERIOD (F_CPU / UART_SPEED / 256)
#elif UART_PERIOD > 2047
#define UART_PRESCALER  ((1 << CS01) | (1 << CS00))
#undef UART_PERIOD
#define UART_PERIOD (F_CPU / UART_SPEED / 64)
#elif UART_PERIOD > 255
#define UART_PRESCALER  (1 << CS01)
#undef UART_PERIOD
#define UART_PERIOD (F_CPU / UART_SPEED / 8)
#else
#define UART_PRESCALER  (1 << CS00)
#endif

#define UART_EMPTY  0xFF

#ifdef UART_RX
#define UART_IN_SIZE  1 // 7
#define UART_ERROR  0
#define UART_OVER   1
#if UART_IN_SIZE == 1
#define UART_FULL   2
#endif
#endif

volatile uint8_t _out;
volatile uint8_t _outbitpos;
#ifdef UART_RX
volatile uint8_t __in;
volatile uint8_t _inbitpos;
#if UART_IN_SIZE > 1
volatile uint8_t _in[UART_IN_SIZE];
volatile uint8_t _inlen;
volatile uint8_t _inpos;
#else
volatile uint8_t _in;
#endif
volatile uint8_t _instatus;
#endif

static const char _CRLF[] PROGMEM = "\r\n";

void uart_init() {
  UART_DDR |= (1 << UART_TX);
#ifdef UART_RX
  UART_DDR &= ~(1 << UART_RX);
  UART_PORT |= (1 << UART_TX) | (1 << UART_RX);
#else
  UART_PORT |= (1 << UART_TX);
#endif

  TCCR0A = (1 << WGM01); // CTC
  OCR0A = UART_PERIOD - 1;
#ifdef UART_RX
  OCR0B = UART_PERIOD / 2 - 1;
#endif

  TCCR0B = UART_PRESCALER; // Start Timer0

  _outbitpos = UART_EMPTY;
#ifdef UART_RX
  _inbitpos = UART_EMPTY;
#if UART_IN_SIZE > 1
  _inlen = _inpos = 0;
#endif
  _instatus = 0;

  PCMSK |= (1 << UART_RX);
  GIFR |= (1 << PCIF);
  GIMSK |= (1 << PCIE);
#endif

  sei();
}

void uart_flush() {
  while (_outbitpos != UART_EMPTY) {
#ifdef SLEEP_IN_FLUSH
    sleep_mode();
#endif
  }
}

void uart_write(char c) {
  uart_flush();
  _out = c;
  _outbitpos = 0;
  UART_PORT &= ~(1 << UART_TX); // Start bit (always 0)
  TCNT0 = 0;
  TIFR0 = (1 << OCF0A);
  TIMSK0 |= (1 << OCIE0A);
}

void uart_print(const char *str) {
  while (*str) {
    uart_write(*str++);
  }
}

void uart_print_P(PGM_P str) {
  while (pgm_read_byte(str)) {
    uart_write(pgm_read_byte(str++));
  }
}

inline void uart_println() {
  uart_print_P(_CRLF);
}

void uart_hex(uint8_t hex) {
  uint8_t d;

  d = hex >> 4;
  uart_write(d > 9 ? 'A' + d - 10 : '0' + d);
  d = hex & 0x0F;
  uart_write(d > 9 ? 'A' + d - 10 : '0' + d);
}

void uart_hex(uint16_t hex) {
  uart_hex((uint8_t)(hex >> 8));
  uart_hex((uint8_t)hex);
}

void uart_print(uint8_t num) {
  for (uint8_t divider = 100; divider > 1; divider /= 10) {
    if (divider <= num) {
      uart_write('0' + (num / divider) % 10);
    }
  }
  uart_write('0' + num % 10);
}

void uart_print(int8_t num) {
  if (num < 0) {
    uart_write('-');
    num = -num;
  }
  uart_print((uint8_t)num);
}

void uart_print(uint16_t num) {
  for (uint16_t divider = 10000; divider > 1; divider /= 10) {
    if (divider <= num) {
      uart_write('0' + (num / divider) % 10);
    }
  }
  uart_write('0' + num % 10);
}

void uart_print(int16_t num) {
  if (num < 0) {
    uart_write('-');
    num = -num;
  }
  uart_print((uint16_t)num);
}

void uart_print(uint32_t num) {
  for (uint32_t divider = 1000000000; divider > 1; divider /= 10) {
    if (divider <= num) {
      uart_write('0' + (num / divider) % 10);
    }
  }
  uart_write('0' + num % 10);
}

void uart_print(int32_t num) {
  if (num < 0) {
    uart_write('-');
    num = -num;
  }
  uart_print((uint32_t)num);
}

#ifdef UART_RX
inline uint8_t uart_available() {
#if UART_IN_SIZE > 1
  return _inlen;
#else
  return _instatus & (1 << UART_FULL) ? 1 : 0;
#endif
}

int16_t uart_read() {
#if UART_IN_SIZE > 1
  if (_inlen) {
#else
  if (_instatus & (1 << UART_FULL)) {
#endif
    uint8_t sreg, result;

    sreg = SREG;
    cli();
#if UART_IN_SIZE > 1
    if (_inlen > _inpos)
      result = UART_IN_SIZE + _inpos - _inlen;
    else
      result = _inpos - _inlen;
    result = _in[result];
    --_inlen;
#else
    result = _in;
    _instatus &= ~(1 << UART_FULL);
#endif
    SREG = sreg;
    return result;
  }
  return -1;
}

inline bool uart_iserror() {
  return (_instatus & (1 << UART_ERROR)) != 0;
}

inline bool uart_isover() {
  return (_instatus & (1 << UART_OVER)) != 0;
}
#endif

class UART {
public:
  static void begin() {
    uart_init();
  }
  static void write(char c) {
    uart_write(c);
  }
  static void print(const char *str) {
    uart_print(str);
  }
  static void print_P(PGM_P str) {
    uart_print_P(str);
  }
  static void println() {
    uart_println();
  }
  static void print(char c) {
    uart_write(c);
  }
  static void print(uint8_t n) {
    uart_print(n);
  }
  static void print(int8_t n) {
    uart_print(n);
  }
  static void print(uint16_t n) {
    uart_print(n);
  }
  static void print(int16_t n) {
    uart_print(n);
  }
  static void print(uint32_t n) {
    uart_print(n);
  }
  static void print(int32_t n) {
    uart_print(n);
  }
  static void hex(uint8_t n) {
    uart_hex(n);
  }
  static void hex(uint16_t n) {
    uart_hex(n);
  }
  static void flush() {
    uart_flush();
  }
#ifdef UART_RX
  static uint8_t available() {
    return uart_available();
  }
  static int16_t read() {
    return uart_read();
  }
  static bool iserror() {
    return uart_iserror();
  }
  static bool isover() {
    return uart_isover();
  }
#endif
};

ISR(TIM0_COMPA_vect, ISR_NAKED) {
  asm volatile ("push r18;2\n"
    "push r19;2\n"
    "in r18, __SREG__;1\n"
    "lds r19, _outbitpos;2\n"
    "cpi r19, 9;1\n"
    "brlo l_data%=;1/2\n"
    "in r19, %[timsk];1\n"
    "cbr r19, (1 << %[ocie0a]);1\n"
    "out %[timsk], r19;1\n"
    "ldi r19, %[empty];1\n"
    "rjmp l_ret%=;2\n"

    "l_data%=:\n"
    "cpi r19, 8;1\n"
    "breq l_out1%=\n"
    "push r20;2\n"
    "lds r20, _out;2\n"
    "lsr r20;1\n"
    "sts _out, r20;2\n"
    "pop r20;2\n"
    "brcs l_out1%=;1/2\n"
    "cbi %[port], %[tx];1\n"
    "rjmp l_inc%=;2\n"

    "l_out1%=:\n"
    "sbi %[port], %[tx];1\n"

    "l_inc%=:\n"
    "inc r19;1\n"

    "l_ret%=:\n"
    "sts _outbitpos, r19;2\n"
    "out __SREG__, r18;1\n"
    "pop r19;2\n"
    "pop r18;2\n"
    "reti;4\n"
    : : [port] "I" (_SFR_IO_ADDR(UART_PORT)), [tx] "I" (UART_TX), [timsk] "I" (_SFR_IO_ADDR(TIMSK0)), [ocie0a] "I" (OCIE0A), [empty] "M" (UART_EMPTY));
}

/*
ISR(TIM0_COMPA_vect) {
  if (_outbitpos <= 8) { // Data or stop bits
    if ((_outbitpos == 8) || (_out & 0x01)) // Stop bit or 1
      UART_PORT |= (1 << UART_TX);
    else
      UART_PORT &= ~(1 << UART_TX);
    _out >>= 1;
    ++_outbitpos;
  } else { // End of stop bit
    TIMSK0 &= ~(1 << OCIE0A);
    _outbitpos = UART_EMPTY;
  }
}
*/

#ifdef UART_RX
ISR(PCINT0_vect) {
  if (! (UART_PIN & (1 << UART_RX))) { // Start bit (0)
    TCNT0 = 0;
    TIFR0 |= (1 << OCF0B);
    TIMSK0 |= (1 << OCIE0B);
    GIMSK &= ~(1 << PCIE);
    _inbitpos = UART_EMPTY;
  }
}

ISR(TIM0_COMPB_vect, ISR_NAKED) {
  asm volatile ("push r18;2\n"
    "push r19;2\n"
    "push r20;2\n"
    "in r18, __SREG__;1\n"
    "in r19, %[pin];1\n"
    "andi r19, (1 << %[rx]);1\n"
    "lds r20, _inbitpos;2\n"
    "cpi r20, 8;1\n"
    "brlo l_data%=;1/2\n"
    "breq l_stopbit%=;1/2\n"
    "tst r19;1\n"
    "brne l_startbiterror%=;1/2\n"
    "clr r20;1\n"
    "sts __in, r20;2\n"
    "sts _inbitpos, r20;2\n"
    "rjmp l_ret%=;2\n"

    "l_startbiterror%=:\n"
    "lds r20, _instatus;2\n"
    "rjmp l_error%=;2\n"

    "l_data%=:\n"
    "push r21;2\n"
    "lds r21, __in;2\n"
    "lsr r21;1\n"
    "tst r19;1\n"
    "breq l_skipbit%=\n"
    "ori r21, 0x80;1\n"

    "l_skipbit%=:\n"
    "sts __in, r21;2\n"
    "pop r21;2\n"
    "inc r20;1\n"
    "sts _inbitpos, r20;2\n"
    "rjmp l_ret%=;2\n"

    "l_stopbit%=:\n"
    "lds r20, _instatus;2\n"
    "tst r19;1\n"
    "breq l_error%=;1/2\n"
#if UART_IN_SIZE > 1
    "lds r19, _inlen;2\n"
    "cpi r19, %[size];1\n"
    "brsh l_over%=;1/2\n"
    "push r21;2\n"
    "push r22;2\n"
    "push r26;2\n"
    "push r27;2\n"
    "lds r21, _inpos;2\n"
    "ldi r26, _in;1\n"
    "clr r27;1\n"
    "add r26, r21;1\n"
//    "adc r27, r27;1\n"
    "lds r22, __in;2\n"
    "st X, r22;2\n"
    "pop r27;2\n"
    "pop r26;2\n"
    "pop r22;2\n"
    "inc r21;1\n"
    "cpi r21, %[size];1\n"
    "brlo l_skipclearpos%=;1/2\n"
    "clr r21;1\n"

    "l_skipclearpos%=:\n"
    "sts _inpos, r21;2\n"
    "pop r21;2\n"
    "inc r19;1\n"
    "sts _inlen, r19;2\n"
#else
    "sbrc r20, (1 << %[full]);1\n"
    "rjmp l_over%=;2\n"
    "lds r19, __in;2\n"
    "sts _in, r19;2\n"
    "sbr r20, (1 << %[full]);1\n"
#endif
    "cbr r20, (1 << %[over]);1\n"
    "rjmp l_noerror%=;2\n"

    "l_over%=:\n"
    "sbr r20, (1 << %[over]);1\n"

    "l_noerror%=:\n"
    "cbr r20, (1 << %[error]);1\n"
    "rjmp l_storestatus%=;2\n"

    "l_error%=:\n"
    "sbr r20, (1 << %[error]);1\n"

    "l_storestatus%=:\n"
    "sts _instatus, r20;2\n"
    "in r20, %[timsk];1\n"
    "cbr r20, (1 << %[ocie0b]);1\n"
    "out %[timsk], r20;1\n"
    "in r20, %[gimsk];1\n"
    "sbr r20, (1 << %[pcie]);1\n"
    "out %[gimsk], r20;1\n"

    "l_ret%=:\n"
    "out __SREG__, r18;1\n"
    "pop r20;2\n"
    "pop r19;2\n"
    "pop r18;2\n"
    "reti;4\n"
#if UART_IN_SIZE > 1
    : : [pin] "I" (_SFR_IO_ADDR(UART_PIN)), [rx] "I" (UART_RX), [timsk] "I" (_SFR_IO_ADDR(TIMSK0)), [ocie0b] "I" (OCIE0B), [gimsk] "I" (_SFR_IO_ADDR(GIMSK)), [pcie] "I" (PCIE), [size] "M" (UART_IN_SIZE), [error] "I" (UART_ERROR), [over] "I" (UART_OVER));
#else
    : : [pin] "I" (_SFR_IO_ADDR(UART_PIN)), [rx] "I" (UART_RX), [timsk] "I" (_SFR_IO_ADDR(TIMSK0)), [ocie0b] "I" (OCIE0B), [gimsk] "I" (_SFR_IO_ADDR(GIMSK)), [pcie] "I" (PCIE), [error] "I" (UART_ERROR), [over] "I" (UART_OVER), [full] "I" (UART_FULL));
#endif
}

/*
ISR(TIM0_COMPB_vect) {
  bool inbit = (UART_PIN & (1 << UART_RX)) != 0;

  if (_inbitpos < 8) {
//    __in |= (inbit << _inbitpos);
    __in >>= 1;
    if (inbit)
      __in |= 0x80;
    ++_inbitpos;
  } else if (_inbitpos == 8) { // Stop bit (must be 1)
    if (inbit) {
#if UART_IN_SIZE > 1
      if (_inlen < UART_IN_SIZE) {
        _in[_inpos] = __in;
        if (++_inpos >= UART_IN_SIZE)
          _inpos = 0;
        ++_inlen;
#else
      if (! (_instatus & (1 << UART_FULL))) {
        _in = __in;
        _instatus |= (1 << UART_FULL);
#endif
        _instatus &= ~(1 << UART_OVER);
      } else {
        _instatus |= (1 << UART_OVER);
      }
      _instatus &= ~(1 << UART_ERROR);
    } else { // Error!
      _instatus |= (1 << UART_ERROR);
    }
    TIMSK0 &= ~(1 << OCIE0B);
    GIMSK |= (1 << PCIE);
  } else { // Start bit (must be 0)
    if (inbit) { // Error!
      TIMSK0 &= ~(1 << OCIE0B);
      GIMSK |= (1 << PCIE);
      _instatus |= (1 << UART_ERROR);
    } else {
      __in = 0;
      _inbitpos = 0;
    }
  }
}
*/
#endif
