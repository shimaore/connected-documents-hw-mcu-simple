/* Functionalities:
*
* - PWM control of motor via L293D;
*/

/* Plarform version. */
#define VERSION 0x00000100

#define F_CPU 8000000

#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/sleep.h>

#define UART_CTS PD2 /* Controlled by module (DCE), indicates the module gives us permission to send data */
#define UART_RTS PD3 /* Controlled by us (DTE), indicate we are ready to receive data */
#define RESETB   PD4
#define SYSTEM_KEY PD5

typedef uint8_t byte;

typedef uint8_t bool;
enum { false = 0, true = 1 };

/* We use 16bits integers for PWM. */
typedef uint16_t word;

/************************ USART *******************************/

inline void usart_9600() {
#undef BAUD
#define BAUD 9600
#include <util/setbaud.h>
UBRRH = UBRRH_VALUE;
UBRRL = UBRRL_VALUE;
#if USE_2X
UCSRA |= _BV(U2X);
#else
UCSRA &= ~_BV(U2X);
#endif
}

/* Bluetooth interface uses the USART in RS-232 mode. */

/* Defaults:
 * - BC-04 uses 9600/1 stop/no parity
 * - HC-05 uses 9600/1 stop/no parity
 * - HM-10 uses 
 */


inline void usart_init() {
  usart_9600();
  UCSRB = _BV(RXCIE)|_BV(RXEN)|_BV(TXCIE)|_BV(TXEN);
  UCSRC = (3<<UCSZ0); // 8 bits
}

enum { received_max = 8 };
volatile uint8_t received[received_max];
volatile uint8_t received_p = 0;

volatile bool received_parse = false;

ISR( USART0_RX_vect, ISR_BLOCK ) {
  /* Get the (inbound) character from the USART. */
  byte c = UDR;
  if( c != '\0' && c != '\n' && c != '\r' && received_p < received_max && !received_parse ) {
    received[received_p++] = c;
  } else {
    received_parse = true;
  }
  toggle_led();
}

volatile uint8_t* send_buffer = NULL;

ISR( USART0_TX_vect, ISR_BLOCK ) {
  if(*send_buffer) {
    UDR = *send_buffer++;
  } else {
    send_buffer = NULL;
  }
  toggle_led();
}

bool usart_send_first( byte t ) {
  /* Send the character out through the USART. */
  while( !( UCSRA & _BV(UDRE))) {
    return false;
  }
  UDR = t;
  return true;
}

bool usart_send_string( const char* s ) {
  // Still transmitting
  if(send_buffer) {
    return false;
  }
  // Nothing to transmit
  if(! *s) {
    return true;
  }
  send_buffer = s+1;
  usart_send_first(*s);
  return true;
}

/********************* PWM control ********************/

/* Patterns may be read from Flash or from RAM over SPI. Patterns must provide a value for base frequency (OCR1A) and duty cycle (OCR1B). */

/*
 * "If the base PWM frequency is actively changed, using the OCR1A as TOP is clearly a better choice."
 * Mode: Fast PWM; OC1A is not connected (used as top) therefor COM1A1=0, COM1A0=0; OC1B is used as PWM output (with output cleared at Compare Match and set at top, therefor COM1B1=1, COM1B0=0); WGM13=1; WGM(12,11,10)=1
 *
 * Also TOV1 must be set so that the interrupt is called to update the new OCR1A and OCR1B at each cycle.
 *
 * See also App Note http://www.atmel.com/Images/doc2542.pdf for filtering (R in series, C between output of R and ground, R=10k, C=100nF for crossover frequency of 1kHz (low-pass filter)).
 */



inline void pwm_init() {
  TCCR1A = _BV(COM1B1) | _BV(WGM10) | _BV(WGM11);
  TCCR1B = _BV(WGM12) | _BV(WGM13);
  TIFR |= _BV(TOV1);
  TIMSK |= _BV(TOIE1);
  // Set OC1A / PB3 as output.
  OC1B_DDR |= _BV(OC1B_BIT);
}

inline void pwm_change( word frequency ) {
  /* The loop description starts with the base frequency of the loop. */
  OCR1A = frequency;

  /* OCR1B is never modified if the loop_length is zero */
  OCR1B = 0;

}

ISR( TIMER1_OVF_vect, ISR_BLOCK ) {
  // OCR1B = read_word(loop_pos);
}

/* Examples of scenarios to test:
 * - base frequency defined but length = 0
 * - base frequency with length = 1 (set duty cycle)
 * - base frequency with length < pwm_buffer_size
 * - base frequency with length = pwm_buffer_size
 * - base frequency with length > pwm_buffer_size (2 blocks)
 * - base frequency with length > pwm_buffer_size (>2 blocks)
 */

byte pos = 0;

inline word mul10( word a ) {
  word a2;
  a <<= 1;
  a2 = a;
  a <<= 2;
  return a+a2;
}

word to_int() {
  word r = 0;
  while(pos < received_p && received[pos] >= '0' && received[pos] <= '9') {
    r = mul10(r);
    r += received[pos++] - '0';
  }
  return r;
}

inline void toggle_led() {
  DDRB |= _BV(PINB0);
  PINB |= _BV(PINB0);
}

int main() {
  usart_init();
  pwm_init();

  toggle_led();

  while(!usart_send_string("AT\r\n")) {
  }

  while(1) {
    word p1;

    if(received_parse) {
      pos = 1;
      switch(received[0]) {
        case 'v':
          p1 = to_int();
          pwm_change(p1);
          usart_send_string("OK\r\n");
          break;
        case 'O':
          if(received[1] == 'K') {
            toggle_led();
          }
          break;
      }
      received_p = 0;
      received_parse = false;
    } else {
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_mode();
    }
  }
}

void exit(int __status) {}
