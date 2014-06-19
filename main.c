/* Functionalities:
*
* - PWM control of motor via L293D;
*/

/* Plarform version. */
#define VERSION 0x00000100

/* Frequency of the quartz */
#define F_QUARTZ 8000000UL
/* Desired divisor */
#define F_CLOCKPR 0
#define F_CPU (F_QUARTZ >> F_CLOCKPR)

#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/sleep.h>

inline void toggle_led0() {
  DDRB |= _BV(DDB0);
  PINB |= _BV(PINB0);
}

inline void toggle_led1() {
  DDRB |= _BV(DDB1);
  PINB |= _BV(PINB1);
}

inline void toggle_led2() {
  DDRB |= _BV(DDB2);
  PINB |= _BV(PINB2);
}

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
  TXD_DDR |= _BV(TXD_BIT); // TxD is output
  usart_9600();
  UCSRB = _BV(RXCIE)|_BV(RXEN)|_BV(TXCIE)|_BV(TXEN);
  UCSRC = _BV(UCSZ1) | _BV(UCSZ0); // | _BV(USBS); // 8 bits, 2 stop -- 2 stops avoid issues with clock slips
}

/* USART RxD */

enum { received_max = 8 };
volatile uint8_t received[received_max];
volatile uint8_t received_p;

volatile bool received_parse;

ISR( USART0_RX_vect, ISR_BLOCK ) {
  /* Get the (inbound) character from the USART. */
  byte c = UDR;
  /* Stop at end of line or buffer full. */
  if( c != '\0' && c != '\n' && c != '\r' && received_p < received_max && !received_parse ) {
    received[received_p++] = c;
  } else {
    received_parse = true;
  }
}

/* USART TxD */

enum { send_max = 16 };
volatile uint8_t send[send_max];
volatile uint8_t send_p;
volatile uint8_t send_len;

ISR( USART0_TX_vect, ISR_BLOCK ) {
  // Nothing, we're just here to trigger the sleep() code.
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
  // Set OC1B as output.
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

#include "util/delay.h"

int main() {
  toggle_led1();
  cli();
  // Ensure the prescaler is disabled.
  CLKPR = _BV(CLKPCE);
  CLKPR = F_CLOCKPR;

  // Ensure the variables are initialized.
  received_p = 0;
  received_parse = false;
  send_p = 0;
  send_len = 0;

  usart_init();
  pwm_init();
  set_sleep_mode(SLEEP_MODE_IDLE);
  sei();

  // Wait for the module to be ready.
  _delay_ms(2000);

  // Send configuration out.
  send_len = 15;
  send_p = 0;
  send[0] = 'T';
  send[1] = '+';
  send[2] = 'N';
  send[3] = 'A';
  send[4] = 'M';
  send[5] = 'E';
  send[6] = '=';
  send[7] = 'F';
  send[8] = 'r';
  send[9] = 'i';
  send[10] = 'f';
  send[11] = 'r';
  send[12] = 'i';
  send[13] = '\r';
  send[14] = '\n';
  UDR = 'A';

  while(1) {
    /* Wait for transmit-complete or receive */
    sleep_mode();

    if(send_len) {
      if(send_p < send_len) {
        DDRB = 0xff;
        PORTB = send_p;
        byte c = send[send_p];
        send_p++;
        _delay_ms(30);
        loop_until_bit_is_set(UCSRA,UDRE);
        // UDR = c;
        UDR = send_p;
      } else {
        // String was fully sent.
        send_len = 0;
        send_p = 0;
      }
    }
    if(!send_len) {
      if(received_parse) {
        word p1;
        pos = 1;
        switch(received[0]) {
          case 'v':
            p1 = to_int();
            pwm_change(p1);
            send_len = 3;
            send_p = 0;
            send[0] = 'K';
            send[1] = '\r';
            send[2] = '\n';
            UDR = 'O';
            break;
          case 'O':
            if(received[1] == 'K') {
              toggle_led1();
              send_len = 4;
              send_p = 0;
              send[0] = 'e';
              send[1] = 's';
              send[2] = '\r';
              send[3] = '\n';
              UDR = 'Y';
            }
            break;
        }
        received_p = 0;
        received_parse = false;
      }
    }

  }
}

void exit(int __status) {}
