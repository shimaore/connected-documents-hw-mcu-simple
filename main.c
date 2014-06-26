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

enum { received_max = 16 };
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
 *
 * OC1B is used as output; TCNT1 will count from 0 (BOTTOM) to OCR1A (TOP), the output will be set on OC1B while OCR1B isn't reached (charge interval), and cleared above that value (discharge interval); this is Compare Output Mode 2 in Fast PWM mode (Table 12-3). ICR is not used. We're using Fast PWM Mode (Section 12.8.3) in mode 15 (Fast PWM, OCR1A is TOP, Update on TOP + TOV1 set on TOP).
 *
 * (Also TOV1 must be set so that the interrupt is called to update the new OCR1A and OCR1B at each cycle.)
 *
 * See also App Note http://www.atmel.com/Images/doc2542.pdf for filtering (R in series, C between output of R and ground, R=10k, C=100nF for crossover frequency of 1kHz (low-pass filter)).
 */

inline void pwm_init() {
  TCCR1A = _BV(COM1B1) /* Clear OC1B on compare match, set at TOP; PWM mode 2 = non-inverted */ | _BV(WGM10) | _BV(WGM11) /* PWM Mode 15 */ ;
  TCCR1B = _BV(WGM12) | _BV(WGM13) /* PWM Mode 15 */ | _BV(CS10) /* Clock = clk_I/O with no prescaling */ ;
}

inline void pwm_change( word limit, word top ) {
  // Set OC1A as output iff top is not zero.
  if(top) {
    OC1B_DDR |= _BV(OC1B_BIT);
  } else {
    OC1B_DDR &= ~ _BV(OC1B_BIT);
  }
  OCR1B = limit;
  OCR1A = top;
}

/* Examples of scenarios to test:
 * - base frequency defined but length = 0
 * - base frequency with length = 1 (set duty cycle)
 * - base frequency with length < pwm_buffer_size
 * - base frequency with length = pwm_buffer_size
 * - base frequency with length > pwm_buffer_size (2 blocks)
 * - base frequency with length > pwm_buffer_size (>2 blocks)
 */

volatile byte pos;

inline word mul10( word a ) {
  word a2;
  a <<= 1;
  a2 = a;
  a <<= 2;
  return a+a2;
}

char hex( uint8_t c ) {
  c &= 0x0f;
  if(c<10) {
    return c+'0';
  } else {
    return c-10+'A';
  }
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
  enum {
    wait_for_module,
    show_name,
    wait_for_connection,
    connected
  } next_step;

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
  pos = 0;

  usart_init();
  pwm_init();
  set_sleep_mode(SLEEP_MODE_IDLE);
  sei();

  // Start sending data out.
  next_step = wait_for_module;

  while(1) {
    /* Wait for transmit-complete or receive */
    sleep_mode();

    /* Handle transmissions first */
    if(send_len) {
      if(send_p < send_len) {
        byte c = send[send_p];
        send_p++;
        loop_until_bit_is_set(UCSRA,UDRE);
        UDR = c;
      } else {
        // String was fully sent.
        send_len = 0;
        send_p = 0;
      }
    }

    /* Handle reception of data from bluetooth */
    if(!send_len) {
      if(received_parse) {
        switch(received[0]) {
          case '!': {
              switch(received[1]) {
                case 'p':
                  send_len = 2;
                  send_p = 0;
                  send[0] = 'P';
                  send[1] = '\n';
                  UDR = '!';
                  break;
                case 'v':
                  {
                    pos = 2;
                    word limit = to_int();
                    pos++; // skip ','
                    word top = to_int();
                    // Set a 50% duty cycle for testing.
                    send_len = 11;
                    send_p = 0;
                    if(limit <= top) {
                      pwm_change(limit,top);
                      send[0] = 'V';
                    } else {
                      send[0] = 'N';
                    }
                    send[1] = hex(limit >> 12);
                    send[2] = hex(limit >>  8);
                    send[3] = hex(limit >>  4);
                    send[4] = hex(limit >>  0);
                    send[5] = ',';
                    send[6] = hex(top >> 12);
                    send[7] = hex(top >>  8);
                    send[8] = hex(top >>  4);
                    send[9] = hex(top >>  0);
                    send[10] = '\n';
                    UDR = '!';
                  }
                  break;
                case 's':
                  pwm_change(0,0);
                  send_len = 2;
                  send_p = 0;
                  send[0] = 'S';
                  send[1] = '\n';
                  UDR = '!';
                  break;
              }
            }
            break;
          case '+':
            // +READY
            if(received[1] == 'R' && received_p == 6) {
              next_step = show_name;
              send_len = 5;
              send_p = 0;
              send[0] = '\n';
              send[1] = 'A';
              send[2] = 'T';
              send[3] = '\r';
              send[4] = '\n';
              UDR = '\r';
            }
            // +PAIRABLE
            // +NAME=
            if(received[1] == 'N' && received[5] == '=') {
              if(received[6] != 'F') {
                send_len = 14;
                send_p = 0;
                send[0] = 'T';
                send[1] = '+';
                send[2] = 'N';
                send[3] = 'A';
                send[4] = 'M';
                send[5] = 'E';
                send[6] = 'F';
                send[7] = 'r';
                send[8] = 'i';
                send[9] = 'f';
                send[10] = 'r';
                send[11] = 'i';
                send[12] = '\r';
                send[13] = '\n';
                UDR = 'A';
              } else {
                send_len = 7;
                send_p = 0;
                send[0] = 'T';
                send[1] = '+';
                send[2] = 'P';
                send[3] = 'I';
                send[4] = 'N';
                send[5] = '\r';
                send[6] = '\n';
                UDR = 'A';
              }
            }
            // +PIN=
            if(received[1] == 'P' && received[4] == '=') {
              if(received[5] != '6') {
                send_len = 11;
                send_p = 0;
                send[0] = 'T';
                send[1] = '+';
                send[2] = 'P';
                send[3] = 'I';
                send[4] = 'N';
                send[5] = '6';
                send[6] = '9';
                send[7] = '6';
                send[8] = '9';
                send[9] = '\r';
                send[10] = '\n';
                UDR = 'A';
              } else {
                next_step = wait_for_connection;
              }
            }
            // +CONNECTING<<00:1B:DC:0F:E3:69
            // +CONNECTED
            if(received[1] == 'R' && received_p == 10) {
              next_step = connected;
            }
            break;
          case 'O':
            if(received[1] == 'K') {
              toggle_led1();
              if(next_step == show_name) {
                send_len = 8;
                send_p = 0;
                send[0] = 'T';
                send[1] = '+';
                send[2] = 'N';
                send[3] = 'A';
                send[4] = 'M';
                send[5] = 'E';
                send[6] = '\r';
                send[7] = '\n';
                UDR = 'A';
              }
            }
            break;
        }
        received_p = 0;
        for( uint8_t i = 0; i < received_max; i++ ) {
          received[i] = 0;
        }
        received_parse = false;
      }
    }

  }
}

void exit(int __status) {}
