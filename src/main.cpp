#include <avr/io.h>
#include <avr/interrupt.h>
#include "util/delay.h"
#include <stdlib.h>

#define bitSet(reg, n) (reg |= 1 << n)
#define bitRead(reg, n) (reg >> n & 1)
#define bitClear(reg, n) (reg &= ~(1 << n))
#define bitCheck(reg, n) (reg & 1 << n)

//Lights
#define green_light PB1
#define yellow_light PB2
#define red_light PB3

//#define delay_time 1000

//Buzzer
#define pin_buzzer PD6

//Buttons
#define button_changeMode PD2
#define button_changeLight PB0
#define button_emergencyMode PD3

//Sonar
#define pin_echo PD4
#define pin_trigger PD5

//Mode Abstraction
#define MODE_AUTO 0
#define MODE_MANUAL 1
#define MODE_EMERGENCY 2

//USART Declerations
void usart_init(float baud);
void usart_send_byte(unsigned char data);
void usart_send_string(char *pstr);
void usart_send_num(float num, char num_int, char num_decimal);
void usart_init_v2(float baud);
void usart_flush(void);

//function declarations
void changeLight();
void makeSound(void);
void autoLights(void);
void manualLights(int light);
void pedestrianAlarm(void);
void beepTimeFunction(float Dmm);
void adc_init();
void pwm_init();
void sonar_init();
void button_init();
void externalInterupts_init();
void resetLightsPWM();
void pinChangeInterupts_init();
void setGreen();
void timer_init();

//global variables
volatile int current_mode = MODE_AUTO;
volatile char pedestrianDetected = 0;
volatile uint8_t timer_count = 0;
int mode_flag = 0;
char handle_mode_change = 0;
char manual_mode = 0;
int current_light = 0;
volatile uint16_t adc = 0;
volatile char adc_ready = 0;
char on_red = 0;
int debug_mode = 0;

// Sonar Variables
volatile uint8_t *ddr_sonar = &DDRD;
volatile uint8_t *port_sonar = &PORTD;
volatile uint8_t *pin_sonar = &PIND;

// USART READLINE VARIABLES
#define BUF_SIZE 50
char usart_buf[BUF_SIZE] = {0};
char *pbuf_usart = usart_buf;
bool flag_read_done = 0;

ISR(USART_RX_vect) {
  char tmp;
  char *ptr = usart_buf;
  while(1) {
    while(!bitCheck(UCSR0A, RXC0));

    tmp = UDR0;
    if(tmp == '\r' || tmp == '\n') {
      *ptr = '\0';
      flag_read_done = 1;
      debug_mode = atoi(usart_buf);
      usart_flush();
      return;
    }
    else {
      *ptr++ = tmp;
    }
  }
}

ISR(TIMER0_OVF_vect) {
  timer_count++;
  if (timer_count >= 6) {
    timer_count = 0;
    pedestrianAlarm();
  }
}

ISR(ADC_vect) {
   adc = ADC;
   //ADCSRA |= (1 << ADSC);
   
 }

ISR(PCINT2_vect) {
  _delay_ms(10);
  if(!bitRead(PIND, button_changeMode)) {
    if(current_mode != MODE_MANUAL) {
      current_mode = MODE_MANUAL;
    }
    else {
      current_mode = MODE_AUTO;
    }
  }
}

//PINCHANGE Interupt
ISR(PCINT0_vect) {
  
  _delay_ms(10);
  // if button is pressed
  if (!bitRead(PINB, button_changeLight)) {
    // Only cycle lights if in manual mode
    if (current_mode == MODE_MANUAL) {
      if (current_light == 2) current_light = 0;
      else current_light++;
      changeLight();
    }
  }
}

//External Inturrupt for Emg modemi

ISR(INT1_vect) {
  _delay_ms(10);
  if (!bitRead(PINC, button_emergencyMode)) {
    if(current_mode != MODE_EMERGENCY) {
      current_mode = MODE_EMERGENCY;
    }
    else {
      current_mode = MODE_AUTO;
    }
  }
}

int main() {
  usart_init_v2(9600);
  adc_init();
  pwm_init();
  sonar_init();
  button_init();
  externalInterupts_init();
  pinChangeInterupts_init();
  timer_init();

  _delay_ms(100);
  usart_flush();

  sei();

  bitSet(DDRD, pin_buzzer);

  _delay_ms(100);

  while(1) {
    if(!bitCheck(ADCSRA, ADSC)) {
      ADCSRA |= (1 << ADSC);
    }

    switch (current_mode) {
      case MODE_AUTO:
        autoLights();
        break;
      case MODE_MANUAL:
        if(!bitCheck(ADCSRA, ADSC)) {
          ADCSRA |= (1 << ADSC);
        }
        changeLight();
        break;
      case MODE_EMERGENCY:
        setGreen();
    }
  }
}

void changeLight() {
  
  resetLightsPWM();
  
  uint8_t temp = adc / 4;
  uint8_t brightness = 0;

  if(temp < 15) {
    brightness = 25;
  }
  else {
    brightness = 5;
  }

  if(debug_mode == 2 || debug_mode == 3) {
    usart_send_string(">adc:");
    usart_send_num(adc, 3, 3);
    usart_send_string("\n");
  }
  usart_send_string(">adc:");
  usart_send_num(adc, 3, 3);
  usart_send_string("\n");

  switch(current_light) {
    case 0:
      //resetLightsPWM();
      OCR1A = brightness;
      break;
    case 1:
      //resetLightsPWM();
      OCR1B = brightness;
      break;
    case 2:
     // resetLightsPWM();
      OCR2A = brightness;
      break;
    default:
      break;
  }
}


void makeSound(void) {
  float T = (1.0 / 523.25) * 1e6;
  int Ton = 0.5 * T;
  int Toff = T - Ton;
  //50% duty cycle
  //sets buzz time off/on for a duty cycle
  //creates about 100ms of sound

  for(int i = 0; i < 1e5/T; i++) {
    
    PORTD = PORTD | (1 << pin_buzzer);
    for(int j = 0; j < Ton; j++) {
      _delay_us(1);
    }

    PORTD = PORTD & ~(1 << pin_buzzer);
    for(int j = 0; j < Toff; j++) {
      _delay_us(1);
    }
  }
}

void autoLights(void) {
  int i = 0;
  while(current_mode == MODE_AUTO) {
    if(i == 3) {
      i = 0;
    }
    current_light = i;
    changeLight();
    for (int t = 0; t < 100; t++) {
      _delay_ms(10);
      
      if (!bitCheck(ADCSRA, ADSC)) {
        ADCSRA |= (1 << ADSC);
      }

      // 10 ms × 100 = 1 second
      // in each step, let the main loop or interrupts run
      // e.g. check if mode changed, check if ADC can start a new conversion
      if (current_mode != MODE_AUTO) break;
    }
    i++;
  }
}

void manualLights(int light) {
  current_light = light;
  changeLight();
  for (int t = 0; t < 100; t++) {
    _delay_ms(10);
    
    if (!bitCheck(ADCSRA, ADSC)) {
      ADCSRA |= (1 << ADSC);
    }

    // 10 ms × 100 = 1 second
    // in each step, let the main loop or interrupts run
    // e.g. check if mode changed, check if ADC can start a new conversion
    if (current_mode != MODE_MANUAL) break;
  }
}

void setGreen() {
  current_light = 0;
  changeLight();
  for (int t = 0; t < 100; t++) {
    _delay_ms(10);
    
    if (!bitCheck(ADCSRA, ADSC)) {
      ADCSRA |= (1 << ADSC);
    }
    pedestrianAlarm();

    // 10 ms × 100 = 1 second
    // in each step, let the main loop or interrupts run
    // e.g. check if mode changed, check if ADC can start a new conversion
    if (current_mode != MODE_EMERGENCY) break;
  }
}

void pedestrianAlarm(void) {
  uint16_t cnt = 0;
  float vel_sound = 343;
  uint16_t timeout = 30000;
  uint16_t timeout_echo =30000;

  bitClear(*port_sonar, pin_trigger);
  _delay_us(2);
  bitSet(*port_sonar, pin_trigger);
  _delay_us(11);
  bitClear(*port_sonar, pin_trigger);

  while(!bitCheck(*pin_sonar, pin_echo) && timeout--) _delay_us(1);

  if (timeout == 0) {
    pedestrianDetected = 0; 
    return;

  }

  while(bitCheck(*pin_sonar, pin_echo) && timeout_echo--) {
    cnt++;
    _delay_us(1);
  }

  float Dmm = (float)cnt / 1.0e6 * vel_sound / 2. * 1000.;

  if(debug_mode == 1 || debug_mode == 3) {
    usart_send_string(">Dmm:");
    usart_send_num(Dmm, 3, 3);
    usart_send_string("\n");
  }

  if(current_mode == MODE_AUTO) {
    if(Dmm < 300 && (current_light != 2 || pedestrianDetected)) {
      pedestrianDetected = 1;
      current_light = 2;
      changeLight();
      beepTimeFunction(Dmm);
    }
    else if(Dmm < 300 && current_light == 2 && !pedestrianDetected) {
      on_red = 1;
      while(on_red) {
        pedestrianAlarm();
        
      }
    }
    else {
      pedestrianDetected = 0;
      on_red = 0;
    }
  }
}

void beepTimeFunction(float Dmm) {
  if (Dmm <= 500) {
    if(Dmm >= 300) {
      makeSound();
      _delay_ms(400);
    }
    else if(Dmm >= 200) {
      makeSound();
      _delay_ms(300);
    }
    else if(Dmm >= 100) {
      makeSound();
      _delay_ms(200);
    }
    else if(Dmm >= 0) {
      makeSound();
      _delay_ms(100);
    }
  }
}

void resetLightsPWM() {
  
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 0;

}

void timer_init() {
  TCCR0B |= (1 << CS02) | (1 << CS00); // 1024 clock mode prescalers
  TIMSK0 |= (1 << TOIE0); // set timer to compare match
}
void externalInterupts_init() {
  EIMSK |= (1 << INT1);
  EICRA |= (1 << ISC11);               // INT1 on falling edge
}

void button_init() {
  bitClear(DDRD, button_changeMode);
  bitSet(PORTD, button_changeMode);

  bitClear(DDRB, button_changeLight);
  bitSet(PORTB, button_changeLight);

  bitClear(DDRD, button_emergencyMode);
  bitSet(PORTD, button_emergencyMode);
}

void sonar_init() {
  bitSet(*ddr_sonar, pin_trigger);
  bitClear(*ddr_sonar, pin_echo);
}

void adc_init() {
  ADMUX |= (1 << REFS0); //Seting the ADC multiplexier register to ref0 for 5vv

  // This sets ADC2 without affecting ref bits only touches the last four bits
  ADMUX = (ADMUX & 0xF0) | (1 << MUX1);

  //ADIE is bit 3 and it enables inturept
  //ADEN is bit 7 and it enables ADC
  //ADPS2 - ADPS0 are prescaler bits

  ADCSRA |= (1 << ADIE) | (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0) | (1 << ADPS1); // ADCSRA is the control and status register
  
  //They divide the system clock from 16mghz, 
  // ADPS2	ADPS1	ADPS0	Division Factor
  // 1	      1	    1 	 128
  // 1	      1	    0	   64
  // 1	      0	    1	   32

  ADCSRA |= (1 << ADSC);
}

void pwm_init() {
  // Set PB1 (OC1A), PB2 (OC1B), and PB3 (OC2A) as output
  bitSet(DDRB, green_light);
  bitSet(DDRB, yellow_light);
  bitSet(DDRB, red_light);
  
  // Timer1 (8-bit Fast PWM for PB1, PB2)
  // COM1A1 & COM1B1 set to clear
  // WGM10 set to 1 & WG12 is on TCR1B those together set fast pwm mode 8 bit
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);

  //TCR1B controls wgm12 is bit 3 cs11 to 1 is prescaller of 8
  TCCR1B |= (1 << WGM12) | (1 << CS11); 

  // Timer2 (8-bit Fast PWM for PB3)
  // COM2A1  set to 1 is clear on compare match
  // WGM20 and WGM21 set to fast pwm
  // CS21 = 1 to prescaler 8 
  TCCR2A |= (1 << COM2A1) | (1 << WGM20) | (1 << WGM21); // Fast PWM
  TCCR2B |= (1 << CS21); 

  //prescaler set to 8 as it doesnt provide a flicker 2 million ticks per second
}

void pinChangeInterupts_init() {
  PCICR |= (1 << PCIE0);  // enable pin change interrupts on PCINT[7:0]
  PCMSK0 |= (1 << PCINT0); // specifically enable PCINT0

  PCICR |= (1 << PCIE2);  // enable pin change interrupts on PCINT[7:0]
  PCMSK2 |= (1 << PCINT18); // specifically enable PCINT0
}



/*USART FUNCTIONS
*
*
*
*/

void usart_init(float baud) {

  float ubrr0 = 1.0e6 / baud;
  int ubrr0a = (int)ubrr0;

  if(ubrr0 - ubrr0a >= 0.5) {
    ubrr0a = ubrr0a + 1;
  }

  UBRR0 = ubrr0a;
  bitSet(UCSR0B, TXEN0);
  UCSR0C |= 3 << UCSZ00;
}

void usart_send_byte(unsigned char data) {
  while(!bitCheck(UCSR0A, UDRE0));
  UDR0 = data;
}

void usart_send_string(char *pstr) {
  while(*pstr != '\0') {
    usart_send_byte(*pstr);
    pstr++;
  }
}

void usart_send_num(float num, char num_int, char num_decimal) {
  char str[20];
  if(num_decimal == 0) {
    dtostrf(num, num_int, num_decimal, str);
  }
  else {
    dtostrf(num, num_int + num_decimal + 1, num_decimal, str);
  }
  str[num_int + num_decimal + 1] = '\0';
  usart_send_string(str);
}

//USART initialization with RX enabled
void usart_init_v2(float baud) {
  usart_init(baud);
  bitSet(UCSR0B, RXCIE0);
  bitSet(UCSR0B, RXEN0);
}

void usart_flush(void) {
  char dummy;
  while(bitCheck(UCSR0A, RXC0)) {
    dummy = UDR0;
  }
}