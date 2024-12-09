/******************************************************************************
 Title:    dir/pwm x analog in, analog feedback  servo loop controller.
 
 
 Author:   rue_mohr
 Date:     Sept 2022
 Software: AVR-GCC 3.3 
 Hardware: attiny13 @ 1Mhz
 
    

PB0 PWM Side A
PB1 DIR Side B
PB2 
PB3 Vfeedback 
PB4 Vctrl

PB5 (RESET)   


                            +-----U-----+    
               RESET    PB5 | o         | VCC
               ADC3     PB3 |           | PB2 ADC1 
               ADC2     PB4 |   Tiny13  | PB1 OC0B
                        GND |           | PB0 OC0A
                            +-----------+    





    
*******************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "avrcommon.h"
#include "stdint.h"
 
#define OUTPUT                   1
#define INPUT                    0

#define PWM0BIT  0

#define DIRBIT 1

#define PWM0PORT PORTB

#define DEADBAND   7

#define Kp 16
//#define Kp 4

volatile int16_t ctrl, fb;



void ADCInit() ;
void PWMInit ( void );

int main (void)  {

  uint8_t  t;
  uint8_t  pwm0;
  
  // Set clock prescaler: 0 gives full 9.6 MHz from internal oscillator.
  CLKPR = (1 << CLKPCE);
  CLKPR = 0;  
         
  DDRB = (OUTPUT << DDB0 | OUTPUT << DDB1 | INPUT << DDB2 | INPUT << DDB3 | INPUT << DDB4 | INPUT << DDB5 ); 
  
  PWMInit();
 
  ADCInit();  
  
  sei();         // turn on interrupts      
  
  
  pwm0 = 0;

              
  while(1) {
                 
     if (0) {
     } else if (fb > ctrl ) {       // go down
       t    = limit(((fb - ctrl )*Kp), 0, 255);
       pwm0 =  (t > DEADBAND) ? t : 0;
       SetBit(DIRBIT, PORTB);
       pwm0 = 255 - pwm0;  
     } else if (fb < ctrl ) {       // go up       
       t    = limit(((ctrl - fb)*Kp), 0, 255);
       pwm0 =  (t > DEADBAND) ? t : 0;
       ClearBit(DIRBIT, PORTB);            
     } 
     
     // update pwm system
    OCR0A = pwm0;
             
   }
}

    
//------------------------| FUNCTIONS |------------------------



void ADCInit() {

  // ADC, 9.6Mhz / 128
  
  ADMUX  = ( (1 << MUX1) ); 
  
  ADCSRA = ( (1 << ADEN) | (1 << ADIF) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)  | (1 << ADATE) | (1 << ADSC)  ) ; 


}
 
void PWMInit() {
  // clear pwm levels
  OCR0A = 0; 
  
  // set up WGM, clock, and mode for timer 0
  TCCR0A = 1 << COM0A1 | 
           0 << COM0A0 | 
           0 << COM0B1 | 
           0 << COM0B0 | 
           1 << WGM00  |
           1 << WGM01  ;
  
  // set up WGM, clock, and mode for timer 2 divide by 64
  TCCR0B = 0 << CS02  |
           1 << CS01  |
           1 << CS00  ;
  
  TIMSK0 |= (1 << TOIE0);  //enable overflow interrupt for accumulator regulation
  
  
 }


// ----------------------| ISR |------------------------------


ISR(  ADC_vect) {    
  if (IsHigh(MUX0, ADMUX)) {  // alternate channels 2 and 3
    ctrl = limit(ADC, 10, 1013);
    ADMUX  = ( (1 << MUX1) ); 
  } else {
    fb = ADC;
    ADMUX  = ( (1 << MUX0)|(1 << MUX1) );
  }
}
























































