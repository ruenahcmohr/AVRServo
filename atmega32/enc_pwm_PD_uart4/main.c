/*

quad servo loop controller by rue_mohr

Run on 16Mhz avr
9600 N81
 

-- added PB0 as ascii replied ID request
-- corrected position report code
-- corrected current report code

 
TODO ideas:

- PID 
- fix offset id reporting to proper 3 ascii digiits
- self correcting polarity (use dERROR/drive?)



*/
/*
 +-------------------------------------+
 |                          Sig Vd Gnd |
 |  +---------+   5V O     PB0 [o o o] | baud test input
 |  | 7805  O |   Vd O     PB1 [o o o] | 
 |  +---------+   V+ .     PB2 [o o o] | 
 |                         PB3 [o o o] | chan0 pwm
 |                         PB4 [o o o] | chan0 dir
 |                         PB5 [o o o] | 
 |                         PB6 [o o o] | 
 |                         PB7 [o o o] | 
 |                         PA0 [o o o] | chan0 current sense
 |                         PA1 [o o o] | 
 |        +----------+     PA2 [o o o] | 
 |        |O         |     PA3 [o o o] | 
 |        |          |     PA4 [o o o] | 
 |        |          |     PA5 [o o o] | 
 |        |          |     PA6 [o o o] | 
 |        |          |     PA7 [o o o] | 
 |        |          |     PC7 [o o o] | 
 |        |          |     PC6 [o o o] | 
 |        |          |     PC5 [o o o] | 
 |        | ATMEGA32 |     PC4 [o o o] | 
 |        |          |     PC3 [o o o] | 
 |        |          |     PC2 [o o o] | 
 |        |          |     PC1 [o o o] | chan0 status -
 |        |          |     PC0 [o o o] | chan0 status +
 |        |          |     PD7 [o o o] | 
 |        |          |     PD2 [o o o] | chan0 encoder A
 |        |          |     PD3 [o o o] | chan0 encoder B
 |        |          |     PD4 [o o o] | 
 |        |          |     PD5 [o o o] | 
 |        +----------+     PD6 [o o o] | Debug
 |      E.D.S BABYBOARD III            |
 +-------------------------------------+


*/
/*
Servo Recieves:

 [0][c3][c2][c1][c0][v2][v1][v0]
 [1][v9][v8][v7][v6][v5][v4][v3]
 [1][v17][[v16][v15][v14][v13][v12][v11][v10]  <-- 17 bit only
 
 c0-c3 are command
 v0-v9 are value
 
 Commands:
 0  listen (servo number) 256 = all                      {always obey command} // sticks through listen once
 1  ignore (servo number) 256 = all                      {always obey command} // overrides listen once
 2  One Time listen (servo number)                       {always obey command}
 3  set flags (flags) (+toggle debug)                    { bitwise obey }
    0 enguage cached position                              {always obey command}
    1 turn servo on                                        {obey if listening}
    2 turn servo off                                       {obey if listening}
    3 set cmdpos to curpos                                 {obey if listening}
 4  set servo position (position)                        {obey if listening}
 5  set cached position (position)                       {obey if listening}
 6 get servo current  (servo number)                    {servo number} 
 7 get servo position (servo number)                    {servo number} 
 8 send device model  (servo number)                    {servo number}
 
 9 set 17 bit position
 10 cache 17 bit position
 11 get 17 bit position
 
 
Servo Sends:

 if the ID line is put low, the controller sends, in ascii, the base channel address.

 [0][p3][p2][p1][p0][v2][v1][v0]
 [1][v9][v8][v7][v6][v5][v4][v3] 
 
 p0-p3 are paramiter number
 v0-v9 are value
 
 Paramiter numbers:
 0 servo position (returns position sensor value)
 1 servo current 
 2 device model   (returns 10 bit device model number)

 3 17 bit position
 

*/
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"

#define Version 13

#define ReplyPos 0
#define ReplyCur 1
#define ReplyVer 2

#define PackBits(V,P) (((V << 5) & 0xFF00)|(V & 0x07)|0x8000|(P<<3))

#define IsHigh(BIT, PORT)    ((PORT & (1<<BIT)) != 0)
#define IsLow(BIT, PORT)     ((PORT & (1<<BIT)) == 0)
#define SetBit(BIT, PORT)     PORT |= (1<<BIT)
#define ClearBit(BIT, PORT)   PORT &= ~(1<<BIT)

#define ABS(a)                ((a) < 0 ? -(a) : (a))
#define SIGN(x)               (x)==0?0:(x)>0?1:-1
#define NOP()                 asm volatile ("nop"::)

// limit v to within low (l) and high (h) limits
#define limit(v, l, h)        ((v) > (h)) ? (h) : ((v) < (l)) ? (l) : (v)

//check bounds
#define inBounds(v, l, h)        ((v) > (h)) ? (0) : ((v) < (l)) ? (0) : (1)

// write value to bit on port
#define WriteBit(b, p, v)    p = (p & ~(1<<b))|((v&1)<<b)

 // debug pin
#define debugH()      SetBit(6, PORTD)
#define debugL()      ClearBit(6, PORTD)
#define debugToggle() PORTD ^= (1<<PD6)


#define pwm0on()  SetBit( COM01, TCCR0)
#define pwm0off() ClearBit( COM01, TCCR0)
#define pwm0stat() IsHigh( COM01, TCCR0)

#define IDRequest() IsLow( 0, PORTB)


#define Status0Red()   SetBit(1, PORTC);   ClearBit(0, PORTC)
#define Status0Green() SetBit(0, PORTC);   ClearBit(1, PORTC)
#define Status0Off()   ClearBit(0, PORTC); ClearBit(1, PORTC)
#define Status0Tog()   PORTC ^=(3<<PC0);


#define OUTPUT   1
#define INPUT    0

#define DIR0BIT  4
#define DIR0PORT PORTB


// -----------------  servo id numbers ----------------------

// ID offset when controller is mimmicing an array
#define IDOFF 0

#define ID0  (0+IDOFF)

#define pwm0  OCR0  


volatile signed int position;           // note that this is signed
volatile char flag;                     // timer flag.
 
#define ERROR 0
signed char offsets[] = {
        /* 0000 */   0,
        /* 0001 */  +1,
        /* 0010 */  -1,
        /* 0011 */   ERROR,
        /* 0100 */  -1,
        /* 0101 */   0,
        /* 0110 */   ERROR,
        /* 0111 */  +1,
        /* 1000 */  +1,
        /* 1001 */   ERROR,
        /* 1010 */   0,
        /* 1011 */  -1,
        /* 1100 */   ERROR,
        /* 1101 */  -1,
        /* 1110 */  +1,
        /* 1111 */   0
};


typedef enum servoFlags_e {
   ENGCACHE = 0,
   SERVOON  ,
   SERVOOFF , 
   CMD2CUR  
} servoFlags_t;
/*****************************| VARIABLES |********************************/
 
volatile unsigned char p0, p1, p2, p3;
volatile unsigned char calcWait;

unsigned char pwmcount;
unsigned char txbuff;
unsigned char listen[4];
unsigned char power[4];
unsigned int cmdpos[4];
unsigned int cachepos[4];
 
/************************| FUNCTION DECLARATIONS |*************************/
 
void AnalogInit (void);
void pwmInit(void) ;
int  Analog (int n);
unsigned int dobyte(char data) ;
unsigned int servoCmd(unsigned int command, unsigned int argument) ;
void setupEncoder ( void );
void updatePos    ( void );

/****************************| CODE... |***********************************/

int main (void) {

  signed int termP, termI, termD;
  signed int error, lastError;
  signed int reaction;
  unsigned int txtemp;
  signed int differ;
  

  // set up directions 
  DDRA = (INPUT << PA0 | INPUT << PA1 |INPUT << PA2 |INPUT << PA3 |INPUT << PA4 |INPUT << PA5 |INPUT << PA6 |INPUT << PA7);
  DDRB = (INPUT << PB0 | INPUT << PB1 |INPUT << PB2 |OUTPUT << PB3 |OUTPUT << PB4 |OUTPUT << PB5 |OUTPUT << PB6 |OUTPUT << PB7);
  DDRC = (OUTPUT << PC0 | OUTPUT << PC1 |OUTPUT << PC2 |OUTPUT << PC3 |OUTPUT << PC4 |OUTPUT << PC5 |OUTPUT << PC6 |OUTPUT << PC7);
  DDRD = (INPUT << PD0 | INPUT << PD1 |INPUT << PD2 |INPUT << PD3 |OUTPUT << PD4 |OUTPUT << PD5 |OUTPUT << PD6 |OUTPUT << PD7);        

  PORTB = 0x01; // pullup on baud test

  pwm0 = 0;
  
  pwm0off();

  Status0Red();
  
  USART_Init( 103 ); // 9600 @ 16Mhz
  pwmInit();
  setupEncoder(); 
  TIMSK |= 1<<TOIE0; // turn on pwm overflow interrupts.
  sei();            // turn interrupts on
  
  flag = 0;
  
  termP = termI = termD  = 0;    
  
  // calculate servo loops
  while(1){
    // while we wait to be cleared for recalculating, check the uart

    // check for uart byte
    if ( (UCSRA & (1<<RXC)) ) {  
       if ((txtemp = dobyte(USART_Receive())) != 0) {
         USART_Transmit( txtemp & 0x00FF );
         txbuff = (txtemp >> 8);
       }  
    } 

    if ( (UCSRA & (1<<UDRE)) ) {
      if (txbuff != 0) {
        USART_Transmit( txbuff );
        txbuff = 0;
      } else { // check for id request
        if (IDRequest())  USART_Transmit( '0'+IDOFF );           
      }
    }

    /* update servo loop status leds
       yellow = off
       red    = overcurrent
       off    = error greater than thresh
       green  = ok         
    */

    if (0) {
    } else if (!pwm0stat()) {  // channel off
      Status0Tog();
    } else if (0) { // Overload !!!???!!!
      Status0Red();
    } else if (pwm0 > 64) {  // piles of error yet
      Status0Off();
    } else {                   // ok
      Status0Green();
    }      
           
   
    if (flag) {
       flag = 0;
       /* these values were tuned for a HP servomotor 
	  Set Ki and Kd to 0
	  Dial up Kp as far as you can with it being stable
	  Dial up kd to slow rate of error correction of Kp
	  Dial up both to stable limit
	  Adjust Ki for correction (low values)
	  IRATE will control speed of integral correction
       */

       #define Kp 40      /* proportionate gain */
       #define Plim 200   /* proportionate component limit */

       #define Kd 30	    /* derivitive gain */
       #define Dlim 200   /* derivitive component limit */	

        error    =  cmdpos[0] - position ;
        differ   =  lastError - error;
        termP    =  limit(error/10, -Plim, Plim);
        termD    =  limit(differ, -Dlim, Dlim);
        reaction =  ((termP * Kp) - (termD * Kd));
    
      //  if (ABS(reaction) < 55) reaction = 0;  // deadzone (SILENCE!)
        
        pwm0 = limit(ABS(reaction), 0, 255);            
        if (reaction > 0)    SetBit(DIR0BIT, DIR0PORT);
        else               ClearBit(DIR0BIT, DIR0PORT);
        			
	lastError = error;
    }

  }

}

//------------------------| FUNCTIONS |------------------------



unsigned int dobyte(char data) {

  static unsigned char state = 0;
  static unsigned int command;
  static unsigned int  argument;

  if (state == 0) {
    if ((data & 0x80) == 0) {
      state    = 1;
      command  = (data >> 3);
      argument = (argument & 0xFFF8) | (data & 0x07); // glue in its 0 through 2
    } 
  } else {
    state = 0;
    if ((data & 0x80) != 0) {
      argument = (argument & 0x0007) | ((data & 0x7F) << 3); //glue in bits 3 through 9
      return servoCmd(command, argument);   
    }
  } 

  return 0;
}



/*
 
 0  listen (servo number) 256 = all                      {always obey command} // sticks through listen once
 1  ignore (servo number) 256 = all                      {always obey command} // overrides listen once
 2  One Time listen (servo number)                       {always obey command}
 3  set flags (flags) (+toggle debug)                    { bitwise obey }
    0 enguage cached position                              {always obey command}
    1 turn servo on                                        {obey if listening}
    2 turn servo off                                       {obey if listening}
    3 set cmdpos to curpos                                 {obey if listening}
 4  set servo position (position)                        {obey if listening}
 5  set cached position (position)                       {obey if listening}
 6 get servo current  (servo number)                    {servo number} 
 7 get servo position (servo number)                    {servo number} 
 8 send device model  (servo number)                    {servo number}

*/


unsigned int servoCmd(unsigned int command, unsigned int argument) {

  unsigned int        reply;
  static unsigned int chainAddress = 1023;
  
  reply = 0;

  switch (command) {
     
     case 0: // listen(id)
       chainAddress = 1023 ;
       if (argument == ID0)      listen[0] |= 2;      
       else if (argument == 256) { listen[0] |= 2 }      
     break;
     
     case 1: // ignore(id)
       chainAddress = 1023 ;
       if (argument == ID0)      listen[0] = 0;
       else if (argument == 256) { listen[0] = 0;  }
     break;
          
     case 2: // listen to only the next command
       chainAddress = 1023;
       if (argument == ID0)      listen[0] |= 1;      
       else if (argument == 256) { listen[0] |= 1;  }
       else if (argument >= 512) { 
         chainAddress = argument - 512; 
         listen[0] = 0; if (chainAddress == ID0) listen[0] |= 1;        
       }       
     break;
               
     /*
       0 enguage cached position                              {always obey command}
       1 turn servo on                                        {obey if listening}
       2 turn servo off                                       {obey if listening}
       3 set cmdpos to curpos                                 {obey if listening}
     */          
     case 3: // set flags
     
       debugToggle();
       
       if (IsHigh(ENGCACHE, argument)) {
         cmdpos[0] = cachepos[0];        
       }
       
       if (IsHigh(CMD2CUR, argument)) {
         if (listen[0]){ cmdpos[0] = position;}       
       } 
       
       if (IsHigh(SERVOON, argument)) {
         if (listen[0]){ pwm0on(); }        
       } else if (IsHigh(SERVOOFF, argument)) {
         if (listen[0]){ pwm0off(); Status0Green(); }      
       }
                           
     break;
     
     case 4: // set servo position 
       if (listen[0]){ cmdpos[0] = argument;  }
     break; 
     
     case 5: // set cached position 
       if (listen[0]){ cachepos[0] = argument;  }
     break; 

     case 6: // get servo current
       if      (argument == ID0){ reply = PackBits(0, ReplyCur); }
     break; 
     
     case 7: // get servo position
       if      (argument == ID0){ reply = PackBits(position, ReplyPos);}
     break; 
     
     case 8: // get model
       if ((argument == ID0)){         
         reply = PackBits(Version, ReplyVer);
       }
     break;
     
   }
   
   switch(command) { // clear one time flags
     case 3:
     case 4:
     case 5:
       listen[0] &= 2;        
       if (chainAddress != 1023) {
         chainAddress++;
         if      (chainAddress == ID0)  listen[0] = 1;
       }
     break;
   }
      
   return reply;
   
}


/*

initialize pwm channels but leave off

16Mhz
/64 = ~1khz
/256 = ~240hz
/1024 = ~61Hz

*/
void pwmInit() {
  // clear pwm levels
  OCR0  = 0; 
  OCR2  = 0;
  OCR1A = 0;
  OCR1B = 0;
  
  // set up WGM, clock, and mode for timer 0
  TCCR0 = 0 << FOC0  | /* force output compare */
          1 << WGM00 | /* fast pwm */
          0 << COM01 | /* ** normal polarity */
          0 << COM00 | /*   this bit 1 for interted, 0 for normal  */
          1 << WGM01 | /* fast pwm */
          0 << CS02  | /* CLKio/64 */
          1 << CS01  |
          1 << CS00  ;
  
 }
 




//SIGNAL (SIG_INTERRUPT0) {  // fix this signal name!!! INT0
ISR(INT0_vect){
   updatePos();
}

//SIGNAL (SIG_INTERRUPT1) { // fix this signal name!!! INT1
ISR(INT1_vect){ 
   updatePos();
}

ISR(TIMER0_OVF_vect) {
    flag  = 1;
}

void setupEncoder() {
        // clear position
        position = 0; 
 
        // we need to set up int0 and int1 to 
        // trigger interrupts on both edges
        MCUCR = (1<<ISC10) | (1<<ISC00);
 
        // then enable them.
        GICR  = (1<<INT0) | (1<<INT1);
}
 
 
 

 
void updatePos() {
  static unsigned char oldstate;
/*      get the bits        PIND     = XXXXiiXX
        clear space       & 0x0C     = 0000ii00
        or in old status  | oldstate = XX00iijj
       clean up          & 0x0F     = 0000iijj  */  
       
        oldstate = oldstate | (PIND & 0x0C); // Update Oldstate 
        position += offsets[oldstate];       // Update Position
        oldstate = oldstate >> 2; 
	
}




