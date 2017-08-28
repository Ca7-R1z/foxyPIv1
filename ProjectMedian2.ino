#include <RunningMedian.h>
#include <JeeLib.h> // Low power functions library


//AVR
#include <avr/io.h>
#include <avr/power.h>
#include <util/delay.h>

//bits clear
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
//bits set
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//Arduino pins
//D02 - btn
//D03 - CLK
//D04 - Din
//D05 - DC
//D06 - AIN0
//D07 - AIN1
//D08 - gen
//D09 - CE
//D10 - RST
//D11 - beep
//D13 - LED

//constants
//------------ pins
const int genPin = 8; //gen pin
const int btnPin = 2; //btn pin
const int ain0Pin = 6; //AIN0 pin
const int ain1Pin = 7; //AIN1 pin
const int ledPin = 13; //LED pin
const int beepPin = 11; //beep pin
//------------ delays
const unsigned int PULSE_US = 170; //pulse length, us
const unsigned int DELAY_US = 50; //delay time, us
const unsigned int PAUSE_MS = 20;  //pause length, ms
const unsigned int BLINK_MS = 250; //blink length, ms
const unsigned int BEEP_MS = 750; //beep length, ms

const int BLINK_TIMES = 3;//init blink times
//PWM
const byte BEEP_ON = 128;//PWM beep on
const byte BEEP_OFF = 0;//PWM beep off

const int PROTECT = 0.5;
const int ZERO_COUNTS = 800;
const int MovingAvgWindow = 8;

//------------ btn

//vars
int count;
float count_sum;

int i;
int zero = 0;
int counter;//counter
float count_avg;
int btnState = 0;
boolean zeroing = false;
unsigned int zero_count = 0;
int delta = 0;//diff

ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup the watchdog

boolean target = false;

void setup()   {
  
  
  pinMode(genPin, OUTPUT);//gen pin -> out
  pinMode(btnPin, INPUT_PULLUP); //btn pin <- in
  digitalWrite(btnPin, HIGH); //btn pin pull-up en
  pinMode(ledPin, OUTPUT);//LED pin -> out
  digitalWrite(ledPin,LOW);//LED off
  //beep
  pinMode(beepPin, OUTPUT);//beep pin -> out
  analogWrite(beepPin, BEEP_OFF);//the duty cycle: between 0 (always off) and 255 (always on)  490 (500) Hz
  //comparator init
  pinMode(ain0Pin, INPUT);//AIN0 -> input
  pinMode(ain1Pin, INPUT);//AIN1 -> input
  ACSR = 0b00000000;
  ADCSRA = 0b10000000;
  ADCSRB = 0b00000000;
  //zeroing init
  zero_count = ZERO_COUNTS;
  zero = 0;
  zeroing = true;
  target = false;
  
  //blink $ beep 3 times
  for (i = 0 ; i < BLINK_TIMES ; i++)
  {
    digitalWrite(ledPin,HIGH);//LED on
    analogWrite(beepPin, BEEP_ON);//beep on
    delay(BLINK_MS);
    digitalWrite(ledPin,LOW);//LED off
    analogWrite(beepPin, BEEP_OFF);//beep off
    delay(BLINK_MS);  
  }
  power_adc_disable();//ADC disable
  power_spi_disable();//SPI disable
  power_twi_disable();//Two Wire Interface disable
  power_usart0_disable();//USART0 disable

}


void loop() 
{
  RunningMedian RM(MovingAvgWindow);

  while (true) //infinite loop
  { 
    noInterrupts();//interrupts disable
    count = 0;
    //pulse
    PORTB |= 0b00000001; //1
    delayMicroseconds(PULSE_US);
    PORTB &= ~0b00000001; //0
    //delay time
    delayMicroseconds(DELAY_US);
    //low wait
    do
    {
    }
    while ((ACSR & 0b00100000) != 0);
    //hi wait
    
    
    do
    {
      count=count +1;
    }
    while ((ACSR & 0b00100000) == 0);
    interrupts();//interrupts enable

    //zeroing
    if (zeroing) {

      count_sum=count_sum+count;
      zero_count--;//zero counter decrement
      if (zero_count == 0) {
        
        //beep
        digitalWrite(ledPin,HIGH);//LED on
        analogWrite(beepPin, BEEP_ON);
        delay(BEEP_MS);
        digitalWrite(ledPin,LOW);//LED off
        analogWrite(beepPin, BEEP_OFF);
        zero=count_sum/ZERO_COUNTS;
        zeroing = false;//zeroing O.K. 
        counter = 0;//average counter reset
        count_avg = 0;
      }

    }
    else {
      RM.add(count);
      //target check
      if(RM.getMedian() > (zero+PROTECT))
      target = true;
      else
      target = false;
    }
        

         
       

                
      if (target){
        digitalWrite(ledPin,HIGH);//LED on
        analogWrite(beepPin, BEEP_ON);//beep on
        //delta indication
        //???
      }
      else
      {
        //target not found
        digitalWrite(ledPin,LOW);//LED off
        analogWrite(beepPin, BEEP_OFF);//beep off
      }


    
    
    //pause between pulses
    if (target) {
        delay(PAUSE_MS);  
      }
      else {
        Sleepy::loseSomeTime(PAUSE_MS);
    }

    //next pulse
  }
}

