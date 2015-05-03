#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define SAMPLE_RATE 8000

/*
 * The audio data needs to be unsigned, 8-bit, 8000 Hz, and small enough
 * to fit in flash. 10000-13000 samples is about the limit.
 *
 * sounddata.h should look like this:
 *     const int sounddata_length=10000;
 *     const unsigned char sounddata_data[] PROGMEM = { ..... };
 *
 * You can use wav2c from GBA CSS:
 *     http://thieumsweb.free.fr/english/gbacss.html
 * Then add "PROGMEM" in the right place. I hacked it up to dump the samples
 * as unsigned rather than signed, but it shouldn't matter.
 *
 * http://musicthing.blogspot.com/2005/05/tiny-music-makers-pt-4-mac-startup.html
 * mplayer -ao pcm macstartup.mp3
 * sox audiodump.wav -v 1.32 -c 1 -r 8000 -u -1 macstartup-8000.wav
 * sox macstartup-8000.wav macstartup-cut.wav trim 0 10000s
 * wav2c macstartup-cut.wav sounddata.h sounddata
 *
 * (starfox) nb. under sox 12.18 (distributed in CentOS 5), i needed to run
 * the following command to convert my wav file to the appropriate format:
 * sox audiodump.wav -c 1 -r 8000 -u -b macstartup-8000.wav
 */

#include "HEV_KICK.WAV.raw.h"

int ledPin = 13;
int speakerPin = 11; // Can be either 3 or 11, two PWM outputs connected to Timer 2
volatile uint16_t sample;

byte lastSample;
//these pins can not be changed 2/3 are special pins
int encoderPin1 = 2;
int encoderPin2 = 3;
 int encoderSwitchPin = 4; //push button switch

 
volatile int lastEncoded = 0;
volatile long
encoderValue = 0;
 
long lastencoderValue = 0;
 
int lastMSB = 0;
int lastLSB = 0;
  
// Variables will change:
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
 

void stopPlayback()
{
    // Disable playback per-sample interrupt.
    TIMSK1 &= ~_BV(OCIE1A);

    // Disable the per-sample timer completely.
    TCCR1B &= ~_BV(CS10);

    // Disable the PWM timer.
    TCCR2B &= ~_BV(CS10);

    digitalWrite(speakerPin, LOW);
}

// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
    if (sample >= sounddata_length) {
        if (sample == sounddata_length + lastSample) {
            stopPlayback();
        }
        else {
            if(speakerPin==11){
                // Ramp down to zero to reduce the click at the end of playback.
                OCR2A = sounddata_length + lastSample - sample;
            } else {
                OCR2B = sounddata_length + lastSample - sample;                
            }
        }
    }
    else {
        if(speakerPin==11){
            OCR2A = pgm_read_byte(&sounddata_data[sample]);
        } else {
            OCR2B = pgm_read_byte(&sounddata_data[sample]);            
        }
    }

    ++sample;
}

void startPlayback()
{
    pinMode(speakerPin, OUTPUT);

    // Set up Timer 2 to do pulse width modulation on the speaker
    // pin.

    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);

    if(speakerPin==11){
        // Do non-inverting PWM on pin OC2A (p.155)
        // On the Arduino this is pin 11.
        TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
        TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

        // Set initial pulse width to the first sample.
        OCR2A = pgm_read_byte(&sounddata_data[0]);
    } else {
        // Do non-inverting PWM on pin OC2B (p.155)
        // On the Arduino this is pin 3.
        TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);
        TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

        // Set initial pulse width to the first sample.
        OCR2B = pgm_read_byte(&sounddata_data[0]);
    }





    // Set up Timer 1 to send a sample every interrupt.

    cli();

    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
    TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);

    lastSample = pgm_read_byte(&sounddata_data[sounddata_length-1]);
    sample = 0;
    sei();
}


void setup()
{
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    
      pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);
  
  pinMode(encoderSwitchPin, INPUT);
  
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  digitalWrite(encoderSwitchPin, HIGH);
 
  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

}

void loop()
{
  if(digitalRead(encoderSwitchPin)){
    // not pushed
  }else{
    // pushed
  }
    checkLed();
}

void checkLed() {
    unsigned long currentMillis = millis();
 
    if(currentMillis - previousMillis > encoderValue) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   
 
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
      startPlayback();
    }
    else {
     ledState = LOW;
    }
 
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}
 
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
 
  lastEncoded = encoded; //store this value for next time
}
