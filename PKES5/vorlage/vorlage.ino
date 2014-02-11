#include <math.h>
#include <util/delay.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "Flydurino.h"
#include "avr/interrupt.h"

void* operator new(size_t s, void* ptr) {return ptr;}
void setup();
void loop();
int8_t checkButtons();
uint8_t linearizeDistance(uint16_t distanceRaw);
uint16_t readADC(int8_t channel);
void writetoDisplay(char digit1, char digit2, char digit3);
char flydurinoPtr[sizeof(Flydurino)];

unsigned long time;
int8_t  modus;
int16_t acc_x, acc_y, acc_z;

// Kanal des ADC Wandlers
// -------------------------------------------------------------
int8_t channel_0 = 3;
int8_t channel_1 = 2;
// -------------------------------------------------------------

volatile int cntLeft  = 0;
volatile int cntRight = 0;

// speed control
// -------------------------------------------------------------
int vCurrent;
int vmax;
int vmin;
float ratio;
int maxSpeed;
int minSpeed;
int changeStep;
float odometryRatio;
float odometryMaxDiff;
int calibrationMode;
// -------------------------------------------------------------

ISR(INT4_vect) {
    // Serial.println("INT4_vect");
    ++cntRight;
}

ISR(PCINT0_vect) {
    // Serial.println("PCINT0_vect");
    ++cntLeft;
}

// the setup routine runs once when you press reset:
void setup() {
    // setup interrupts
    // -----------------------------------------------------------------------
    sei();                 // enable global interrupts
    EIMSK  |= _BV(INT4);   // enable external interrupt 4
    PCMSK0 |= _BV(PCINT4); // enable pin change interrupt 4
    
    EICRB |= _BV(ISC40);   // generate interrupt on any logical change on INT4
    PCICR |= _BV(PCIE0);   // any change on PCINT7:0 will cause interrupt
    // -----------------------------------------------------------------------
  
  
    // initialize serial communication
    Serial.begin(9600);
    
    Serial.println("----------------------------------" );
    Serial.println("PKES Wintersemester 2013/14" );
    Serial.println("Vorlage 5. Aufgabe " );
    Serial.println("----------------------------------\r\n");
    
    // -------------------------------------------------------------
     // Single Onboard LED configuration
     // -------------------------------------------------------------
     // set leds LED_X as output
     // alternative
     // - intermediate
     // DDRA=1+2+4+8;
     // - using processor makros
     // DDRA=((1<<DDA0) | (1<<DDA1) | (1<<DDA2) | (1<<DDA3));
     // - using hardware specific makros
     DDRB |= (1<<DDA7);
     // disable leds
     PORTB &= ~(1<<7);
     // -------------------------------------------------------------
     // Serial bus lines
     // -------------------------------------------------------------
     // Pin 5 = PORT E 3 = clock
     DDRE |= (1<<DDE3);
     // Pin 6 = PORT H 3 = data
     DDRH |= (1<<DDH3);
     // Pin 7 = PORT H 4 = enable
     DDRH |= (1<<DDH4);
     // -------------------------------------------------------------
     // Button configuration
     // -------------------------------------------------------------
     // not necessary but for completion
     // S1 as input
     DDRF &=~(1<<DDF4);
     // S2 as input
     DDRG &=~(1<<DDG5);

    // Configuration of ADC
    // -----------------------------------------------------
    analogReference(INTERNAL2V56);
    // -----------------------------------------------------
    
    // Configure buttons
    // -----------------------------------------------------
    digitalRead(4); // S1
    
    // Configure PWM
    // -----------------------------------------------------
    pinMode(3, OUTPUT); // channel A (left motor)
    
    TCCR3A = _BV(WGM31) | _BV(WGM30); // -> fast PWM with OCR3A top
    TCCR3B = _BV(CS32) | _BV(WGM32); // -> prescaler 64
    
    pinMode(11, OUTPUT); // channel B (right motor)
    
    TCCR1A = _BV(WGM11) | _BV(WGM10); // -> fast PWM with OCR1A top
    TCCR1B = _BV(CS12) | _BV(WGM12); // -> prescaler 64
    // -----------------------------------------------------

    // Configure Flydurino
    new(flydurinoPtr) Flydurino;
    // -----------------------------------------------------
    // set Digital Low Pass Filter
    ((Flydurino*)flydurinoPtr)->configureZGyro(MPU6050_DLPF_BW_5);
    
    // -----------------------------------------------------
    
    modus           = 0;
    vCurrent        = 30;
    vmin            = 0;
    vmax            = 0;
    ratio           = 1.0;
    maxSpeed        = 150;
    minSpeed        = 80;
    changeStep      = 5; // 10
    odometryRatio   = 0.85;
    odometryMaxDiff = 0.2;
    calibrationMode = 0;
}

void loop() {
  // Receive acceleration values
  ((Flydurino*)flydurinoPtr)->getAcceleration(&acc_x, &acc_y, &acc_z);
  
  // reset if taken up
  /*if (acc_z > 19000) {
    resetAll();
  }*/
    
  // default state - avoids crash situations due to suddenly starting
   if (modus==0){
       writetoDisplay(0b10011111,0b11111101,0b10110111);
       
       while(modus==0){
         modus=checkButtons();
       }
       time = millis();
   }
   
   // calibration
   if (modus==1){
     if (millis() - time > 400) {
       Serial.print("speed left:  "); Serial.println(cntLeft);
       Serial.print("speed right: "); Serial.println(cntRight);
       //Serial.print("ratio: "); Serial.println(ratio);
       
       goAhead(vCurrent);
       
       // find min and max PWM
       if (calibrationMode == 0) {
         if ((cntLeft < maxSpeed || cntRight < maxSpeed) && vCurrent < 250) {
           vCurrent += changeStep;
         }
         else {
           vmax            = vCurrent;
           calibrationMode = 1;
           Serial.println("Callibration:");
           Serial.print("vmax = "); Serial.println(vmax);
         }
       }
       
       if (calibrationMode == 1) {
         if ((cntLeft > minSpeed && cntRight > minSpeed) && vCurrent > 0) {
           vCurrent -= changeStep;
         }
         else {
           vmin = vCurrent + 20;
           Serial.print("vmin = "); Serial.println(vmin);
           Serial.print("ratio = "); Serial.println(ratio);
           resetAll();
           //cli();
           EIMSK  &= ~_BV(INT4);
           PCMSK0 &= ~_BV(PCINT4);
         }
       }
       
       /*
       // find left/right PWM ratio
       if (cntRight > 0) {
         double currentRatio = (double) cntLeft / cntRight;
         if (currentRatio < odometryRatio) {
           if (currentRatio < (odometryRatio - odometryMaxDiff)) {
             ratio += 0.1;
           }
         }
         else if (currentRatio > odometryRatio) {
           if (currentRatio > (odometryRatio + odometryMaxDiff)) {
             ratio -= 0.1;
           }
         }
       }
       */
       
       cntLeft  = 0;
       cntRight = 0;
       time     = millis();
     }
   }
   
   // drive in the circle
   if (modus==2) {
        // get distances
        uint8_t distance_left,distance_right;
        distance_right = linearizeDistance(readADC(channel_1));
        distance_left  = linearizeDistance(readADC(channel_0));
        
        //Serial.print("distance_right: "); Serial.println(distance_right);
        
        //Serial.print("distance_left: "); Serial.println(distance_left);
        
        int v = vmax;
        
        if (distance_right < 8) {
          // obstacle
          goBackwards(vmin);
        }
        else {
          // calculate speed depending on distance to next
          if (distance_right < 25) {
            v = ((distance_right - 8) / 17.0) * (vmax - vmin) + vmin;
          }
          
          if (distance_left < 10) {
            goAheadRightFaster(vmin);
            //turnLeft(vmin);
          }
          //else if (distance_left > 12) {
          //  goAheadLeftFaster(vmin);
          //}
          else {
            goAhead(v);
          }
        }
        
        Serial.print("v = "); Serial.println(v);
        
        delay(200);
   }
   
   modus=checkButtons();
}

void resetAll() {
  stopTheMotors();
  modus       = 0;
  cntLeft     = 0;
  cntRight    = 0;
}

void stopTheMotors() {
  // stop the motors
  TCCR1A &= ~_BV(COM1A1);
  TCCR3A &= ~_BV(COM3C1);  
}

void startTheMotors() {
  // start the motors
  TCCR1A |= _BV(COM1A1); // clear on compare match
  TCCR3A |= _BV(COM3C1);
}

/*
calculation of left and right speed:
left/right = ratio and (left + right)/2 = _speed

=> right = (2 * _speed)/ (ratio + 1)
   left  = ratio * right;
*/

void goAhead(int _speed) {
   int right = 2 * _speed / (ratio + 1);
  
   // left motor
   digitalWrite(12, HIGH); // direction (forward)
   OCR3C = ratio * right; // fix was 250
   
   // right motor
   digitalWrite(13, LOW); // direction (forward)
   OCR1A = right; // fix was 180
   
   startTheMotors();
}

void goBackwards(int _speed) {
   int right = 2 * _speed / (ratio + 1);
  
   // left motor
   digitalWrite(12, LOW); // direction (backward)
   OCR3C = ratio * right; // fix was 120
   
   // right motor
   digitalWrite(13, HIGH); // direction (backward)
   OCR1A = right; // fix was 100
   
   startTheMotors();
}

void goAheadLeftFaster(int _speed) {
   int right = 2 * _speed / (ratio + 0.2 + 1);
  
   // left motor
   digitalWrite(12, HIGH); // direction (forward)
   OCR3C = (ratio + 0.2) * right; // fix was 250
   
   // right motor
   digitalWrite(13, LOW); // direction (forward)
   OCR1A = right; // fix was 140
   
   startTheMotors();
}

void goAheadRightFaster(int _speed) {
   int right = 2 * _speed / (ratio - 0.5 + 1);
  
   // left motor
   digitalWrite(12, HIGH); // direction (forward)
   OCR3C = (ratio - 0.5) * right; // fix was 180
   
   // right motor
   digitalWrite(13, LOW); // direction (forward)
   OCR1A = right; // fix was 160
   
   startTheMotors();
}

void turnRight(int _speed) {
   int right = 2 * _speed / (ratio + 1);
   
   // left motor
   digitalWrite(12, HIGH); // direction (forward)
   OCR3C = ratio * right; // speed
   
   // right motor
   digitalWrite(13, HIGH); // direction (backward)
   OCR1A = right; // speed
   
   startTheMotors();
}

void turnLeft(int _speed) {
   int right = 2 * _speed / (ratio + 1);
  
   // left motor
   digitalWrite(12, LOW); // direction (backward)
   OCR3C = ratio * right; // speed
   
   // right motor
   digitalWrite(13, LOW); // direction (forward)
   OCR1A = right; // speed
   
   startTheMotors();
}

int8_t checkButtons(){
   int8_t modus_new=modus;
   // Abfrage der Buttons und Moduswechsel
   // -----------------------------------------------------
   
   if (digitalRead(4)) {
     modus_new=1;
   }
   if (analogRead(4) > 800){
     modus_new=2;
     cntLeft  = 0;
     cntRight = 0;
   }
   return modus_new;
   
}

uint8_t linearizeDistance(uint16_t distance_raw){
   double distance_cm=0;
   // Transformation der Spannungsbezogenen Distanzwerte in
   // eine Entfernung in cm
   // -----------------------------------------------------
   distance_cm = (6787/(distance_raw - 3));
   // -----------------------------------------------------
   return (int8_t)ceil(distance_cm);
}

uint16_t readADC (int8_t channel){
   int distance_raw;
   int maxCnt = 16;
   int sum = 0;
   // möglicherweise mehrmaliges Lesen des ADC Kanals
   // Mittelwertbildung
   // -----------------------------------------------------
   
   for (int i = 0; i < maxCnt; i++) {
         sum += analogRead(channel);
   }
   
   distance_raw = sum / maxCnt;
        
   // -----------------------------------------------------
   return distance_raw;
}

void writetoDisplay(char digit1, char digit2, char digit3){

        char stream[36];
        stream[0]=1;
        int i;
        for ( i=1; i<36; i++ ) {
                stream[i]=0;
        }

        for ( i=0; i<8; i++ ) {
                if (digit1 & (1<<(7-i))) stream[i+ 1]=1;
                if (digit2 & (1<<(7-i))) stream[i+9]=1;
                if (digit3 & (1<<(7-i))) stream[i+17]=1;
        }

        for ( i=0; i<36; i++ ) {
                // clock low
                PORTE &= ~(1<<3);
                // data enable low
                PORTH &= ~(1<<4);
                _delay_us (1);
                // data
                if (stream[i]==1)
                        PORTH |= (1<<3);
                else
                        PORTH &=~(1<<3);
                _delay_us (1);
                // clock high - Transmission finished
                PORTE |= (1<<3);
                _delay_us (1);
                // data enable high - ready for next cycle
                PORTH |= (1<<4);
        }
}
