#include <math.h>
#include <util/delay.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "Flydurino.h"

void* operator new(size_t s, void* ptr) {return ptr;}
void setup();
void loop();
int8_t checkButtons();
void displaySpiritLevel(int16_t acc_x, int16_t acc_y, int16_t acc_z);
uint8_t linearizeDistance(uint16_t distanceRaw);
void displayDistance (int8_t dist);
uint16_t readADC(int8_t channel);
void writetoDisplay(char digit1, char digit2, char digit3);
uint8_t displayMask(char val);

void returnBobbyToOrigin(int dir, int v);

char flydurinoPtr[sizeof(Flydurino)];
// aktuelle Beschleunigungswerte, Kompassmessungen
int16_t acc_x, acc_y, acc_z;
int16_t ori_x, ori_y, ori_z;
int16_t rot_x, rot_y, rot_z;
float current_rot_deg, sum_rot;
// Wasserwaage oder Distanzmessung
int8_t modus;
// Kanal des ADC Wandlers
// -------------------------------------------------------------
int8_t channel_0 = 3; // korrekte Werte bestimmen !
int8_t channel_1 = 2;
// -------------------------------------------------------------
// Laufzeit des Programms
unsigned long time;

int8_t minimum_distance = 10;
int8_t medium_distance = 20;

enum number {
    ZERO = 0b11111101,
    ONE = 0b01100001,
    TWO = 0b11011011,
    THREE = 0b11110011,
    FOUR = 0b01100111,
    FIVE = 0b10110111,
    SIX = 0b10111111,
    SEVEN = 0b11100001,
    EIGHT = 0b11111111,
    NINE = 0b11110111,
    NOTHING = 0b00000001,
    MINUS = 0b00000010
};

// the setup routine runs once when you press reset:
void setup() {
    // initialize serial communication
    Serial.begin(9600);
    
    Serial.println("----------------------------------" );
    Serial.println("PKES Wintersemester 2013/14" );
    Serial.println("Vorlage 3. Aufgabe " );
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
    /*
Fast PWM, 8-bit = Mode 5 (101)
WGM-Register
TCCR2A - [COM2A1, COM2A0, COM2B1, COM2B0, reserved, reserved, WGM21, WGM20]
TCCR2B - [FOC2A, FOC2B, reserved, reserved, WGM22, CS22, CS21, CS20]
COM2x: toggle(01)/clear(10)/set(11) OC2x on compare match [Compare Output Mode]
WGM2n: Fast PWM = 011 (0xFF TOP) / 111 (OCR2A TOP) [Waveform Generation Mode]
FOC2x: have to be zero when operating in PWM mode [FOrce Output Compare]
CS2n: select clock source, 001 = no prescaling [Clock Select]
configure OC2A as output? DDB4 set 1
*/
    
    pinMode(3, OUTPUT);
    pinMode(11, OUTPUT);
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS20); // CS22?
    OCR2A = 180; // set top value
    OCR2B = 50;
    
    // -----------------------------------------------------

    // Configure Flydurino
    new(flydurinoPtr) Flydurino;
    // -----------------------------------------------------
    // set Digital Low Pass Filter
    ((Flydurino*)flydurinoPtr)->configureZGyro(MPU6050_DLPF_BW_5);
    
    // -----------------------------------------------------
    
    modus=0;
    sum_rot=0;
    drive = false;
}

void loop() {
   // default state - avoids crash situations due to suddenly starting
   // PWM modus
   if (modus==0){
       writetoDisplay(0b10011111,0b11111101,0b10110111);
       
       while(modus==0){
         modus=checkButtons();
       }
       time = millis();
   }
   
   // Gyro task
   if (modus==1){
       // Receive acceleromation values
       ((Flydurino*)flydurinoPtr)->getAcceleration(&acc_x, &acc_y, &acc_z);
       // Get compass data
       ((Flydurino*)flydurinoPtr)->getOrientation(&ori_x, &ori_y, &ori_z);
       // Get gyro data
       ((Flydurino*)flydurinoPtr)->getRotationalSpeed(&rot_x, &rot_y, &rot_z);
      
	  	//measure time interval
      	trigger = millis() - time;
      	time += trigger;
		
      	//subtract offset
      	if (rot_z > 105 && rot_z < 125)
    		rot_z = 0;
      	else
    		rot_z -= 115;

      	//rot_z -> degree
      	//90° is equal to 12k sum_rot (experimental)
      	//=> 1° is equal to 90/12k = 0.075
      	current_rot_deg = (trigger * (rot_z * 0.001f)) * 0.0075f;

      	//sum_rot = old sum_rot + current_rot_deg
      	sum_rot += current_rot_deg;

      	//some output
      	Serial.print("Trigger: ");
      	Serial.print(trigger);
      	Serial.print(". Current_rot: ");
      	Serial.print(current_rot_deg);
      	Serial.print(". Sum_rot: ");
      	Serial.print(sum_rot);
	
      	//get direction of rotation
      	int dir = 1;

      	//display rot
      	if (sum_rot < 0) {
    		dir = -1;
    		sum_rot *= -1;
      	}
      	char digit_2 = displayMask( (int) (sum_rot + 0.5) % 10);
      	char digit_1 = displayMask( (int) ((sum_rot + 0.5) / 10) % 10);
      	char digit_0 = displayMask( (int) ((sum_rot + 0.5) / 100) % 10);
      	if (dir < 0){
    		if (digit_0 == displayMask(0))
    			digit_0 = displayMask(' ');
    	  	digit_0 |= displayMask('-');
    	  	sum_rot *= -1;
      	}
      	writetoDisplay(digit_0, digit_1, digit_2);

      	// degree -> speed
      	// v = -omega * k
      	// speed is [0, 255] (255 is quiet fast, so lets set the limit to 200)
      	// 360° is equal to v=200
      	// 1° is equal to 200/360 = 0.55
      	int v = 0;
	
      	if (dir < 0) sum_rot *= -1;

      	//compute speed
      	if (sum_rot <= 18)
      	//minimum speed = 10
    		v = 10;
      	else if (sum_rot >= 360)
      	//maximum speed = 200
    		v = 200;
      	else
    		v = sum_rot * 0.55f;

      	if (dir < 0) sum_rot *= -1;

      	Serial.print(". Speed: ");
      	Serial.print(v);

      	//if current_rot == 0 AND sum_rot != 0 go into drive mode
      	//leave drive mode, if sum_rot == 0
      	if (current_rot_deg == 0.0 && sum_rot != 0.0)
    		drive = true;

      	if (sum_rot == 0.0)
    		drive = false;

      	Serial.print(". Drive: ");
      	Serial.println(drive);
	
      	if (drive)
    		returnToOrigin(dir, v);      
   }
   
   // Driving without any collision
   if (modus==2) {
        // show modus
        displayDistance(2);

        // get distances
        uint8_t distance_left,distance_right;
        distance_right = linearizeDistance(readADC(channel_1));
        distance_left = linearizeDistance(readADC(channel_0));
   
        Serial.print("distance_right: ");
        Serial.print(distance_right);
        Serial.print("\r\n");
   
        Serial.print("distance_left: ");
        Serial.print(distance_left);
        Serial.print("\r\n");
   
        // Motor control
        // -----------------------------------------------------
        
        boolean turn_right = false;
        
        // show distances on screen
        char disp[3];
        disp[0] = NOTHING;
        disp[1] = NOTHING;
        disp[2] = NOTHING;

        if (distance_left < medium_distance) {
          if (distance_left < minimum_distance) {
            turn_right = true;
            disp[0] |= 0b01100011;
          }
          disp[0] |= 0b10001101;
        }
            
        if (distance_right < medium_distance) {
          if (distance_right < minimum_distance) {
            turn_right = true;
            disp[0] |= 0b00000011;
            disp[1] |= 0b00000011;
            disp[2] |= 0b00000011;
          }
          disp[0] |= 0b10000001;
          disp[1] |= 0b10000001;
          disp[2] |= 0b10000001;
        }
        
        writetoDisplay(disp[0]^=1, disp[1]^=1, disp[2]^=1);
        
        // move
        if (turn_right) {
          turnRight();
        }
        else {
          goAhead();
        }
        
        // -----------------------------------------------------
       
        delay(200);
   }
   
   modus=checkButtons();
}

void goAhead() {
   Serial.print("going ahead");
   Serial.print("\r\n");
   
   // left motor
   digitalWrite(12, HIGH); // direction (forward)
   analogWrite(3, 130); // speed
   
   // right motor
   digitalWrite(13, LOW); // direction (forward)
   analogWrite(11, 120); // speed
}

void turnRight() {
   Serial.print("turning right");
   Serial.print("\r\n");
   
   // left motor
   digitalWrite(12, HIGH); // direction (forward)
   analogWrite(3, 130); // speed
   
   // right motor
   digitalWrite(13, HIGH); // direction (backward)
   analogWrite(11, 120); // speed
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

void displayDistance (int8_t dist){
   /*
        char disp [3];
// Darstellung der Distanz in cm auf dem Display
// -----------------------------------------------------
// last digit
disp[2] = dist % 10;

// middle digit
disp[1] = (dist / 10) % 10;
if (disp[1] == 0 && dist < 100) {
         disp[1] = ' ';
}
// first digit
disp[0] = dist / 100;
if (disp[0] == 0) {
         disp[0] = ' ';
}
writetoDisplay(displayMask(disp[0])^=1, displayMask(disp[1])^=1, displayMask(disp[2])^=1);
// -----------------------------------------------------
*/
        
        char disp [3];
         // Darstellung der Distanz in cm auf dem Display
         // -----------------------------------------------------
        
         // last digit
        
         switch (dist % 10) {
         case 0: disp[2] = ZERO; break;
         case 1: disp[2] = ONE; break;
         case 2: disp[2] = TWO; break;
         case 3: disp[2] = THREE; break;
         case 4: disp[2] = FOUR; break;
         case 5: disp[2] = FIVE; break;
         case 6: disp[2] = SIX; break;
         case 7: disp[2] = SEVEN; break;
         case 8: disp[2] = EIGHT; break;
         case 9: disp[2] = NINE; break;
         default: break;
         }

         // middle digit
         switch ((dist / 10) % 10) {
         case 0:
         if (dist < 100) {
         disp[1] = NOTHING;
         }
         else {
         disp[1] = ZERO;
         }
         break;
         case 1: disp[1] = ONE; break;
         case 2: disp[1] = TWO; break;
         case 3: disp[1] = THREE; break;
         case 4: disp[1] = FOUR; break;
         case 5: disp[1] = FIVE; break;
         case 6: disp[1] = SIX; break;
         case 7: disp[1] = SEVEN; break;
         case 8: disp[1] = EIGHT; break;
         case 9: disp[1] = NINE; break;
         default: break;
         }
        
         // first digit
         switch (dist / 100) {
         case 0: disp[0] = NOTHING;break;
         case 1: disp[0] = ONE; break;
         case 2: disp[0] = TWO; break;
         case 3: disp[0] = THREE; break;
         case 4: disp[0] = FOUR; break;
         case 5: disp[0] = FIVE; break;
         case 6: disp[0] = SIX; break;
         case 7: disp[0] = SEVEN; break;
         case 8: disp[0] = EIGHT; break;
         case 9: disp[0] = NINE; break;
         default: break;
         }
        
         writetoDisplay(disp[0]^=1, disp[1]^=1, disp[2]^=1);
         // -----------------------------------------------------
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

uint8_t displayMask(char val){
        switch(val){
                case ' ': return 0b00000000;
                case 0: return 0b11111100;

                case 1: return 0b01100000;
                case 2: return 0b11011010;
                case 3: return 0b11110010;
                case 4: return 0b01100110;
                case 5: return 0b10110110;
                case 6: return 0b10111110;
                case 7: return 0b11100000;
                case 8: return 0b11111110;
                case 9: return 0b11110110;

                case 'a':
                case 'A': return 0b11101110;
                case 'b':
                case 'B': return 0b00111110;
                case 'c': return 0b00011010;
                case 'C': return 0b10011100;
                case 'd':
                case 'D': return 0b01111010;
                case 'e':
                case 'E': return 0b10011110;
                case 'f':
                case 'F': return 0b10001110;

                case '-': return 0b00000010;

                default: return 0b00000001;
        }
}

void returnBobbyToOrigin(int dir, int v){
//TODO: write method
//HINT: dir =  1 -> rotate clockwise
//		dir = -1 -> rotate counter clockwise
}
