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
int8_t channel_0 =  3;  // korrekte Werte bestimmen !
int8_t channel_1 =  2;
// -------------------------------------------------------------
// Laufzeit des Programms
unsigned long time;

enum number {
    ZERO    = 0b11111101,
    ONE     = 0b01100001,
    TWO     = 0b11011011,
    THREE   = 0b11110011,
    FOUR    = 0b01100111,
    FIVE    = 0b10110111,
    SIX     = 0b10111111,
    SEVEN   = 0b11100001,
    EIGHT   = 0b11111111,
    NINE    = 0b11110111,
    NOTHING = 0b00000001,
    MINUS   = 0b00000010
};

// the setup routine runs once when you press reset:
void setup() {                
    // initialize serial communication
    Serial.begin(9600);
    
    Serial.println("----------------------------------"    );
    Serial.println("PKES Wintersemester 2013/14"           );
    Serial.println("Vorlage 3. Aufgabe "                   );
    Serial.println("----------------------------------\r\n");
    
    // -------------------------------------------------------------
     // Single Onboard LED configuration
     // -------------------------------------------------------------
     // set leds LED_X as output
     // alternative
     //     - intermediate
     //       DDRA=1+2+4+8;
     //     - using processor makros
     //       DDRA=((1<<DDA0) | (1<<DDA1) | (1<<DDA2) | (1<<DDA3));
     //     - using hardware specific makros
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
    digitalRead(4);  // S1    
    
    // Configure PWM 
    // -----------------------------------------------------  
    
    
    // -----------------------------------------------------

    // Configure Flydurino
    new(flydurinoPtr) Flydurino;
    // -----------------------------------------------------  
    //((Flydurino*)flydurinoPtr)->configureZGyro(   );    
    
    // -----------------------------------------------------
    
    modus=0;
    sum_rot=0;
}

void loop() {
   // default state - avoids crash situations due to suddenly starting
   // PWM modus
   if (modus==0){
       writetoDisplay(0b10011111,0b11111101,0b10110111);
       
       while(modus==0){
	      modus=checkButtons();
              Serial.print("checking buttons. modus = 0");
              Serial.print("|\r\n");
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
            
      digitalWrite(12, HIGH);
      analogWrite(3, 100); // linker Motor (rückwärts)
      
      
      
       //current_rot_deg = ... ;
       //sum_rot=  sum_rot+ ...;
       
   }  
   // Driving without any collision
   if (modus==2){
       uint8_t distance_left,distance_right;
       distance_right = linearizeDistance(readADC(channel_1));
       distance_left = linearizeDistance(readADC(channel_0));
 
	// Motor control
	// -----------------------------------------------------
        displayDistance(distance_left);
	
        digitalWrite(12, LOW);
        analogWrite(3, 100); // rechter Motor (vorwärts)
	// -----------------------------------------------------    
       
       delay(50);
   }
   
   modus=checkButtons();
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
   int maxCnt       = 16;
   int sum          = 0;
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
		case '0': return 0b11111100;

		case '1': return 0b01100000;
		case '2': return 0b11011010;
		case '3': return 0b11110010;
		case '4': return 0b01100110;
		case '5': return 0b10110110;
		case '6': return 0b10111110;
		case '7': return 0b11100000;
		case '8': return 0b11111110;
		case '9': return 0b11110110;

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
