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
int8_t linearizeDistance(int distanceRaw);
void displayDistance (int8_t dist);
int readADC(int8_t channel);
void writetoDisplay(char digit1, char digit2, char digit3);

char flydurinoPtr[sizeof(Flydurino)];
// aktuelle Beschleunigungswerte, Kompassmessungen
int16_t acc_x, acc_y, acc_z;
int16_t ori_x, ori_y, ori_z;
// Wasserwaage oder Distanzmessung
int8_t modus;
// Kanal des ADC Wandlers
int8_t channel = 3;
int8_t trigger = 50;
int8_t MAXTRIGGER = 50;

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
	NOTHING = 0b00000001
};

enum spiritlevel {
        MIDDLE  = 0b00000011,
        RIGHT   = 0b01100001,
        LEFT    = 0b00001101,
        UP      = 0b10000001,
        DOWN    = 0b00010001,
        FLAT    = RIGHT | LEFT
};
        

// the setup routine runs once when you press reset:
void setup() {                
    // initialize serial communication
    Serial.begin(9600);
    new(flydurinoPtr) Flydurino;
    
    Serial.println("----------------------------------"    );
    Serial.println("PKES Wintersemester 2013/14"           );
    Serial.println("Vorlage 2. Aufgabe "                   );
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
    DDRF &=~(1<<DDG4);
    // S2 as input
    DDRG &=~(1<<DDG5);

    // Configuration of ADC 
    // -----------------------------------------------------
    
    
    // -----------------------------------------------------       
    
    // Configure buttons
    // -----------------------------------------------------
    
    
    // -----------------------------------------------------
    modus=1;
    
}

// the loop routine runs over and over again forever:
void loop() {
  
   if (modus==1){
       // ----------------------------------------------------------------
       // level 
       // ----------------------------------------------------------------
       // Receive acceleromation values
       ((Flydurino*)flydurinoPtr)->getAcceleration(&acc_x, &acc_y, &acc_z);
       // Get compass data
       ((Flydurino*)flydurinoPtr)->getOrientation(&ori_x, &ori_y, &ori_z);
      
       Serial.print(acc_x); Serial.print("\t");
       Serial.print(acc_y); Serial.print("\t");
       Serial.print(acc_z); Serial.print("|\t");
       Serial.print(ori_x); Serial.print("\t");
       Serial.print(ori_y); Serial.print("\t");
       Serial.print(ori_z); Serial.print("|\r\n"); 
       
       displaySpiritLevel(acc_x, acc_y, acc_z);
   }
   
   if (modus==2){
       int8_t distance;
       // ----------------------------------------------------------------
       // Distance sensor
       // ----------------------------------------------------------------
       
       if (trigger == 0) {
         distance = linearizeDistance(readADC(channel));
         displayDistance (distance);
         trigger = MAXTRIGGER;
       }
       else { trigger--; }

   }

   modus=checkButtons();
}

int8_t checkButtons(){
   //int8_t modus=0;
   // Abfrage der Buttons und Moduswechsel
   // -----------------------------------------------------
   if (PINF & (1<<4)) {
     modus = 1;
   }
   if (PING & (1<<5)) {
     modus = 2;   
   }
   // -----------------------------------------------------
   return modus;
}


void displaySpiritLevel(int16_t acc_x, int16_t acc_y, int16_t acc_z)
{
   //   3 cases for roll and pitch
   // -15 Grad <= alpha,
   // -15 Grad <= alpha  <= 15 Grad
   //  15 Grad <= alpha 
   // -----------------------------------------------------
   char disp[3];
   
   if (acc_y <= -4000)
     for (int i = 0; i < 3; i++)
       disp[i] |= UP;
   else if (acc_y >= 4000)
     for (int i = 0; i < 3; i++)
       disp[i] |= DOWN;
   else
     for (int i = 0; i < 3; i++)
       disp[i] |= MIDDLE;
       
   //   7 cases for tilt
   if (acc_x <= 9000)
     disp[0] |= LEFT;
   else if (acc_x <= 6000)
     disp[0] |= RIGHT;
   else if (acc_x <= 3000)
     disp[1] |= LEFT;
   else if (acc_x >= -9000)
     disp[2] |= RIGHT;
   else if (acc_x >= -6000)
     disp[2] |= LEFT;
   else if (acc_x >= -3000)
     disp[1] |= RIGHT;
   else
     disp[1] |= FLAT;
   
   writetoDisplay(disp[0]^=1, disp[1]^=1, disp[2]^=1);
      
   // -----------------------------------------------------
}


int8_t linearizeDistance(int distance_raw){
   int8_t distance_cm=0; 
   // Transformation der Spannungsbezogenen Distanzwerte in
   // eine Entfernung in cm
   // -----------------------------------------------------
   
   // -----------------------------------------------------
   //return distance_cm;
   return distance_raw; 
}

void displayDistance (int8_t dist)
{
   // Darstellung der Distanz in cm auf dem Display
   // -----------------------------------------------------
   Serial.print("Distance sensor:   ");
   Serial.print(dist);
   Serial.print("\r\n");
   
   char disp[3];
   
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
	case 0: if (dist < 100) {
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
      switch ((dist / 100) % 10) {
        case 0: disp[0] = NOTHING; break;
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

int readADC (int8_t channel){
   int distance_raw=255;
   // möglicherweise mehrmaliges Lesen des ADC Kanals
   // Mittelwertbildung 
   // -----------------------------------------------------
   long sum = 0;
   int COUNTER = 10;
   for (int i = 1; i <= COUNTER; i++) {
     sum += analogRead(channel);
   }
   distance_raw = sum / COUNTER;
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
