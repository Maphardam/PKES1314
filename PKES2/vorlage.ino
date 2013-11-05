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
       
       distance = linearizeDistance(readADC(channel));
       displayDistance (distance);

   }

   modus=checkButtons();
}

int8_t checkButtons(){
   int8_t modus=0;
   // Abfrage der Buttons und Moduswechsel
   // -----------------------------------------------------
   
   
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
   
   
   // -----------------------------------------------------
}


int8_t linearizeDistance(int distance_raw){
   int8_t distance_cm=0; 
   // Transformation der Spannungsbezogenen Distanzwerte in
   // eine Entfernung in cm
   // -----------------------------------------------------
   
   
   // -----------------------------------------------------
   return distance_cm; 
}

void displayDistance (int8_t dist)
{
   // Darstellung der Distanz in cm auf dem Display
   // -----------------------------------------------------
   
   
   // -----------------------------------------------------
}

int readADC (int8_t channel){
   int distance_raw=255;
   // möglicherweise mehrmaliges Lesen des ADC Kanals
   // Mittelwertbildung 
   // -----------------------------------------------------
    
    
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
