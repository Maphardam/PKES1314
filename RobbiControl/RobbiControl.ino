//https://github.com/cmassicot/P2PMQTT_Arduino_Libraries/tree/master/UsbHost

#include <AndroidAccessory.h>
#include <util/delay.h>
#include "Handbag.h"

uint8_t linearizeDistance(uint16_t distanceRaw);
uint16_t readADC(int8_t channel);
uint8_t displayMask(char val);
void writetoDisplay(char digit1, char digit2, char digit3);

AndroidAccessory acc("rancidbacon.com",
		     "Handbag",
		     "Handbag (Arduino Board)",
		     "0.1",
		     "http://HandbagDevices.com/#i");

HandbagApp Handbag(acc);
unsigned int displayWidget1Id; 

void callMeGO() {
  Serial.println("GO called."); 
         
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  analogWrite(3, 640);
  analogWrite(11, 640);
}

void callMeSTOP() {
  Serial.println("STOP called."); 
         
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  analogWrite(3, 0);
  analogWrite(11, 0);
}

void textInputCallback(char *msg) {
  Serial.println(msg);
  writetoDisplay(displayMask(msg[0]),
    	      displayMask(msg[1]),
    	      displayMask(msg[2]));
}

void setupUI() {
  /*
   */
  Handbag.addLabel("PKES 2013/14", 32, 0x01);
  Handbag.addLabel("Praktische Aufgabe 5", 32, 0x01);
  Handbag.addLabel(" ");
  Handbag.addLabel("Messdaten links/rechts [cm]"); 
  displayWidget1Id = Handbag.addLabel("0.00", 32, 0x01);  
  Handbag.addLabel(" "); 
  Handbag.addLabel("Motorsteuerung"); 
  Handbag.addButton("GO", callMeGO); 
  Handbag.addButton("STOP", callMeSTOP); 
  Handbag.addLabel(" "); 
  Handbag.addLabel("Change button label (press enter):");  
  Handbag.addLabel("", 6);
  Handbag.addTextInput(textInputCallback);
}


void setup() {
  Serial.begin(9600);
  Serial.println("----------------------------------"    );
  Serial.println("PKES Wintersemester 2013/14"           );
  Serial.println("Vorlage 5. Aufgabe "                   );
  Serial.println("----------------------------------\r\n"); 
  // Configuration of ADC 
  // -----------------------------------------------------
  analogReference(INTERNAL2V56);
  // -----------------------------------------------------  
  
  // Configure PWM 
  // -----------------------------------------------------  
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
    
  pinMode(2, OUTPUT);
  pinMode(11, OUTPUT); 
  
  // -------------------------------------------------------------
  // Serial bus lines
  // -------------------------------------------------------------
  // Pin 5 = PORT E 3 = clock
  DDRE |= (1<<DDE3);
  // Pin 6 = PORT H 3 = data
  DDRH |= (1<<DDH3);
  // Pin 7 = PORT H 4 = enable
  DDRH |= (1<<DDH4);
  
  writetoDisplay(0b10011111,0b11111101,0b10110111);
  
  Handbag.begin(setupUI);
}


void loop() {
    Handbag.refresh(); 
    uint8_t distance_left,distance_right;
    distance_right = linearizeDistance(readADC(3));
    distance_left = linearizeDistance(readADC(2));
    
    char result[11];
  
    if (distance_right<80){
      result[0] = (distance_right / 10) + '0';
      result[1] = (distance_right % 10) + '0';
    }else{
      result[0] = '-';
      result[1] = '-';
    }
    result[2] = ' ';
    result[3] = 'c'; 
    result[4] = 'm';
    result[5] = ' '; 
    if (distance_left<80){
      result[6] = (distance_left / 10) + '0';
      result[7] = (distance_left % 10) + '0';
    }else{
      result[6] = '-';
      result[7] = '-';
    }
    result[8] = ' ';
    result[9] = 'c';
    result[10]= 'm';
    Handbag.setText(displayWidget1Id, result);
}


uint16_t readADC (int8_t channel){
   uint16_t distance_raw=0xFFFF;
   // möglicherweise mehrmaliges Lesen des ADC Kanals
   // Mittelwertbildung 
   // -----------------------------------------------------
   distance_raw = analogRead(channel);

   // -----------------------------------------------------
   return distance_raw;
}

uint8_t linearizeDistance(uint16_t distance_raw){
   double distance_cm=0;
   // Transformation der Spannungsbezogenen Distanzwerte in
   // eine Entfernung in cm
   // -----------------------------------------------------
   //1.791172e+02, -8.711005e-01,  1.819673e-03, -1.711544e-06,  5.923521e-

   distance_cm= 179.1172 +
		        -0.8711005 * distance_raw +
		         0.001819673 *pow(distance_raw,2)+
		        -0.000001711544 * pow(distance_raw,3)+
		         0.0000000005923521 * pow(distance_raw, 4);
   //Serial.println(distance_cm);
   // -----------------------------------------------------
   return (int8_t)ceil(distance_cm);
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
