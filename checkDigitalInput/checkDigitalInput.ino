#include "Arduino.h"
#include "PCF8574.h"  //Library:https://github.com/xreef/PCF8574_library

#define LED8  P7
#define LED7  P6
#define LED6  P5
#define BUZ   P4
#define S1    P0
#define S2    P1
#define S3    P2
#define S4    P3
#define S5    P4

// Set i2c address
PCF8574 IC1 (0x21);
PCF8574 IC2 (0x20);


void setup()
{
	Serial.begin(115200);

  IC1.pinMode(S1, INPUT);
  IC1.pinMode(S2, INPUT);
  IC1.pinMode(S3, INPUT);
  IC1.pinMode(S4, INPUT);
  IC1.pinMode(S5, INPUT);
	// Set pinMode to OUTPUT
	IC2.pinMode(LED8, OUTPUT);
	IC2.pinMode(LED7, OUTPUT);
  IC2.pinMode(LED6, OUTPUT);
  IC2.pinMode(BUZ, OUTPUT);
	IC1.begin();
	IC2.begin();
}

void loop()
{
	IC2.digitalWrite(BUZ, HIGH);
  IC2.digitalWrite(LED8,HIGH);
  IC2.digitalWrite(LED7,HIGH);
  IC2.digitalWrite(LED6,HIGH);
	delay(500);
	IC2.digitalWrite(BUZ, LOW);
  IC2.digitalWrite(LED8, LOW);
  IC2.digitalWrite(LED7, LOW);
  IC2.digitalWrite(LED6, LOW);
	delay(500);
 Serial.print("Sensor Array:");
 Serial.print(IC1.digitalRead(S1));
 Serial.print(IC1.digitalRead(S2));
 Serial.print(IC1.digitalRead(S3));
 Serial.print(IC1.digitalRead(S4));
 Serial.println(IC1.digitalRead(S5));
}
