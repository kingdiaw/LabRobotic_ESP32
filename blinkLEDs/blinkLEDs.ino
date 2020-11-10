/*
 Blink led on PIN0
 by Mischianti Renzo <http://www.mischianti.org>

 https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
*/

#include "Arduino.h"
#include "PCF8574.h"

#define LED8  P7
#define LED7  P6
#define LED6  P5
#define BUZ   P4

// Set i2c address
PCF8574 IC2(0x20);

void setup()
{
	Serial.begin(115200);

	// Set pinMode to OUTPUT
	IC2.pinMode(LED8, OUTPUT);
	IC2.pinMode(LED7, OUTPUT);
  IC2.pinMode(LED6, OUTPUT);
  IC2.pinMode(BUZ, OUTPUT);
	IC2.begin();
}

void loop()
{
	IC2.digitalWrite(BUZ, HIGH);
  IC2.digitalWrite(LED8,HIGH);
  IC2.digitalWrite(LED7,HIGH);
  IC2.digitalWrite(LED6,HIGH);
	delay(1000);
	IC2.digitalWrite(BUZ, LOW);
  IC2.digitalWrite(LED8, LOW);
  IC2.digitalWrite(LED7, LOW);
  IC2.digitalWrite(LED6, LOW);
	delay(1000);
}
