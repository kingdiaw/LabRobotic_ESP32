#include "SPI.h"
const byte sw1=39;
const byte ledPin=2;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(sw1, INPUT);
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,LOW);
  while(HIGH==digitalRead(sw1));
  digitalWrite(ledPin,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("MOSI:");
  Serial.println(MOSI);
  Serial.print("MISO:");
  Serial.println(MISO);
  Serial.print("SCK:");
  Serial.println(SCK);
  Serial.print("SS:");
  Serial.println(SS);
  while(1);
}
