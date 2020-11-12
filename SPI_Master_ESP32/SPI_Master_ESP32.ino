#include "SPI.h"
#include "PCF8574.h"  //Library:https://github.com/xreef/PCF8574_library
#include <Wire.h>

#define BUZ   P4
const byte sw1=39;
const byte ledPin=2;

static const int spiClk = 1000000; // 1 MHz
SPIClass * vspi = NULL;

PCF8574 IC2 (0x20);

//Global Variable
bool state=false;
byte Mastersend,Mastereceive; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  vspi = new SPIClass(VSPI);
  vspi->begin();
  pinMode(SS,OUTPUT);
  pinMode(sw1, INPUT);
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,LOW);
  IC2.pinMode(BUZ, OUTPUT);
  IC2.begin();
  IC2.digitalWrite(BUZ, LOW);
  while(HIGH==digitalRead(sw1));
  digitalWrite(ledPin,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  state ^= 1;
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(SS, LOW);
  Mastereceive = vspi->transfer(state);
  digitalWrite(SS, HIGH);
  vspi->endTransaction();
  if(Mastereceive == 1){
    digitalWrite(ledPin,HIGH);
  }
  else if(Mastereceive == 0){
    digitalWrite(ledPin,LOW);
  }
  delay(1000);
}
