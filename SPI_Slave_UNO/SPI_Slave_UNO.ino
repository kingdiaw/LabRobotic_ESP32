/*
Example code taken from:
https://circuitdigest.com/microcontroller-projects/arduino-spi-communication-tutorial
*/

/*
CIRCUIT CONNECTION
MASTER (ESP32) <--> SLAVE (UNO)
  MOSI (23) <-->  MOSI (11)
  MISO (19) <-->  MISO (12)
  CLK (18)  <-->  SCK (13)
  CS0(5)    <-->  SS (10)
*/

//SPI Slave (UNO)

#include<SPI.h>
#define buttonpin 2
#define LEDpin  7

volatile boolean received;
volatile byte Slavereceived,Slavesend;
int buttonvalue;
int x;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(buttonpin, INPUT_PULLUP);
pinMode(LEDpin,OUTPUT);
pinMode(MISO, OUTPUT);  //Sets MISO as OUTPUT (Have to Send data to Master IN
SPCR |= _BV(SPE);                       //Turn on SPI in Slave Mode
received = false;

SPI.attachInterrupt();                  //Interuupt ON is set for SPI commnucation
  
}

ISR (SPI_STC_vect)                        //Inerrrput routine function 
{
  Slavereceived = SPDR;         // Value received from master if store in variable slavereceived
  received = true;                        //Sets received as True 
}

void loop() {
  // put your main code here, to run repeatedly:
if(received)                            //Logic to SET LED ON OR OFF depending upon the value recerived from master
   {
      if (Slavereceived==1) 
      {
        digitalWrite(LEDpin,HIGH);         //Sets pin 7 as HIGH LED ON
        Serial.println("Slave LED ON");
      }else
      {
        digitalWrite(LEDpin,LOW);          //Sets pin 7 as LOW LED OFF
        Serial.println("Slave LED OFF");
      }
      
      buttonvalue = digitalRead(buttonpin);  // Reads the status of the pin 2
      
      if (buttonvalue == HIGH)               //Logic to set the value of x to send to master
      {
        x=1;
        
      }else
      {
        x=0;
      }
      
  Slavesend=x;                             
  SPDR = Slavesend;                           //Sends the x value to master via SPDR 
  delay(1000);
}  

}
