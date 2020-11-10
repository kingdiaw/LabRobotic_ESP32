#include "Arduino.h"
#include "BluetoothSerial.h"
#include "PCF8574.h"  //Library:https://github.com/xreef/PCF8574_library

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Mapping I/O
//================================================
#define LED8  P7
#define LED7  P6
#define LED6  P5
#define BUZ   P4
#define S1    P0
#define S2    P1
#define S3    P2
#define S4    P3
#define S5    P4
#define PB1   39
#define PB2   34
//================================================

//Interrupt Mapping
//================================================
#define ESP32_INTERRUPTED_PIN 26

//Function ISR
void readSensorArray();
//================================================

//Mapping Object
//===============================================
// Set i2c address
PCF8574 IC1 (0x21,ESP32_INTERRUPTED_PIN,readSensorArray);
PCF8574 IC2 (0x20);
BluetoothSerial SerialBT;

//================================================

//Global Variable
bool sensorDetected = false;
bool PB1_old = true;
bool PB2_old = true;
bool PB1_new, PB2_new;
bool state;
unsigned char command;
unsigned long ledTick;

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

 pinMode(PB1, INPUT);
 pinMode(PB2, INPUT);

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
  //Handle Serial-Bluetooh Comm
  //======================================
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    command = SerialBT.read();
    Serial.write(command);   
  }  
  //======================================

  //Handle Blinking LEDs and buzzer
  //======================================
  if(millis()>ledTick){
    ledTick = millis()+500;
    state ^=1;  //toggle it
    IC2.digitalWrite(BUZ, state);
    IC2.digitalWrite(LED8,state);
    IC2.digitalWrite(LED7,state);
    IC2.digitalWrite(LED6,state);    
  }
  //=======================================

  //Handle Read PB state
  //=======================================
 PB1_new = digitalRead(PB1);
 PB2_new = digitalRead(PB2);
 if(PB1_old != PB1_new){
  PB1_old = PB1_new;
    if(PB1_new == LOW){
      Serial.println("PB1 Pressed!");
    }
 }
 if(PB2_old != PB2_new){
  PB2_old = PB2_new;
    if(PB2_new == LOW){
      Serial.println("PB2 Pressed!");
    }
 }
 //==========================================

 //Handle IC1 Pin Change (Interrupt)
 //========================================
 if(sensorDetected){
   Serial.print("Sensor Array:");
   Serial.print(IC1.digitalRead(S1));
   Serial.print(IC1.digitalRead(S2));
   Serial.print(IC1.digitalRead(S3));
   Serial.print(IC1.digitalRead(S4));
   Serial.println(IC1.digitalRead(S5));
   sensorDetected = false;
   }
 //=======================================
 
}

//Interrupt Service Routine
//========================================
void readSensorArray(){
  sensorDetected = true;
  delay(100);
}
//========================================
