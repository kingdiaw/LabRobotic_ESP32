/*
 * ESP32 NodeMCU AJAX Demo
 * Updates and Gets data from webpage without page refresh
 */
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
 
#include "index.h"  //Web page header file
 
WebServer server(80);
 
//Enter your SSID and PASSWORD
const char* ssid = "OPPO F9";
const char* password = "12345678";

//Global Variable
//===============================================================
unsigned long ledTick=0;

//GPIO Mapping
//===============================================================
const byte LED1 = 26;
const byte LED2 = 27;
const byte ledPin = 2;
 
//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot() {
 String s = MAIN_page; //Read HTML contents
 server.send(200, "text/html", s); //Send web page
}
 
void handleADC() {
 int a = analogRead(35);
 String adcValue = String(a);
 
 server.send(200, "text/plane", adcValue); //Send ADC value only to client ajax request
}
 
void handleLED() {
 String ledState = "OFF";
 String t_state = server.arg("LEDstate"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
 Serial.println(t_state);
 if(t_state == "26/on")
 {
  digitalWrite(LED1,HIGH); //LED ON
  ledState = "26ON"; //Feedback parameter
 }
 else if(t_state =="26/off")
 {
  digitalWrite(LED1,LOW); //LED OFF
  ledState = "26OFF"; //Feedback parameter  
 }
 else if(t_state =="27/on")
 {
  digitalWrite(LED2,HIGH); //LED ON
  ledState = "27ON"; //Feedback parameter  
 } 
 else if(t_state =="27/off")
 {
  digitalWrite(LED2,LOW); //LED ON
  ledState = "27OFF"; //Feedback parameter  
 } 
 
 server.send(200, "text/plane", ledState); //Send web page
}
//==============================================================
//                  SETUP
//==============================================================
void setup(void){
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");
 
  //Onboard LED port Direction output
  pinMode(ledPin,OUTPUT);
  pinMode(LED1,OUTPUT); 
  pinMode(LED2,OUTPUT);
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
 
  server.on("/", handleRoot);      //Which routine to handle at root location. This is display page
  server.on("/setLED", handleLED);
  server.on("/readADC", handleADC);
 
  server.begin();                  //Start server
  Serial.println("HTTP server started");
}
//==============================================================
//                     LOOP
//==============================================================
void loop(void){
  server.handleClient();          //Handle client requests
  if(millis()>ledTick){
    ledTick = millis()+300;
    digitalWrite(ledPin,digitalRead(ledPin)^1);
  }
}
