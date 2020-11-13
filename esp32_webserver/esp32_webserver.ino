#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiAP.h>

#define ledPin  2
String ledState;


// Replace with your network credentials
const char *ssid = "yourAP";
const char *password = "";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

//const char index_html[] PROGMEM = R"rawliteral(
//<!DOCTYPE html>
//<html>
//<head>
// <title>ESP Web Server</title>
// <meta name="viewport" content="width=device-width, initial-scale=1">
// <link rel="icon" href="data:,">
// <style>
//html {
// font-family: Arial, Helvetica, sans-serif;
// text-align: center;
//}
//h1 {
// font-size: 1.8rem;
// color: white;
//}
//.topnav {
// overflow: hidden;
// background-color: #0A1128;
//}
//body {
// margin: 0;
//}
//.content {
// padding: 50px;
//}
//.card-grid {
// max-width: 800px;
// margin: 0 auto;
// display: grid;
// grid-gap: 2rem;
// grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
//}
//.card {
// background-color: white;
// box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
//}
//.card-title {
// font-size: 1.2rem;
// font-weight: bold;
// color: #034078
//}
//.state {
// font-size: 1.2rem;
// color: #1282A2;
//}
//Button {
// border: none;
// color: #FEFCFB;
// padding: 15px 32px;
// text-align: center;
// text-decoration: none;
// font-size: 16px;
// width: 100px;
// border-radius: 4px;
// transition-duration: 0.4s;
//}
//.button-on {
// background-color: #034078;
//}
//.button-on:hover {
// background-color: #1282A2;
//}
//.button-off {
// background-color: #858585;
//}
//.button-off:hover {
// background-color: #252524;
//}
// </style>
// </head>
//<body>
// <div class="topnav">
// <h1>ESP WEB SERVER</h1>
// </div>
// <div class="content">
// <div class="card-grid">
// <div class="card">
// <p class="card-title"><i class="fas fa-lightbulb"></i> GPIO 2</p>
// <p>
// <a href="on"><button class="button-on">ON</button></a>
// <a href="off"><button class="button-off">OFF</button></a>
// </p>
// <p class="state">State: %STATE%</p>
// </div>
// </div>
// </div>
//
//</body>
//</html>
//)rawliteral";

void initWiFi() {
  WiFi.softAP(ssid,password );
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  Serial.println("Server started");
}

// Replaces placeholder with LED state value
String processor(const String& var) {
 if(var == "STATE") {
 if(digitalRead(ledPin)) {
 ledState = "OFF";
 }
 else {
 ledState = "ON";
 }
 return ledState;
 }
 return String();
}

void setup() {
 // Serial port for debugging purposes
 Serial.begin(115200);
 pinMode(ledPin, OUTPUT);
 initWiFi();
// server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
// request->send_P(200, "text/html", index_html, processor);
// });
//  // Route to set GPIO state to HIGH (inverted logic for ESP8266)
// server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request){
// digitalWrite(ledPin, HIGH);
//  request->send_P(200, "text/html", index_html, processor);
// });
//
//  server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request){
// digitalWrite(ledPin, LOW);
// request->send_P(200, "text/html", index_html, processor);
// });


 // Start server
 server.begin();
}
void loop() {
 // put your main code here, to run repeatedly:
}
