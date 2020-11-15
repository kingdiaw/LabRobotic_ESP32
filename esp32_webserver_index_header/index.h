const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
</head>
<style>
html{
font-family:verdana;
font-size:110%;
text-align:center;
}
.button{
font-size:120%;
color:white;
background-color:green;
border:none;
margin:7px;
padding:13px;
padding-right:30px;
padding-left:30px;
}
.button2{
background-color:gray;
}
.stateFrame{
border-style: groove;  
}
</style>
<body>
 
<div>
<h2>ESP32 NodeMCU Web Server</h1>
  <p class="stateFrame">LED 1 State - <span id="txt1">NA<span></p>
	<button type="button" class="button" onclick="sendData('26/on')">ON</button> <br>
	<button type="button" class="button button2" onclick="sendData('26/off')">OFF</button><br>
  <p class="stateFrame">LED 2 State - <span id="txt2">NA</span></p>
  <button type="button" class="button" onclick="sendData('27/on')">ON</button> <br>
  <button type="button" class="button button2" onclick="sendData('27/off')">OFF</button><br>
</div>
 
<div id="adcValue" style="border-style: groove;">
	ADC Value is : <span id="ADCValue">0</span><br>
</div>
<script>
function sendData(led) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      if(this.responseText =="26ON"){
      document.getElementById("txt1").innerHTML ="ON";        
      }
      else if(this.responseText =="26OFF"){
      document.getElementById("txt1").innerHTML ="OFF";        
      }
      else if(this.responseText =="27ON"){
      document.getElementById("txt2").innerHTML ="ON";        
      }   
      else if(this.responseText =="27OFF"){
      document.getElementById("txt2").innerHTML ="OFF";        
      }         
    }
  };
  
  xhttp.open("GET", "setLED?LEDstate="+led , true);
  xhttp.send();
}
 
setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getData();
}, 2000); //2000mSeconds update rate
 
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ADCValue").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "readADC", true);
  xhttp.send();
}
</script>
</body>
</html>
)=====";
