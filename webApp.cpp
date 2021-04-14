#include "webApp.h"

// Replace with your network credentials
//const char* ssid = "WillAF-WiFi";
//const char* password = "blackcarrot361";
const char* ssid = "EE162-2.4-g";
const char* password = "";

// Set LED GPIO
const int ledPin = 2;
// Stores LED state
String ledState;

// Create AsyncWebServer object on port 80
AsyncWebServer ayncServer(80);

// Replaces placeholder with LED state value
String processor(const String& var){
  Serial.println(var);
  if(var == "STATE"){
    if(digitalRead(ledPin)){
      ledState = "ON";
    }
    else{
      ledState = "OFF";
    }
    Serial.print(ledState);
    return ledState;
  }
  return String();
}
 
void setupWeb(){
  // Serial port for debugging purposes
  //Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Connect to Wi-Fi
  //IPAddress ip(192,168,3,154);
  WiFi.config(IPAddress(192,168,3,154), IPAddress(192,168,0,2), IPAddress(255,255,255,0));
  WiFi.begin(ssid, password);
  
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  ayncServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/home.html", String(), false, processor);
  });
  
  // Route to go to login page
  ayncServer.on("/login", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/login.html", String(), false, processor);
  });
  
  // Route to go to create account
  ayncServer.on("/create_account", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/createAccount.html", String(), false, processor);
  });

  /// Route to go to main menu
  ayncServer.on("/menu", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/menu.html", String(), false, processor);
  });

  /*// Route to sign out
  ayncServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/menu.html", String(), false, processor);
  });*/

  
  // Start server
  ayncServer.begin();
}
 
void loopApp(){
  
}
