#include <Arduino.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"

/*------------------------------ DEFS ------------------------------*/


/*------------------------------ Fn's ------------------------------*/
String processor(const String& var);
void setupWeb();
void loopApp();
