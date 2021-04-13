#include <Wire.h>
#include <SPI.h>
//#include <WiFi.h>
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <ArduCAM.h>


void IRAM_ATTR detectsMovement();
void start_capture();
void camCapture(ArduCAM myCAM);
void serverCapture();
void serverStream();
void handleNotFound();
void serverHome();
void setupCam();
void pollCam();
