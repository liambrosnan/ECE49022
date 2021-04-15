#include <ETH.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <SPI.h>
#include <WiFiManager.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
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
