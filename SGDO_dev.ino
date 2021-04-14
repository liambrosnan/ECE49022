#include "DoorControls.h"
#include "DistanceSensor.h"
#include "ESP32_ARDUCAM.h"
#include "webApp.h"

bool userDoorRequest = false;
int fixedCode = 250;
//bool needsInterrupt = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("SGDO Begin");

  AsyncWebServer ayncServer(80);
  
  setupWeb(); //get fixed code each startup for door controls
  setupDistanceSensor();
  //setupDoorControl();
  setupCam();
}

void loop() {
  // put your main code here, to run repeatedly:
  pollDistanceSensor(); //return data to send to webapp on request
  /*if(userDoorRequest){
    changeDoorState(fixedCode);
    userDoorRequest = false;
  }*/
  pollCam(); //blocking, cannot poll cam.
  /* Potential fix:
   * set a timer/interrupt to poll door status. On state change, send data to webapp.
   * set interrupt for requests from webapp to either start/stop camera or change door state
   * how do make interrupts go do working for esp in cpp? what does?
   */

}
