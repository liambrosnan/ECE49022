#include "DoorControls.h"
#include "DistanceSensor.h"

bool userDoorRequest = false;
int fixedCode = 256;

void setup() {
  // put your setup code here, to run once:
  //setupWeb(); //get fixed code each startup for door controls
  setupDistanceSensor();
  setupDoorControl();
  //setupCam();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  pollDistanceSensor(); //return data to send to webapp on request
  if(userDoorRequest){
    changeDoorState(fixedCode);
    userDoorRequest = false;
  }

}
