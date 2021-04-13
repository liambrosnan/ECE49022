#include "DistanceSensor.h"

/* Globals */
// defines pins numbers
//const int trigPin = 13;
const int trigPin = 32;
//const int echoPin = 12;
const int echoPin = 35;

// defines variables
long duration;
int distance;

const int statusClosedLower = 33;     // arbitrary low distance
const int statusClosedHigher = 42;    // arbitrary high distance
//const int statusClosedHigher = 400;    // arbitrary high distance
const int timeSec = 20;
unsigned long startTime = millis();
unsigned long lastTrig = 0;
// Prompt user for distance to door: lower = dist - 5, higher = dist + 5

int doorStatus = 0;
int count = 0;
int statusChange = 0;

/*
void measureDistance() {
  pollDistanceSensor();
}
*/

void setupDistanceSensor(){
Serial.print("Setup Distance Sensor\n");    
    pinMode(trigPin, OUTPUT);
    //pinMode(echoPin, INPUT);
    //Serial.begin(9600);
    
    // PIR Motion Sensor mode INPUT_PULLUP
    //pinMode(echoPin, INPUT_PULLUP);
    pinMode(echoPin, INPUT);
  // Set echo pin as interrupt, assign interrupt function and set CHANGE mode
  //if(count >= 100) {
  if(startTime - lastTrig > (timeSec*1000)) {
    attachInterrupt(digitalPinToInterrupt(echoPin), pollDistanceSensor, CHANGE);
  }
}

void pollDistanceSensor(){

    lastTrig = millis();
   
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);

    distance= duration*0.034/2;
    //Serial.print(distance);    

    if (distance >= statusClosedHigher){
        //count = 0;
        count++;
        if(doorStatus != 1 & count >= 100){
            delayMicroseconds(100);
            doorStatus = 1;
            Serial.print("Door Status: OPENED\n");
            statusChange = 1;
        }
    }
    else if(distance <= statusClosedLower){
        //count = 0;
        count++;
        if(doorStatus != 2 & count >= 100){
            delayMicroseconds(100);
            doorStatus = 2;
            Serial.print("Door Status: OBSTRUCTION\n");
            statusChange = 1;
        }
    }
    else{
        //count = 0;
        count++;
        if(doorStatus != 3 & count >= 100){
            delayMicroseconds(100);
            doorStatus = 3;
            Serial.print("Door Status: CLOSED\n");
            statusChange = 1;
        }
    }

    if(statusChange == 1){
        count = 0;
        statusChange = 0;
    }

    //Serial.print("Distance: ");
    //Serial.println(distance);
    //Serial.println(count);
}
