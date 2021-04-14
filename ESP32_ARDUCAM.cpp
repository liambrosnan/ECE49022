#include <Arduino.h>
#include "ESP32_ARDUCAM.h"

#define timeSeconds 30

// Set GPIOs for LED, PIR Motion Sensor, and CS
const int led = 26;
const int motionSensor = 27;
const int CS = 5;

// Timer: Auxiliary variables
unsigned long lastMillis;
unsigned long frameCount;
unsigned int framesPerSecond;
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

WebServer server(81);
ArduCAM myCAM(OV2640, CS);

static const size_t bufferSize = 4096;
static uint8_t buffer[bufferSize] = {0xFF};
uint8_t temp = 0, temp_last = 0;
int i = 0;
bool is_header = false;
int imgMode = 0; // 0: stream  1: capture

/*-----------------------Functions------------------------------------*/
// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  if (!startTimer) {
    Serial.println("MOTION DETECTED!!!");
    digitalWrite(led, HIGH);
    startTimer = true;
    lastTrigger = millis();
  }

}

void start_capture() {
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
}

void camCapture(ArduCAM myCAM) {
  WiFiClient client = server.client();
  uint32_t len  = myCAM.read_fifo_length();
  if (len >= MAX_FIFO_SIZE) //8M
  {
    //Serial.println(F("Over size."));
  }
  if (len == 0 ) //0 kb
  {
    //Serial.println(F("Size is 0."));
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  if (!client.connected()) return;
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: image/jpeg\r\n";
  response += "Content-len: " + String(len) + "\r\n\r\n";
  server.sendContent(response);
  i = 0;
  while ( len-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buffer[i++] = temp;  //save the last  0XD9
      //Write the remain bytes in the buffer
      if (!client.connected()) break;
      client.write(&buffer[0], i);
      is_header = false;
      i = 0;
      myCAM.CS_HIGH();
      break;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < bufferSize)
        buffer[i++] = temp;
      else
      {
        //Write bufferSize bytes image data to file
        if (!client.connected()) break;
        client.write(&buffer[0], bufferSize);
        i = 0;
        buffer[i++] = temp;
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      buffer[i++] = temp_last;
      buffer[i++] = temp;
    }
  }
}

void serverStream() {
  WiFiClient client = server.client();

  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  int count = 0;
  unsigned long startTime = millis();
  unsigned long endTime = millis();
  while (client.connected() & ((endTime - startTime) / 1000 < 30)) {
    endTime = millis();
    start_capture();
    unsigned long startCapture = millis();
    while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
    size_t len = myCAM.read_fifo_length();
    if (len >= MAX_FIFO_SIZE) //8M
    {
      //Serial.println(F("Over size."));
      continue;
    }
    if (len == 0 ) //0 kb
    {
      //Serial.println(F("Size is 0."));
      continue;
    }
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();
    if (!client.connected()) break;
    response = "--frame\r\n";
    response += "Content-Type: image/jpeg\r\n\r\n";
    server.sendContent(response);
    while ( len-- )
    {
      temp_last = temp;
      temp =  SPI.transfer(0x00);

      //Read JPEG data from FIFO
      if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
      {
        buffer[i++] = temp;  //save the last  0XD9
        //Write the remain bytes in the buffer
        myCAM.CS_HIGH();;
        if (!client.connected()) break;
        client.write(&buffer[0], i);
        is_header = false;
        i = 0;
      }
      if (is_header == true)
      {
        //Write image data to buffer if not full
        if (i < bufferSize)
          buffer[i++] = temp;
        else
        {
          //Write bufferSize bytes image data to file
          myCAM.CS_HIGH();
          if (!client.connected()) break;
          client.write(&buffer[0], bufferSize);
          i = 0;
          buffer[i++] = temp;
          myCAM.CS_LOW();
          myCAM.set_fifo_burst();
        }
      }
      else if ((temp == 0xD8) & (temp_last == 0xFF))
      {
        is_header = true;
        buffer[i++] = temp_last;
        buffer[i++] = temp;
      }
    }
    if (!client.connected()) break;

    // Calculate FPS
    unsigned long endCapture = millis();
    int frameRate = 1000 / (endCapture - startCapture);
    count++;
    if (count > 30 & frameRate > 14) {
      Serial.print("FPS = ");
      Serial.println(frameRate);
      count = 0;
    }

    /*String numFrames = String(frameRate);
      response = "Content-Type: text/html; charset=UTF-8";
      response += numFrames;
      server.sendContent(response);
      String message = "<div><h3>FPS: " + numFrames + "</h3></div> \n";
      server.send(200, "text/html", message);
    */
  }
}

void handleNotFound() {
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text/plain", message);
}

void serverHome() {
  //// default HTML
  String message = "<html><head>\n";
  message += "</head><body>\n";
  //message += "<form action=\"http://" + ipStr + "/submit\" method=\"POST\">";
  message += "<h1 style=\"font-size:11px\">Garage Door Live Feed</h1>\n";
  //if (errMsg != "")
  //  message += "<div style=\"color:red\">" + errMsg + "</div>";


  if (imgMode == 0) // stream mode
  {
    //message += "<div><h2>Video Streaming</h2></div> \n";
    message += "<div><h2 style=\"font-size:11px\">Video Streaming</h2></div> \n";
    message += "<div><img id=\"ArduCam\" src=\"http://192.168.3.155:81/stream\" ></div>\n";
    //message += "<div><iframe src=\"http://" + ipStr + "/stream\" ></iframe></div>\n";
    //imgMode = 1;

  }

  //message += "</form> \n";
  message += "</body></html>\n";

  server.send(200, "text/html", message);
}

/*void fps(int seconds) {
  frameCount ++;
  if ((millis() - lastMillis) > (seconds * 1000)) {
    framesPerSecond = (frameCount / seconds);
    Serial.print("FPS:  ");
    Serial.println(framesPerSecond);
    frameCount = 0;
    lastMillis = millis();
  }
  }*/

void setupCam() {
  uint8_t vid, pid;
  uint8_t temp;
  pinMode(CS, OUTPUT);

  //I2C START SDA, SCL
  Wire.begin(21, 22);

  // initialize SPI: SCK, MISO, MOSI, SS
  SPI.begin(18, 19, 23, 5);
  SPI.setFrequency(4000000); //4MHz

  //Check if the ArduCAM SPI bus is OK
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55) {
    Serial.println(F("SPI1 interface Error!"));
    //while(1);
  }

  //Check if the camera module type is OV2640
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
  // if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 )))
  //Serial.println(F("Can't find OV2640 module!"));
  // else
  //Serial.println(F("OV2640 detected."));


  //Change to JPEG capture mode and initialize the OV2640 module
  myCAM.set_format(JPEG);
  myCAM.InitCAM();

  // Sets the jpeg size of one frame
  myCAM.OV2640_set_JPEG_size(OV2640_160x120);
  //myCAM.OV2640_set_JPEG_size(OV2640_640x480);
  myCAM.clear_fifo_flag();
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);
  //wifiManager.setAPStaticIPConfig(IPAddress(192,168,3,155), IPAddress(192,168,0,1), IPAddress(255,255,255,0));
  wifiManager.setSTAStaticIPConfig(IPAddress(192, 168, 3, 155), IPAddress(192, 168, 0, 1), IPAddress(255, 255, 255, 0)); // optional DNS 4th argument
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("ESP32_CAM")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, may not work for PCB
    ESP.restart();
    delay(5000);
  }
  // Start the server
  server.on("/", HTTP_GET, serverHome);
  server.on("/stream", HTTP_GET, serverStream);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println(F("Server started"));

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  // Set LED to LOW
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
}

void pollCam() {
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }
  // Current time
  now = millis();
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if (startTimer && (now - lastTrigger > (timeSeconds * 1000))) {
    Serial.println("Motion stopped...");
    digitalWrite(led, LOW);
    startTimer = false;
  }
}
