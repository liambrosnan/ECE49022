#include <Arduino.h>
#include "ESP32_ARDUCAM.h"

const int CS = 5;

/////////////////////////////////////////////////////////////////////
#define timeSeconds 10
// Set GPIOs for LED and PIR Motion Sensor
const int led = 26;
const int motionSensor = 27;

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;
/////////////////////////////////////////////////////////////////////

WebServer server(80);
AsyncWebServer asyncserver(81);
AsyncWebSocket ws("/ws");
//SSD1306  display(0x3c, 5, 4);
ArduCAM myCAM(OV2640, CS);

static const size_t bufferSize = 4096;
static uint8_t buffer[bufferSize] = {0xFF};
uint8_t temp = 0, temp_last = 0;
int i = 0;
bool is_header = false;
int imgMode = 0; // 0: stream  1: capture

/*-----------------------Functions------------------------------------*/

void start_capture(){
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
}

void camCapture(ArduCAM myCAM){
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

void serverCapture(){
  delay(1000);
  start_capture();
  //Serial.println(F("CAM Capturing"));

  int total_time = 0;

  total_time = millis();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  total_time = millis() - total_time;
  //Serial.print(F("capture total_time used (in miliseconds):"));
  //Serial.println(total_time, DEC);

  total_time = 0;

  //Serial.println(F("CAM Capture Done."));
  total_time = millis();
  camCapture(myCAM);
  total_time = millis() - total_time;
  //Serial.print(F("send total_time used (in miliseconds):"));
  //Serial.println(total_time, DEC);
  //Serial.println(F("CAM send Done."));
}

void serverStream(){
    WiFiClient client = server.client();

    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server.sendContent(response);

    while (1){
        start_capture();
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
    }
}

void handleNotFound(){
    String message = "Server is running!\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET)?"GET":"POST";
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
  message += "<h1>Garage Door Live Feed</h1>\n";
  //if (errMsg != "")
  //  message += "<div style=\"color:red\">" + errMsg + "</div>";


  if (imgMode == 0) // stream mode
  {
    message += "<div><h2>Video Streaming</h2></div> \n";
    message += "<div><img id=\"ArduCam\" src=\"http://10.0.0.112/stream\" ></div>\n";
    //message += "<div><iframe src=\"http://" + ipStr + "/stream\" ></iframe></div>\n";
    //imgMode = 1;

  }

  //message += "</form> \n";
  message += "</body></html>\n";

  server.send(200, "text/html", message);
}

void setupCam() {
    uint8_t vid, pid;
    uint8_t temp;
    pinMode(CS,OUTPUT);

    //I2C START SDA, SCL
    Wire.begin(21,22);
    //display.init();
    //display.flipScreenVertically();
    //display.setFont(ArialMT_Plain_10);
    //Serial.begin(115200);

// initialize SPI: SCK, MISO, MOSI, SS
    SPI.begin(18,19,23,5);
    SPI.setFrequency(4000000); //4MHz

//Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55){
        //Serial.println(F("SPI1 interface Error!"));
        while(1);
}

//Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 )))
        //Serial.println(F("Can't find OV2640 module!"));
    else
        //Serial.println(F("OV2640 detected."));


//Change to JPEG capture mode and initialize the OV2640 module
    myCAM.set_format(JPEG);
    myCAM.InitCAM();

// Sets the jpeg size of one frame
    myCAM.OV2640_set_JPEG_size(OV2640_640x480);
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

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
    if(!wifiManager.autoConnect("ESP32_CAM")) {
        //Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.restart();
        delay(5000);
    }
// Start the server
    server.on("/", HTTP_GET, serverHome);
    server.on("/capture", HTTP_GET, serverCapture);
    server.on("/stream", HTTP_GET, serverStream);
    server.onNotFound(handleNotFound);
    server.begin();
    //Serial.println(F("Server started"));

/////////////////////////////////////////////////////////////////////
  // PIR Motion Sensor mode INPUT_PULLUP
    pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
    attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  // Set LED to LOW
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);
  /////////////////////////////////////////////////////////////////////
}
void pollCam() {
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
/*    display.setFont(ArialMT_Plain_24);
    IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
    display.drawString(0, 26, String(WiFi.softAPIP()));
    display.display();
    delay(10);*/
    }

    /////////////////////////////////////////////////////////////////////
  // Current time
  now = millis();
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    //Serial.println("Motion stopped...");
    digitalWrite(led, LOW);
    startTimer = false;
  }
  /*if(startTimer) {
    serverStream();
  }*/
  /////////////////////////////////////////////////////////////////////
  }
