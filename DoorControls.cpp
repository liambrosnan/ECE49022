#include "DoorControls.h"

/*----------------------------- Globals ----------------------------*/

CC1101 radio;

unsigned long lastSend = 0;
unsigned int sendDelay = 10000;

unsigned long lastStatusDump = 0;
unsigned int statusDelay = 5000;

String rec_payload;

float RT, VR, ln, TXX, VRT;

int   counter = 0;
char  output[64] = {0};
char * return_data;


void setupDoorControl(){
  // enable radio module. Leaving in serial startup code for later use maybe

    // Start Serial
    //delay (100);
    //Serial.begin(115200);
    //delay (100);
    Serial.println("Starting...");

    radio.enableSerialDebug(); 

    // SAM add relay setup, don't setup cc1101 if unnecessary

    // Start RADIO
    while ( !radio.begin(CFREQ_315, KBPS_250, /* channel num */ 16, /* address */ THIS_DEVICE_ADDRESS, GDO0_INTERRUPT_PIN /* Interrupt */) ) // channel 16! Whitening enabled
    {
        yield();
    }

    radio.printCConfigCheck();
    Serial.println(F("CC1101 radio initialized."));
    delay(1000);


    rec_payload.reserve(100);
}

void changeDoorState(int fixedCode){
  //called on user request to open/close door

  if(fixedCode < 256){ //fixed code present, use wireless
    memset(output, 0x00, sizeof(output));
    radio.sendChars("----------------------------------", DESINATION_DEVICE_ADDRESS);
  }
  else{ //no fixed code, use relay
    //SAM add relay code
  }
}

void pairDevice(){
     /*WIP, only example code in here rn*/
    /*unsigned long now = millis();

    if ( radio.dataAvailable() )
    {


        // Data format from CC1110:
        //  V|33|D|000907|000393|000138|002047|002047|002047|001421|0
        //  V|33|D|000902|000393|000138|002047|002047|002047|001423|0

       rec_payload  = String(radio.getChars()); // pointer to memory location of start of string
       //Serial.print("Payload size recieved: "); Serial.println(radio.getSize());
       Serial.println(rec_payload);

       byte *payload = radio.getBytes();

       float battery           =  rec_payload.substring(2,4).toInt();
       Serial.print("battery: " ); Serial.println(battery);

       int movement           =  rec_payload.substring(8,14).toInt();
       //Serial.print("movement: " ); Serial.println(movement);

       int thermopile_surface_ir = rec_payload.substring(15,21).toInt();
       //Serial.print("thermopile_surface_ir: "); Serial.println(thermopile_surface_ir);

       int thermistor_ambient_temp = rec_payload.substring(22).toInt();
       //Serial.print("thermistor_ambient_temp: "); Serial.println(thermistor_ambient_temp);

       VRT  = thermistor_ambient_temp;     // Acquisition analog value of VRT
       VRT  = (3.30 / 1023.00) * VRT;      // Conversion to voltage
       VR   = VCC - VRT;
       RT   = VRT / (VR / R);              // Resistance of RT

       ln = log(RT / RT0);
       TXX = (1 / ((ln / B) + (1 / T0o))); //Temperature from thermistor

       TXX = TXX - 273.15;                 //Conversion to Celsius

       Serial.print("Temperature:");
       Serial.print("\t");
       Serial.print(TXX);
       Serial.print("C\t\t");
       Serial.print(TXX + 273.15);        //Conversion to Kelvin
       Serial.print("K\t\t");
       Serial.print((TXX * 1.8) + 32);    //Conversion to Fahrenheit
       Serial.println("F");

       // Infrared camer / Thermopile attempt
       float ir_temp = thermopile_surface_ir*(battery/33); // normalise to 3.3 volts due to the ir value going up as battery deplets
       ir_temp /= (float)4.95; // guestimate

       Serial.print("IR temperature:");
       Serial.print("\t");
       Serial.println(ir_temp);
   }*/

}
