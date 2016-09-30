/**************************************************************************/
/*!
    @file     xplane_dials.ino
    @author   @DavidHowlett (David Howlett, Peter Dobson)
    @license  BSD (see license.txt)
    This example reads an array of 4 sensors and prints the results to serial.
    This was originally developed to upload dial positions to a flight simulator
    called X-Plane using a modified version of Arduino called Teensyduino.
    The Teensyduino specific code has now been commented out.
    If you want to enable the live upload of values to xplane you need to:
        - Download and install Teensyduino
        - Set the board type to Teensy++ 2.0 (or which ever version of Teensy you are using)
        - Set the USB type to "Flight Sim Controls" in the tools menu (this adds some definitions)
        - Uncomment the marked lines

    @section  HISTORY
    v1.0.0 - First release
    v1.0.1 - Now works without Teensyduino, also added build instructions
*/
/**************************************************************************/

#include <ams_as5048b.h>

// the constant U_DEG means the results are given in degrees
#define U_DEG 3

// declare an array of sensors
AMS_AS5048B sensors[4] = {AMS_AS5048B(0x40), AMS_AS5048B(0x41), AMS_AS5048B(0x42), AMS_AS5048B(0x43)};

int led = 6;

int ADFdialR = 0; //Reverse direction dial rotation from the sensor
int ADFdial = 0; //define ADF parameters.
int ADFdiff = 0;
int ADFprev = 3; //ensures X-Plane takes the value at first start up.
int ADF = 0;
int ADFfine = 0;
int ADFcourse = 0;
int ADFobs = 0;

int VORdialR = 0; //Reverse direction dial rotation from the sensor
int VORdial = 0; //define VOR parameters.
int VORdiff = 0;
int VORprev = 3; //ensures X-Plane takes the value at first start up.
int VOR = 0;
int VORfine = 0;
int VORcourse = 0;
int VORobs = 0;

float Baro1DialR = 0; //Reverse direction dial rotation from the sensor
float Baro1Dial = 0; //define altimeter 1 parameters.
float Baro1Diff = 0;
float Baro1Prev = 3; //ensures X-Plane takes the value at first start up.
float Baro1Sub = 1013;
float Baro1Fine = 0;
float Baro1Course = 127;

float Baro2DialR = 0; //Reverse direction dial rotation from the sensor
float Baro2Dial = 0; //define altimeter 2 parameters.
float Baro2Diff = 0;
float Baro2Prev = 3; //ensures X-Plane takes the value at first start up.
float Baro2Sub = 1013;
float Baro2Fine = 0;
float Baro2Course = 127;

FlightSimInteger VORout;
FlightSimInteger ADFout;
FlightSimFloat Baro1Ins;
FlightSimFloat Baro2Ins;

void setup() {
  VORout = XPlaneRef("sim/cockpit/radios/nav2_obs_degm"); //X-Plane Data Ref for VOR2 OBS
  ADFout = XPlaneRef("sim/cockpit/radios/adf1_cardinal_dir"); //X-Plane Data Ref for ADF OBS
  Baro1Ins = XPlaneRef("sim/cockpit/misc/barometer_setting"); //Data Ref for X-plane Baro sub scale in ins of Hg
  Baro2Ins = XPlaneRef("sim/cockpit/misc/barometer_setting2"); //Data Ref for X-plane Baro sub scale in ins of Hg
  pinMode(led, OUTPUT);
  for (int i = 0; i < 4; i++) {
    sensors[i].begin();
  }
  //Wait for the flight simulator to be ready
  while (!FlightSim.isEnabled()){
    FlightSim.update();
  }
}

void loop() {
  //Read in angles from the 4 sensors
  ADFdialR = sensors[0].angleR(U_DEG, true);
  VORdialR = sensors[1].angleR(U_DEG, true);
  Baro1DialR = sensors[2].angleR(U_DEG, true);
  Baro2DialR = sensors[3].angleR(U_DEG, true);
  FlightSim.update();
  //ADF Calcs.
  ADFdial = 360 - ADFdialR; // change rotation from clockwise to anticlock.
  ADFdiff = ADFdial - ADFprev;  //look for changes in the value of ADFdial
  if (ADFdiff > 180){ //decriment ADF if shaft passes North clockwise.
    ADF--;
    if (ADF < 0){
      ADF = 5;
    }
  }
  if (ADFdiff < -180){ //increment ADF if shaft passes North anticlock.
    ADF++;
    if (ADF > 5){  //limit ADFobs to 0 to 360
      ADF = 0;
    }
  }
  //calculate ADFobs using 6 turns of ADFdial per 360 degrees of ADFobs.
  ADFfine = ADFdial / 6;
  ADFcourse = ADF * 60;
  ADFobs = ADFcourse + ADFfine;
  //Output to X-Plane only if data changes.
  if (ADFdiff >= 2 || ADFdiff <= -2){  //output if ADFdial is rotated
    ADFout = ADFobs;
    ADFprev = ADFdial;  // ready for next change in value
  }
  //end of ADF
  //VOR Calcs
  VORdial = 360 - VORdialR;
  VORdiff = VORdial - VORprev;  //look for changes in the value of VORdial.
  if (VORdiff > 180){ //decriment VOR if shaft passes North clockwise.
    VOR--;
    if (VOR < 0){  //limit VORobs to 0 to 360
      VOR = 5;
    }
  }
  if (VORdiff < -180){ //increment VOR if shaft passes North anticlock.
    VOR++;
    if (VOR > 5){ //limit VOR to 0 to 360
      VOR = 0;
    }
  }
  //calculate VORobs using 6 turns of VORdial per 360 degrees of VORobs.
  VORfine = VORdial / 6;
  VORcourse = VOR * 60;
  VORobs = VORcourse + VORfine;
  //Output to X-Plane only if data changes.
  if (VORdiff >= 2 || VORdiff <= -2){
    VORout = VORobs;
    VORprev = VORdial;  //ready for next change in value
  }
  //End of VOR Calcs.
  //Baro1 Calcs.
  Baro1Dial = 360 - Baro1DialR; // change rotation from clockwise to anticlock.
  Baro1Diff = Baro1Dial - Baro1Prev;
  Baro1Fine = Baro1Dial / 45; // each revolution of the Barodial chnages the altimeter by 8mb.
  if (Baro1Diff >= 180){
    Baro1Course = Baro1Course - 1;
  }
  if (Baro1Diff <= -180){
    Baro1Course = Baro1Course + 1;
  }
  Baro1Course = constrain(Baro1Course, 117, 132);
  Baro1Sub = (Baro1Course * 8) + ( Baro1Fine);
  Baro1Sub = constrain(Baro1Sub, 945, 1050); //altimeter sub scale limits
  //Output to X-Plane only if data changes.
  if (Baro1Diff >= 2 || Baro1Diff <= -2){
    Baro1Ins = Baro1Sub * 0.02966436; //convert mb to ins of Hg
    Baro1Prev = Baro1Dial;
  }
  //End of Baro1 Calcs.
  //Baro2 Calcs.
  Baro2Dial = 360 - Baro2DialR; // change rotation from clockwise to anticlock.
  Baro2Diff = Baro2Dial - Baro2Prev;
  Baro2Fine = Baro2Dial / 45; // each revolution of the Barodial chnages the altimeter by 8mb.
  if (Baro2Diff >= 180){
    Baro2Course = Baro2Course - 1;
  }
  if (Baro2Diff <= -180){
    Baro2Course = Baro2Course + 1;
  }
  Baro2Course = constrain(Baro2Course, 117, 132);
  Baro2Sub = (Baro2Course * 8) + ( Baro2Fine);
  Baro2Sub = constrain(Baro2Sub, 945, 1050); //altimeter sub scale limits
  //Output to X-Plane only if data changes.
  if (Baro2Diff >= 2 || Baro2Diff <= -2){
    Baro2Ins = Baro2Sub * 0.02966436; //convert mb to ins of Hg;
    Baro2Prev = Baro2Dial;
  }
  //End of Baro2 Calcs.
  digitalWrite (led, ! digitalRead(led)); // toggle the internal LED to check program execution speed
  delay(10);
}
