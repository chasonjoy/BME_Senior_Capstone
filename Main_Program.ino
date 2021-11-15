#include <Wire.h>
#include <Sparkfun_DRV2605L.h>
#include <DFRobotDFPlayerMini.h>
#include <HX711.h>

SFE_HMD_DRV2605L HMDNeck;
SFE_HMD_DRV2605L HMDChest;
SFE_HMD_DRV2605L HMDMouth;
SFE_HMD_DRV2605L HMDHead;
DFRobotDFPlayerMini audioOutput;
HX711 scale;

const float calibration_factor = -7050.0;
const int neckSensor_PIN = 8;
const int chestComp_PIN = A0;
const int noseClosure_PIN = A1;
const int headTilt_PIN = 9;
const int lungInf_PIN = A2;

void setup() {
  Serial.begin(9600);

  // Initializing haptic motors
  HMDNeck.begin();HMDChest.begin();HMDMouth.begin();HMDHead.begin();
  HMDNeck.Mode(0);HMDChest.Mode(0);HMDMouth.Mode(0);HMDHead.Mode(0); // Internal trigger input mode -- Must use the GO() function to trigger playback.
  HMDNeck.MotorSelect(0x36);HMDChest.MotorSelect(0x36);HMDMouth.MotorSelect(0x36);HMDHead.MotorSelect(0x36); // ERM motor, 4x Braking, Medium loop gain, 1.365x back EMF gain
  HMDNeck.Library(7);HMDChest.Library(7);HMDMouth.Library(7);HMDHead.Library(7); //1-5 & 7 for ERM motors, 6 for LRA motors 

  // Initializing mp3 module
  audioOutput.begin(Serial);
  if (!audioOutput.begin(Serial)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
  }
  Serial.println(F("DFPlayer Mini online."));

  // Initializing load cell for chest compression
  scale.set_scale(calibration_factor);
  scale.tare();
  
  pinMode(neckSensor_PIN, INPUT); 
  pinMode(chestComp_PIN, INPUT); 
  pinMode(noseClosure_PIN, INPUT);
  pinMode(headTilt_PIN, INPUT);
  pinMode(lungInf_PIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Pulse Check Module
  // 1. run audio to introduce the pulse check module 
  audioOutput.play(1);
  delay(10000);
  // 2. once audio clip says to check pulse, run haptic motor to indicate location
  int touchSensorReading = digitalRead(neckSensor_PIN);
  while (touchSensorReading == LOW) {
    HMDNeck.Waveform(1,145);
    HMDNeck.go();
    delay(500);  
  }
  // 3. once touchSensorReading detects high (user activated), stop haptic motor
    //stop haptic motor
    audioOutput.play(2); // tell users that device is moving to next module

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Chest Compression Module
  // 1. introduce chest compression technique
  audioOutput.play(3);
  delay(10000);
  // 2. use haptic motors to indicate location to compress
  float chestCompSensor = scale.get_units();
  while (chestCompSensor == 0) {
    HMDChest.Waveform(1,145);
    HMDChest.go();
    delay(500);
  }

  while (chestCompSensor < 125) { // if chest compression is detected but not strong enough, then have user retry
    audioOutput.play(4); // indicate that user compression is not enough and try again
  }

  // 3. have user perform 15 successful compressions
  audioOutput.play(5); //let user know previous compression is successful, now must perform 15 successful compressions
  int chestCompressions = 0;
  while (chestCompressions < 15) {
    if (chestCompSensor >= 125) {
      chestCompressions += 1;
      audioOutput.play(6); // indicate good compression;
      delay(5000);
    }
    else {
      audioOutput.play(7); // indicate unsuccessful compression;
      delay(5000);
    }
  }

  audioOutput.play(2); // tell users that device is moving to next module

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Head Tilt, Nose Closure, and Breathing Check
  // 1. introduce breathing technique and let users know to tilt head first
  audioOutput.play(8);
  delay(15000);
  
  // 2. activate haptic motor at head region to indicate user to tilt head
  int headTiltReading = digitalRead(headTilt_PIN);
  int timerCount = 0;
  
  while (headTiltReading == LOW) {
    delay(1000);
    timerCount += 1;
    if (timerCount%3 == 0) {    // every three seconds of idle time, system lets user know to tilt head
      audioOutput.play(9);
    }
  }
  
  // 3. next nose closure
  audioOutput.play(10);
  delay(10000);
  
  float noseSensorReading = analogRead(noseClosure_PIN);
  while (noseSensorReading < 23) {
    HMDMouth.Waveform(1,145);
    HMDMouth.go();
    delay(500); 
  }
  
  // 4. now breathing technique 
  audioOutput.play(10);
  delay(10000);
  int lungInflations = 0;
  float lungInfSensor = analogRead(lungInf_PIN);
  while (lungInflations < 2) {
    if (lungInfSensor >= 1024) {
      lungInflations += 1;
      audioOutput.play(11); // indicate good breath;
      delay(5000);
    }
    else {
      audioOutput.play(12); // indicate lacking breath;
      delay(5000);
    }
  }

  audioOutput(13); // let users know that after two breaths, will still need to continue with compressions if patient doesn't regain consciousness
  delay(5000);
  
  audioOutput(2); // tell users that device is moving to next module

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // tell users they have completed the complete hands on training
  audioOutput.play(14);
}
