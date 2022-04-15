#include <Wire.h>
#include <Sparkfun_DRV2605L.h>
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>

// global variables and object instantiations
const int chestHapticMotor_EN_PIN = 23;
const int redPistonLED = A8;
const int greenPistonLED = A9;

const int neckHapticMotor_EN_PIN = 22;
const int rightNeckSensor_PIN = 52;
const int leftNeckSensor_PIN = 53;

const int headHapticMotor_EN_PIN = 24;
const int noseClosure_PIN = A1;
const int lungInf_PIN = A2;
const int headTilt_PIN = 31;

const int redLED_PIN = 6;
const int blueLED_PIN = 4;
const int greenLED_PIN = 5;

#define TCAADDR 0x70
SFE_HMD_DRV2605L HapticMotor;
DFRobotDFPlayerMini audioOutput;
SoftwareSerial mySoftwareSerial(10, 11); // Rx, Tx

// Function to use I2C multiplexer board
void tcaselect(uint8_t i) {
 if (i > 7) return;
 Wire.beginTransmission(TCAADDR);
 Wire.write(1 << i);
 Wire.endTransmission(); 
}

// Function to set Colors for LED
void setColor(int redVal, int greenVal, int blueVal) {
 analogWrite(redLED_PIN, redVal);
 analogWrite(greenLED_PIN, greenVal);
 analogWrite(blueLED_PIN, blueVal);
}

// Function to set LED colors to red, green, or off
void redLEDs() {
  setColor(255,0,0);
}

void greenLEDs() {
  setColor(0,255,0);
}

void turnOffLEDs() {
  setColor(0,0,0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Function that runs pulse check module
void pulseModuleCheck() {
  
  // initialize haptic motors to not enabled
  digitalWrite(neckHapticMotor_EN_PIN, LOW);
  
  // 1. run audio to introduce the pulse check module 
  audioOutput.play(1);
  delay(11500);
  
  // 2. once audio clip says to check pulse, run haptic motor to indicate location
  bool rightTouchSensorReading = digitalRead(rightNeckSensor_PIN);
  bool leftTouchSensorReading = digitalRead(leftNeckSensor_PIN);
  Serial.print(rightTouchSensorReading); Serial.println(leftTouchSensorReading);
  while (!rightTouchSensorReading /*&& !leftTouchSensorReading*/) {
    // Initialize and keep red LEDs on until user taps pulse point
    redLEDs();
    // enable haptic motors so they can start running
    digitalWrite(neckHapticMotor_EN_PIN, HIGH);
    // select and play specific waveform - will update in future iterations
    tcaselect(6);
    HapticMotor.go();
    delay(500);
    // read condition of sensor again for while loop
    rightTouchSensorReading = digitalRead(rightNeckSensor_PIN);
    leftTouchSensorReading = digitalRead(leftNeckSensor_PIN);
    Serial.print(rightTouchSensorReading); Serial.println(leftTouchSensorReading);
  }

  greenLEDs();         // Output satisfactory check with green LEDs
  delay(500);
  audioOutput.play(2); // tell users that device is moving to next module
  turnOffLEDs();
  delay(6500);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Function to check nose closure
void noseClosureCheck() { 
  // initialize haptic motors to not enabled
  digitalWrite(headHapticMotor_EN_PIN, LOW);
  // Pulse Check Module
  // 1. run audio to introduce the pulse check module 
  audioOutput.play(7);
  delay(6500);
  int noseSensorReading = analogRead(noseClosure_PIN);
  Serial.println(noseSensorReading);
  while (noseSensorReading > 50) {
    redLEDs();
    digitalWrite(headHapticMotor_EN_PIN, HIGH);
    Serial.println(noseSensorReading);
    tcaselect(7);
    HapticMotor.Waveform(1,145);
    HapticMotor.go();
    delay(500);
    noseSensorReading = analogRead(noseClosure_PIN); 
    Serial.println(noseSensorReading);
  }

  greenLEDs();
  delay(500);
  audioOutput.play(2); // tell users that device is moving to next module
  turnOffLEDs();
  delay(6500);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Function for lung inflation check
void lungInflationCheck() {
  digitalWrite(headHapticMotor_EN_PIN, LOW);

  // breathing technique
  audioOutput.play(8);
  delay(16000);
  int lungInflations = 0;
  float lungInfSensor = analogRead(lungInf_PIN);
  Serial.println(lungInfSensor);
  while (lungInflations < 2) {
    redLEDs();
    digitalWrite(headHapticMotor_EN_PIN, HIGH);
    tcaselect(7);
    HapticMotor.Waveform(1,145);
    HapticMotor.go();
    lungInfSensor = analogRead(lungInf_PIN);
    Serial.println(lungInfSensor);
    if (lungInfSensor <= 5.00) {
      lungInflations += 1;
      greenLEDs();
      delay(500);
    }
    else {
      redLEDs();
      delay(500);
    }
  }

  greenLEDs();
  delay(500);
  audioOutput.play(2);
  turnOffLEDs();
  delay(6500);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Function for head tilt check
void headTiltCheck() {
  // 1. introduce breathing technique and let users know to tilt head first
  audioOutput.play(5);
  delay(15000);
  
  // 2. activate haptic motor at head region to indicate user to tilt head
  digitalWrite(headHapticMotor_EN_PIN, HIGH); 
  bool headTiltReading = digitalRead(headTilt_PIN);
  // Serial.println(headTiltReading);   // for debugging purposes
  int timerCount = 0;
  
  while (headTiltReading) {   // active LOW when one pin of the tilt sensor is connected to Vcc
    redLEDs();
    headTiltReading = digitalRead(headTilt_PIN);
    Serial.println(headTiltReading);
    if (!headTiltReading) {
      break;
    }
    delay(1000);
    timerCount += 1;
    if (timerCount%6 == 0) {    // every three seconds of idle time, system lets user know to tilt head
      audioOutput.play(6);
    }
  }

  greenLEDs();
  delay(500);
  
  turnOffLEDs();
  audioOutput.play(2); // tell users that device is moving to next module
  delay(3000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Function for chest compressions
void chestCompCheck() {
  // Don't forget to include haptic motor -- see other functions to use it
  // Also use chestComp_PIN variable to get readings

  // 1. Introduce chest compression technique
  // Enable chestHapticMotor
  digitalWrite(chestHapticMotor_EN_PIN, HIGH);
  audioOutput.play(20);
  delay(25000);
  signed int greenReading = analogRead(greenPistonLED);
  signed int redReading = analogRead(redPistonLED);
  while (greenReading < 300 && redReading < 300) {
    Serial.print(greenReading);
    Serial.print("\t");
    Serial.print(redReading);
    redLEDs();
    tcaselect(3);
    HapticMotor.Waveform(1,145);
    HapticMotor.go();
    delay(500);
    greenReading = analogRead(greenPistonLED);
    if (greenReading >= 374) {
      break;
    }
  }
  greenLEDs();
  delay(1500);
  
  // 3. have user perform compressions for 60 s
  // Update on audio file: "Notice how my eyes light up...if they are green, then you've got the right compression and depth, but if they're red, then you'll have to speed up or slow down
  audioOutput.play(21); //let user know previous compression is successful, now must perform 30 successful compressions
  delay(12000);
  
  // FSM 
  audioOutput.play(11);
  // play the song for a few seconds to get user used to metronome
  delay(5000);
  unsigned long currentMillis = millis();
  unsigned long startOfSong = millis();
  unsigned long prevMillis = 0;
  // perform checks based on user performance after acclimating to metronome
  while (currentMillis - startOfSong < 60000) {
     if (millis() - prevMillis == 500) {
      signed int currentGreenReading = analogRead(greenPistonLED);
      signed int currentRedReading = analogRead(redPistonLED);
      if (currentGreenReading > 400) {
        greenLEDs();
      }
      
      if (currentRedReading > 375) {
        redLEDs();
      }
      
      if (currentGreenReading < 400 && currentRedReading > 375) {
        turnOffLEDs();
      }

      // update values
      signed int prevGreenReading = currentGreenReading;
      signed int prevRedReading = currentRedReading;
     }
     
     prevMillis = currentMillis;
     currentMillis = millis();
  }

  greenLEDs();
  delay(500);

  turnOffLEDs();
  audioOutput.play(2); // tell users that device is moving to next module
  delay(3000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Function for Traditional CPR flow
void traditionalCPR() {
  audioOutput.play(16); // track 'Welcome to Traditional CPR. You would need to use this after the patient has been
  delay(85000);         // unconscious for more time than you've seen them conscious...'

  // Modules applicable to traditional CPR
  pulseModuleCheck();
  chestCompCheck();
  headTiltCheck();
  noseClosureCheck();
  lungInflationCheck();

  audioOutput.play(17); // track 'you have now completed traditional CPR training. IRL, you will have to
  delay(13500);         // cycle b/w 30 chest compressions and 2 breaths'
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Function for Hands-Only CPR flow
void handsOnlyCPR() {
  audioOutput.play(18);  // track 'Welcome to Hands-only CPR training. You can only use this training...'
  delay(6500);

  // Modules applicable hands-only CPR
  pulseModuleCheck();
  chestCompCheck();

  audioOutput.play(19); // track 'you have now completed hands-only CPR training. IRL, you would continue this
  delay(18500);         // CPR method indefinitely until the victim regains consciousness or you switch to traditional CPR'
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  mySoftwareSerial.begin(9600);

  // Initializing haptic motors
  HapticMotor.begin();
  HapticMotor.Mode(0); // Internal trigger input mode -- Must use the GO() function to trigger playback.
  HapticMotor.MotorSelect(0x36); // ERM motor, 4x Braking, Medium loop gain, 1.365x back EMF gain
  HapticMotor.Library(7); //1-5 & 7 for ERM motors, 6 for LRA motors 

  // Initializing mp3 module
  audioOutput.begin(mySoftwareSerial);
  if (!audioOutput.begin(mySoftwareSerial)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
  }
  Serial.println(F("DFPlayer Mini online."));
  
  pinMode(rightNeckSensor_PIN, INPUT); 
  pinMode(leftNeckSensor_PIN, INPUT); 
  pinMode(redPistonLED, INPUT); 
  pinMode(greenPistonLED, INPUT); 
  pinMode(noseClosure_PIN, INPUT);
  pinMode(headTilt_PIN, INPUT);
  pinMode(lungInf_PIN, INPUT);
  
  pinMode(neckHapticMotor_EN_PIN, OUTPUT);
  pinMode(chestHapticMotor_EN_PIN, OUTPUT);
  pinMode(headHapticMotor_EN_PIN, OUTPUT);

  pinMode(redLED_PIN, OUTPUT);
  pinMode(blueLED_PIN, OUTPUT);
  pinMode(greenLED_PIN, OUTPUT);

  // Orient the tilt sensor to know which value it starts at
  digitalWrite(headTilt_PIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Initialize Haptic Motors to not enabled
  digitalWrite(neckHapticMotor_EN_PIN, LOW);
  digitalWrite(chestHapticMotor_EN_PIN, LOW);
  digitalWrite(headHapticMotor_EN_PIN, LOW);
  digitalWrite(headHapticMotor_EN_PIN, LOW);

  // test run for traditional CPR (circumventing menu select mode)
//  int init = analogRead(noseClosure_PIN);
//  if (init < 1) {traditionalCPR();};
  
  // Menu select
  int init = analogRead(noseClosure_PIN);
  Serial.println(init);
  int timerCount = 0;
  if (init < 1) {
    audioOutput.play(12);  // ## = track should say 'Welcome! Please select which training mode you would like to learn...'
    delay(13500);

    // Check to see if user presses pulse points
    bool modeSelectRight = digitalRead(rightNeckSensor_PIN);
    bool modeSelectLeft = digitalRead(leftNeckSensor_PIN);
    while (!modeSelectRight || !modeSelectLeft) {
      modeSelectRight = digitalRead(rightNeckSensor_PIN);
      modeSelectLeft = digitalRead(leftNeckSensor_PIN);
      
      if (modeSelectLeft) {
        audioOutput.play(14);  // ## = track 'Traditional CPR. Please confirm by pressing both of my pulse points'
        delay(2500);
        audioOutput.play(13); // ## = 'Please confirm by pressing both pulse points'
        delay(2500);
        bool modeSelectRight = digitalRead(rightNeckSensor_PIN);
        bool modeSelectLeft = digitalRead(leftNeckSensor_PIN);
        if (modeSelectLeft && modeSelectRight) {
          traditionalCPR();
        }
      }

      if (modeSelectRight) {
        audioOutput.play(15);  // ## = track 'Hands-only CPR.'
        delay(2500);
        audioOutput.play(13); // ## = 'Please confirm by pressing both pulse points'
        delay(2500);
        bool modeSelectRight = digitalRead(rightNeckSensor_PIN);
        bool modeSelectLeft = digitalRead(leftNeckSensor_PIN);
        if (modeSelectLeft && modeSelectRight) {
          handsOnlyCPR();
        }
      }

      delay(1000);
      timerCount++;
      if (timerCount == 60) { // After 60s, of no activity, return to powered off state
        break;
      }
    } 
  }
}
