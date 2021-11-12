#include <Wire.h>
#include <Sparkfun_DRV2605L.h>

SFE_HMD_DRV2605L HMD;
const int neckSensor_PIN = 8;

void setup() {
  // put your setup code here, to run once:
  HMD.begin();
  HMD.Mode(0); // Internal trigger input mode -- Must use the GO() function to trigger playback.
  HMD.MotorSelect(0x36); // ERM motor, 4x Braking, Medium loop gain, 1.365x back EMF gain
  HMD.Library(7); //1-5 & 7 for ERM motors, 6 for LRA motors 
  Serial.begin(9600);

  pinMode(neckSensor_PIN, INPUT);  
}

void loop() {
  // put your main code here, to run repeatedly:
  int neckTouchCount;
  int touchSensorReading = digitalRead(neckSensor_PIN);
  Serial.print(touchSensorReading);
  if (touchSensorReading == HIGH) {
    neckTouchCount += 1;
  }

  do {
    if (touchSensorReading == HIGH) {
      HMD.Waveform(1, 134);
      HMD.go();
      delay(600);
    }
  } while (neckTouchCount == 1);
}
