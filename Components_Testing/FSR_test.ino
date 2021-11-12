const int FSR_PIN = A0; // Pin connected to FSR/resistor divider
int fsrReading;         // the analog reading from the FSR resistor divider
int fsrVoltage;         // the analog reading converted to voltage
const int indicatorLED_PIN = 8;

void setup() 
{
  Serial.begin(9600);
  pinMode(FSR_PIN, INPUT);
  pinMode(indicatorLED_PIN, OUTPUT);
}

void loop() 
{
  int fsrADC = analogRead(FSR_PIN);
  double fsrVoltage;
  
//  if (fsrADC != 0) {
    Serial.println(fsrADC);
    fsrVoltage = map(fsrADC, 0, 1023, 0, 5000);
    Serial.println("Voltage of FSR: " + String(fsrVoltage/1000) + " V");

    if (fsrADC >= 400) {
      digitalWrite(indicatorLED_PIN, HIGH);
    }
    else {
      digitalWrite(indicatorLED_PIN, LOW);
    }

    delay(250);
//  }
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
//  if (fsrADC != 0) // If the analog reading is non-zero
//  {
//    // Use ADC reading to calculate voltage:
//    float fsrV = fsrADC * VCC / 1023.0;
//    // Use voltage and static resistor value to 
//    // calculate FSR resistance:
//    float fsrR = R_DIV * (VCC / fsrV - 1.0);
//    Serial.println("Resistance: " + String(fsrR) + " ohms");
//    // Guesstimate force based on slopes in figure 3 of
//    // FSR datasheet:
//    float force;
//    float fsrG = 1.0 / fsrR; // Calculate conductance
//    // Break parabolic curve down into two linear slopes:
//    if (fsrR <= 600) 
//      force = (fsrG - 0.00075) / 0.00000032639;
//    else
//      force =  fsrG / 0.000000642857;
//    Serial.println("Force: " + String(force) + " g");
//    Serial.println();
//
//    delay(500);
//  }
//  else
//  {
//    // No pressure detected
//  }
}
