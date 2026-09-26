#include <Arduino.h>

// Inputs from human
const int steeringADC = 34; 
const int throttleADC = 35; 

// Outputs to Remote PCB
const int steeringDAC = 25; 
const int throttleDAC = 26; 

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Reads 0 to 4095
}

void loop() {
  // 1. Read the human input (0 to 4095)
  int humanSteer = analogRead(steeringADC);
  int humanThrot = analogRead(throttleADC);

  // 2. Convert 12-bit ADC to 8-bit DAC scale (divide by 16)
  // 4095 / 16 = 255 maximum
  int dacSteer = humanSteer / 16;
  int dacThrot = humanThrot / 16;

  // 3. Inject the voltage into the remote's brain!
  dacWrite(steeringDAC, dacSteer);
  dacWrite(throttleDAC, dacThrot);

  // 4. Print for debugging
  // Format: STEER,THROTTLE
  Serial.print(dacSteer);
  Serial.print(",");
  Serial.println(dacThrot);

  delay(20);
}