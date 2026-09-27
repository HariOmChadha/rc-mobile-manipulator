#include <Arduino.h>
#include "remote_control.h"

RemoteControl remoteControl;
char commandBuffer[96];
size_t commandLength = 0;
bool commandOverflow = false;

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
  // Bounded serial handling: malformed/oversized input cannot stall the watchdog.
  int budget = 128;
  while (Serial.available() && budget-- > 0) {
    char c = Serial.read();
    if (c == '\n') {
      if (!commandOverflow) {
        commandBuffer[commandLength] = 0;
        char reply[80];
        remoteControl.command(commandBuffer, millis(), reply, sizeof(reply));
        Serial.println(reply);
      }
      commandLength = 0; commandOverflow = false;
    } else if (c != '\r') {
      if (commandLength + 1 < sizeof(commandBuffer)) commandBuffer[commandLength++] = c;
      else commandOverflow = true;
    }
  }
  remoteControl.tick(millis());
  // 1. Read the human input (0 to 4095)
  int humanSteer = analogRead(steeringADC);
  int humanThrot = analogRead(throttleADC);

  // 2. Convert 12-bit ADC to 8-bit DAC scale (divide by 16)
  // 4095 / 16 = 255 maximum
  int dacSteer = humanSteer / 16;
  int dacThrot = humanThrot / 16;

  if (dacThrot < 50){
    dacThrot = 50;
  }
  if (dacThrot > 145){
    dacThrot = 145;
  }


  if (remoteControl.mode != RemoteControl::MANUAL) {
    dacSteer = remoteControl.steer;
    dacThrot = remoteControl.throttle;
  }

  // 3. Write the selected manual or host-commanded outputs.
  dacWrite(steeringDAC, dacSteer);
  dacWrite(throttleDAC, dacThrot);

  // 4. Print for debugging
  // Format: STEER,THROTTLE
  Serial.print(dacSteer);
  Serial.print(",");
  Serial.println(dacThrot);

  delay(20);
}
