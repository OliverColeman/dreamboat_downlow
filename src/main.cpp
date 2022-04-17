/*
Steering controller code for the Dreamboat.
Designed to run on Teensy 4.1.
Controls steering for 4 wheels.
Receives commands that specify the desired position of the wheels.
Sends telemetry for the current position of the wheels.
*/

#include "../lib/ADC/ADC.h"

#include "wheel.h"

// Set up the quadrature encoders.
// QuadEncoder( channel [1:4], A pin, B pin )
QuadEncoder encoders[4] = {
  QuadEncoder(1, 1, 2),
  QuadEncoder(2, 3, 4),
  QuadEncoder(3, 5, 7),
  QuadEncoder(4, 30, 31)
};
// Set up wheels.
// Wheel( wheelNumber, homeSwitchPin, motorDirectionPin, motorPwmPin, encoder )
Wheel wheels[4] = {
  Wheel(0, 9, 38, 18, &encoders[0]),
  Wheel(1, 10, 39, 19, &encoders[1]),
  Wheel(2, 11, 40, 22, &encoders[2]),
  Wheel(3, 12, 41, 23, &encoders[3])
};

// Current sensing inputs (one per motor controller)
const int currentPins[] = { 20, 21 };
// The current currently being drawn by each motor controller.
const double current[2] = { 0, 0 };
// Precalculated value to convert sensor reading to amps.
double currentSenseConversionFactor;

ADC *adc = new ADC(); // Make an ADC to read the current sensor values.

void setup() {
  Serial.begin(9600);

  pinMode(currentPins[0], INPUT);
  pinMode(currentPins[1], INPUT);
  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(10);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  currentSenseConversionFactor = 3.3 / adc->adc0->getMaxValue() * CURRENT_SENSOR_VOLTS_PER_AMP;
}

uint32_t lastWheelUpdateTime = 0;
uint32_t now;
byte readBuffer[2];
Buffer sendBuffer = Buffer(32);

void loop() {
  if (Serial.available() > 0) {
    byte command = Serial.read();

    // If a Set command has been received.
    if (command == 'S') {
      for (int wi = 0; wi < 4; wi++) {
        Serial.readBytes((char*) readBuffer, 2);
        uint16_t shortVal = readBuffer[0] << 8 | readBuffer[1];
        double unitAngle = shortVal / 65535.0;
        wheels[wi].setWheelPositionTarget(unitAngle * 360 - 180);
      }
    }
    // If a Get command has been received.
    else if (command == 'G') {
      byte flags = 0; // ready status flags.
      // For each wheel.
      for (int wi = 0; wi < 4; wi++) {
        flags |= wheels[wi].isReady() << wi;
        // 2 bytes to represent current angle of wheel, in range [0-65535].
        double normalisedTo360 = normaliseValueToRange(0, wheels[wi].getPosition(), 360);
        uint16_t shortVal = round(normalisedTo360 / 360.0 * 65535);
        sendBuffer.write((shortVal >> 8) & 0xff);
        sendBuffer.write((shortVal >> 0) & 0xff);
        // 1 byte to represent time wheel has been stuck, in tenths of a second. 
        sendBuffer.write(round((wheels[wi].getStuckTime() * 10)) & 0xff);
      }
      sendBuffer.write(flags);

      // For each motor controller.
      for (int mci = 0; mci < 2; mci++) {
        int currentSensorReading = adc->adc0->analogRead(currentPins[mci]);
        double amps = currentSensorReading * currentSenseConversionFactor;
        // 1 byte for current being consumed by the motor controller, in halves of an amp.
        byte halfAmps = round(amps * 2) & 0xff;
        sendBuffer.write(halfAmps);
      }
      sendBuffer.send();
      Serial.println();
    }
  }

  now = micros();
  if ((uint32_t)(now - lastWheelUpdateTime) > WHEEL_UPDATE_PERIOD) {
    lastWheelUpdateTime = now;

    for (int wi = 0; wi < 4; wi++) {
      wheels[wi].update();
    }

    #ifdef DEBUG
    for (int wi = 0; wi < 4; wi++) {
      Serial.print(wheels[wi].isReady() ? "r P:" : "n P:");
      Serial.print(wheels[wi].getPosition());
      Serial.print(" S:");
      Serial.print(wheels[wi].getStuckTime());
      Serial.print("    ");
    }

    for (int mci = 0; mci < 2; mci++) {
      int currentSensorReading = adc->adc0->analogRead(currentPins[mci]);
      double amps = currentSensorReading * currentSenseConversionFactor;
      Serial.print("MC ");
      Serial.print(mci);
      Serial.print(amps);
      Serial.print("   ");
    }

    Serial.println();
    #endif
  }
}
