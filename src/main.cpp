/**
 * Steering controller for the Dreamboat: https://github.com/OliverColeman/dreamboat)
 * a motorised four-poster bed designed to move in surreal and dreamlike ways.
 * 
 * Connects to the Dreamboat controller (Raspberry Pi) via USB.
 * Receives commands that specify the desired angles of the wheels.
 * Implements servo control of the four wheels.
 * Sends telemetry including the current position of the wheels and the motor controller current draw.
 * Designed to run on a Teensy 4.1: https://www.pjrc.com/store/teensy41.html.
 * PlatformIO is used for development.
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
  Wheel(0, 9, 16, 18, &encoders[0]),
  Wheel(1, 10, 17, 19, &encoders[1]),
  Wheel(2, 11, 20, 22, &encoders[2]),
  Wheel(3, 12, 21, 23, &encoders[3])
};

// Current sensing inputs (one per motor controller)
const int currentPins[] = { 
  24, 25, 26, 27, // DC0-3  Drive motor current sensors
  38, 39, 40, 41  // SC0-3  Steering motor current sensors
};
// The current currently being drawn by each motor controller.
double motorCurrentDraw[8];
// Precalculated value to convert sensor reading to amps.
double currentSenseConversionFactor;
// Make an ADC to read the current sensor values.
ADC *adc = new ADC();


void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(currentPins[i], INPUT);
  }
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
boolean aliveFlag = false;

void loop() {
  // If a command has been received over USB.
  if (Serial.available() > 0) {
    byte command = Serial.read();

    // If a Set command has been received.
    if (command == 'S') {
      for (int wi = 0; wi < 4; wi++) {
        Serial.readBytes((char*) readBuffer, 2);
        uint16_t shortVal = readBuffer[0] << 8 | readBuffer[1];
        double unitAngle = shortVal / 65535.0;
        wheels[wi].setWheelPositionTarget(normaliseValueToRange(-180, unitAngle * 360, 180));
      }
    }
    // If a Get command has been received.
    else if (command == 'G') {
      // 1 byte for wheel ready status flags, bit format [ x x x x w3 w2 w1 w0 ]
      byte flags = 0;
      // For each wheel.
      for (int wi = 0; wi < 4; wi++) {
        flags |= wheels[wi].isReady() << wi;

        // 2 bytes to represent current angle of wheel, in range [0-65535].
        double normalisedTo360 = normaliseValueToRange(0, wheels[wi].getPosition(), 360);
        uint16_t shortVal = round(normalisedTo360 / 360.0 * 65535);
        sendBuffer.write((shortVal >> 8) & 0xff);
        sendBuffer.write((shortVal >> 0) & 0xff);

        // 1 byte to represent rate the steering motor is being driven at. 0=full reverse, 127=stopped, 254=full forward.
        sendBuffer.write(round(wheels[wi].getDriveRate() * 127 + 127) & 0xff);

        // 1 byte to represent time wheel has been stuck, in tenths of a second. 
        sendBuffer.write(round((wheels[wi].getStuckTime() * 10)) & 0xff);

        // 1 byte to represent the current being drawn by the steering motor, in halves of an amp. 0 = -63.5A, 127=0A, 254=63.5A
        sendBuffer.write(round(motorCurrentDraw[wi+4] * 2 + 127) & 0xff);

        // 1 byte to represent the current being drawn by the drive motor, in halves of an amp. 0 = -63.5A, 127=0A, 254=63.5A
        sendBuffer.write(round(motorCurrentDraw[wi] * 2 + 127) & 0xff);
      }
      sendBuffer.write(flags);

      sendBuffer.send();
    }
  }

  // Update wheel and motor current draw info every WHEEL_UPDATE_PERIOD us.
  now = micros();
  if ((uint32_t)(now - lastWheelUpdateTime) > WHEEL_UPDATE_PERIOD) {
    aliveFlag = !aliveFlag;
    digitalWrite(LED_BUILTIN, aliveFlag);

    lastWheelUpdateTime = now;

    for (int wi = 0; wi < 4; wi++) {
      wheels[wi].update();
    }

     for (int csi = 0; csi < 8; csi++) {
      // Subtract half the maximum analogRead value as the middle of the possible range of analogRead values is 0 amps.
      int currentSensorReading = adc->adc0->analogRead(currentPins[csi]) - adc->adc0->getMaxValue() * 0.5;
      motorCurrentDraw[csi] = currentSensorReading * currentSenseConversionFactor;
    }

    #ifdef DEBUG
    for (int wi = 0; wi < 4; wi++) {
      Serial.print(wheels[wi].isReady() ? "r P:" : "n P:");
      Serial.print(wheels[wi].getPosition());
      Serial.print(" D:");
      Serial.print(round(wheels[wi].getDriveRate() * 127 + 127) & 0xff);
      Serial.print(" S:");
      Serial.print(wheels[wi].getStuckTime());
      Serial.print("    ");
    }
    Serial.println();
    #endif
  }
}
