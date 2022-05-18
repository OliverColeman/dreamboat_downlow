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

// Set up the quadrature encoders for steering position.
// QuadEncoder( channel [1:4], A pin, B pin )
QuadEncoder encoders[4] = {
  QuadEncoder(1, 1, 2),
  QuadEncoder(2, 3, 4),
  QuadEncoder(3, 5, 7),
  QuadEncoder(4, 30, 31)
};

USBSabertoothSerial driveMCSerial0(DRIVE_MOTOR_SERIAL0);
USBSabertoothSerial driveMCSerial1(DRIVE_MOTOR_SERIAL1);
USBSabertooth driveMC[2] = {
  USBSabertooth(driveMCSerial0, 128),
  USBSabertooth(driveMCSerial1, 128)
};

// Set up wheels.
// Wheel(wheelNumber, driveMotorController, driveMotorControllerChannel, steeringMotorDirectionPin, steeringMotorPwmPin, steeringMotorFaultPin, homeSwitchPin, encoder )
Wheel wheels[4] = {
  Wheel(0, &driveMC[0], 1, 8, 18, 33, 9, &encoders[0]),
  Wheel(1, &driveMC[0], 2, 6, 19, 34, 10, &encoders[1]),
  Wheel(2, &driveMC[1], 1, 20, 22, 35, 11, &encoders[2]),
  Wheel(3, &driveMC[1], 2, 21, 23, 36, 12, &encoders[3])
};

// Current sensing inputs (one per motor controller)
const int currentPins[8] = { 
  24, 25, 26, 27, // DC0-3  Drive motor current sensors
  38, 39, 40, 41  // SC0-3  Steering motor current sensors
};
// Baseline raw value readings for the current sensor ADCs, for calibration.
int motorCurrentSensorBaseline[8];
// The current currently being drawn by each motor.
double motorCurrentDraw[8];
// Make an ADC to read the current sensor values.
ADC *adc = new ADC();


void setup() {
  for (int mci = 0; mci < 2; mci++) {
    driveMC[mci].setGetTimeout(50);
    driveMC[mci].setGetRetryInterval(20);
    driveMC[mci].setRamping(-16383);
  }

  Serial.begin(9600);
  DRIVE_MOTOR_SERIAL0.begin(9600);
  DRIVE_MOTOR_SERIAL1.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(currentPins[i], INPUT);
  }
  adc->adc0->setAveraging(32);
  adc->adc0->setResolution(CURRENT_SENSOR_ADC_RESOLUTION);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  adc->adc1->setAveraging(32);
  adc->adc1->setResolution(CURRENT_SENSOR_ADC_RESOLUTION);
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  for (int i = 0; i < 8; i++) {
    motorCurrentSensorBaseline[i] = adc->analogRead(currentPins[i]);
  }
}


uint32_t lastWheelUpdateTime = 0;
uint32_t lastStatusLedUpdateTime = 0;
uint32_t lastDebugOutputTime = 0;
uint32_t lastSetCommandReceivedTime = micros();
uint32_t now;
byte readBuffer[2];
Buffer sendBuffer = Buffer(32);
boolean ledState = false;

void loop() {
  #ifndef DEBUG

  // If a command has been received over USB.
  if (Serial.available() > 0) {
    byte command = Serial.read();

    // If a Set command has been received.
    if (command == 'S') {
      lastSetCommandReceivedTime = micros();

      for (int wi = 0; wi < 4; wi++) {
        // 2 bytes for angle of wheel, in range [0-65535], for [0, 360] degrees.
        Serial.readBytes((char*) readBuffer, 2);
        uint16_t shortVal = readBuffer[0] << 8 | readBuffer[1];
        double unitAngle = shortVal / 65535.0;
        wheels[wi].setWheelPositionTarget(normaliseValueToRange(-180, unitAngle * 360, 180));

        // 1 byte for drive rate, 0 = full reverse, 127 = stop, 254 = full forward.
        uint8_t driveRate = Serial.read();
        wheels[wi].setDriveRate((driveRate - 127.0) / 127.0);
      }
    }
    // If a Get command has been received.
    else if (command == 'G') {
      // 1 byte for wheel fault (of steering motor driver) and ready status flags, bit format [ w3f w2f w1f w0f w3r w2r w1r w0r ]
      byte flags = 0;
      // For each wheel.
      for (int wi = 0; wi < 4; wi++) {
        flags |= wheels[wi].getSteeringDriverFault() << (wi+4);
        flags |= wheels[wi].isReady() << wi;

        // 2 bytes to represent current angle of wheel, in range [0-65535].
        double normalisedTo360 = normaliseValueToRange(0, wheels[wi].getPosition(), 360);
        uint16_t shortVal = round(normalisedTo360 / 360.0 * 65535);
        sendBuffer.write((shortVal >> 8) & 0xff);
        sendBuffer.write((shortVal >> 0) & 0xff);

        // 1 byte to represent rate the drive motor is being driven at. 0=full reverse, 127=stopped, 254=full forward.
        sendBuffer.write(round(wheels[wi].getDriveRate() * 127 + 127) & 0xff);
        
        // 1 byte to represent rate the steering motor is being driven at. 0=full reverse, 127=stopped, 254=full forward.
        sendBuffer.write(round(wheels[wi].getSteeringMotorDriveRate() * 127 + 127) & 0xff);

        // 1 byte to represent time wheel has been stuck, in tenths of a second. 
        sendBuffer.write(round((wheels[wi].getStuckTime() * 10)) & 0xff);

        // // 1 byte to represent temperature of the drive motor controller channel for the wheel, in degrees C. 
        // sendBuffer.write(round((wheels[wi].getDriveControllerTemperature())) & 0xff);

        // 1 byte to represent the current being drawn by the steering motor, in halves of an amp. 0 = -63.5A, 127=0A, 254=63.5A
        sendBuffer.write(round(motorCurrentDraw[wi+4] * 2 + 127) & 0xff);

        // 1 byte to represent the current being drawn by the drive motor, in halves of an amp. 0 = -63.5A, 127=0A, 254=63.5A
        sendBuffer.write(round(motorCurrentDraw[wi] * 2 + 127) & 0xff);
      }
      sendBuffer.write(flags);

      // Get battery voltage from first drive motor controller, in tenths of a volt.
      int batteryVoltage = driveMC[0].getBattery(1);
      // 2 bytes to represent battery voltage in tenths of a volt.
      sendBuffer.write((batteryVoltage >> 8) & 0xff);
      sendBuffer.write((batteryVoltage >> 0) & 0xff);

      sendBuffer.send();
    }
  }
  #endif

  bool setCommandReceivedTimeout = (uint32_t)(micros() - lastSetCommandReceivedTime) > SET_COMMAND_RECEIVED_TIMEOUT;
  // If set command has not been received for too long then stop.
  if (setCommandReceivedTimeout) {
    for (int wi = 0; wi < 4; wi++) {
      wheels[wi].setDriveRate(0);
    }
  }

  // Update wheel and motor current draw info every WHEEL_UPDATE_PERIOD us.
  now = micros();
  if ((uint32_t)(now - lastWheelUpdateTime) > WHEEL_UPDATE_PERIOD) {
    lastWheelUpdateTime = now;

    for (int csi = 0; csi < 8; csi++) {
      int rawCurrentSensorReading = adc->analogRead(currentPins[csi]);
      motorCurrentDraw[csi] = (rawCurrentSensorReading - motorCurrentSensorBaseline[csi]) * CURRENT_SENSOR_CONVERSION_FACTOR;
    }

    for (int wi = 0; wi < 4; wi++) {
      wheels[wi].update(motorCurrentDraw[wi], motorCurrentDraw[wi+4]);
    }

    #ifdef DEBUG
    now = micros();
    if ((uint32_t)(now - lastDebugOutputTime) > 1 * 1000000) {
      lastDebugOutputTime = now;
      for (int wi = 0; wi < 4; wi++) {
        Serial.print(wheels[wi].getSteeringDriverFault() ? "f" : "");
        Serial.print(wheels[wi].isReady() ? "r" : "h");
        Serial.print(" P:");
        Serial.print(wheels[wi].getPosition());
        Serial.print(" D:");
        Serial.print(round(wheels[wi].getDriveRate() * 127 + 127) & 0xff);
        Serial.print(" S:");
        Serial.print(wheels[wi].getStuckTime());
        Serial.print("    ");
      }
      Serial.println();
    }
    
    #endif
  }

  now = micros();
  uint32_t ledBlinkPeriod = setCommandReceivedTimeout ? STATUS_ERROR_BLINK_PERIOD : STATUS_OKAY_BLINK_PERIOD;
  if ((uint32_t)(now - lastStatusLedUpdateTime) > ledBlinkPeriod) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastStatusLedUpdateTime = now;
  }
}
