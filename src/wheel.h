#include "../lib/QuadEncoder/QuadEncoder.h"
#include "../lib/USBSabertooth/USBSabertooth.h"

#include "./constants.h"
#include "./util.h"


/** 
 * Determine new drive ratet for a motor while ensuring 
 * - change in motor drive rate for a time step is no more than MOTOR_RAMPING_DELTA, and
 * - the drive rate is capped to maintain current draw less than the maximum allowed. 
 */
double getRampedRateForMotorDrive(double currentRate, double targetRate, double motorCurrent, double maxCurrent) {
  // Determine change in rate while allowing for ramping.
  double rateDelta = capRange(-MOTOR_RAMPING_DELTA, targetRate - currentRate, MOTOR_RAMPING_DELTA);
  double rate = currentRate + rateDelta;
  // Determine maximum drive rate while allowing for current limiting.
  if (abs(motorCurrent) > maxCurrent) { 
    double maxRate = currentRate * (maxCurrent / abs(motorCurrent));
    rate = capRange(-maxRate, rate, maxRate);
  }
  return rate * MOTOR_THROTTLING_FACTOR;
}


/**
 * Encapsulates steering telemetry and control functions for a wheel.
 */
class Wheel {
  private:
    uint8_t wheelNumber;
    /** Pointer to the drive motor controller. */
    USBSabertooth *driveMotorController;
    /** Channel on the drive motor controller this wheels drive motor is attached to. */
    uint8_t driveMotorControllerChannel;
    uint8_t steeringMotorDirectionPin;
    uint8_t steeringMotorPwmPin;
    uint8_t steeringMotorFaultPin;
    uint8_t homeSwitchPin;
    /** Pointer to the quadrature encoder for this wheel. */
    QuadEncoder *encoder;

    /** Indicates if this wheel is ready - it's ready when it's reset itself to the home and so knows it's position. */
    bool ready = false;
    /** True iff the last update found the wheel at the home position as indicated by the home switch. */
    bool atHomePosition = false;
    /** Current position of the wheel in degrees, in range [-180, 180] relative to home position, 
     * as determined in last call to update() */
    double position = 0;
    /** Target position of the wheel in degrees. */
    double targetPosition = 0;
   /** Steering motor drive rate in the last step. */
    double steeringDriveRate = 0;
     /** Drive motor target drive rate in the last step. */
    double targetDriveRate = 0;
    /** Drive motor actual drive rate in the last step. This can be different from target rate due to ramping. */
    double driveRate = 0;
    /** Time of previous step. */
    uint32_t previousStepTime = 0;
    /** Whether the motor controller (channel) is experiencing a fault. */
    bool steeringMotorDriverFault = false;
    /** Amount of time that wheel appears to have become stuck for, in seconds. */
    double stuckTime = 0;

    /** Update the drive motor, working towards target rate allowing for ramping. Rate range is [-1, 1]. */
    void updateDriveMotor(double motorCurrent) {
      double rampedRate = getRampedRateForMotorDrive(
        this->driveRate, this->targetDriveRate, motorCurrent, DRIVE_MOTOR_MAX_CURRENT
      );
      this->driveMotorController->motor(this->driveMotorControllerChannel, round(rampedRate * 2047));
      this->driveRate = rampedRate;
    }

    /** Update the steering motor. Rate range is [-1, 1]. Negative values indicate counter-clockwise. */
    void updateSteeringMotor(double rate, double motorCurrent) {
      double rampedRate = getRampedRateForMotorDrive(
        this->steeringDriveRate, rate, motorCurrent, STEERING_MOTOR_MAX_CURRENT
      );
      bool direction = rampedRate > 0 ? CW : CCW;
      digitalWrite(this->steeringMotorDirectionPin, direction);
      analogWrite(this->steeringMotorPwmPin, abs(round(rampedRate * 256)));
      this->steeringDriveRate = rampedRate;
    }

  
  public:
    Wheel(
      int wheelNumber, 
      USBSabertooth *driveMotorController,
      uint8_t driveMotorControllerChannel,
      uint8_t steeringMotorDirectionPin,
      uint8_t steeringMotorPwmPin,
      uint8_t steeringMotorFaultPin,
      uint8_t homeSwitchPin,
      QuadEncoder * encoder
    ):
      wheelNumber(wheelNumber),
      driveMotorController(driveMotorController),
      driveMotorControllerChannel(driveMotorControllerChannel),
      steeringMotorDirectionPin(steeringMotorDirectionPin),
      steeringMotorPwmPin(steeringMotorPwmPin),
      steeringMotorFaultPin(steeringMotorFaultPin),
      homeSwitchPin(homeSwitchPin),
      encoder(encoder)
    { 
      pinMode(this->homeSwitchPin, INPUT_PULLUP);
      pinMode(this->steeringMotorFaultPin, INPUT_PULLUP);
      pinMode(this->steeringMotorDirectionPin, OUTPUT);
      pinMode(this->steeringMotorPwmPin, OUTPUT);
      analogWriteFrequency(this->steeringMotorPwmPin, STEERING_PWM_FREQUENCY);
      this->encoder->setInitConfig();  // Loads default configuration for the encoder channel
      this->encoder->init();  // Initialise the encoder for the channel selected
    }

    void update(double driveMotorCurrent, double steeringMotorCurrent) {
      uint32_t now = micros();

      this->updateDriveMotor(driveMotorCurrent);
      
      this->steeringMotorDriverFault = digitalRead(this->steeringMotorFaultPin) == LOW;

      this->atHomePosition = digitalRead(this->homeSwitchPin) == LOW;
      // Reset encoder position to 0 when home position found.
      if (this->atHomePosition) this->encoder->write(0);

      double currentPosition = (double) this->encoder->read() / ENCODER_TICKS_PER_DEGREE;
      currentPosition = normaliseValueToRange(-180, currentPosition, 180);

      // If this isn't the first iteration and the drive rate is "large enough",
      // determine the amount the wheel is expected to turn, and check whether the wheel turned
      // within some threshold factor of that.
      if (this->previousStepTime > 0 && fabs(this->steeringDriveRate) > STUCK_CHECK_DRIVE_RATE_THRESHOLD) {
        double actualMovement = normaliseValueToRange(-180, currentPosition - this->position, 180);
        uint32_t elapsedTimeMicroSec = now - this->previousStepTime; // Note: this handles wrap around of timestamp.
        double elapsedTimeSec = elapsedTimeMicroSec * 0.000001;
        double expectedMovement = NOMINAL_MAX_DEGREES_PER_SECOND * elapsedTimeSec * this->steeringDriveRate;

        // If the wheel has turned less, in the right direction, than 
        // STUCK_MOVEMENT_THRESHOLD proportion of the expected amount.
        // Note: important to keep the sign of the expected and actual movement, 
        // to handle the case that the wheel turns the opposite direction.
        if (actualMovement / expectedMovement < STUCK_MOVEMENT_THRESHOLD) {
          this->stuckTime += elapsedTimeSec;
        }
        else {
          this->stuckTime = 0;
        }
      }
      else {
        this->stuckTime = 0;
      }

      this->position = currentPosition;

      // If the absolute position is not known, drive the wheel clockwise until we find the home position.
      if (!this->ready) {
        // Start/continue driving the wheel to the home position, as determined by the home switch.
        this->updateSteeringMotor(0.333, steeringMotorCurrent);
        if (this->atHomePosition) { 
          this->ready = true;
        }
      }
      else {
        double error = this->targetPosition - this->position;
        // Allow wrapping around, eg going directly from current position of -180.0 to target position of 180.1
        error = normaliseValueToRange(-180, error, 180);
        // Output is proportional to the error, but only within a few degrees of error,
        // otherwise it's capped to the maximum drive rate.
        double output = capRange(-1, error * STEERING_PID_K, 1);
        this->updateSteeringMotor(output, steeringMotorCurrent);
      }

      this->previousStepTime = now;
    }

    /** Set (target) drive rate, in range [-1, 1]. */
    void setDriveRate(double rate) { this->targetDriveRate = rate; }

    /** Get the current drive rate of the drive motor from the motor controller, in range [-1, 1]. */
    double getDriveRate() { return this->driveRate; }

    /** Set target position of the wheel in degrees, in range [-180, 180]. */
    void setWheelPositionTarget(double newTargetPosition) { this->targetPosition = newTargetPosition; }

    /** Get current position of the wheel in degrees, in range [-180, 180] relative to home position, 
     * as determined in last call to update(). */
    double getPosition() { return this->position; }

    /** Returns true iff this wheel is ready (knows its position). */
    bool isReady() { return this->ready; }

    /** Returns true iff the motor controller (channel) for this wheels' steering motor is experiencing a fault. 
     * Possible faults are output short-circuit, low source voltage, over-temperature (only of the board, not directly of the FETs!) */
    bool getSteeringDriverFault() { return this->steeringMotorDriverFault; }

    /** Get the current drive rate of the steering motor, in range [-1, 1]. */
    double getSteeringMotorDriveRate() { return this->steeringDriveRate; }

    /** Get the amount of time that the wheel has been stuck, in seconds. */
    double getStuckTime() { return this->stuckTime; }

    /** Returns the temperature of the drive motor controller channel for this wheel in degrees C,
     * or SABERTOOTH_GET_TIMED_OUT if the request timed out.
     */
    int getDriveControllerTemperature() {
      return this->driveMotorController->getTemperature(this->driveMotorControllerChannel);
    }
};
