#include "../lib/QuadEncoder/QuadEncoder.h"

#include "./constants.h"
#include "./util.h"


/**
 * Encapsulates steering telemetry and control functions for a wheel.
 */
class Wheel {
  private:
    uint8_t wheelNumber; 
    uint8_t homeSwitchPin;
    uint8_t steeringMotorDirectionPin;
    uint8_t steeringMotorPwmPin;
    uint8_t steeringMotorFaultPin;
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
    /** Drive rate in the last step. */
    double driveRate = 0;
    /** Time of previous step. */
    uint32_t previousStepTime = 0;
    /** Whether the motor controller (channel) is experiencing a fault. */
    bool steeringMotorDriverFault = false;
    /** Amount of time that wheel appears to have become stuck for, in seconds. */
    double stuckTime = 0;


    /** Drive the steering motor. Rate range is [-1, 1]. Negative values indicate counter-clockwise. */
    void driveMotor(double rate) {
      // Ensure change in drive speed this time step is no more than MOTOR_RAMPING_DELTA.
      double rateDelta = capRange(-MOTOR_RAMPING_DELTA, rate - this->driveRate, MOTOR_RAMPING_DELTA);
      double rampedRate = this->driveRate + rateDelta;
      
      // If the motor direction is changing, turn off PWM output before performing the direction change.
      // This avoids the situation where the A and B direction input on the motor controller are both potentially 
      // enabled momentarily due to the signal propagation delay for the NOT gate driving the B input.
      bool direction = rampedRate > 0 ? CW : CCW;
      digitalWrite(this->steeringMotorDirectionPin, direction);
      analogWrite(this->steeringMotorPwmPin, abs(round(rampedRate * 256)));
      this->driveRate = rampedRate;
    }

  
  public:
    Wheel(
      int wheelNumber, 
      uint8_t steeringMotorDirectionPin,
      uint8_t steeringMotorPwmPin,
      uint8_t steeringMotorFaultPin,
      uint8_t homeSwitchPin,
      QuadEncoder * encoder
    ):
      wheelNumber(wheelNumber),
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
      analogWriteFrequency(this->steeringMotorPwmPin, PWM_FREQUENCY);
      this->encoder->setInitConfig();  // Loads default configuration for the encoder channel
      this->encoder->init();  // Initialise the encoder for the channel selected
    }

    void update() {
      uint32_t now = micros();

      this->steeringMotorDriverFault = digitalRead(this->steeringMotorFaultPin) == LOW;

      this->atHomePosition = digitalRead(this->homeSwitchPin) == LOW;
      // Reset encoder position to 0 when home position found.
      if (this->atHomePosition) this->encoder->write(0);

      double currentPosition = (double) this->encoder->read() / ENCODER_TICKS_PER_DEGREE;
      currentPosition = normaliseValueToRange(-180, currentPosition, 180);

      // If this isn't the first iteration and the drive rate is "large enough",
      // determine the amount the wheel is expected to turn, and check whether the wheel turned
      // within some threshold factor of that.
      if (this->previousStepTime > 0 && fabs(this->driveRate) > STUCK_CHECK_DRIVE_RATE_THRESHOLD) {
        double actualMovement = normaliseValueToRange(-180, currentPosition - this->position, 180);
        uint32_t elapsedTimeMicroSec = now - this->previousStepTime; // Note: this handles wrap around of timestamp.
        double elapsedTimeSec = elapsedTimeMicroSec * 0.000001;
        double expectedMovement = NOMINAL_MAX_DEGREES_PER_SECOND * elapsedTimeSec * this->driveRate;

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

      this->position = currentPosition;

      // If the absolute position is not known, drive the wheel clockwise until we find the home position.
      if (!this->ready) {
        // Start/continue driving the wheel to the home position, as determined by the home switch.
        this->driveMotor(1);
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
        double output = capRange(-1, error * PID_K, 1);
        this->driveMotor(output);
      }

      this->previousStepTime = now;
    }

    /** Set target position of the wheel in degrees, in range [-180, 180]. */
    void setWheelPositionTarget(double newTargetPosition) {
      this->targetPosition = newTargetPosition;
    }

    /** Get current position of the wheel in degrees, in range [-180, 180] relative to home position, 
     * as determined in last call to update(). */
    double getPosition() { return this->position; }

    /** Returns true iff this wheel is ready (knows its position). */
    bool isReady() { return this->ready; }

    /** Returns true iff the motor controller (channel) for this wheels' steering motor is experiencing a fault. 
     * Possible faults are output short-circuit, low source voltage, over-temperature (only of the board, not directly of the FETs!) */
    bool getSteeringDriverFault() { return this->steeringMotorDriverFault; }

    /** Get the current drive rate of the steering motor, in range [-1, 1]. */
    double getDriveRate() { return this->driveRate; }

    /** Get the amount of time that the wheel has been stuck, in seconds. */
    double getStuckTime() { return this->stuckTime; }
};
