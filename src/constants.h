const int MOTOR_PINION_TEETH = 9;
const int WHEEL_SPROCKET_TEETH = 60;
const int ENCODER_PPR = 400; // PPR describes the number of high pulses the encoder has on either of its outputs per revolution.
// The encoder library (in|de)crements the count for every change on either of the encoder outputs, 
// so the count given by the encoder library will be double the PPR for every revolution of the encoder.
// The encoder is attached to the motor output, so factor in the gearing ratio of that to the actual wheel.
const int ENCODER_TICKS_PER_WHEEL_REVOLUTION = 2 * ENCODER_PPR * ((double) WHEEL_SPROCKET_TEETH / (float) MOTOR_PINION_TEETH); 
const int ENCODER_TICKS_PER_DEGREE = ENCODER_TICKS_PER_WHEEL_REVOLUTION / 360;

enum Direction {
  /** Clockwise */
  CW = LOW,
  /** Counter-clockwise */
  CCW = HIGH
};

const double PID_K = 0.5;

const int PWM_FREQUENCY = 20000;

const double CURRENT_SENSOR_VOLTS_PER_AMP = 0.0068;

/** The expected nominal degrees per second turn rate at the maximum turn rate. */
const double NOMINAL_MAX_DEGREES_PER_SECOND = (37.48 / 60) * 360;

/** If a wheel has turned less than this proportion of the expected amount, consider the wheel stuck. */
const double STUCK_MOVEMENT_THRESHOLD = 0.3;

/** Only check if the wheel is stuck if the drive rate is greater than this. */ 
const double STUCK_CHECK_DRIVE_RATE_THRESHOLD = 0.2;

/** Period of wheel updates, in us. */
const uint32_t WHEEL_UPDATE_PERIOD = 500000;

// #define DEBUG
