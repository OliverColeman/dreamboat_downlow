const double STEERING_SPROCKET_TEETH = 60; // Use doubles to save conversion to double below.
const double ENCODER_SPROCKET_TEETH = 26;
const double ENCODER_PPR = 400; // PPR describes the number of high pulses the encoder has on either of its outputs per revolution.
// The encoder library (in|de)crements the count for every change on either of the encoder outputs, 
// so the count given by the encoder library will be double the PPR for every revolution of the encoder.
// The encoder is attached to the motor output, so factor in the gearing ratio of that to the actual wheel.
const double ENCODER_TICKS_PER_WHEEL_REVOLUTION = 4 * ENCODER_PPR * STEERING_SPROCKET_TEETH / ENCODER_SPROCKET_TEETH;
const double ENCODER_TICKS_PER_DEGREE = ENCODER_TICKS_PER_WHEEL_REVOLUTION / 360.0;

enum Direction {
  /** Clockwise */
  CW = LOW,
  /** Counter-clockwise */
  CCW = HIGH
};

const double STEERING_PID_K = 0.03;

const int STEERING_PWM_FREQUENCY = 20000;

// Sensor output is 0.052v/A, 5v to 3.3v level shifting is via 2.2k and 3.3k resistors.
const double CURRENT_SENSOR_VOLTS_PER_AMP = 0.052 * (3.3 / 5.5);
const double CURRENT_SENSOR_MAX_V_IN = 3.3;
const int CURRENT_SENSOR_ADC_RESOLUTION = 10;
/** Factor to convert raw current sensor readings to amps. */
const double CURRENT_SENSOR_CONVERSION_FACTOR = (CURRENT_SENSOR_VOLTS_PER_AMP * ((2^CURRENT_SENSOR_ADC_RESOLUTION) - 1)) / CURRENT_SENSOR_MAX_V_IN;

/** The expected nominal degrees per second turn rate at the maximum turn rate. */
const double NOMINAL_MAX_DEGREES_PER_SECOND = (37.48 / 60) * 360;

/** If a wheel has turned less than this proportion of the expected amount, consider the wheel stuck. */
const double STUCK_MOVEMENT_THRESHOLD = 0.3;

/** Only check if the wheel is stuck if the drive rate is greater than this. */ 
const double STUCK_CHECK_DRIVE_RATE_THRESHOLD = 0.1;

/** Frequency of wheel updates in Hz. */
const int WHEEL_UPDATE_FREQ = 50;
/** Period of wheel updates, in us. */
const uint32_t WHEEL_UPDATE_PERIOD = 1000000 / WHEEL_UPDATE_FREQ;

/** Time it takes in seconds to go from full stop to full speed for the drive and steering motors. */
const double DRIVE_MOTOR_RAMPING_TIME = 0.5;
/** The maximum percentile change in motor drive rate per update step. */
const double DRIVE_MOTOR_RAMPING_DELTA = 1.0 / (DRIVE_MOTOR_RAMPING_TIME * WHEEL_UPDATE_FREQ);
/** Time it takes in seconds to go from full stop to full speed for the drive and steering motors. */
const double STEERING_MOTOR_RAMPING_TIME = 0.2;
/** The maximum percentile change in motor drive rate per update step. */
const double STEERING_MOTOR_RAMPING_DELTA = 1.0 / (STEERING_MOTOR_RAMPING_TIME * WHEEL_UPDATE_FREQ);

/** Throttle (multiply) final motor drive rates by this amount. 1 means no throttling. */
const double MOTOR_THROTTLING_FACTOR = 1;

/** The maximum current draw allowed for drive motors, in amps. */
const double DRIVE_MOTOR_MAX_CURRENT = 9999;
/** The maximum current draw allowed for steering motors, in amps. */
const double STEERING_MOTOR_MAX_CURRENT = 9999;

/** Proportion of maximum steering rate when finding the home position. */
const double STEERING_HOMING_DRIVE_RATE = 0.15;

#define DRIVE_MOTOR_SERIAL0 Serial7
#define DRIVE_MOTOR_SERIAL1 Serial3

/** Time before it's considered an error to not have recived a set command from the controller, in us. */
const uint32_t SET_COMMAND_RECEIVED_TIMEOUT = 1 * 1000000;

const uint32_t STATUS_OKAY_BLINK_PERIOD = 1000000 / 1;
const uint32_t STATUS_ERROR_BLINK_PERIOD = 1000000 / 10;

// #define DEBUG
