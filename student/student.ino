#include <SignalSource.h>
#include <math.h>


// Global Constants
constexpr float ERROR_OFFSET = 22.0f; // Degrees
constexpr float DELTA_T = 0.011f;
constexpr float OFFSET_TORQUE = 5.0f;
constexpr int16_t MIN_POS = 0;
constexpr int16_t RANGE = 1023 - MIN_POS;

// Global variables for the derivative and integral calculations
int16_t last_error = 0;   // To store the last error for derivative calculation
float accumulated_error = 0; // To store the accumulated error for integral calculation

//Resets the controller state when an experiment is started.
void reset(){
  last_error = 0;
  accumulated_error = 0;
}

/**
 * Rescales the position value from the motor.
 *
 * @param position The original position value to be rescaled.
 * @return The rescaled position value, adjusted to remain within a range of 0 to 360.
 */
float rescalepos(int16_t position) { // Rescaling function
  float rescaled;
  float degrees;

  // Observed min: 6 - max: 1014
  rescaled = (position - MIN_POS) * (2 * PI) / RANGE;
  degrees = fmod(((rescaled * 180.0f) / PI - ERROR_OFFSET), 360.0f);

  if (degrees < 0) {
    degrees += 360.0f;
  }

  return degrees;
}

/**
 * Calculates the error between the desired setpoint and the current position.
 *
 * @param setpoint The desired target value.
 * @param currentpos The current position value.
 * @return The error value, normalized to account for smallest angle to target value from current value.
 */
int16_t calculate_error(int16_t setpoint, int16_t currentpos){
  int16_t error1;
  int16_t error2;

  error1 = currentpos-setpoint;
  error2 = -((setpoint-MIN_POS)+(1023-currentpos));

  if (abs(error1) < abs(error2)) {
    return error1;
  }
  else {
    return error2;
  }
}

/**
 * Calculates the derivative of the error for use in a PID controller.
 *
 * @param error The current error value.
 * @return The change in error (delta) since the last call.
 */
int16_t error_derivative(int16_t error) {
  int16_t delta = (error - last_error) / DELTA_T;
  last_error = error;
  return delta;
}

int16_t error_integral(int16_t error) {
  accumulated_error += float(error) * DELTA_T;
  return accumulated_error;
}

/**
 * Implements a basic controller using proportional, integral, and derivative (PID) control.
 *
 * @param error The current error value.
 * @param error_i The integral of the error.
 * @param error_d The derivative of the error.
 * @param measured_disturbance The measured disturbance value.
 * @return The calculated control torque.
 */
int16_t controller(int16_t error, int16_t error_i, int16_t error_d, int16_t measured_disturbance){
  float Kp = 0.228f; // for PID
  // float Kp = 0.171f; // for PI

  float Ti = 0.1797f; // for PID
  // float Ti = 0.2983f; // for PI

  float Td = 0.0449f; // for PID
  // float Td = 0.0f; // for PI

  float control_torque = Kp * (error + (1.0f / Ti) * error_i + Td * error_d);

  return int16_t(control_torque + OFFSET_TORQUE - measured_disturbance);
  // return int16_t(control_torque + OFFSET_TORQUE);
}

/**
 * Placeholder for the main program loop. Do not change this.
 */
void loop(){
  start_loop();
}

/**
 * Sets up the controller by registering student-defined functions and initializing signals.
 * Do not change this.
 */
void setup(){
  register_student_fcns((studentFcns){
    .calculate_position=rescalepos,
    .calculate_error=calculate_error,
    .calculate_error_integral=error_integral,
    .calculate_error_derivative=error_derivative,
    .calculate_control=controller,
    .reset=reset});
  setup_signal();
}
