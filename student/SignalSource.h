// Variables for Offset  and Torque
#ifndef HEADERFILE_H
#define HEADERFILE_H

#include <Arduino.h>
#include <Servo.h>

void start_loop();
void setup_signal();

typedef struct {
  float (*calculate_position)(int16_t);
  int16_t (*calculate_error)(int16_t, int16_t);
  int16_t (*calculate_error_integral)(int16_t);
  int16_t (*calculate_error_derivative)(int16_t);
  int16_t (*calculate_control)(int16_t, int16_t, int16_t, int16_t);
  void (*reset)();
} studentFcns;

void register_student_fcns(studentFcns);
#endif
