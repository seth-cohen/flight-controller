#include "pid.h"

void InitPid(Pid* pid, float PGain, float IGain, float DGain) {
  pid->_integrator = 0.0f;
  pid->_lastError = 0.0f;
  pid->_lastDerivative = 0.0f;

  pid->kP = PGain;
  pid->kI = IGain;
  pid->kD = DGain;
}

// @TODO - Add integrator and derivative
float RunPid(Pid *self, float error, float deltaSeconds) {
  float output = 0;

  if (deltaSeconds > 1.0f) {
    // reset the integrator
    self->_integrator = 0.0f;
  }

  // Handle the P value
  output += error * self->kP;

  return output;
}
