/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

typedef struct {
  float kP;
  float kI;
  float kD;

  float _lastError;
  float _integrator;
  float _lastDerivative;
} Pid;

void InitPid(Pid* pid, float PGain, float IGain, float DGain);
float RunPid(Pid *self, float error, float deltaSeconds);

#endif /* __PID_H */
