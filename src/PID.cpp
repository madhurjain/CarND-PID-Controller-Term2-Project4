#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  // For Twiddle
  nextState             = FIRSTRUN;
  resetCar              = false;
  moveToNextParamTuning = true;
  tuneParameter         = 0;
  dp[0]                 = 1;
  dp[1]                 = 1;
  dp[2]                 = 1;
  tolerance             = 0.005;
  iteration             = 0;
  stepCount             = 0;
  newErr                = 0.0;
  bestErr               = 0.0;
}

void PID::UpdateError(double cte) {
  // p_error holds previous value of cte
  d_error = cte - p_error;
  p_error = cte;
  i_error = i_error + cte;
}

double PID::TotalError() {
  return (Kp * p_error + Kd * d_error + Ki * i_error);
}

