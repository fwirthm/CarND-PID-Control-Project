#include "PID.h"
#include <stdlib.h>

/**
 * Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  LastError = 0.0;
  TotalError = 0.0;

}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  LastError = cte;
  TotalError += abs(cte);

}

double PID::GetTotalError() {
  /**
   * Calculate and return the total error
   */

  return TotalError; 
}


double PID::CalcResponse(double cte) {
  /**
   * Calculate the controler resposne
   */
  p_error = Kp*cte;
  d_error = Kd*(cte-LastError);
  UpdateError(cte);
  i_error = Ki*TotalError;
  
  //double response = -p_error -(Kd*(cte-LastError)) ;//-(Ki*np.sum(ctes));
  double response = - p_error - i_error - d_error;

  return response; 
}

double PID::GetKp(){
  return Kp; 
}
double PID::GetKi(){
  return Ki; 
}
double PID::GetKd(){
  return Kd; 
}
