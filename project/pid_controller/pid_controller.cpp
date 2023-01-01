/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  err_p = 0.0;
  err_i = 0.0;
  err_d = 0.0;
  coeff_p = Kpi;
  coeff_i = Kii;
  coeff_d = Kdi;
  limit_max = output_lim_maxi;
  limit_min = output_lim_mini;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  if (dt > 0) {
    int prev = err_p;
    err_d = (cte - prev) / dt; // NOTE: dt must > 0
    err_i += cte * dt;
    err_p = cte;
  } else {
    dt = 0.0;
  }
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
//     double control = -coeff_p * err_p - coeff_i * err_i - coeff_d * err_d;
    double control = coeff_p * err_p + coeff_i * err_i + coeff_d * err_d;
//     double control = coeff_p * err_p - coeff_i * err_i - coeff_d * err_d;
    control = min(max(control, limit_min), limit_max);
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  dt = new_delta_time;
}