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
  	kp = Kpi;
  	ki = Kii;
  	kd = Kdi;
  	minlimit = output_lim_mini;
  	maxlimit = output_lim_maxi;
  	prev_cte = 0.0;
    error = 0.0;
  	d_cte = 0.0;
  	sum_cte = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  	error = cte;
  	d_cte = (error - prev_cte)/deltatime;
  	sum_cte += error*deltatime;
  	
  	prev_cte = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
  	control = (kp * error + kd * d_cte + ki*sum_cte);
  
//   	if(control > maxlimit) {
//       control = maxlimit;
//     } else if(control < minlimit) {
//       control = minlimit;
//     }
  	control = min(control, maxlimit);
   	control = max(control, minlimit);
      
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
    deltatime = new_delta_time;
  	
  	return deltatime;
}