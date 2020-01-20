#include "PID.h"
#include <stdlib.h> 
#include <iostream>
#include <math.h>

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
	p_error = 0.0;
  	i_error = 0.0;
  	d_error = 0.0;

	pre_cte = 0.0;  
	first_itr = true;

	total_cte = 0;
	count=0;
}

void PID::UpdateError(double cte) {
	if (count > 200) {
		total_cte += fabs(cte);
	}
	p_error = cte;
	if (first_itr)
	{
		d_error = 0.0;
		pre_cte = cte;
		first_itr = false;
	} else {
		d_error = cte - pre_cte;
		pre_cte = cte; 
	}
 	
 	i_error += cte;

}

double PID::OutputControl() {
	double out = -Kp*p_error - Ki*i_error - Kd * d_error;
	count++;
	return out;
}

void PID::setPrevTotalCte(double totalCte){
	prev_total_cte = totalCte;
}

void PID::setPrevK(double Kp, double Ki, double Kd){
	prev_kp = Kp;
	prev_ki = Ki;
	prev_kd = Kd;
}

