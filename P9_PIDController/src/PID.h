#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double pre_cte;
  bool first_itr;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  int count;
  double total_cte;
  double prev_total_cte;

  double prev_kp;
  double prev_ki;
  double prev_kd;
  
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double OutputControl();

  void setPrevTotalCte(double totalCte);
  void setPrevK(double Kp, double Ki, double Kd);
};

#endif /* PID_H */
