#include "kalman_filter.h"
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PI 3.14159265


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z, int mod) {
	MatrixXd H_tmp;
	VectorXd z_pred;
	if (mod == 0) {  // Lidar
		H_tmp = H_;
		z_pred = H_tmp * x_;
	} else if (mod == 1) {  // Radar
		H_tmp = Hj_;
		z_pred = VectorXd(3);
		float px = x_(0);
		float py = x_(1);
		float vx = x_(2);
		float vy = x_(3);

		float rho = sqrt(px*px+py*py);
		float theta = atan2(py, px);
		//cout<<"============"<<z(1) -theta <<endl;

		if (fabs(z(1)-theta)>6.0) { 
			// for example car drives from -pi to pi or from pi to -pi
			if (theta < 0){
				// car drives from -pi to pi
				theta += 2*PI;
			} else if (theta > 0) {
				// car drives from pi to -pi
				theta -= 2*PI;
			}
		}
		if (rho < 0.0001){
			rho = 0.0001;
		}
		float rho_p = (px*vx+py*vy)/(rho);
		z_pred << rho, theta, rho_p;

	} else {
		cout<<"False Mode: "<<mod<<endl;
	}
	
	VectorXd y = z - z_pred;  // y = z - H*x'
	MatrixXd Ht = H_tmp.transpose();
	MatrixXd S = H_tmp * P_ * Ht + R_;  // S
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;  // K

	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_tmp) * P_;
}
