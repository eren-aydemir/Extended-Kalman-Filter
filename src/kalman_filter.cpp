#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
     TODO:
     * predict the state
     */
    x_ = F_ * x_;
    
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
     TODO:
     * update the state by using Kalman Filter equations
     */
    
    VectorXd y = z - H_ * x_;
    
    //angle normalization
    if (y(1) < -M_PI) {
        y(1) = y(1) + (2 * M_PI);
    } else if (y(1) > M_PI) {
        y(1) = y(1) - (2 * M_PI);
    }
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
    
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    
    x_ = x_ + (K * y);
    
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
     TODO:
     * update the state by using Extended Kalman Filter equations
     */
    
    VectorXd y = z - h(x_);
    
    
    //angle normalization
    if (y(1) < -M_PI) {
        y(1) = y(1) + (2 * M_PI);
    } else if (y(1) > M_PI) {
        y(1) = y(1) - (2 * M_PI);
    }
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
    
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    
    x_ = x_ + (K * y);
    
    P_ = (I - K * H_) * P_;
}

VectorXd KalmanFilter::h(const VectorXd &x) {
    // extract position and velocity
    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);
    float rho_dot;
    
    VectorXd hx = VectorXd(3);
    
    float rho = sqrt(px*px + py*py);
    
    if (fabs(rho) < 0.0001) {
        rho_dot = 0;
    } else {
        rho_dot = (px*vx + py*vy) /rho;
    }
    
    float phi = atan2(py,px);
    
    hx << rho, phi, rho_dot;
    
    
    return hx;
}
