#pragma once

#include <Eigen/Dense>
#include "measurement.hpp"
#include "system_qcar.hpp"

class AccelMeasurement : public Measurement {
public:
    Eigen::Vector2d z;  // Raw acceleration [a_x; a_y]
    Eigen::Matrix2d R;  // Measurement noise covariance

    // Position and orientation of accelerometer relative to body
    Eigen::Vector3d rMBb = Eigen::Vector3d(0.1278, 0.0223, 0.0895);
    Eigen::Matrix3d Rbm = Eigen::Matrix3d::Identity();  // Default aligned

    AccelMeasurement(const Eigen::Vector2d& accel_data) {
        z = accel_data;
        double sigma_acc = 0.1;
        R = sigma_acc * sigma_acc * Eigen::Matrix2d::Identity();
    }

    int dim() const override {
        return 2;
    }

    // Measurement function h(x): expected accelerometer reading given state
    Eigen::VectorXd h(const Eigen::VectorXd& x, const SystemQCAR& sys) const override {
        double u = x(0);
        double v = x(1);
        double r = x(2);
        double psi = x(8);

        // Rotation from inertial to body frame
        Eigen::Matrix3d Rbn;
        Rbn << cos(psi), -sin(psi), 0,
               sin(psi),  cos(psi), 0,
               0,         0,        1;

        Eigen::Matrix3d Rnb = Rbn.transpose();

        Eigen::Vector3d omegaBNb(0, 0, r);
        Eigen::Matrix3d Somega = skew(omegaBNb);
        Eigen::Matrix3d SrMBb = skew(rMBb);

        // Assume zero acceleration in this stub
        Eigen::Vector3d vBNb(u, v, 0);
        Eigen::Vector3d aBNb(0, 0, 0);
        Eigen::Vector3d omegadot(0, 0, 0);

        Eigen::Vector3d aMNb = aBNb
            + Somega * vBNb
            - SrMBb * omegadot
            - Somega * SrMBb * omegaBNb;

        Eigen::Vector3d gn(0, 0, sys.g);
        Eigen::Vector2d h = (Rbm * (aMNb - Rnb * gn)).head<2>();

        return h;
    }

    // Jacobian H = ∂h/∂x
    Eigen::MatrixXd H(const Eigen::VectorXd& x, const SystemQCAR& sys) const override {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, x.size());

        double u = x(0);
        double v = x(1);
        double r = x(2);

        H(0, 1) = -r;
        H(0, 2) = -v - rMBb(0) * 2 * r;

        H(1, 0) = r;
        H(1, 2) = u - rMBb(1) * 2 * r;

        return H;
    }

    Eigen::MatrixXd covariance() const override {
        return R;
    }

private:
    Eigen::Matrix3d skew(const Eigen::Vector3d& v) const {
        Eigen::Matrix3d S;
        S <<     0, -v.z(),  v.y(),
              v.z(),     0, -v.x(),
             -v.y(),  v.x(),     0;
        return S;
    }
};
