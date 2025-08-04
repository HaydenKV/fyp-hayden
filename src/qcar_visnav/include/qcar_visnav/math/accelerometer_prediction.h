#ifndef QCAR_VISNAV_ACCELEROMETER_PREDICTION_H
#define QCAR_VISNAV_ACCELEROMETER_PREDICTION_H

#include <Eigen/Dense>
#include "qcar_visnav/math/system_params.h"

namespace qcar_visnav {

struct PredictionResult {
    Eigen::Vector2d h;           // Predicted measurement h(x)
    Eigen::MatrixXd H;           // Jacobian matrix dh/dx
    bool valid;                  // Prediction validity
    
    PredictionResult() : valid(false) {
        h = Eigen::Vector2d::Zero();
        H = Eigen::MatrixXd::Zero(2, 10); // 2 measurements, 10 states
    }
};

class AccelerometerPredictor {
public:
    AccelerometerPredictor();
    
    // Main prediction function - ports MATLAB predict.m
    PredictionResult predict(const Eigen::VectorXd& x, const SystemParams& system);
    
    // Set accelerometer mounting parameters
    void setMountingParams(const Eigen::Vector3d& rMBb, const Eigen::Matrix3d& Rbm);
    
private:
    // Accelerometer mounting parameters (from MATLAB)
    Eigen::Vector3d rMBb_;  // Position of accelerometer w.r.t B expressed in {b}
    Eigen::Matrix3d Rbm_;   // Rbm = [m1b, m2b, m3b], accelerometer axes in {b}
    
    // Helper functions
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
    Eigen::MatrixXd computeSystemDynamicsJacobian(const Eigen::VectorXd& x, const SystemParams& system);
    Eigen::VectorXd getSystemDynamics(const Eigen::VectorXd& x, const SystemParams& system);
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_ACCELEROMETER_PREDICTION_H