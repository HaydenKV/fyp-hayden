#pragma once
#include "qcar_visnav/estimation/model/kinematic_model.h"

Eigen::VectorXd h_gyro(const qcar_nav::KinematicModel::StateVec& x, const qcar_nav::ModelParams& params);
Eigen::MatrixXd H_gyro(const qcar_nav::KinematicModel::StateVec& x, const qcar_nav::ModelParams& params);
