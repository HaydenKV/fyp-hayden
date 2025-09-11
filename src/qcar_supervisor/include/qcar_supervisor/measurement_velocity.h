#pragma once
#include "qcar_visnav/estimation/model/kinematic_model.h"

Eigen::VectorXd h_vel(const qcar_nav::KinematicModel::StateVec& x);
Eigen::MatrixXd H_vel();
