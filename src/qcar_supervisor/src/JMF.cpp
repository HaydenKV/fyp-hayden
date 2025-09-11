#include "qcar_supervisor/JMF.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <stdexcept>

namespace qcar_nav {

JumpMarkovFilter::JumpMarkovFilter(int max_components)
  : max_components_(max_components) {}

void JumpMarkovFilter::initialize(const std::vector<JmfComponent>& initial) {
  if (initial.empty()) {
    throw std::runtime_error("JMF initialization requires at least one component");
  }
  components_ = initial;
}

void JumpMarkovFilter::step(double dt,
                            const KinematicModel::ModelInput& input,
                            const KinematicModel::StateVec& dxdt,
                            const KinematicModel::StateMat& ddxdtdx,
                            double gravity,
                            double vel_meas,
                            double gyro_meas,
                            const TransitionModel& T_acc,
                            const TransitionModel& T_vel) {
  expandComponents(dt, input, dxdt, ddxdtdx, gravity, vel_meas, gyro_meas, T_acc, T_vel);
  normalizeWeights();
  reduceComponents();
}

void JumpMarkovFilter::expandComponents(double dt,
                                        const KinematicModel::ModelInput& input,
                                        const KinematicModel::StateVec& dxdt,
                                        const KinematicModel::StateMat& ddxdtdx,
                                        double gravity,
                                        double vel_meas,
                                        double gyro_meas,
                                        const TransitionModel& T_acc,
                                        const TransitionModel& T_vel) {
  std::vector<JmfComponent> new_components;

  Eigen::Matrix2d T_acc_dt = T_acc.getTransition(dt);
  Eigen::Matrix2d T_vel_dt = T_vel.getTransition(dt);

  for (const auto& comp : components_) {
    for (int acc_status = 1; acc_status <= 2; ++acc_status) {
      for (int vel_status = 1; vel_status <= 2; ++vel_status) {
        SensorStatus new_status{acc_status, vel_status};

        double p_acc = T_acc_dt(comp.status.acc - 1, acc_status - 1);
        double p_vel = T_vel_dt(comp.status.vel - 1, vel_status - 1);
        double log_trans = std::log(p_acc) + std::log(p_vel);

        JmfComponent new_comp = comp;
        new_comp.status = new_status;

        // Measurement updates
        double log_likelihood = 0.0;

        // Accelerometer
        MeasurementAccelerometer meas_acc;
        Eigen::Vector2d h_acc = meas_acc.predict(comp.mean, input, dxdt, ddxdtdx, gravity);
        auto H_acc = meas_acc.jacobian(comp.mean, ddxdtdx, gravity);
        auto R_acc = meas_acc.noiseCovariance(acc_status);
        Eigen::Vector2d y_acc(input.a_meas_x, input.a_meas_y);
        Eigen::Vector2d innov_acc = y_acc - h_acc;
        Eigen::Matrix2d S_acc = H_acc * comp.cov * H_acc.transpose() + R_acc;
        log_likelihood += -0.5 * innov_acc.transpose() * S_acc.inverse() * innov_acc
                          - 0.5 * std::log(S_acc.determinant());

        // Velocity
        Eigen::VectorXd h_v = h_vel(comp.mean);
        Eigen::MatrixXd H_v = H_vel();
        Eigen::MatrixXd R_v = Eigen::MatrixXd::Identity(1,1) * (vel_status == 1 ? 0.01 : 0.5);
        Eigen::VectorXd innov_v(1);
        innov_v(0) = vel_meas - h_v(0);
        Eigen::MatrixXd S_v = H_v * comp.cov * H_v.transpose() + R_v;
        log_likelihood += -0.5 * innov_v.transpose() * S_v.inverse() * innov_v
                          - 0.5 * std::log(S_v.determinant());

        // Gyro
        KinematicModel model;
        model.setParams(KinematicModel::ModelParams());
        Eigen::VectorXd h_g = h_gyro(comp.mean, model.getParams());
        Eigen::MatrixXd H_g = H_gyro(comp.mean, model.getParams());
        Eigen::MatrixXd R_g = Eigen::MatrixXd::Identity(1,1) * 0.01;
        Eigen::VectorXd innov_g(1);
        innov_g(0) = gyro_meas - h_g(0);
        Eigen::MatrixXd S_g = H_g * comp.cov * H_g.transpose() + R_g;
        log_likelihood += -0.5 * innov_g.transpose() * S_g.inverse() * innov_g
                          - 0.5 * std::log(S_g.determinant());

        // EKF update (simple linear form)
        Eigen::MatrixXd H_all(4, comp.mean.size());
        H_all << H_acc, H_v, H_g;
        Eigen::VectorXd innov_all(4);
        innov_all << innov_acc, innov_v, innov_g;
        Eigen::MatrixXd R_all(4, 4);
        R_all.setZero();
        R_all.block<2,2>(0,0) = R_acc;
        R_all(2,2) = R_v(0,0);
        R_all(3,3) = R_g(0,0);

        Eigen::MatrixXd S = H_all * comp.cov * H_all.transpose() + R_all;
        Eigen::MatrixXd K = comp.cov * H_all.transpose() * S.inverse();

        new_comp.mean = comp.mean + K * innov_all;
        new_comp.cov = (Eigen::MatrixXd::Identity(comp.mean.size(), comp.mean.size()) - K * H_all) * comp.cov;

        new_comp.log_weight = comp.log_weight + log_trans + log_likelihood;
        new_components.push_back(new_comp);
      }
    }
  }

  components_ = std::move(new_components);
}

void JumpMarkovFilter::normalizeWeights() {
  double max_log = components_.front().log_weight;
  for (const auto& comp : components_) {
    if (comp.log_weight > max_log) {
      max_log = comp.log_weight;
    }
  }

  double sum = 0.0;
  for (auto& comp : components_) {
    comp.log_weight = std::exp(comp.log_weight - max_log);
    sum += comp.log_weight;
  }

  for (auto& comp : components_) {
    comp.log_weight /= sum;
    comp.log_weight = std::log(comp.log_weight);
  }
}

void JumpMarkovFilter::reduceComponents() {
  std::sort(components_.begin(), components_.end(),
            [](const JmfComponent& a, const JmfComponent& b) {
              return a.log_weight > b.log_weight;
            });

  if (components_.size() > max_components_) {
    components_.resize(max_components_);
  }
}

KinematicModel::StateVec JumpMarkovFilter::getMAPEstimate() const {
  auto it = std::max_element(components_.begin(), components_.end(),
                             [](const JmfComponent& a, const JmfComponent& b) {
                               return a.log_weight < b.log_weight;
                             });
  return it->mean;
}

std::vector<double> JumpMarkovFilter::getAccStatusMarginals() const {
  std::vector<double> marginals(2, 0.0);
  for (const auto& comp : components_) {
    marginals[comp.status.acc - 1] += std::exp(comp.log_weight);
  }
  return marginals;
}

std::vector<double> JumpMarkovFilter::getVelStatusMarginals() const {
  std::vector<double> marginals(2, 0.0);
  for (const auto& comp : components_) {
    marginals[comp.status.vel - 1] += std::exp(comp.log_weight);
  }
  return marginals;
}

} // namespace qcar_nav
