#ifndef QCAR_SUPERVISOR_JMF_H
#define QCAR_SUPERVISOR_JMF_H

#include "qcar_visnav/estimation/model/kinematic_model.h"
#include "qcar_supervisor/jmf_component.h"
#include "qcar_supervisor/transition_model.h"
#include "qcar_supervisor/measurement_accel/measurement_accel.h"
#include "qcar_supervisor/measurement_velocity.h"
#include "qcar_supervisor/measurement_gyro.h"

namespace qcar_nav {

class JumpMarkovFilter {
public:
  JumpMarkovFilter(int max_components);

  void initialize(const std::vector<JmfComponent>& initial);
  void step(double dt,
            const KinematicModel::ModelInput& input,
            const KinematicModel::StateVec& dxdt,
            const KinematicModel::StateMat& ddxdtdx,
            double gravity,
            double vel_meas,
            double gyro_meas,
            const TransitionModel& T_acc,
            const TransitionModel& T_vel);

  KinematicModel::StateVec getMAPEstimate() const;
  std::vector<double> getAccStatusMarginals() const;
  std::vector<double> getVelStatusMarginals() const;

private:
  std::vector<JmfComponent> components_;
  int max_components_;

  void expandComponents(double dt,
                        const KinematicModel::ModelInput& input,
                        const KinematicModel::StateVec& dxdt,
                        const KinematicModel::StateMat& ddxdtdx,
                        double gravity,
                        double vel_meas,
                        double gyro_meas,
                        const TransitionModel& T_acc,
                        const TransitionModel& T_vel);

  void normalizeWeights();
  void reduceComponents();
};

} // namespace qcar_nav

#endif // QCAR_SUPERVISOR_JMF_H
