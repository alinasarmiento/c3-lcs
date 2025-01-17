#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

class EndEffectorTrajectoryGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  EndEffectorTrajectoryGenerator(const Eigen::Vector3d& neutral_pose);

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  void SetRemoteControlParameters(const Eigen::Vector3d& neutral_pose,
                                  double x_scale, double y_scale,
                                  double z_scale);

 private:
  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  drake::systems::InputPortIndex trajectory_port_;
  drake::systems::InputPortIndex radio_port_;

  Eigen::Vector3d neutral_pose_ = {0.55, 0, 0.40};
  double x_scale_;
  double y_scale_;
  double z_scale_;
};

}  // namespace dairlib
