#pragma once

#include <string>
#include <vector>
#include <drake/multibody/plant/multibody_plant.h>
#include "examples/pushbot/systems/pushbot_kinematics_vector.h"

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"

namespace dairlib {
namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class PushbotKinematics : public drake::systems::LeafSystem<double> {
 public:
  explicit PushbotKinematics(const drake::multibody::MultibodyPlant<double>& pushbot_plant,
			     drake::systems::Context<double>* pushbot_context);
  
  const drake::systems::InputPort<double>& get_input_port_pushbot_state() const {
    return this->get_input_port(pushbot_state_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_lcs_state() const {
    return this->get_output_port(lcs_state_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_lcs_input() const {
    return this->get_output_port(lcs_input_port_);
  }

 private:
  void ComputeLCSState(
      const drake::systems::Context<double>& context,
      PushbotKinematicsVector<double>* lcs_state) const;
  void ComputeLCSInput(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* lcs_input) const;

  drake::systems::InputPortIndex pushbot_state_port_;
  drake::systems::OutputPortIndex lcs_state_port_;
  drake::systems::OutputPortIndex lcs_input_port_;

  const drake::multibody::MultibodyPlant<double>& pushbot_plant_;
  drake::systems::Context<double>* pushbot_context_;
  const drake::multibody::Frame<double>& world_;
};

}  // namespace systems
}  // namespace dairlib
