#pragma once

#include <vector>
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

class C3SolutionToRobotCommand : public drake::systems::LeafSystem<double> {
public:
  explicit C3SolutionToRobotCommand(int u_size);

  const drake::systems::InputPort<double>& get_input_port_c3_solution() const {
    return this->get_input_port(c3_solution_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(robot_state_);
  }

  const drake::systems::OutputPort<double>& get_output_port_robot_command() const {
    return this->get_output_port(robot_command_);
  }

private:
  void AbstractToVector(const drake::systems::Context<double>& context,
			systems::TimestampedVector<double>* output) const;

  drake::systems::InputPortIndex c3_solution_;
  drake::systems::InputPortIndex robot_state_;
  drake::systems::OutputPortIndex robot_command_;

  int u_size_;
};
    
} // namespace systems
} // namespace dairlib
