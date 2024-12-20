#pragma once

#include <vector>
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

class C3SolutionToRobotCommand : public drake::systems::LeafSystem<double> {
public:
  explicit C3SolutionToRobotCommand(int vector_size);

  const drake::systems::InputPort<double>& get_input_port_c3_solution() const {
    return this->get_input_port(c3_solution_);
  }

  const drake::systems::OutputPort<double>& get_output_port_robot_command() const {
    return this->get_output_port(robot_command_);
  }

private:
  void AbstractToVector(const drake::systems::Context<double>& context,
			drake::systems::BasicVector<double>* output) const;

  drake::systems::InputPortIndex c3_solution_;
  drake::systems::OutputPortIndex robot_command_;

  int vector_size_;
};
    
} // namespace systems
} // namespace dairlib
