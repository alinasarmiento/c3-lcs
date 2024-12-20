#include "examples/pushbot/systems/c3soln_to_command.h"

#include <drake/systems/framework/leaf_system.h>
#include "common/eigen_utils.h"
#include "solvers/c3_output.h"

namespace dairlib {

namespace systems {

C3SolutionToRobotCommand::C3SolutionToRobotCommand(int vector_size) {
  this->set_name("c3_soln_to_command");

  vector_size_ = vector_size;

  c3_solution_ = this->DeclareAbstractInputPort("c3_solution", drake::Value<C3Output::C3Solution>{}).get_index();
  robot_command_ = this->DeclareVectorOutputPort("robot_command", drake::systems::BasicVector<double>(vector_size_),
				    &C3SolutionToRobotCommand::AbstractToVector).get_index();

}

void C3SolutionToRobotCommand::AbstractToVector(const drake::systems::Context<double>& context,
		     drake::systems::BasicVector<double>* output) const {
  const auto& c3_solution = this->get_input_port(0).Eval<C3Output::C3Solution>(context);
  Eigen::VectorXd u_sol_next = c3_solution.u_sol_.col(0).cast<double>();
  output->SetFromVector(u_sol_next);
}

} // namespace systems
} //namespace dairlib
