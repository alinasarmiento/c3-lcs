#include "examples/pushbot/systems/c3soln_to_command.h"

#include <drake/systems/framework/leaf_system.h>
#include "common/eigen_utils.h"
#include "solvers/c3_output.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/framework/output_vector.h"

namespace dairlib {

namespace systems {

C3SolutionToRobotCommand::C3SolutionToRobotCommand(int u_size) {
  this->set_name("c3_soln_to_command");

  u_size_ = u_size;

  c3_solution_ = this->DeclareAbstractInputPort("c3_solution", drake::Value<C3Output::C3Solution>{}).get_index();
  robot_state_ = this->DeclareVectorInputPort("x,u,t", OutputVector<double>(2, 2, u_size_)).get_index();
  robot_command_ = this->DeclareVectorOutputPort("robot_command", TimestampedVector<double>(u_size_),
				    &C3SolutionToRobotCommand::AbstractToVector).get_index();

}

void C3SolutionToRobotCommand::AbstractToVector(const drake::systems::Context<double>& context,
		     systems::TimestampedVector<double>* output) const {
  const auto& c3_solution = this->get_input_port(0).Eval<C3Output::C3Solution>(context);
  Eigen::VectorXd u_sol_next = c3_solution.u_sol_.col(0).cast<double>();

  auto robot_state = dynamic_cast<const OutputVector<double>*>(EvalVectorInput(context, robot_state_));
  
  // Set output
  output->SetDataVector(u_sol_next);
  output->set_timestamp(robot_state->get_timestamp());
}

} // namespace systems
} //namespace dairlib
