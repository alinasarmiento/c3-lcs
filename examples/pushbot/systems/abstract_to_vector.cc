#include "c3soln_to_command.h"

#include <drake/systems/framework/leaf_system.h>
#include "common/eigen_utils.h"
#include "solvers/c3_output.h"

class C3SolutionToRobotCommand : public drake::systems::LeafSystem<double> {
public:
  C3SolutionToRobotCommand(int vector_size) : vector_size_(vector_size) {
    c3_solution_port = 
      this->DeclareAbstractInputPort("c3_solution", drake::Value<C3Output::C3Solution>{}).get_index();
    robot_command_port =
      this->DeclareVectorOutputPort("robot_command", drake::systems::BasicVector<double>(vector_size_),
				    &C3SolutionToRobotCommand::ConvertToVector).get_index();
  }

private:
  void ConvertToVector(const drake::systems::Context<double>& context,
		       drake::systems::BasicVector<double>* output) const {
    const auto& abstract_data = this->get_input_port(c3_solution_port).Eval<C3Output::C3Solution>(context);
    Eigen::VectorXd vector_data = abstract_data.ToVector();
    output->SetFromVector(vector_data);
  }

  int vector_size_;
};
