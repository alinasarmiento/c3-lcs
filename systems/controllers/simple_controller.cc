#include "systems/controllers/simple_controller.h"
#include <drake/multibody/plant/multibody_plant.h>
#include "multibody/multibody_utils.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::Context;
using drake::multibody::MultibodyPlant;


namespace dairlib::systems::controllers {

SimpleController::SimpleController(const MultibodyPlant<double>& plant,
				   drake::systems::Context<double>* context)
    : plant_(plant),
      context_(context) {
    this->set_name("SimpleController");

    n_q_ = plant.num_positions();
    n_v_ = plant.num_velocities();
    n_u_ = plant.num_actuated_dofs();
    
  // Input port that corresponds to the state information
  input_port_info_index_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(n_q_, n_v_, n_u_))
          .get_index();

  // Ouput port for the actuator efforts
  output_port_control_index_ =
      this->DeclareVectorOutputPort("u, t",
                                    TimestampedVector<double>(n_u_),
                                    &SimpleController::CalcControl)
          .get_index();
}

void SimpleController::CalcControl(const Context<double>& context,
                                   TimestampedVector<double>* control) const {
  const OutputVector<double>* info =
      (OutputVector<double>*)this->EvalVectorInput(context,
                                                   input_port_info_index_);

  VectorXd u = info->GetEfforts();

  control->SetDataVector(u);
  control->set_timestamp(info->get_timestamp());
}

}  // namespace dairlib
