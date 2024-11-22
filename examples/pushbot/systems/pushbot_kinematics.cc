#include "examples/pushbot/systems/pushbot_kinematics.h"

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"

namespace dairlib {

using drake::multibody::MultibodyPlant;
using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using systems::OutputVector;
using systems::StateVector;
using systems::TimestampedVector;

namespace systems {

PushbotKinematics::PushbotKinematics(const MultibodyPlant<double>& pushbot_plant,
				     Context<double>* pushbot_context)
    : pushbot_plant_(pushbot_plant),
      pushbot_context_(pushbot_context),
      world_(pushbot_plant_.world_frame()) {
  this->set_name("pushbot_kinematics");
  pushbot_state_port_ =
      this->DeclareVectorInputPort(
              "x_pushbot", OutputVector<double>(pushbot_plant.num_positions(),
                                               pushbot_plant.num_velocities(),
                                               pushbot_plant.num_actuators()))
          .get_index();

  int num_joint_positions_ = 2;
  int num_joint_velocities_ = 2;
  lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              PushbotKinematicsVector<double>(num_joint_positions_,
                  num_joint_velocities_),
              &PushbotKinematics::ComputeLCSState)
          .get_index();
  lcs_input_port_ =
      this->DeclareVectorOutputPort(
              "u_lcs",
              BasicVector<double>(2),
              &PushbotKinematics::ComputeLCSInput)
          .get_index();
}

void PushbotKinematics::ComputeLCSState(
    const drake::systems::Context<double>& context,
    PushbotKinematicsVector<double>* lcs_state) const {
  const OutputVector<double>* pushbot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, pushbot_state_port_);

  VectorXd q_pushbot = pushbot_output->GetPositions();
  VectorXd v_pushbot = pushbot_output->GetVelocities();
  multibody::SetPositionsIfNew<double>(pushbot_plant_, q_pushbot,
                                       pushbot_context_);
  multibody::SetVelocitiesIfNew<double>(pushbot_plant_, v_pushbot,
                                        pushbot_context_);

  lcs_state->SetJointPositions(q_pushbot);
  lcs_state->SetJointVelocities(v_pushbot);
  lcs_state->set_timestamp(pushbot_output->get_timestamp());
}

void PushbotKinematics::ComputeLCSInput(
    const drake::systems::Context<double>& context,
    BasicVector<double>* lcs_input) const {
  const OutputVector<double>* pushbot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, pushbot_state_port_);

  lcs_input->SetFromVector(pushbot_output->GetEfforts());
}

}  // namespace systems
}  // namespace dairlib
