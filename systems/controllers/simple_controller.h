#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"
#include <drake/multibody/plant/multibody_plant.h>


namespace dairlib::systems::controllers {

/*
 * SimpleController class that generates a control input of the form
 * u = u
 * The controller has two input ports.
 * The first port is of type OutputVector<double> and has the state information
 * of the system.
 * The second input port is of type SimpleParams and has the parameters
 * (K, E and x_desired) of the controller.
 * The controller has a single output port of type TimestampedVector<double>
 * that holds the control inputs plus a timestamp that is taken from the
 * timestamp of the OutputVector input port.
 */
class SimpleController : public drake::systems::LeafSystem<double> {
 public:
  SimpleController(const drake::multibody::MultibodyPlant<double>& plant,
		   drake::systems::Context<double>* context);

  const drake::systems::InputPort<double>& get_input_port_info() const {
    return this->get_input_port(input_port_info_index_);
  }

  const drake::systems::OutputPort<double>& get_output_port_control() const {
    return this->get_output_port(output_port_control_index_);
  }

  int get_input_port_info_index() { return input_port_info_index_; }

  int get_output_port_control_index() { return output_port_control_index_; }

 private:
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* output) const;

  int input_port_info_index_;
  int output_port_control_index_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  int n_q_;
  int n_v_;
  int n_u_;
  
};

}  // namespace dairlib
