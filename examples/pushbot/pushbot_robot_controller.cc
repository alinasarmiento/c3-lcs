#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
  
DEFINE_string(osqp_settings,
              "examples/pushbot/parameters/pushbot_ctrl_qp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_string(controller_parameters,
              "examples/pushbot/parameters/pushbot_llc_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/pushbot/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  PushbotControllerParams controller_params =
      drake::yaml::LoadYamlFile<PushbotControllerParams>(
          FLAGS_controller_parameters);
  PushbotLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<PushbotLcmChannels>(FLAGS_lcm_channels);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(FLAGS_osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());
  DiagramBuilder<double> builder;

  // ######### controller plant ################
  
  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, nullptr);
  parser.AddModels(FindResourceOrThrow("examples/pushbot/urdf/pushbot.urdf"));

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"), X_WI);

  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // ########## wiring ##############
  
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto pushbot_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.pushbot_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto pushbot_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);
  auto llc = builder.AddSystem<systems::controllers::LowLevelController>(plant, plant_context.get());
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  auto radio_to_vector = builder.AddSystem<systems::RadioToVector>();

  builder.Connect(*radio_sub, *radio_to_vector);
  builder.Connect(pushbot_command_sender->get_output_port(),
                  pushbot_command_pub->get_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  pushbot_command_sender->get_input_port(0));

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("pushbot_robot_controller"));
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver,
      lcm_channel_params.pushbot_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();
  return 0;

} // DoMain

} // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
