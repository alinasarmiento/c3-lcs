#include <math.h>

#include <vector>

#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/pushbot/parameters/pushbot_lcm_channels.h"
#include "examples/pushbot/parameters/pushbot_sim_params.h"
#include "examples/pushbot/parameters/pushbot_sim_scene_params.h"
#include "examples/pushbot/systems/external_force_generator.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

namespace dairlib {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::GeometrySet;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::AddActuationRecieverAndStateSenderLcm;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(lcm_channels,
              "examples/pushbot/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // load parameters
  PushbotSimParams sim_params = drake::yaml::LoadYamlFile<PushbotSimParams>(
      "examples/pushbot/parameters/pushbot_sim_params.yaml");
  PushbotLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<PushbotLcmChannels>(FLAGS_lcm_channels);
  PushbotSimSceneParams scene_params =
      drake::yaml::LoadYamlFile<PushbotSimSceneParams>(
          sim_params.sim_scene_file[sim_params.scene_index]);

  DiagramBuilder<double> builder;
  double sim_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  Parser parser(&plant);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex pushbot_index = parser.AddModels(FindResourceOrThrow("examples/pushbot/urdf/pushbot.urdf"))[0];
  drake::multibody::ModelInstanceIndex wall1_index = parser.AddModels(FindResourceOrThrow("examples/pushbot/urdf/wall.urdf"))[0];
  drake::multibody::ModelInstanceIndex wall2_index = parser.AddModels(FindResourceOrThrow("examples/pushbot/urdf/wall.urdf"))[0];
  
  Vector3d pushbot_origin = Eigen::VectorXd::Zero(3);

  RigidTransform<double> T_X_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), pushbot_origin);

  Vector3d wall1_origin = scene_params.wall1_position[0];
  Vector3d wall2_origin = scene_params.wall2_position[0];
  
  RigidTransform<double> T_E1_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), wall1_origin);
  RigidTransform<double> T_E2_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), wall2_origin);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", pushbot_index), T_X_W);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", wall1_index), T_E1_W);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", wall2_index), T_E2_W);
  
  const drake::geometry::GeometrySet ee_geom = plant.CollectRegisteredGeometries({&plant.GetBodyByName("end_effector")});

  plant.Finalize();
  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm;
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, lcm_channel_params.pushbot_input_channel,
      lcm_channel_params.pushbot_state_channel, sim_params.pushbot_publish_rate,
      pushbot_index, sim_params.publish_efforts, sim_params.actuator_delay);

  auto disturbance_force_generator = builder.AddSystem<ExternalForceGenerator>(plant.GetBodyByName("end_effector", pushbot_index).index());
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &drake_lcm));
  auto radio_to_vector = builder.AddSystem<systems::RadioToVector>();
  builder.Connect(*radio_sub, *radio_to_vector);
  builder.Connect(radio_to_vector->get_output_port(),
                  disturbance_force_generator->get_input_port_radio());
  builder.Connect(disturbance_force_generator->get_output_port_spatial_force(),
                  plant.get_applied_spatial_force_input_port());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  if (sim_params.visualize_drake_sim) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  VectorXd q = VectorXd::Zero(nq);

  q.head(plant.num_positions(pushbot_index)) = sim_params.q_init;

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
