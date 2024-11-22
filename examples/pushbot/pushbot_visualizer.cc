#include <drake/multibody/parsing/parser.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "common/find_resource.h"
#include "examples/pushbot/parameters/pushbot_lcm_channels.h"
#include "examples/pushbot/parameters/pushbot_sim_scene_params.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"
#include "systems/visualization/lcm_visualization_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"


namespace dairlib {

  using Eigen::Vector3d;
  using Eigen::VectorXd;

  using dairlib::systems::RobotOutputReceiver;
  using dairlib::systems::SubvectorPassThrough;
  using drake::geometry::DrakeVisualizer;
  using drake::geometry::SceneGraph;
  using drake::math::RigidTransformd;
  using drake::multibody::MultibodyPlant;
  using drake::multibody::RigidBody;
  using drake::systems::Simulator;
  using drake::systems::lcm::LcmSubscriberSystem;
  using drake::systems::rendering::MultibodyPositionToGeometryPose;

  using drake::math::RigidTransform;
  using drake::multibody::AddMultibodyPlantSceneGraph;
  using drake::multibody::Parser;
  using drake::systems::DiagramBuilder;
  
  int do_main() {
    PushbotLcmChannels lcm_channel_params = drake::yaml::LoadYamlFile<PushbotLcmChannels>("examples/pushbot/parameters/lcm_channels_simulation.yaml");
    PushbotSimSceneParams scene_params = drake::yaml::LoadYamlFile<PushbotSimSceneParams>("examples/pushbot/parameters/scene.yaml");
    drake::systems::DiagramBuilder<double> builder;

    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    scene_graph.set_name("scene_graph");

    MultibodyPlant<double> plant(0.0);

    Parser parser(&plant, &scene_graph);
    parser.SetAutoRenaming(true);
    drake::multibody::ModelInstanceIndex pushbot_index = parser.AddModels(FindResourceOrThrow("examples/pushbot/urdf/pushbot.urdf"))[0];
    drake::multibody::ModelInstanceIndex wall1_index = parser.AddModels(FindResourceOrThrow("examples/pushbot/urdf/wall.urdf"))[0];
    drake::multibody::ModelInstanceIndex wall2_index = parser.AddModels(FindResourceOrThrow("examples/pushbot/urdf/wall.urdf"))[0];

    Vector3d pushbot_origin = Eigen::VectorXd::Zero(3);
    RigidTransform<double> R_X_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), pushbot_origin);

    Vector3d wall1_origin = scene_params.wall1_position[0];
    Vector3d wall2_origin = scene_params.wall2_position[0];
    RigidTransform<double> T_E1_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), wall1_origin);
    RigidTransform<double> T_E2_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), wall2_origin);
    
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", pushbot_index), R_X_W);
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", wall1_index), T_E1_W);
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", wall2_index), T_E2_W);

    plant.Finalize();

    auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

    auto pushbot_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(lcm_channel_params.pushbot_state_channel, lcm));
    auto pushbot_state_receiver =
      builder.AddSystem<RobotOutputReceiver>(plant, pushbot_index);

    auto pushbot_passthrough = builder.AddSystem<SubvectorPassThrough>(pushbot_state_receiver->get_output_port(0).size(), 0, plant.num_positions(pushbot_index));

    auto to_pose = builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

    std::vector<int> input_sizes = {plant.num_positions(pushbot_index)};
    auto mux = builder.AddSystem<drake::systems::Multiplexer<double>>(input_sizes);
    
    drake::geometry::MeshcatVisualizerParams params;
    params.publish_period = 1.0 / 32;
    auto meshcat = std::make_shared<drake::geometry::Meshcat>();

    // meshcat::AddButton("disturbance_force");
    // builder.Connect();
    
    builder.Connect(pushbot_passthrough->get_output_port(),
		    mux->get_input_port(0));
    builder.Connect(*mux, *to_pose);
    builder.Connect(to_pose->get_output_port(), scene_graph.get_source_pose_port(plant.get_source_id().value()));
    builder.Connect(*pushbot_state_receiver, *pushbot_passthrough);
    builder.Connect(*pushbot_state_sub, *pushbot_state_receiver);
    
    auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat, std::move(params));

    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();

    auto& pushbot_state_sub_context = diagram->GetMutableSubsystemContext(*pushbot_state_sub, context.get());
    pushbot_state_receiver->InitializeSubscriberPositions(plant, pushbot_state_sub_context);

    auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
    simulator->set_publish_every_time_step(false);
    simulator->set_publish_at_initialization(false);
    simulator->set_target_realtime_rate(1.0);
    simulator->Initialize();
    
    simulator->AdvanceTo(std::numeric_limits<double>::infinity());

    return 0;
  }
}

int main() { return dairlib::do_main(); }
