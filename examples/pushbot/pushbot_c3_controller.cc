#include <dairlib/lcmt_radio_out.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "solvers/lcs_factory.h"
#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "common/find_resource.h"
#include "examples/pushbot/parameters/pushbot_lcm_channels.h"
#include "examples/pushbot/parameters/pushbot_sim_scene_params.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"

#include "examples/pushbot/parameters/pushbot_c3_controller_params.h"
#include "examples/pushbot/parameters/pushbot_sim_params.h"
#include "examples/pushbot/systems/c3_state_sender.h"
#include "examples/pushbot/systems/c3_trajectory_generator.h"
#include "examples/pushbot/systems/pushbot_kinematics.h"
#include "examples/pushbot/systems/c3soln_to_command.h"
#include "solvers/solver_options_io.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/c3/lcs_factory_system.h"
#include "systems/controllers/c3/c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/trajectory_optimization/c3_output_systems.h"
#include "systems/system_utils.h"


namespace dairlib {

  using dairlib::solvers::LCSFactory;
  using drake::SortedPair;
  using drake::geometry::GeometryId;
  
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
  using drake::systems::lcm::LcmPublisherSystem;
  using drake::systems::lcm::LcmSubscriberSystem;

  using drake::math::RigidTransform;
  using drake::multibody::AddMultibodyPlantSceneGraph;
  using drake::multibody::Parser;
  using drake::systems::DiagramBuilder;
  using drake::systems::TriggerType;
  using drake::systems::TriggerTypeSet;

DEFINE_string(controller_settings,
              "examples/pushbot/parameters/pushbot_c3_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/pushbot/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

    // load parameters
    // drake::yaml::LoadYamlOptions yaml_options;
    // yaml_options.allow_yaml_with_no_cpp = true;
    PushbotC3ControllerParams controller_params =
	drake::yaml::LoadYamlFile<PushbotC3ControllerParams>(
	    FLAGS_controller_settings);
    PushbotLcmChannels lcm_channel_params =
	drake::yaml::LoadYamlFile<PushbotLcmChannels>(FLAGS_lcm_channels);
    PushbotSimSceneParams scene_params =
        drake::yaml::LoadYamlFile<PushbotSimSceneParams>("examples/pushbot/parameters/scene.yaml");
    C3Options c3_options = drake::yaml::LoadYamlFile<C3Options>(
	controller_params.c3_options_file[controller_params.scene_index]);
    drake::solvers::SolverOptions solver_options =
	drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
	    FindResourceOrThrow(controller_params.osqp_settings_file))
	    .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

    // ####### plant_pushbot ###########
    
    MultibodyPlant<double> plant_pushbot(0.0);
    Parser parser_pushbot(&plant_pushbot, nullptr);
    parser_pushbot.SetAutoRenaming(true);
    
    // add pushbot model
    drake::multibody::ModelInstanceIndex pushbot_index = parser_pushbot.AddModels(FindResourceOrThrow("examples/pushbot/urdf/pushbot.urdf"))[0];
    std::cout << "pushbot index in plant_pushbot: " << pushbot_index << std::endl;
    Vector3d pushbot_origin = Eigen::VectorXd::Zero(3);
    RigidTransform<double> R_X_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), pushbot_origin);
    plant_pushbot.WeldFrames(plant_pushbot.world_frame(), plant_pushbot.GetFrameByName("base", pushbot_index), R_X_W);

    plant_pushbot.Finalize();
    auto pushbot_context = plant_pushbot.CreateDefaultContext();

    // ####### plant_lcs ###########
    
    drake::systems::DiagramBuilder<double> plant_builder;
    
    auto [plant_lcs, scene_graph] = AddMultibodyPlantSceneGraph(&plant_builder, 0.0);

    Parser parser_lcs(&plant_lcs);
    parser_lcs.SetAutoRenaming(true);

    // add plant_lcs models
    drake::multibody::ModelInstanceIndex pushbot_index_lcs = parser_lcs.AddModels(FindResourceOrThrow("examples/pushbot/urdf/pushbot.urdf"))[0];
    drake::multibody::ModelInstanceIndex wall1_index = parser_lcs.AddModels(FindResourceOrThrow("examples/pushbot/urdf/wall.urdf"))[0];
    drake::multibody::ModelInstanceIndex wall2_index = parser_lcs.AddModels(FindResourceOrThrow("examples/pushbot/urdf/wall.urdf"))[0];

    std::cout << "pushbot index in plant_lcs: " << pushbot_index_lcs << std::endl;
    std::cout << "wall1 index in plant_lcs: " << wall1_index << std::endl;
    std::cout << "wall2 index in plant_lcs: " << wall2_index << std::endl;
    
    Vector3d wall1_origin = scene_params.wall1_position[0];
    Vector3d wall2_origin = scene_params.wall2_position[0];
    RigidTransform<double> T_E1_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), wall1_origin);
    RigidTransform<double> T_E2_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), wall2_origin);
    
    plant_lcs.WeldFrames(plant_lcs.world_frame(), plant_lcs.GetFrameByName("base", pushbot_index_lcs), R_X_W);
    plant_lcs.WeldFrames(plant_lcs.world_frame(), plant_lcs.GetFrameByName("base", wall1_index), T_E1_W);
    plant_lcs.WeldFrames(plant_lcs.world_frame(), plant_lcs.GetFrameByName("base", wall2_index), T_E2_W);

    plant_lcs.Finalize();

    // build diagram, context, autodiff for wall
    std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_lcs);
    
    auto plant_diagram = plant_builder.Build();
    std::unique_ptr<drake::systems::Context<double>> diagram_context = plant_diagram->CreateDefaultContext();
    auto& plant_lcs_context = plant_diagram->GetMutableSubsystemContext(plant_lcs, diagram_context.get());
    auto wall_context_ad = plant_autodiff->CreateDefaultContext();

    // announce contact geometries
    std::vector<drake::geometry::GeometryId> wall1_geom = plant_lcs.GetCollisionGeometriesForBody(plant_lcs.GetBodyByName("base", wall1_index));
    std::vector<drake::geometry::GeometryId> wall2_geom = plant_lcs.GetCollisionGeometriesForBody(plant_lcs.GetBodyByName("base", wall2_index));
    std::vector<drake::geometry::GeometryId> ee_contact_geom = plant_lcs.GetCollisionGeometriesForBody(plant_lcs.GetBodyByName("end_effector", pushbot_index));
    std::unordered_map<std::string, std::vector<drake::geometry::GeometryId>>contact_geoms;
    contact_geoms["WALL1"] = wall1_geom ;
    contact_geoms["WALL2"] = wall2_geom ;
    contact_geoms["EE"] = ee_contact_geom;
    
    std::vector<SortedPair<GeometryId>> contact_pairs;
    contact_pairs.emplace_back(contact_geoms["EE"][0], contact_geoms["WALL1"][0]);
    contact_pairs.emplace_back(contact_geoms["EE"][0], contact_geoms["WALL2"][0]);

    // ############### wiring #############
    
    DiagramBuilder<double> builder;
    auto pushbot_state_receiver =
	builder.AddSystem<systems::RobotOutputReceiver>(plant_pushbot);
    // std::cout << "plant pushbot: " << plant_pushbot.num_positions() << plant_pushbot.num_velocities() << plant_pushbot.num_actuators() << std::endl;

    auto c3_output_publisher =
	builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
	    lcm_channel_params.c3_debug_output_channel, &lcm,
	    TriggerTypeSet({TriggerType::kForced})));
    auto c3_target_state_publisher =
	builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
	    lcm_channel_params.c3_target_state_channel, &lcm,
	    TriggerTypeSet({TriggerType::kForced})));
    auto c3_actual_state_publisher =
	builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
	    lcm_channel_params.c3_actual_state_channel, &lcm,
	    TriggerTypeSet({TriggerType::kForced})));
    auto c3_forces_publisher =
	builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
	    lcm_channel_params.c3_force_channel, &lcm,
	    TriggerTypeSet({TriggerType::kForced})));
    auto radio_sub =
	builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
	    lcm_channel_params.radio_channel, &lcm));
    auto radio_to_vector = builder.AddSystem<systems::RadioToVector>();

    std::vector<int> input_sizes = {2, 2};
    auto target_source =
	builder.AddSystem<drake::systems::ConstantVectorSource>(
	    VectorXd::Zero(4));
    
    auto lcs_factory = builder.AddSystem<systems::LCSFactorySystem>(plant_lcs, plant_lcs_context, *plant_autodiff,
								    *wall_context_ad, contact_pairs, c3_options);
    // #################

    auto controller = builder.AddSystem<systems::C3Controller>(plant_lcs, c3_options);

    auto c3_trajectory_generator = builder.AddSystem<systems::C3TrajectoryGenerator>(plant_lcs, c3_options);

    std::vector<std::string> state_names = {
      "theta",  "ee_x", "dtheta", "ee_dx"
    };
    auto c3_state_sender =
	builder.AddSystem<systems::C3StateSender>(4, state_names);
    auto c3_output_sender = builder.AddSystem<systems::C3OutputSender>();
    controller->SetOsqpSolverOptions(solver_options);
    auto kinematics_model = builder.AddSystem<systems::PushbotKinematics>(plant_pushbot, pushbot_context.get());
    // ########################
    auto pushbot_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.pushbot_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
    auto pushbot_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant_pushbot);
    auto c3_to_command = builder.AddSystem<systems::C3SolutionToRobotCommand>(3);

    // size checking
    
    builder.Connect(*radio_sub, *radio_to_vector);
    
    builder.Connect(target_source->get_output_port(),
		    controller->get_input_port_target());

    builder.Connect(lcs_factory->get_output_port_lcs(),
		    controller->get_input_port_lcs());

    builder.Connect(pushbot_state_receiver->get_output_port(),
		    kinematics_model->get_input_port_pushbot_state());

    builder.Connect(kinematics_model->get_output_port_lcs_state(),
		    lcs_factory->get_input_port_lcs_state());
    builder.Connect(kinematics_model->get_output_port_lcs_input(),
		    lcs_factory->get_input_port_lcs_input());

    builder.Connect(lcs_factory->get_output_port_lcs_contact_jacobian(),
		    c3_output_sender->get_input_port_lcs_contact_info());
    builder.Connect(kinematics_model->get_output_port_lcs_state(), 
		    c3_state_sender->get_input_port_actual_state());

    builder.Connect(target_source->get_output_port(),
		    c3_state_sender->get_input_port_target_state());    
    builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
		    c3_target_state_publisher->get_input_port());
    builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
		    c3_actual_state_publisher->get_input_port());
    builder.Connect(controller->get_output_port_c3_solution(),
		    c3_output_sender->get_input_port_c3_solution());
    builder.Connect(controller->get_output_port_c3_intermediates(),
		    c3_output_sender->get_input_port_c3_intermediates());
    builder.Connect(c3_output_sender->get_output_port_c3_debug(),
		    c3_output_publisher->get_input_port());
    builder.Connect(c3_output_sender->get_output_port_c3_force(),
		    c3_forces_publisher->get_input_port());

    builder.Connect(pushbot_command_sender->get_output_port(),
		    pushbot_command_pub->get_input_port());
    // type checking
    std::cout << "c3 soln 2 command size: " << c3_to_command->get_output_port_robot_command().size() << std::endl;
    std::cout << "pushbot command size: " << pushbot_command_sender->get_input_port(0).size() << std::endl;
    builder.Connect(controller->get_output_port_c3_solution(), // test
		    c3_to_command->get_input_port_c3_solution());
    builder.Connect(c3_to_command->get_output_port_robot_command(),
		    pushbot_command_sender->get_input_port(0));
    
    auto owned_diagram = builder.Build();
    owned_diagram->set_name(("pushbot_c3_controller"));
    plant_diagram->set_name(("pushbot_c3_plant"));

    systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
	&lcm, std::move(owned_diagram), pushbot_state_receiver,
	lcm_channel_params.pushbot_state_channel, true);
    DrawAndSaveDiagramGraph(*loop.get_diagram());

    loop.Simulate();

    return 0;
  }
}

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
