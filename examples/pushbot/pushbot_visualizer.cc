#include <drake/multibody/parsing/parser.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "common/find_resource.h"
#include "multibody/visualization_utils.h"

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {

  using Eigen::Vector3d;
  using Eigen::VectorXd;

  using drake::geometry::DrakeVisualizer;
  using drake::geometry::SceneGraph;
  using drake::math::RigidTransformd;
  using drake::multibody::MultibodyPlant;
  using drake::multibody::RigidBody;
  using drake::systems::Simulator;

  using drake::math::RigidTransform;
  using drake::multibody::AddMultibodyPlantSceneGraph;
  using drake::multibody::Parser;
  using drake::systems::DiagramBuilder;

  int do_main() {

    drake::systems::DiagramBuilder<double> builder;

    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    scene_graph.set_name("scene_graph");

    MultibodyPlant<double> plant(0.0);

    Parser parser(&plant, &scene_graph);
    parser.SetAutoRenaming(true);
    drake::multibody::ModelInstanceIndex pushbot_index = parser.AddModels(FindResourceOrThrow("examples/pushbot/urdf/pushbot.urdf"))[0];

    Vector3d pushbot_origin = Eigen::VectorXd::Zero(3);
    RigidTransform<double> R_X_W = RigidTransform<double>(drake::math::RotationMatrix<double>(), pushbot_origin);
    
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"), R_X_W);

    plant.Finalize();

    drake::geometry::MeshcatVisualizerParams params;
    params.publish_period = 1.0 / 32;
    auto meshcat = std::make_shared<drake::geometry::Meshcat>();
    
    auto visualizer = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat, std::move(params));

    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();

    auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
    simulator->set_publish_every_time_step(false);
    simulator->set_publish_at_initialization(false);
    simulator->set_target_realtime_rate(1.0);

    simulator->AdvanceTo(std::numeric_limits<double>::infinity());

    return 0;
  }
}

int main() { return dairlib::do_main(); }
