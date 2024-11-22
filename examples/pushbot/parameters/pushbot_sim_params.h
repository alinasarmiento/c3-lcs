#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct PushbotSimParams {
  std::vector<std::string> sim_scene_file;
  
  double dt;
  double realtime_rate;
  double actuator_delay;
  double pushbot_publish_rate;
  double visualizer_publish_rate;

  int scene_index;
  bool visualize_drake_sim;
  bool publish_efforts;
  bool publish_object_velocities;

  Eigen::VectorXd q_init;

  std::vector<double> external_force_scaling;

  bool visualize_pose_trace;
  bool visualize_center_of_mass_plan;
  bool visualize_c3_forces;
  bool visualize_workspace;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sim_scene_file));

    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(pushbot_publish_rate));
    a->Visit(DRAKE_NVP(visualizer_publish_rate));

    a->Visit(DRAKE_NVP(scene_index));
    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(publish_object_velocities));

    a->Visit(DRAKE_NVP(q_init));

    a->Visit(DRAKE_NVP(external_force_scaling));

    a->Visit(DRAKE_NVP(visualize_pose_trace));
    a->Visit(DRAKE_NVP(visualize_center_of_mass_plan));
    a->Visit(DRAKE_NVP(visualize_c3_forces));
    a->Visit(DRAKE_NVP(visualize_workspace));
  }
};
