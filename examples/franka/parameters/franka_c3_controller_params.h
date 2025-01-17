#pragma once

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct FrankaC3ControllerParams {
  int scene_index;
  std::vector<std::string> c3_options_file;
  std::vector<std::string> c3_scene_file;
  std::string osqp_settings_file;

  bool include_end_effector_orientation;
  double target_frequency;

  double near_target_threshold;
  std::vector<Eigen::Vector3d> first_target;
  std::vector<Eigen::Vector3d> second_target;
  std::vector<Eigen::Vector3d> third_target;
  double x_scale;
  double y_scale;
  double z_scale;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(c3_options_file));
    a->Visit(DRAKE_NVP(c3_scene_file));
    a->Visit(DRAKE_NVP(osqp_settings_file));

    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(target_frequency));
    a->Visit(DRAKE_NVP(scene_index));

    a->Visit(DRAKE_NVP(first_target));
    a->Visit(DRAKE_NVP(second_target));
    a->Visit(DRAKE_NVP(third_target));
    a->Visit(DRAKE_NVP(x_scale));
    a->Visit(DRAKE_NVP(y_scale));
    a->Visit(DRAKE_NVP(z_scale));
    a->Visit(DRAKE_NVP(near_target_threshold));
  }
};