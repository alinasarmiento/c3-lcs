#pragma once

#include "solvers/c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct PushbotC3ControllerParams {
  int scene_index;
  std::vector<std::string> c3_options_file;
  std::string osqp_settings_file;

  double target_frequency;

  double near_target_threshold;
  std::vector<Eigen::VectorXd> first_target;
  double x_scale;
  double y_scale;
  double z_scale;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(c3_options_file));
    a->Visit(DRAKE_NVP(osqp_settings_file));

    a->Visit(DRAKE_NVP(target_frequency));
    a->Visit(DRAKE_NVP(scene_index));

    a->Visit(DRAKE_NVP(first_target));
    a->Visit(DRAKE_NVP(x_scale));
    a->Visit(DRAKE_NVP(y_scale));
    a->Visit(DRAKE_NVP(z_scale));
    a->Visit(DRAKE_NVP(near_target_threshold));
  }
};
