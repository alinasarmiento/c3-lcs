#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

// Currently this scene only defines static environment obstacles
struct PushbotSimSceneParams {
  std::vector<Eigen::Vector3d> wall1_position;
  std::vector<Eigen::Vector3d> wall2_position;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(wall1_position));
    a->Visit(DRAKE_NVP(wall2_position));

  }
};
