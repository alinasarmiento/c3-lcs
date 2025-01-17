#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct PushbotLcmChannels {
  std::string pushbot_state_channel;
  std::string pushbot_input_channel;
  std::string radio_channel;

  std::string c3_force_channel;
  std::string c3_debug_output_channel;
  std::string c3_target_state_channel;
  std::string c3_actual_state_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(pushbot_state_channel));
    a->Visit(DRAKE_NVP(pushbot_input_channel));
    a->Visit(DRAKE_NVP(radio_channel));
    a->Visit(DRAKE_NVP(c3_force_channel));
    a->Visit(DRAKE_NVP(c3_debug_output_channel));
    a->Visit(DRAKE_NVP(c3_target_state_channel));
    a->Visit(DRAKE_NVP(c3_actual_state_channel));
  }
};
