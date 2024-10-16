#pragma once

struct PushbotLcmChannels {
  std::string pushbot_state_channel;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(pushbot_state_channel));
  }
};
