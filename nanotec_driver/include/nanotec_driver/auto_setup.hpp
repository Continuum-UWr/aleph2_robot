#pragma once

#include <memory>

#include "nanotec_driver/mode.hpp"

namespace nanotec_driver
{

class AutoSetupMode : public ModeHelper
{
protected:
  std::shared_ptr<ros2_canopen::LelyDriverBridge> driver;
  std::atomic_bool execute_;

  std::mutex status_mutex_;
  std::condition_variable status_cond_;
  uint16_t status_;

  enum SW_bits
  {
    SW_AutoSetupCompleted = State402::SW_Operation_mode_specific0,
  };

  enum CW_bits
  {
    CW_StartAutoSetup = Command402::CW_Operation_mode_specific0,
  };

public:
  explicit AutoSetupMode(std::shared_ptr<ros2_canopen::LelyDriverBridge> driver);

  bool start() override;
  bool read(const uint16_t & sw) override;
  bool write(OpModeAccesser & cw) override;
  bool execute_auto_setup();
};

}  // namespace nanotec_driver
