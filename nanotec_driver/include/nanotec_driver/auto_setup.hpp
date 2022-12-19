#pragma once

#include <memory>

#include "canopen_402_driver/motor.hpp"

namespace nanotec_driver
{

class AutoSetupMode : public ros2_canopen::Mode
{
protected:
  std::shared_ptr<LelyMotionControllerBridge> driver;
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
  explicit AutoSetupMode(std::shared_ptr<LelyMotionControllerBridge> driver);

  bool start() override;
  bool read(const uint16_t & sw) override;
  bool write(OpModeAccesser & cw) override;
  bool executeAutoSetup();
};

}  // namespace nanotec_driver
