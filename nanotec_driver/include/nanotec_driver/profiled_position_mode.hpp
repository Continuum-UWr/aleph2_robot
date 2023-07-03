#pragma once

#include <cstdint>
#include <memory>
#include <limits>

#include "nanotec_driver/mode.hpp"

namespace nanotec_driver
{
class ProfiledPositionMode : public ModeTargetHelper<int32_t>
{
  const uint16_t index = 0x607A;
  std::shared_ptr<ros2_canopen::LelyDriverBridge> driver;

  double last_target_;
  uint16_t sw_;

public:
  enum SW_masks
  {
    MASK_Reached = (1 << State402::SW_Target_reached),
    MASK_Acknowledged = (1 << State402::SW_Operation_mode_specific0),
    MASK_Error = (1 << State402::SW_Operation_mode_specific1),
  };
  enum CW_bits
  {
    CW_NewPoint = Command402::CW_Operation_mode_specific0,
    CW_Immediate = Command402::CW_Operation_mode_specific1,
    CW_Blending = Command402::CW_Operation_mode_specific3,
  };
  explicit ProfiledPositionMode(std::shared_ptr<ros2_canopen::LelyDriverBridge> driver)
  : ModeTargetHelper(Mode::Profiled_Position)
  {
    this->driver = driver;
  }

  virtual bool start()
  {
    sw_ = 0;
    last_target_ = std::numeric_limits<double>::quiet_NaN();
    return ModeTargetHelper::start();
  }
  virtual bool read(const uint16_t & sw)
  {
    sw_ = sw;
    return (sw & MASK_Error) == 0;
  }
  virtual bool write(OpModeAccesser & cw)
  {
    cw.set(CW_Immediate);
    if (hasTarget()) {
      int32_t target = getTarget();
      if ((sw_ & MASK_Acknowledged) == 0 && target != last_target_) {
        if (cw.get(CW_NewPoint)) {
          cw.reset(CW_NewPoint);  // reset if needed
        } else {
          driver->universal_set_value(index, 0x0, target);
          cw.set(CW_NewPoint);
          last_target_ = target;
        }
      } else if (sw_ & MASK_Acknowledged) {
        cw.reset(CW_NewPoint);
      }
      return true;
    }
    return false;
  }
};
}  // namespace nanotec_driver
