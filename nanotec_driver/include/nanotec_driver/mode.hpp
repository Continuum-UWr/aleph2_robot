#pragma once

#include <cstdint>
#include <memory>
#include <limits>

#include "canopen_core/exchange.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"

#include "nanotec_driver/command.hpp"
#include "nanotec_driver/state.hpp"
#include "nanotec_driver/word_accessor.hpp"

namespace nanotec_driver
{
enum class Mode : int8_t
{
  Auto_Setup = -2,
  No_Mode = 0,
  Profiled_Position = 1,
  Profiled_Velocity = 3,
  Profiled_Torque = 4,
};

class ModeHelper
{
public:
  const Mode mode;
  explicit ModeHelper(Mode mode)
  : mode(mode) {}
  typedef WordAccessor<
      (1 << Command402::CW_Operation_mode_specific0) |
      (1 << Command402::CW_Operation_mode_specific1) |
      (1 << Command402::CW_Operation_mode_specific2) |
      (1 << Command402::CW_Operation_mode_specific3)>
    OpModeAccesser;
  virtual bool start() = 0;
  virtual bool read(__attribute__((unused)) const uint16_t & sw) {return true;}
  virtual bool write(OpModeAccesser & cw) = 0;
  virtual bool setTarget(__attribute__((unused)) const double & val) {return false;}
  virtual ~ModeHelper() {}
};

template<typename T>
class ModeTargetHelper : public ModeHelper
{
  T target_;
  std::atomic<bool> has_target_;

public:
  explicit ModeTargetHelper(Mode mode)
  : ModeHelper(mode) {}
  bool hasTarget() {return has_target_;}
  T getTarget() {return target_;}
  virtual bool setTarget(const double & val)
  {
    if (std::isnan(val)) {
      return false;
    }

    using boost::numeric_cast;
    using boost::numeric::negative_overflow;
    using boost::numeric::positive_overflow;

    try {
      target_ = numeric_cast<T>(val);
    } catch (negative_overflow &) {
      target_ = std::numeric_limits<T>::min();
    } catch (positive_overflow &) {
      target_ = std::numeric_limits<T>::max();
    } catch (...) {
      return false;
    }

    has_target_ = true;
    return true;
  }
  virtual bool start()
  {
    has_target_ = false;
    return true;
  }
};

template<Mode MODE, typename TYPE, uint16_t OBJ, uint8_t SUB, uint16_t CW_MASK>
class ModeForwardHelper : public ModeTargetHelper<TYPE>
{
  std::shared_ptr<ros2_canopen::LelyDriverBridge> driver;

public:
  explicit ModeForwardHelper(std::shared_ptr<ros2_canopen::LelyDriverBridge> driver)
  : ModeTargetHelper<TYPE>(MODE)
  {
    this->driver = driver;
  }
  virtual bool write(ModeHelper::OpModeAccesser & cw)
  {
    if (this->hasTarget()) {
      cw = cw.get() | CW_MASK;

      driver->universal_set_value<TYPE>(OBJ, SUB, this->getTarget());
      return true;
    } else {
      cw = cw.get() & ~CW_MASK;
      return false;
    }
  }
};

typedef ModeForwardHelper<Mode::Profiled_Velocity, int32_t, 0x60FF, 0, 0> ProfiledVelocityMode;
typedef ModeForwardHelper<Mode::Profiled_Torque, int16_t, 0x6071, 0, 0> ProfiledTorqueMode;

}  // namespace nanotec_driver
