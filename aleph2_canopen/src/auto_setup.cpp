#include "aleph2_canopen/motor.hpp"

namespace aleph2_canopen
{

AutoSetupMode::AutoSetupMode(std::shared_ptr<LelyMotionControllerBridge> driver)
  : ros2_canopen::Mode(MotorNanotec::Auto_Setup)
{
  this->driver = driver;
}

bool AutoSetupMode::start()
{
  execute_ = false;
  return read(0);
}

bool AutoSetupMode::read(const uint16_t &sw)
{

  return true;
}

bool AutoSetupMode::write(OpModeAccesser &cw)
{
  cw = 0;
  if (execute_)
  {
    cw.set(CW_StartAutoSetup);
    return true;
  }
  return true;
}

bool AutoSetupMode::executeAutoSetup()
{
  execute_ = true;
  return true;
}


} // namespace aleph2_canopen