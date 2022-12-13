#include "nanotec_driver/auto_setup.hpp"
#include "nanotec_driver/motor.hpp"

namespace nanotec_driver
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

bool AutoSetupMode::read(const uint16_t & sw)
{
  std::scoped_lock lock(status_mutex_);
  uint16_t old = status_;
  status_ = sw & (1 << SW_AutoSetupCompleted);
  if (old != status_) {
    status_cond_.notify_all();
  }
  return true;
}

bool AutoSetupMode::write(OpModeAccesser & cw)
{
  cw = 0;
  if (execute_) {
    cw.set(CW_StartAutoSetup);
    return true;
  }
  return true;
}

bool AutoSetupMode::executeAutoSetup()
{
  std::unique_lock lock(status_mutex_);
  bool okay = true;

  execute_ = true;

  std::chrono::steady_clock::time_point finish_time =
    std::chrono::steady_clock::now() + std::chrono::seconds(30);
  if (!status_cond_.wait_until(
      lock, finish_time, [this] {
        return status_ & (1 << SW_AutoSetupCompleted);
      }))
  {
    std::cout << "Auto Setup failed!" << std::endl;
    okay = false;
  } else {
    std::cout << "Auto Setup completed!" << std::endl;
  }

  execute_ = false;

  return okay;
}


} // namespace nanotec_driver
