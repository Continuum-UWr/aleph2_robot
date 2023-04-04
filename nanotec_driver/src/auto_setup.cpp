// Copyright (c) 2023 Team Continuum
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "nanotec_driver/auto_setup.hpp"
#include "nanotec_driver/motor.hpp"

namespace nanotec_driver
{

AutoSetupMode::AutoSetupMode(std::shared_ptr<LelyNanotecBridge> driver)
: ModeHelper(Mode::Auto_Setup)
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

bool AutoSetupMode::execute_auto_setup()
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


}  // namespace nanotec_driver
