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

#pragma once

#include <memory>

#include "mode.hpp"

namespace nanotec_driver
{

class AutoSetupMode : public ModeHelper
{
protected:
  std::shared_ptr<LelyNanotecBridge> driver;
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
  explicit AutoSetupMode(std::shared_ptr<LelyNanotecBridge> driver);

  bool start() override;
  bool read(const uint16_t & sw) override;
  bool write(OpModeAccesser & cw) override;
  bool execute_auto_setup();
};

}  // namespace nanotec_driver
