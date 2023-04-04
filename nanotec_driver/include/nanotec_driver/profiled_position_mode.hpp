// Copyright (c) 2022-2023 ROS2 Canopen Stack Contributors
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

#include <cstdint>
#include <memory>
#include <limits>

#include "lely_nanotec_bridge.hpp"
#include "mode.hpp"

namespace nanotec_driver
{
class ProfiledPositionMode : public ModeTargetHelper<int32_t>
{
  const uint16_t index = 0x607A;
  std::shared_ptr<LelyNanotecBridge> driver;
  std::shared_ptr<RemoteObject> obj;

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
  explicit ProfiledPositionMode(std::shared_ptr<LelyNanotecBridge> driver)
  : ModeTargetHelper(Mode::Profiled_Position)
  {
    this->driver = driver;
    obj = driver->create_remote_obj(index, 0U, CODataTypes::COData32);
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
          driver->set_remote_obj(obj, target);
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
