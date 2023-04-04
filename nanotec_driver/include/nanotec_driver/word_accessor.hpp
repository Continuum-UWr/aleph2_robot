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

namespace nanotec_driver
{
template<uint16_t MASK>
class WordAccessor
{
  uint16_t & word_;

public:
  explicit WordAccessor(uint16_t & word)
  : word_(word) {}
  bool set(uint8_t bit)
  {
    uint16_t val = MASK & (1 << bit);
    word_ |= val;
    return val;
  }
  bool reset(uint8_t bit)
  {
    uint16_t val = MASK & (1 << bit);
    word_ &= ~val;
    return val;
  }
  bool get(uint8_t bit) const {return word_ & (1 << bit);}
  uint16_t get() const {return word_ & MASK;}
  WordAccessor & operator=(const uint16_t & val)
  {
    word_ = (word_ & ~MASK) | (val & MASK);
    return *this;
  }
};
}  // namespace nanotec_driver
