// Copyright (c) 2022-2023 ROS2 Canopen Stack Contributors
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

#include "nanotec_driver/lely_nanotec_bridge.hpp"

namespace nanotec_driver
{

LelyNanotecBridge::LelyNanotecBridge(
  ev_exec_t * exec, canopen::AsyncMaster & master, uint8_t id, std::string name)
: LelyDriverBridge(exec, master, id, name)
{}

std::shared_ptr<RemoteObject> LelyNanotecBridge::create_remote_obj(
  uint16_t index, uint8_t subindex, CODataTypes type)
{
  RemoteObject obj = {index, subindex, 0, type, false, false, true};
  for (auto it = objs.begin(); it != objs.end(); ++it) {
    if (((*it)->index == index) && ((*it)->subindex == subindex) && ((*it)->type == type)) {
      return *it;
    }
  }
  std::shared_ptr<RemoteObject> objp = std::make_shared<RemoteObject>(obj);
  objs.push_back(objp);
  return objp;
}

void LelyNanotecBridge::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
  for (auto it = objs.begin(); it != objs.end(); ++it) {
    std::shared_ptr<RemoteObject> obj = *it;
    if (obj->index == idx && obj->subindex == subidx) {
      if (obj->type == CODataTypes::COData8) {
        obj->data = rpdo_mapped[idx][subidx].Read<uint8_t>();
      } else if (obj->type == CODataTypes::COData16) {
        obj->data = rpdo_mapped[idx][subidx].Read<uint16_t>();
      } else if (obj->type == CODataTypes::COData32) {
        obj->data = rpdo_mapped[idx][subidx].Read<uint32_t>();
      }
      break;
    }
  }
}

void LelyNanotecBridge::validate_objs()
{
  for (auto it = objs.begin(); it != objs.end(); ++it) {
    std::shared_ptr<RemoteObject> obj = *it;

    try {
      switch (obj->type) {
        case CODataTypes::COData8:
          this->tpdo_mapped[obj->index][obj->subindex].Read<uint8_t>();
          break;
        case CODataTypes::COData16:
          this->tpdo_mapped[obj->index][obj->subindex].Read<uint16_t>();
          break;
        case CODataTypes::COData32:
          this->tpdo_mapped[obj->index][obj->subindex].Read<uint32_t>();
          break;
        default:
          throw lely::canopen::SdoError(
                  this->get_id(), obj->index, obj->subindex,
                  std::make_error_code(std::errc::function_not_supported),
                  "Unknown used, type must be 8, 16 or 32.");
          break;
      }
      obj->tpdo_mapped = true;
    } catch (lely::canopen::SdoError & e) {
      obj->tpdo_mapped = false;
    }

    try {
      switch (obj->type) {
        case CODataTypes::COData8:
          obj->rpdo_mapped = this->rpdo_mapped[obj->index][obj->subindex].Read<uint8_t>();
          break;
        case CODataTypes::COData16:
          obj->rpdo_mapped = this->rpdo_mapped[obj->index][obj->subindex].Read<uint16_t>();
          break;
        case CODataTypes::COData32:
          obj->rpdo_mapped = this->rpdo_mapped[obj->index][obj->subindex].Read<uint32_t>();
          break;
        default:
          throw lely::canopen::SdoError(
                  this->get_id(), obj->index, obj->subindex,
                  std::make_error_code(std::errc::function_not_supported),
                  "Unknown used, type must be 8, 16 or 32.");
          break;
      }
      obj->rpdo_mapped = true;
    } catch (lely::canopen::SdoError & e) {
      obj->rpdo_mapped = false;
    }

    try {
      switch (obj->type) {
        case CODataTypes::COData8:
          obj->data = get_remote_obj<uint8_t>(obj);
          break;
        case CODataTypes::COData16:
          obj->data = get_remote_obj<uint16_t>(obj);
          break;
        case CODataTypes::COData32:
          obj->data = get_remote_obj<uint32_t>(obj);
          break;
        default:
          break;
      }
    } catch (lely::canopen::SdoError & e) {
      RCLCPP_ERROR(rclcpp::get_logger(name_), "Could not fetch data. %s", e.what());
    }
    RCLCPP_INFO(
      rclcpp::get_logger(name_),
      "Initialised object: node_id %hu, index %x, subindex %hhu, data %u, RPDO: %s, TPDO: %s",
      this->get_id(), obj->index, obj->subindex, obj->data, (obj->rpdo_mapped ? "yes" : "no"),
      (obj->tpdo_mapped ? "yes" : "no"));
  }
}

void LelyNanotecBridge::trigger_tpdo_event(std::shared_ptr<RemoteObject> obj)
{
  if (obj->tpdo_mapped) {
    this->tpdo_mapped[obj->index][obj->subindex].WriteEvent();
  }
}

}  // namespace nanotec_driver
