#pragma once

#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <boost/container/flat_map.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include "canopen_base_driver/lely_driver_bridge.hpp"

using ros2_canopen::CODataTypes;

namespace nanotec_driver
{
struct RemoteObject
{
  uint16_t index;
  uint8_t subindex;
  uint32_t data;
  CODataTypes type;
  bool tpdo_mapped;
  bool rpdo_mapped;
  bool valid;
};

class LelyNanotecBridge : public ros2_canopen::LelyDriverBridge
{
private:
  std::vector<std::shared_ptr<RemoteObject>> objs;

public:
  LelyNanotecBridge(
    ev_exec_t * exec, canopen::AsyncMaster & master, uint8_t id, std::string name);


  std::shared_ptr<RemoteObject> create_remote_obj(
    uint16_t index, uint8_t subindex, CODataTypes type);

  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;

  template<typename T>
  void set_remote_obj(std::shared_ptr<RemoteObject> obj, T data)
  {
    T data_ = data;
    std::memcpy(&(obj->data), &data_, sizeof(T));

    ros2_canopen::COData d = {obj->index, obj->subindex, obj->data, obj->type};
    if (!obj->tpdo_mapped) {
      auto f = this->async_sdo_write(d);
      f.wait();
    } else {
      this->tpdo_mapped[obj->index][obj->subindex] = data;
    }
  }

  template<typename T>
  T get_remote_obj(std::shared_ptr<RemoteObject> obj)
  {
    if (!obj->rpdo_mapped) {
      ros2_canopen::COData d = {obj->index, obj->subindex, 0U, obj->type};
      auto f = this->async_sdo_read(d);
      f.wait();
      try {
        obj->data = f.get().data_;
      } catch (std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger(name_), e.what());
        obj->valid = false;
      }
    }
    T data;
    std::memcpy(&data, &(obj->data), sizeof(T));
    return data;
  }

  template<typename T>
  T get_remote_obj_cached(std::shared_ptr<RemoteObject> obj)
  {
    T data;
    std::memcpy(&data, &(obj->data), sizeof(T));
    return data;
  }

  template<typename T>
  void set_remote_obj_cached(std::shared_ptr<RemoteObject> obj, const T data)
  {
    T data_ = data;
    std::memcpy(&(obj->data), &data_, sizeof(T));
  }

  void validate_objs();
  void trigger_tpdo_event(std::shared_ptr<RemoteObject> obj);
};
}  // namespace nanotec_driver
