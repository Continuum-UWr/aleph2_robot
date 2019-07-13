#pragma once

#include <string>

#include "value.h"
#include "types.h"

static kaco::Value int_to_value_of_type(int64_t val, kaco::Type type)
{
    switch(type)
    {
    case kaco::Type::uint8:
        return kaco::Value(static_cast<uint8_t>(val));
    case kaco::Type::uint16:
        return kaco::Value(static_cast<uint16_t>(val));
    case kaco::Type::uint32:
        return kaco::Value(static_cast<uint32_t>(val));
    case kaco::Type::uint64:
        return kaco::Value(static_cast<uint64_t>(val));
    case kaco::Type::int8:
        return kaco::Value(static_cast<int8_t>(val));
    case kaco::Type::int16:
        return kaco::Value(static_cast<int16_t>(val));
    case kaco::Type::int32:
        return kaco::Value(static_cast<int32_t>(val));
    case kaco::Type::int64:
        return kaco::Value(val);
    }
}