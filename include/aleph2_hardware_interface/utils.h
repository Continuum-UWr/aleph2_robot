#ifndef ALEPH2_HARDWARE_INTERFACE_UTILS_H
#define ALEPH2_HARDWARE_INTERFACE_UTILS_H

#include <map>
#include <string>
#include "ros/ros.h"
#include "xmlrpcpp/XmlRpcValue.h"

void LoadNanotecParametersFromStruct(std::map<std::string, int64_t>& parameters, XmlRpc::XmlRpcValue str)
{
    for (auto& param : str)
    {
        if (param.second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            for (auto& subparam : param.second)
            {
                ROS_ASSERT(subparam.second.getType() == XmlRpc::XmlRpcValue::TypeInt);
                std::string key = param.first + "/" + subparam.first;
                int64_t value = static_cast<int64_t>((int)subparam.second);
                parameters[key] = value;
            }
        }
        else
        {
            ROS_ASSERT(param.second.getType() == XmlRpc::XmlRpcValue::TypeInt);
            const std::string& key = param.first;
            int64_t value = static_cast<int64_t>((int)param.second);
            parameters[key] = value;
        }
    }
}

std::string LoadOptionalRubiFieldFromStruct(const std::string field_name, XmlRpc::XmlRpcValue& str)
{
    if (str.hasMember(field_name))
    {
        ROS_ASSERT(str[field_name].getType() == XmlRpc::XmlRpcValue::TypeString);
        return static_cast<std::string>(str[field_name]);
    }
    return "";
}

#endif