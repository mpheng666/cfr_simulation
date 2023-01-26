#ifndef CFR_PROTOCOL_INTERFACES_PROTOCOL_SERVER_HPP_
#define CFR_PROTOCOL_INTERFACES_PROTOCOL_SERVER_HPP_

#include "cfr_protocol_interfaces/protocol.hpp"
#include "cfr_protocol_interfaces/srv/trigger_service.hpp"

#include "rclcpp/rclcpp.hpp"

namespace cfr_protocol
{
    class ProtocolServer
    {
        rclcpp::Service<cfr_protocol_interfaces::srv::TriggerService>::SharedPtr protocol_service_;

    };
}

#endif