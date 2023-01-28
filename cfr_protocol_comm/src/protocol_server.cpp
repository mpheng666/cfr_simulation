#include "cfr_protocol_comm/protocol_server.hpp"

namespace cfr_protocol {
    ProtocolServer::ProtocolServer()
        : Node("cfr_protocol_server")
        , protocol_service_(
          this->create_service<cfr_protocol_interfaces::srv::TriggerService>(
          "~/cfr_protocol_service",
          std::bind(&ProtocolServer::protocolServiceCb, this, _1, _2)))
        , axis_control_sub_(this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/axis_control", 10, std::bind(&ProtocolServer::AxisControlCallback, this, _1)))
        , cmdvel_control_sub_(this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/cmdvel_control",
          10,
          std::bind(&ProtocolServer::CmdVelControlCallback, this, _1)))
    {
        robot_state_machine_.initiate();
    }

    void ProtocolServer::protocolServiceCb(
    const std::shared_ptr<cfr_protocol_interfaces::srv::TriggerService::Request> req,
    std::shared_ptr<cfr_protocol_interfaces::srv::TriggerService::Response> res)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Receive request: " << req->service_name
                                                                   << " with data size "
                                                                   << req->data.size());
        res->success = true;
    }

    void ProtocolServer::AxisControlCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
    }

    void ProtocolServer::CmdVelControlCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        robot_state_machine_.process_event(
        cfr_sm::EventControl(msg->data[0], msg->data[1], msg->data[2], msg->data[3]));
    }

    void ProtocolServer::getSMState()
    {
        for (auto it = robot_state_machine_.state_begin();
             it != robot_state_machine_.state_end(); ++it) {
            std::cout << typeid(*it).name() << "\n";
        }
    }

    void ProtocolServer::triggerEvent(const std::string& service_name,
                                      const std::vector<double>& data)
    {
        using namespace cfr_sm;
        if (service_name == "PSTATE") {
            getSMState();
        }
        else if (service_name == "MODE") {
            robot_state_machine_.process_event(EventGetMode());
        }
        else if (service_name == "NYPAUTO") {
            robot_state_machine_.process_event(EventSetMode(1));
        }
        else if (service_name == "FB") {
            robot_state_machine_.process_event(EventFeedBack(1));
        }
        else if (service_name == "INIT") {
            robot_state_machine_.process_event(EventInit());
        }
        else if (service_name == "START") {
            robot_state_machine_.process_event(EventStart());
        }
        else if (service_name == "STOP") {
            robot_state_machine_.process_event(EventStop());
        }
        else if (service_name == "STARTENGINE") {
            robot_state_machine_.process_event(EventStartEngine());
        }
        else if (service_name == "CTRL") {
            robot_state_machine_.process_event(EventControl(1,2,3,4));
        }
        else if (service_name == "AXIS") {
            robot_state_machine_.process_event(EventAxis(1,2,3,4));
        }
        else if (service_name == "BLADEANG") {
            robot_state_machine_.process_event(EventSetBladeAngle(1));
        }
        else if (service_name == "BEACONS") {
            robot_state_machine_.process_event(EventBeacons());
        }
    }

} // namespace cfr_protocol