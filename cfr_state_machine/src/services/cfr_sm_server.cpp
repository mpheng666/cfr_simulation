
#include "cfr_state_machine/services/cfr_sm_server.hpp"

namespace cfr_sm {
    CFRControlServer::CFRControlServer()
        : Node("cfr_sm_node")
        , control_sub_(this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "control_channel", 10, std::bind(&CFRControlServer::controlCallback, this, _1)))
    {
    }

    void CFRControlServer::start()
    {
        std::cout << "START CFR SM SERVER!! \n";
        cfr_sm_.initiate();

        status_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/status_service",
        [this](
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventStatus());
            res->success = true;
            res->message = "Get status successfully!";
        });

        feedback_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/feedback_service",
        [this](
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventFeedBack());
            res->success = true;
            res->message = "Get feedback successfully!";
        });

        init_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/init_service",
        [this](
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventInit());
            res->success = true;
            res->message = "Init successfully!";
        });

        start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/start_service",
        [this](
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventStart());
            res->success = true;
            res->message = "Start successfully!";
        });

        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/stop_service",
        [this](
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventStop());
            res->success = true;
            res->message = "Stop successfully!";
        });

        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reset_service",
        [this](
        [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventReset());
            res->success = true;
            res->message = "Reset successfully!";
        });
    }

    void CFRControlServer::controlCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::cout << "server do work \n";
        for(const auto& m : msg->data)
        {
            std::cout << m << "|";
        }
        std::cout << "\n";

        cfr_sm_.process_event(
        cfr_sm::EventControl(msg->data[0], msg->data[1], msg->data[2], msg->data[3]));
    }

} // namespace cfr_sm