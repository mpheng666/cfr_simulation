#include "cfr_state_machine/cfr_sm_server.hpp"

namespace cfr_sm {
    CFRControlServer::CFRControlServer()
        : Node("cfr_sm_node")
    {
    }

    void CFRControlServer::start()
    {
        cfr_sm_.initiate();

        init_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/init_service", [this]([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventInit());
            res->success = true;
            res->message = "Init successfully!";
        });

        start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/start_service",
        [this]([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventStart());
            res->success = true;
            res->message = "Start successfully!";
        });

        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/stop_service", [this]([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventStop());
            res->success = true;
            res->message = "Stop successfully!";
        });

        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reset_service",
        [this]([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            cfr_sm_.process_event(cfr_sm::EventReset());
            res->success = true;
            res->message = "Reset successfully!";
        });
    }

} // namespace cfr_sm