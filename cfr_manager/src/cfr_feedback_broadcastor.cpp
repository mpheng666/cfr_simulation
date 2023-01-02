#include "cfr_manager/cfr_feedback_broadcastor.hpp"

namespace cfr_manager {
    CFRFeedBackBroadcastor::CFRFeedBackBroadcastor(const int port, const double frequency)
        : feedback_server_(port)
        , broadcast_frequency_(frequency)
    {
        
    }

    void CFRFeedBackBroadcastor::start()
    {
        std::thread(&cfr_feedback_server::CFRFeedBackServer::run, &feedback_server_)
        .detach();
    }

    void CFRFeedBackBroadcastor::updateMsg(const CFRFeedbackMsg& msg) {
        feedback_msg_ = msg;
        feedback_server_.write(feedback_msg_.getMsgString());
    }

    void CFRFeedBackBroadcastor::startBroadcastTimer() {
    }
} // namespace cfr_manager