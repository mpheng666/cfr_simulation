#ifndef CFR_MANAGER_CFR_FEEDBACK_BROADCASTPR_HPP_
#define CFR_MANAGER_CFR_FEEDBACK_BROADCASTPR_HPP_

#include "cfr_feedback_server/cfr_feedback_server.hpp"
#include "cfr_manager/cfr_feedback_msg.hpp"

namespace cfr_manager {
    class CFRFeedBackBroadcastor {
    public:
        CFRFeedBackBroadcastor(/* const */ int port, /* const */ double frequency);
        void start();
        void updateMsg(const CFRFeedbackMsg& msg);
        void startBroadcastTimer();

    private:
        cfr_feedback_server::CFRFeedBackServer feedback_server_;
        CFRFeedbackMsg feedback_msg_{};
        double broadcast_frequency_{10.0};
    };
} // namespace cfr_manager

#endif