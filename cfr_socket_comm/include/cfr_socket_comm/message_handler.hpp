#ifndef CFR_SOCKET_COMM_MESSAGE_HANDLER_HPP_
#define CFR_SOCKET_COMM_MESSAGE_HANDLER_HPP_

#include "cfr_state_machine/cfr_sm_client.hpp"
#include <string>
#include <vector>

namespace cfr_socket_comm {

    class MessageHandler {
    public:
        MessageHandler() = default;
        static std::string handleMessage(const std::string& msg);

    private:
        const std::string HOST_NAME_{"CFR"};
    };
} // namespace cfr_socket_comm

#endif