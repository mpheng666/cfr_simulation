#ifndef CFR_SOCKET_COMM_MESSAGE_HANDLER_HPP_
#define CFR_SOCKET_COMM_MESSAGE_HANDLER_HPP_

#include <string>
#include <vector>
#include "cfr_state_machine/cfr_sm_client.hpp"

namespace cfr_socket_comm {

    enum class request : int { STATUS = 0, FB, INIT, START, STOP, CTRL };

    enum class respond : int {

    };

    struct Message {
    };

    class MessageHandler {
    public:
        MessageHandler() = default;
        std::string handleMessage(const std::string& msg);

    private:
        const std::string HOST_NAME_{"CFR"};
    };
} // namespace cfr_socket_comm

#endif