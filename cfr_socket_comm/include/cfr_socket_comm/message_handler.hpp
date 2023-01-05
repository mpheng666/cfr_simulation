#ifndef CFR_SOCKET_COMM_MESSAGE_HANDLER_HPP_
#define CFR_SOCKET_COMM_MESSAGE_HANDLER_HPP_

#include "cfr_state_machine/cfr_sm_client.hpp"
#include <string>
#include <vector>

namespace cfr_socket_comm {

    class MessageHandler {
    public:
        MessageHandler();
        ~MessageHandler();
        void handleMessage(const std::string& msg);

    private:
        const std::string HOST_NAME_{"CFR"};
        const std::string DELIMITER_{','};
        const std::string CLIENT_NODE_NAME_{"cfr_sm_client_node"};

        std::shared_ptr<cfr_sm_client::CFRSMClient> cfr_state_machine_client_;

        std::vector<std::string> split_string(const std::string& s,
                                              const std::string& delimiter);
        void takeAction(const std::vector<std::string>& token);
    };
} // namespace cfr_socket_comm

#endif