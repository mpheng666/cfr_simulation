#include "cfr_socket_comm/message_handler.hpp"

namespace cfr_socket_comm {

    std::string MessageHandler::handleMessage(const std::string& msg)
    {
        if (msg == "STATUS") {
            cfr_sm_client::CFRSMClient cfr_state_machine_client_("cfr_sm_client_node");
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &cfr_state_machine_client_, msg);
        }
        if (msg == "FB") {
            cfr_sm_client::CFRSMClient cfr_state_machine_client_("cfr_sm_client_node");
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &cfr_state_machine_client_, msg);
        }
        if (msg == "INIT") {
            cfr_sm_client::CFRSMClient cfr_state_machine_client_("cfr_sm_client_node");
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &cfr_state_machine_client_, msg);
        }
        if (msg == "START") {
            cfr_sm_client::CFRSMClient cfr_state_machine_client_("cfr_sm_client_node");
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &cfr_state_machine_client_, msg);
        }
        if (msg == "STOP") {
            cfr_sm_client::CFRSMClient cfr_state_machine_client_("cfr_sm_client_node");
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &cfr_state_machine_client_, msg);
        }
        if (msg == "RESET") {
            cfr_sm_client::CFRSMClient cfr_state_machine_client_("cfr_sm_client_node");
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &cfr_state_machine_client_, msg);
        }
        if (msg == "CTRL") {
        }
        else {
            std::cout << "Discard invalid message \n";
        }
        
    }

} // namespace cfr_socket_comm