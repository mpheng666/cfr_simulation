#include "cfr_socket_comm/message_handler.hpp"

namespace cfr_socket_comm {

    MessageHandler::MessageHandler()
    // : cfr_state_machine_client_(
    //   std::make_shared<cfr_sm_client::CFRSMClient>(CLIENT_NODE_NAME_))
    {
        rclcpp::init(0, nullptr);
        cfr_state_machine_client_ =
        std::make_shared<cfr_sm_client::CFRSMClient>(CLIENT_NODE_NAME_);
        std::thread([&]() { rclcpp::spin(cfr_state_machine_client_); }).detach();
    }

    MessageHandler::~MessageHandler() { rclcpp::shutdown(); }

    void MessageHandler::handleMessage(const std::string& msg)
    {
        auto result = split_string(msg, DELIMITER_);
        takeAction(result);
    }

    std::vector<std::string> MessageHandler::split_string(const std::string& s,
                                                          const std::string& delimiter)
    {
        std::size_t pos_start = 0, pos_end, delim_len = delimiter.length();
        std::string token;
        std::vector<std::string> res;

        while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
            token = s.substr(pos_start, pos_end - pos_start);
            pos_start = pos_end + delim_len;
            res.push_back(token);
        }

        res.push_back(s.substr(pos_start));
        return res;
    }

    void MessageHandler::takeAction(const std::vector<std::string>& token)
    {

        if (token.at(0) == "STATUS") {
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &*cfr_state_machine_client_, token.at(0));
        }
        else if (token.at(0) == "FB") {
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &*cfr_state_machine_client_, token.at(0));
        }
        else if (token.at(0) == "INIT") {
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &*cfr_state_machine_client_, token.at(0));
        }
        else if (token.at(0) == "START") {
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &*cfr_state_machine_client_, token.at(0));
        }
        else if (token.at(0) == "STOP") {
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &*cfr_state_machine_client_, token.at(0));
        }
        else if (token.at(0) == "RESET") {
            auto a_service = std::async(&cfr_sm_client::CFRSMClient::callCFRService,
                                        &*cfr_state_machine_client_, token.at(0));
        }
        else if (token.at(0) == "CTRL") {
            if (token.size() == 5) {
                std::vector<float> command_vec;
                for (auto it = token.begin() + 1; it != token.end(); it++) {
                    command_vec.push_back(std::stof(*it));
                }
                cfr_state_machine_client_->callControl(command_vec);
            }
        }
        else {
            std::cout << "Discard invalid message \n";
        }
    }
} // namespace cfr_socket_comm