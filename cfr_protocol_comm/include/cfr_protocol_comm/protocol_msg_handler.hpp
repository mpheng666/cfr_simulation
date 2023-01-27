#ifndef CFR_PROTOCOL_COMM_PROTOCOL_MSG_HANDLER_HPP_
#define CFR_PROTOCOL_COMM_PROTOCOL_MSG_HANDLER_HPP_

#include "cfr_protocol_comm/protocol.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

namespace cfr_protocol {
    using VecStr = std::vector<std::string>;
    class ProtocolMessageHandler {
    public:
        static bool process(const std::string& msg, VecStr& output_tokens)
        {
            output_tokens = splitString(msg, ",");
            output_tokens = removeWhiteSpaces(output_tokens);
            return isValidCommand(output_tokens);
        }

    private:
        static VecStr splitString(const std::string& input, const std::string& delimiter)
        {
            std::size_t pos_start = 0, pos_end, delim_len = delimiter.length();
            std::string token;
            std::vector<std::string> res;

            while ((pos_end = input.find(delimiter, pos_start)) != std::string::npos) {
                token = input.substr(pos_start, pos_end - pos_start);
                pos_start = pos_end + delim_len;
                if (token.size()) {
                    res.push_back(token);
                }
            }
            if((input.substr(pos_start)).size())
            {
                res.push_back(input.substr(pos_start));
            }

            return res;
        }

        static VecStr removeWhiteSpaces(VecStr& input)
        {
            for (auto& token : input) {
                token.erase(std::remove_if(token.begin(), token.end(),
                                           [](char c) {
                                               return (c == ' ' || c == '\n' ||
                                                       c == '\r' || c == '\t' ||
                                                       c == '\v' || c == '\f');
                                           }),
                            token.end());
            }
            return input;
        }

        static bool isValidCommand(const VecStr& input)
        {
            if (input.size() == 0)
                return false;
            Protocol protocol = ProtocolUtility::getProtocolFromString(input.at(0));

            return protocol != Protocol::UNKNOWN;
        }
    };
} // namespace cfr_protocol

#endif