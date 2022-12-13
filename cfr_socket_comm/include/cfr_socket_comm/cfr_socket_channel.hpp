#ifndef CFR_SOCKET_CHANNEL_HPP_
#define CFR_SOCKET_CHANNEL_HPP_

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <set>
#include <memory>

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

namespace cfr_socket_comm {

    class SocketSubscriber {
        virtual ~SocketSubscriber()
        {

        }
        virtual void deliver(const std::string& msg) = 0;
    };

    typedef std::shared_ptr<SocketSubscriber> SockSubPtr;

    class CFRSocketChannel {
    public:
        void join(SockSubPtr subscriber) { sub_ptrs_.insert(subscriber); };
        void leave(SockSubPtr subscriber){sub_ptrs_.erase(subscriber)};

    private:
        std::set<SockSubPtr> sub_ptrs_;
    };
} // namespace cfr_socket_comm

#endif