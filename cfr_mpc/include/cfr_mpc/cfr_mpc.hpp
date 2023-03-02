#ifndef _CFR_MPC_CFR_MPC_HPP_
#define _CFR_MPC_CFR_MPC_HPP_

// platform type definition
#include "rtwtypes.h"

// platform compiler definition
#include "lib/mpcmoveCodeGeneration_spec.h"

// mpc type definition
#include "lib/mpcmoveCodeGeneration_types.h"

// optimization function
#include "lib/qpkwik.h"


#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cfr_mpc {

    class CFRMPC : public rclcpp::Node {
    public:
        CFRMPC();
        ~CFRMPC();

    private:
        rclcpp::WallTimer mpc_compute_cb_timer_;

        void MPCComputeCb();
    };

} // namespace cfr_mpc

#endif