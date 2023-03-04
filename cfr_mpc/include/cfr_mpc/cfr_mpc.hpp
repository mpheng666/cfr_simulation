#ifndef _CFR_MPC_CFR_MPC_HPP_
#define _CFR_MPC_CFR_MPC_HPP_

// platform type definition
#include "lib/rtwtypes.h"

// platform compiler definition
#include "lib/mpcmoveCodeGeneration_spec.h"

// optimization function
#include "lib/qpkwik.h"

// actual mpc
#include "lib/mpcmoveCodeGeneration.h"

#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Type Definitions
// struct struct10_T {
//     double Uopt[33];
//     double Yopt[33];
//     double Xopt[66];
//     double Topt[11];
//     double Slack;
//     double Iterations;
//     double Cost;
// };

// struct struct4_T {
//     double Plant[3];
//     double Disturbance[3];
//     double LastMove[3];
//     double Covariance[36];
//     boolean_T iA[72];
// };

// struct struct6_T {
//     double ym[3];
//     double ref[3];
// };

// struct struct5_T {
//     struct6_T signals;
// };

namespace cfr_mpc {
    class CFRMPC : public rclcpp::Node {
    public:
        CFRMPC();
        ~CFRMPC();

    private:
        rclcpp::TimerBase::SharedPtr mpc_compute_cb_timer_;

        void argInit_3x1_real_T(double result[3]);
        void argInit_6x6_real_T(double result[36]);
        void argInit_72x1_boolean_T(boolean_T result[72]);
        boolean_T argInit_boolean_T();
        double argInit_real_T();
        void argInit_struct4_T(struct4_T* result);
        struct5_T argInit_struct5_T();
        struct6_T argInit_struct6_T();

        void MPCComputeCb();
    };

} // namespace cfr_mpc

#endif