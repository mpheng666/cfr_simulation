#include "cfr_mpc/cfr_mpc.hpp"

namespace cfr_mpc {
    CFRMPC::CFRMPC()
        : Node("cfr_mpc")
        , mpc_compute_cb_timer_(
          this->create_wall_timer(100ms, std::bind(&CFRMPC::MPCGenerationCb, this)))
    {
    }

    CFRMPC::~CFRMPC() { mpcmoveCodeGeneration_terminate(); }

    void CFRMPC::MPCComputeCb()
    {
        struct10_T Info;
        struct4_T statedata;
        struct5_T r;
        double u[3];
        // Initialize function 'mpcmoveCodeGeneration' input arguments.
        // Initialize function input argument 'statedata'.
        // Initialize function input argument 'onlinedata'.
        // Call the entry-point 'mpcmoveCodeGeneration'.
        argInit_struct4_T(&statedata);
        r = argInit_struct5_T();
        r.signals.ref[2] = 0.5;
        printf("output r is %f, %f %f\n", r.signals.ref[0], r.signals.ref[1],
               r.signals.ref[2]);
        coder::mpcmoveCodeGeneration(&statedata, &r, u, &Info);
        printf("output u is %f, %f %f\n", u[0], u[1], u[2]);
    }

} // namespace cfr_mpc