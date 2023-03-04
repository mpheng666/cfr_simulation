#include "cfr_mpc/cfr_mpc.hpp"

namespace cfr_mpc {
    CFRMPC::CFRMPC()
        : Node("cfr_mpc")
        , mpc_compute_cb_timer_(
          this->create_wall_timer(100ms, std::bind(&CFRMPC::MPCComputeCb, this)))
    {
    }

    CFRMPC::~CFRMPC()
    {
        // mpcmoveCodeGeneration_terminate();
    }

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
        argInit_struct4_T(&statedata); // set 0
        r = argInit_struct5_T();       // set 0
        r.signals.ref[2] = 0.5;        // set ref to 0.5
        printf("output r is %f, %f %f\n", r.signals.ref[0], r.signals.ref[1],
               r.signals.ref[2]);
        // statedata = plant[3], disturbance[3], lastmove[3], covariance[36], ia[72]
        // r = signal.ym[3], signal.ref[3]
        // u = double[3]
        // Info = Uopt[33], Yopt[33], Xopt[66], Topt[11], Slack, Iterations, Cost;
        coder::mpcmoveCodeGeneration(&statedata, &r, u, &Info);
        printf("output u is %f, %f %f\n", u[0], u[1], u[2]);
    }

    void CFRMPC::argInit_3x1_real_T(double result[3])
    {
        // Loop over the array to initialize each element.
        for (int idx0{0}; idx0 < 3; idx0++) {
            // Set the value of the array element.
            // Change this value to the value that the application requires.
            result[idx0] = argInit_real_T();
        }
    }

    void CFRMPC::argInit_6x6_real_T(double result[36])
    {
        // Loop over the array to initialize each element.
        for (int idx0{0}; idx0 < 6; idx0++) {
            for (int idx1{0}; idx1 < 6; idx1++) {
                // Set the value of the array element.
                // Change this value to the value that the application requires.
                result[idx0 + 6 * idx1] = argInit_real_T();
            }
        }
    }

    void CFRMPC::argInit_72x1_boolean_T(boolean_T result[72])
    {
        // Loop over the array to initialize each element.
        for (int idx0{0}; idx0 < 72; idx0++) {
            // Set the value of the array element.
            // Change this value to the value that the application requires.
            result[idx0] = argInit_boolean_T();
        }
    }

    boolean_T CFRMPC::argInit_boolean_T() { return false; }

    double CFRMPC::argInit_real_T() { return 0.0; }

    void CFRMPC::argInit_struct4_T(struct4_T* result)
    {
        argInit_3x1_real_T(result->Plant);      // set all fields to double zero
        argInit_6x6_real_T(result->Covariance); // set all fields to double zero
        argInit_72x1_boolean_T(result->iA);     // set all fields to false
        // set all disturbance and lastmove to plant
        result->Disturbance[0] = result->Plant[0];
        result->Disturbance[1] = result->Plant[1];
        result->Disturbance[2] = result->Plant[2];
        result->LastMove[0] = result->Plant[0];
        result->LastMove[1] = result->Plant[1];
        result->LastMove[2] = result->Plant[2];
    }

    struct5_T CFRMPC::argInit_struct5_T()
    {
        struct5_T result;
        // Set the value of each structure field.
        // Change this value to the value that the application requires.
        result.signals = argInit_struct6_T();

        return result;
    }

    struct6_T CFRMPC::argInit_struct6_T()
    {
        struct6_T result;
        // Set the value of each structure field.
        // Change this value to the value that the application requires.
        // result.ref[1]=1;
        // printf("result is %f, %f %f\n",result.ref[0],result.ref[1],result.ref[2]);
        argInit_3x1_real_T(result.ym);
        argInit_3x1_real_T(result.ref);
        return result;
    }

} // namespace cfr_mpc