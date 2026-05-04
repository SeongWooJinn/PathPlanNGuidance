#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "base_controller.h"
#include "acados_c/ocp_nlp_interface.h"

// extern "C" {
//     #include "acados_c/ocp_nlp_interface.h"
// }

// BaseController를 상속
class MpcController : public BaseController 
{
protected:
    ocp_nlp_config* nlp_config_ = nullptr;
    ocp_nlp_dims* nlp_dims_ = nullptr;
    ocp_nlp_in* nlp_in_ = nullptr;
    ocp_nlp_out* nlp_out_ = nullptr;
    ocp_nlp_solver *nlp_solver_ = nullptr;

    int N_;                 // 예측 호라이즌
    int NX_;                // 상태 변수 개수 
    int NU_;                // 제어 입력 개수 
    int NBX0_;              // 

    // for stats 
    // double min_time_ = 1e12;
    double kkt_norm_inf_;
    double elapsed_time_;
    int sqp_iter_;

public:
    virtual ~MpcController() = default;

///////////// 공통 로직 (BaseController의 가상 함수 구현) /////////////
    // 초기 예상 궤적 
    void setInitialGuess(double* x_init, double* u_init) override {
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u_init);
        }
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, N_, "x", x_init);
    }
    // 현재 로봇의 물리적 위치(상태) 제약 세팅
    void setInitialState(double* lbx0, double* ubx0) override {
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", lbx0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", ubx0);
    }

    void setTargetTrajectory(const double* yref, const double* yref_e) override {
        for (int i = 0; i < N_; i++) {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", const_cast<double*>(yref));
        }
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "yref", const_cast<double*>(yref_e));
    }

    void getControlInput(double* u_out) override {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u_out);
    }

    void getPredictedState(int step, double* x_pred) override {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, step, "x", x_pred);
    }
///////////// 공통 로직 (BaseController의 가상 함수 구현) /////////////


////////////// 자식에서 반드시 구현해야 할 함수 /////////////
    virtual void freeSolver() = 0;
    virtual void printStats() {

        // get solution
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf_);
        ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter_);

        printf("\nSolver info:\n");
        printf(" SQP iterations %2d\n solve %f [ms]\n KKT %e\n",
            sqp_iter_, elapsed_time_*1000, kkt_norm_inf_);

    }
////////////// 자식에서 반드시 구현해야 할 함수 /////////////

};

#endif