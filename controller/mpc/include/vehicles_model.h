#ifndef VEHICLES_MODEL_H
#define VEHICLES_MODEL_H

#include "mpc_controller.h"

// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/external_function_interface.h"

#include "acados_solver_bicycle_model.h"
#include "acados_solver_parallel_model.h"
#include "acados_solver_spin_model.h"

// blasfeo
#include "blasfeo_d_aux_ext_dep.h"

// #define NP_GLOBAL   BICYCLE_MODEL_NP_GLOBAL


// struct RobotState {
//     double x, y, theta, v, delta;
// };

// struct ControlInput {
//     double a, delta_dot;
// };


//////////////////////////////////////////////////
/////////////// BicycleMode Class ///////////////
//////////////////////////////////////////////////
class BicycleMode : public MpcController {

private:
    bicycle_model_solver_capsule* capsule_ = nullptr;
    int status_;

public:
    BicycleMode() {
        N_ = BICYCLE_MODEL_N;   // 헤더에 정의된 값
        NX_ = BICYCLE_MODEL_NX;
        NU_ = BICYCLE_MODEL_NU;
        NBX0_ = BICYCLE_MODEL_NBX0;
    }

    ~BicycleMode() override {
        freeSolver();
    }

    bool initialize() override {
        capsule_ = bicycle_model_acados_create_capsule();
        // int status = spin_model_acados_create_with_discretization(capsule_, N_, nullptr);
        status_ = bicycle_model_acados_create_with_discretization(capsule_, N_, nullptr);
        
        if (status_ != 0) return false;

        // 부모(MpcController)의 공통 포인터에 캡슐 연결
        nlp_config_ = bicycle_model_acados_get_nlp_config(capsule_);
        nlp_dims_   = bicycle_model_acados_get_nlp_dims(capsule_);
        nlp_in_     = bicycle_model_acados_get_nlp_in(capsule_);
        nlp_out_    = bicycle_model_acados_get_nlp_out(capsule_);
        nlp_solver_ = bicycle_model_acados_get_nlp_solver(capsule_);

        return true;
    }

    bool solve() override {
        status_ = bicycle_model_acados_solve(capsule_);

        ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time_);
        return (status_ == 0);
    }

    void freeSolver() override {
        if (capsule_ != nullptr) {
            bicycle_model_acados_free(capsule_);
            if (status_) {
                printf("bicycle_model_acados_free() returned status %d. \n", status_);
            }

            bicycle_model_acados_free_capsule(capsule_);
            if (status_) {
                printf("bicycle_model_acados_free_capsule() returned status %d. \n", status_);
            }
            capsule_ = nullptr;
        }
    }

    void printStats() override {

        MpcController::printStats();
        bicycle_model_acados_print_stats(capsule_);

    }
};

///////////////////////////////////////////////////
///////////////// ParallelMode Class ///////////////
///////////////////////////////////////////////////
// 모든 바퀴가 같은 각도(alpha)로 정렬되어 헤딩방향을 고정한채 이동하는 모드
class ParallelMode : public MpcController {

private:
    parallel_model_solver_capsule* capsule_ = nullptr;
    int status_;

public:
    ParallelMode() {
        N_ = PARALLEL_MODEL_N;   // 헤더에 정의된 값
        NX_ = PARALLEL_MODEL_NX;
        NU_ = PARALLEL_MODEL_NU;
        NBX0_ = PARALLEL_MODEL_NBX0;
    }

    ~ParallelMode() override {
        freeSolver();
    }

    bool initialize() override {
        capsule_ = parallel_model_acados_create_capsule();
        // int status = spin_model_acados_create_with_discretization(capsule_, N_, nullptr);
        status_ = parallel_model_acados_create_with_discretization(capsule_, N_, nullptr);
        
        if (status_ != 0) return false;

        // 부모(MpcController)의 공통 포인터에 캡슐 연결
        nlp_config_ = parallel_model_acados_get_nlp_config(capsule_);
        nlp_dims_   = parallel_model_acados_get_nlp_dims(capsule_);
        nlp_in_     = parallel_model_acados_get_nlp_in(capsule_);
        nlp_out_    = parallel_model_acados_get_nlp_out(capsule_);
        nlp_solver_ = parallel_model_acados_get_nlp_solver(capsule_);

        return true;
    }

    bool solve() override {
        status_ = parallel_model_acados_solve(capsule_);

        ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time_);
        return (status_ == 0);
    }

    void freeSolver() override {
        if (capsule_ != nullptr) {
            parallel_model_acados_free(capsule_);
            if (status_) {
                printf("parallel_model_acados_free() returned status %d. \n", status_);
            }

            parallel_model_acados_free_capsule(capsule_);
            if (status_) {
                printf("parallel_model_acados_free_capsule() returned status %d. \n", status_);
            }
            capsule_ = nullptr;
        }
    }

    void printStats() override {

        MpcController::printStats();
        parallel_model_acados_print_stats(capsule_);

    }
};

///////////////////////////////////////////////////
///////////////// SpinMode Class ///////////////
///////////////////////////////////////////////////
// 위치는 고정한채 제자리 회전하여 헤딩만 바꾸는 모드
class SpinMode : public MpcController {

private:
    spin_model_solver_capsule* capsule_ = nullptr;
    int status_;

public:
    SpinMode() {
        N_ = SPIN_MODEL_N;   // 헤더에 정의된 값
        NX_ = SPIN_MODEL_NX;
        NU_ = SPIN_MODEL_NU;
        NBX0_ = SPIN_MODEL_NBX0;
    }

    ~SpinMode() override {
        freeSolver();
    }

    bool initialize() override {
        capsule_ = spin_model_acados_create_capsule();
        // int status = spin_model_acados_create_with_discretization(capsule_, N_, nullptr);
        status_ = spin_model_acados_create_with_discretization(capsule_, N_, nullptr);
        
        if (status_ != 0) return false;

        // 부모(MpcController)의 공통 포인터에 캡슐 연결
        nlp_config_ = spin_model_acados_get_nlp_config(capsule_);
        nlp_dims_   = spin_model_acados_get_nlp_dims(capsule_);
        nlp_in_     = spin_model_acados_get_nlp_in(capsule_);
        nlp_out_    = spin_model_acados_get_nlp_out(capsule_);
        nlp_solver_ = spin_model_acados_get_nlp_solver(capsule_);

        return true;
    }

    bool solve() override {
        status_ = spin_model_acados_solve(capsule_);

        ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time_);
        return (status_ == 0);
    }

    void freeSolver() override {
        if (capsule_ != nullptr) {
            spin_model_acados_free(capsule_);
            if (status_) {
                printf("spin_model_acados_free() returned status %d. \n", status_);
            }

            spin_model_acados_free_capsule(capsule_);
            if (status_) {
                printf("spin_model_acados_free_capsule() returned status %d. \n", status_);
            }
            capsule_ = nullptr;
        }
    }

    void printStats() override {

        MpcController::printStats();
        spin_model_acados_print_stats(capsule_);

    }
};

#endif