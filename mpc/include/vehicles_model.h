#ifndef VEHICLES_MODEL_H
#define VEHICLES_MODEL_H
#define _USE_MATH_DEFINES

#include "mpc_controller.h"
#include <math.h>

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
        nlp_solver_ = bicycle_model_acados_get_nlp_solver(acados_ocp_capsule);

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




#endif