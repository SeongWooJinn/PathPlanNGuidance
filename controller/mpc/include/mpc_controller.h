#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "base_controller.h"
#include "acados_c/ocp_nlp_interface.h"
#include "structs.h"  

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

    // for ref trajectory, velocity propile
    std::vector<ReferenceTraj> ref_traj_;
    int current_closest_idx_ = 0;

//////// helper functions ////////
    inline double calcDistance(double x1, double y1, double x2, double y2) const 
    {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }

public:
    virtual ~MpcController() = default;

///////////// 공통 로직 (BaseController의 가상 함수 구현) /////////////
    bool getRefTraj(
        std::vector<State>& global_path, double v_max,
        double a_lat_max, double a_dec_mag, double dt) override {

        if (global_path.empty()) return false;

        std::vector<ReferenceTraj> spatial_ref;
        for (size_t i = 0; i < global_path.size(); ++i) 
        {
            spatial_ref[i].x = global_path[i].x;
            spatial_ref[i].y = global_path[i].y;
            spatial_ref[i].theta = global_path[i].theta;
            spatial_ref[i].delta = global_path[i].steering;
            spatial_ref[i].v = (global_path[i].gear == 0) ? v_max : -v_max;
        }

        // 곡률 기반 속도 제약 , a = v^2 / r
        // 경로 중간에 있는 급커브나 U턴 구간에 대한 속도 제약을 위해 수행
        for (size_t i = 0; i < spatial_ref.size(); ++i)
        {
            double dx = spatial_ref[i].x - spatial_ref[i+1].x;
            double dy = spatial_ref[i].y - spatial_ref[i+1].y;
            double ds = std::hypot(dx, dy);
            if (ds > 1e-3) {
                double dtheta = std::abs(normalizeAngle(spatial_ref[i].theta - spatial_ref[i+1].theta));
                double kappa = dtheta / ds;
                if (kappa > 1e-5) {
                    double v_safe = std::sqrt(a_lat_max / kappa);
                    // 기어 방향 유지하면서 절대값만 제한
                    double sign = (spatial_ref[i].v > 0) ? 1.0 : -1.0;
                    spatial_ref[i].v = sign * std::min(v_safe, v_max);

                }
            }
        }
        spatial_ref.back().v = 0.0; // 종점 정지

        // 감가속 제약, v^2 - v0^2 = 2as
        // 언제부터 브레이크를 밟을 것인지 check
        decelerationProfile(spatial_ref, a_dec_mag);

        // 시간 기반 리샘플링, 시간 기반 궤적 계산
        ref_traj_ = resampleToTimeBasedTrajectory(spatial_ref, 0.1);
        current_closest_idx_ = 0;

        std::cout << "글로벌 경로 변환 완료: " << ref_traj_.size() << std::endl;

        return !ref_traj_.empty();
    }

    int updateSlidingWindow(const double* curr) override {
        if (ref_traj_.empty()) return -1;

        // 이전 인덱스 기반으로 가장 가까운 점 탐색 (연산 최적화)
        double min_dist = 1e10;
        int search_limit = std::min(current_closest_idx_ + 20, (int)ref_traj_.size());
        for (int i = current_closest_idx_; i < search_limit; ++i) {
            double dist = calcDistance(curr[0], curr[1], ref_traj_[i].x, ref_traj_[i].y);
            if (dist < min_dist) {
                min_dist = dist;
                current_closest_idx_ = i;
            }
        }

        // 내부 버퍼를 사용하여 슬라이딩 윈도우 생성
        std::vector<ReferenceTraj> yref_window(N_);
        for (int i = 0; i < N_; ++i) {
            int target_idx = std::min(current_closest_idx_ + i, (int)ref_traj_.size() - 1);
            yref_window[i] = ref_traj_[target_idx];
        }

        ReferenceTrajTerminal yref_e;
        int terminal_idx = std::min(current_closest_idx_ + N_, (int)ref_traj_.size() - 1);
        yref_e.x = ref_traj_[terminal_idx].x;
        yref_e.y = ref_traj_[terminal_idx].y;
        yref_e.theta = ref_traj_[terminal_idx].theta;
        yref_e.v = ref_traj_[terminal_idx].v;
        yref_e.delta = ref_traj_[terminal_idx].delta;

        // 솔버에 즉시 타겟 주입
        setTargetTrajectory(yref_window, yref_e);
        
        return current_closest_idx_;

    }
    bool isGuidanceFinished() override {
        return !ref_traj_.empty() && (current_closest_idx_ >= ref_traj_.size() - 1);
    }

    // 초기 예상 궤적, 초기 상태(x), 제어입력(u)세팅
    void setInitialGuess(double* x_init, double* u_init) override {
        for (int i = 0; i < N_; i++)
        {
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_init);
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "u", u_init);
        }
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, N_, "x", x_init);
    }
    // 현재 로봇의 물리적 위치(상태) 제약 세팅
    void setInitialState(double* lbx0, double* ubx0) override {
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", lbx0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", ubx0);
    }

    void setTargetTrajectory(
        const std::vector<ReferenceTraj>& yref, 
        const ReferenceTrajTerminal& yref_e) override 
    {
        for (int i = 0; i < N_; i++) {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", (double*)&yref[i]);
        }
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "yref", (double*)&yref_e);
    }

    void getControlInput(double* u_out) override {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u_out);
    }

    void getPredictedState(int step, double* x_pred) override {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, step, "x", x_pred);
    }
///////////// 공통 로직 (BaseController의 가상 함수 구현) /////////////


////////////// 자식 classes에서 반드시 구현해야 할 함수 /////////////
    virtual void freeSolver() = 0;
    virtual void printStats() {

        // get solution
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf_);
        ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter_);

        printf("\nSolver info:\n");
        printf(" SQP iterations %2d\n solve %f [ms]\n KKT %e\n",
            sqp_iter_, elapsed_time_*1000, kkt_norm_inf_);

    }
////////////// 자식 classes에서 반드시 구현해야 할 함수 /////////////

};

#endif