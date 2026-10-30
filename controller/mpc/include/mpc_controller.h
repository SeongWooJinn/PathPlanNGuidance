#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "base_controller.h"
#include "acados_c/ocp_nlp_interface.h"
#include "structs_mpc.h"  
#include "velocity_planner.h"

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
    double time_tot_, time_qp_, time_lin_;
    int sqp_iter_;

    // for ref trajectory, velocity propile
    std::vector<ReferenceTraj> ref_traj_;
    int current_closest_idx_ = 0;

    // for eval
    RealTimePerformance rt_stats;
    // 261011 수정: 재계획의 첫 속도는 측정값으로 시작하고 기어 방향을 보존한다.
    double initial_reference_speed_ = 0.0;

//////// helper functions ////////
    inline double calcDistance(double x1, double y1, double x2, double y2) const 
    {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }

public:
    // 261011 수정: Waffle은 큰 방향 오차를 줄이느라 위치 경로를 벗어나지 않도록 위치 가중치를 높인다.
    void configureTrackingWeights() {
        const double weights[8] = {30.0, 30.0, 8.0, 5.0, 0.5, 1.0, 2.0, 6000.0};
        double matrix[64] = {};
        double terminal[25] = {};
        for (int i = 0; i < 8; ++i) matrix[i * 8 + i] = weights[i];
        for (int i = 0; i < 5; ++i) terminal[i * 5 + i] = weights[i];
        for (int i = 0; i < N_; ++i) {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", matrix);
        }
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "W", terminal);
    }
    void setInitialReferenceSpeed(double speed) { initial_reference_speed_ = speed; }
    // virtual ~MpcController() = default;
    virtual ~MpcController() {
        if (rt_stats.cnt_ > 0) {
            rt_stats.printMetrics();
        }
    }
    inline std::vector<ReferenceTraj> getRefTrajectoryData() {return ref_traj_;}
    inline void setRefTrajectoryData(const std::vector<ReferenceTraj>& traj) { ref_traj_ = traj;}
    inline void setClosestIdx(int idx) {current_closest_idx_ = idx;}

///////////// 공통 로직 (BaseController의 가상 함수 구현) /////////////
    bool getRefTraj(
        std::vector<State>& global_path, double v_max,
        double a_lat_max, double a_acc_mag, double a_dec_mag, double dt) override 
    {
        
        if (global_path.empty()) return false;

        // 1. 기존 공간경로에 v항을 넣어줌
        std::vector<ReferenceTraj> spatial_ref(global_path.size());
        for (size_t i = 0; i < global_path.size(); ++i) 
        {
            spatial_ref[i].x = global_path[i].x;
            spatial_ref[i].y = global_path[i].y;
            spatial_ref[i].theta = global_path[i].theta;
            spatial_ref[i].delta = global_path[i].steering;
            spatial_ref[i].v = (global_path[i].gear == 0) ? v_max : -v_max;
            spatial_ref[i].mode = global_path[i].vehicle;
            spatial_ref[i].gear = global_path[i].gear;  // 261011 수정: 명시적 기어 정보 전달.
            // std::cout << spatial_ref[0].mode << std::endl;
            // 모드 전환 or 기어 전환 지점에서는 v = 0
            if (i > 0) {
                bool mode_changed = (spatial_ref[i].mode != spatial_ref[i-1].mode);
                bool gear_changed = (spatial_ref[i].v * spatial_ref[i-1].v < 0.0); // 부호가 다르면 곱이 음수
                
                if (mode_changed || gear_changed) {
                    spatial_ref[i-1].v = 0.0;
                }
            }
        }

        // for (const auto& gp : spatial_ref)
        //     std::cout << "RP : " << gp.mode << std::endl;

        // 2. 곡률 기반 속도 제약 , a = v^2 / r
        // 경로 중간에 있는 급커브나 U턴 구간에 대한 속도 제약을 위해 수행
        for (size_t i = 0; i < spatial_ref.size() - 1; ++i)
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

        // 261011 수정: 실제 속도와 새 첫 구간의 방향이 일치할 때만 초기 속도를 이어받는다.
        spatial_ref.front().v = spatial_ref.front().v * initial_reference_speed_ > 0.0 ?
            std::copysign(std::min(std::abs(initial_reference_speed_), v_max), initial_reference_speed_) : 0.0;
        // 3. 감가속 제약, v^2 - v0^2 = 2as
        accelerationProfile(spatial_ref, a_acc_mag);
        // 언제부터 브레이크를 밟을 것인지 check
        decelerationProfile(spatial_ref, a_dec_mag);

        // // velocity check
        // for (int i = 0; i < spatial_ref.size(); ++i){
        //     std::cout << i << ": "<< spatial_ref[i].v << std::endl;
        // }

        std::cout << spatial_ref[0].mode << std::endl;

        // 4. 시간 기반 리샘플링, 시간 기반 궤적 계산
        // 261011 수정: 양끝 정지 구간도 설정 가속도/최대 속도 범위 내에서 시간화한다.
        ref_traj_ = resampleTimeBasedTrajectory(spatial_ref, dt, a_acc_mag, v_max);

        // Theta 언랩핑 (각도 점프 제거)
        // 3.14에서 -3.14로 뛰는 현상을 3.14 -> 3.15 로 부드럽게 이어줌
        if (!ref_traj_.empty()) {
            // 궤적의 첫 번째 점을 차량의 실제 target 헤딩에 맞춤 (-pi 와 +pi 점프 억제)
            double diff0 = ref_traj_[0].theta - global_path[0].theta;
            double diff0_norm = normalizeAngle(diff0);
            ref_traj_[0].theta = global_path[0].theta + diff0_norm;

            for (size_t i = 1; i < ref_traj_.size(); ++i) {
                double diff = ref_traj_[i].theta - ref_traj_[i-1].theta;
                double diff_norm = normalizeAngle(diff);
                ref_traj_[i].theta = ref_traj_[i-1].theta + diff_norm;
            }
        }
        

        current_closest_idx_ = 0;
        std::cout << "글로벌 경로 변환 완료: " << ref_traj_.size() << std::endl;

        // std::cout << "x, y, theta, v, a, delta_dot" << std::endl;
        // for (const auto& t : ref_traj_)
        // {
        //     std::cout << t.x << ", " << t.y << ", " << t.theta 
        //         << ", " << t.v << ", " << t.a << ", " << t.delta_dot << std::endl;
        // }
        return !ref_traj_.empty();
    }

    int updateSlidingWindow(const double* curr, VehicleMode& curr_mode, 
        double min_d, double zero_v,
        double bi_dtheta, double spin_dtheta) override {
        if (ref_traj_.empty()) return -1;

        // 1. 이전 인덱스 기반으로 추종 궤적 내 가장 가까운 점 탐색 (연산 최적화)
        // 거리기반 인덱스 탐색
        double min_dist = 1e10;
        int best_idx = current_closest_idx_;
        int search_limit = std::min(current_closest_idx_ + 100, (int)ref_traj_.size());
        for (int i = current_closest_idx_; i < search_limit; ++i) {
            // 261011 수정: 최근접점 검색은 현재 모드/기어 구간을 넘어가지 않는다.
            if (ref_traj_[i].mode != curr_mode) break;
            if (ref_traj_[i].gear != ref_traj_[current_closest_idx_].gear) break;
            
            double dist = 0.0;
            if (curr_mode == VehicleMode::SpinMode) {
                // Spin 모드에서는 위치 이동이 없으므로, 헤딩 각도 차이를 거리로 환산하여 평가
                double dtheta = std::abs(normalizeAngle(curr[2] - ref_traj_[i].theta));
                dist = dtheta;
                // best_idx = std::min(current_closest_idx_ + 1, search_limit - 1);
                
            } else {
                // 자전거 모드 등에서는 기존처럼 유클리디안 거리 사용
                double dx = curr[0] - ref_traj_[i].x;
                double dy = curr[1] - ref_traj_[i].y;
                dist = std::hypot(dx, dy);
            }

            if (dist < min_dist) {
                min_dist = dist;
                best_idx = i;
            }
        }
        double dx_cur = curr[0] - ref_traj_[best_idx].x;
        double dy_cur = curr[1] - ref_traj_[best_idx].y;
        double dtheta_cur = std::abs(normalizeAngle(curr[2] - ref_traj_[best_idx].theta));
        double ddelta_cur = std::abs(normalizeAngle(curr[4] - ref_traj_[best_idx].delta));
        double dist_curr = std::hypot(dx_cur, dy_cur);

        VehicleMode best_mode = ref_traj_[best_idx].mode;
        bool is_arrived = false;

        switch (best_mode) {
            case VehicleMode::ParallelMode:     
                if (dist_curr < min_d) is_arrived = true;
                break;
            case VehicleMode::SpinMode:     
                if (dtheta_cur < spin_dtheta) is_arrived = true;
                break;
            default: // Bicycle Mode
                if (dist_curr < min_d)// && dtheta_cur < bi_dtheta) 
                    is_arrived = true;
                break;
        }
        
        // 도달했을 경우에만 인덱스 전진 (역행 방지)
        // 261011 수정: 모드/기어 경계는 측정 속도가 충분히 작아진 뒤에만 넘긴다.
        const int next = std::min(best_idx + 1, (int)ref_traj_.size() - 1);
        const bool boundary = ref_traj_[next].mode != curr_mode ||
            ref_traj_[best_idx].gear != ref_traj_[next].gear;
        if (is_arrived && (!boundary || std::abs(curr[3]) < zero_v)) {
            // current_closest_idx_가 아닌, 현재 찾은 best_idx를 기준으로 +1 전진
            best_idx = std::min(best_idx + 1, (int)ref_traj_.size() - 1);
        }
        current_closest_idx_ = best_idx;
    


        // 2. 예측 호라이즌 내 추종 궤적(yref_window) 찾기
        // 내부 버퍼를 사용하여 슬라이딩 윈도우 생성
        std::vector<ReferenceTraj> yref_window(N_);
        bool mode_switched = false;
        double last_theta = 0.0;
        double last_delta = 0.0;

        for (int i = 0; i < N_; ++i) {
            int idx = std::min(current_closest_idx_ + i, (int)ref_traj_.size() - 1);
            
            // 미래 예측 윈도우 내에서 모드가 바뀌는 순간 포착
            // 261011 수정: 예측 참조는 모드뿐 아니라 전후진 경계에서도 정지한다.
            if (!mode_switched && (ref_traj_[idx].mode != curr_mode ||
                ref_traj_[idx].gear != ref_traj_[current_closest_idx_].gear)) {
                mode_switched = true;
            }
            
            // 1. 일단 위치(x, y)와 타겟 상태는 원본 궤적을 그대로 복사
            yref_window[i] = ref_traj_[idx];
            // 261011 수정: 다른 모드의 참조는 현재 구간 끝에서 정지시켜 모델 혼합을 방지한다.
            if (mode_switched) {
                const int stop = std::max(current_closest_idx_, idx - 1);
                yref_window[i] = i > 0 ? yref_window[i - 1] : ref_traj_[stop];
                yref_window[i].v = yref_window[i].a = 0.0;
                yref_window[i].delta = yref_window[i].delta_dot = 0.0;
            }
            
            // 2. 만약 미래 궤적이 다른 모드라면, 기구학적으로 불가능한 요구만 마스킹!
            if (mode_switched) {
                if (ref_traj_[idx].mode == VehicleMode::SpinMode) {
                    // v,a는 0으로 확정
                    yref_window[i].v = 0.0; 
                    yref_window[i].a = 0.0;

                    // yref_window[i].delta = 0.0;      // in spinmode delta -> 각속도
                    // yref_window[i].delta_dot = 0.0;     // 각가속도
                }
            }
        }

        ReferenceTrajTerminal yref_e;
        yref_e.x = yref_window.back().x;
        yref_e.y = yref_window.back().y;
        yref_e.theta = yref_window.back().theta;
        yref_e.v = yref_window.back().v;
        yref_e.delta = yref_window.back().delta;

        // 솔버에 즉시 타겟 주입
        setTargetTrajectory(yref_window, yref_e);
        
        return current_closest_idx_;

    }

    bool isGuidanceFinished(const double* curr) override {

        if (ref_traj_.empty()) return true;

        double goal_x = ref_traj_.back().x;
        double goal_y = ref_traj_.back().y;
        double dist_to_goal = calcDistance(goal_x, goal_y, curr[0], curr[1]);
        
        if (dist_to_goal <= GOAL_TOLERANCE)
        {
            std::cout << "Reach Goal !!!" << std::endl;
            return true;
        }
        return false;
    }

    // 초기 예상 궤적, 초기 상태(x), 제어입력(u)세팅
    void setInitialGuess(double* x_init, double* u_init) override {
        for (int i = 0; i < N_; i++)
        {
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_init);
            ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u_init);
        }
        ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, N_, "x", x_init);
    }
    // 현재 로봇의 물리적 위치(상태) 제약 세팅
    void setInitialState(double* lbx0, double* ubx0) override {
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lbx0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", ubx0);
    }

    void setTargetTrajectory(
        const std::vector<ReferenceTraj>& yref, 
        const ReferenceTrajTerminal& yref_e) override 
    {
        for (int i = 0; i < N_; i++) {
            double target_array[8];
            target_array[0] = yref[i].x;
            target_array[1] = yref[i].y;
            target_array[2] = yref[i].theta;
            target_array[3] = yref[i].v;         
            target_array[4] = yref[i].delta;     
            target_array[5] = yref[i].a;
            target_array[6] = yref[i].delta_dot;
            // 쓰레기값 방지를 위해 0.0으로 명시적 초기화, obs 추가하면 수정
            target_array[7] = 0.0;               
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", target_array);
            // ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", (double*)&yref[i]);
            // std::cout << "spin mode 4/6 : "<< target_array[4] << ", " << target_array[6] << std::endl;
        }
        // 종점(Terminal) 타겟도 5칸 배열로 안전하게 패킹
        double target_e_array[5];
        target_e_array[0] = yref_e.x;
        target_e_array[1] = yref_e.y;
        target_e_array[2] = yref_e.theta;
        target_e_array[3] = yref_e.v;
        target_e_array[4] = yref_e.delta;

        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "yref", target_e_array);
        // ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N_, "yref", (double*)&yref_e);
    }

    void getControlInput(double* u_out) override {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u_out);
    }

    void getPredictedState(int step, double* x_pred) override {
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, step, "x", x_pred);
    }

    void setObstacleParameters(const std::vector<Obstacle>& top_obs, int k, double dt) override
    {
        // k -> 파이썬에서 정의
        std::vector<double> p_data(k * 3, 0.0);
        // 예측 호라이즌 내 솔버에서 인식하는 장애물 개수(k)만큼 장애물들의 예측위치 주입
        for (int i = 0; i <= N_; i++) {
            for (int j = 0; j < k; ++j) {
                p_data[j*3]     = top_obs[j].x + top_obs[j].vx * dt;
                p_data[j*3 + 1] = top_obs[j].y + top_obs[j].vy * dt;
                p_data[j*3 + 2] = top_obs[j].radius;

            }
            ocp_nlp_in_set(nlp_config_, nlp_dims_, nlp_in_, i, "parameter_values", p_data.data());
        }
        
    }
///////////// 공통 로직 (BaseController의 가상 함수 구현) /////////////

////////////// 자식 classes에서 반드시 구현해야 할 함수 /////////////
    virtual void freeSolver() = 0;
    virtual void printStatus() {

        // get solution
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf_);
        ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter_);

        // printf("\nSolver info:\n");
        // printf(" SQP iterations %2d\n solve %f [ms]\n KKT %e\n",
        //     sqp_iter_, time_tot_*1000, kkt_norm_inf_);

    }
////////////// 자식 classes에서 반드시 구현해야 할 함수 /////////////

};

#endif
