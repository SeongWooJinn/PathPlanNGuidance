#include "vehicles_mpc.h"
#include "visualize.h"
#include "maps.h"
#include "eval_guidance.h"

int main()
{
    // 1. 컨트롤러 초기화
    // 초기화 등 acados 내부 오류 발생 시 수동으로 make : cd ~/c_generated_xx -> make shared_lib
    BicycleMode mpc_bi;
    ParallelMode mpc_parallel;
    SpinMode mpc_spin;

    if (!mpc_bi.initialize()) return -1;
    if (!mpc_parallel.initialize()) return -1;
    if (!mpc_spin.initialize()) return -1;
    
    // 2. 경로 로드 (파일 로드 및 내부 포맷 변환 자동 수행)
    std::vector<State> g_path = loadPathFromBin("/tmp/hybrid_astar_path");

    // 제어 파라미터 세팅 (실제 차량 사양에 맞게 튜닝)
    // nav2_params.yaml controller server
    // 제동거리 고려해서 설정(d = v^2 / 2a)
    double v_max = 0.5;        // 최대 허용 속도 (m/s),generate_mpc의 v limit보다 작게 설정
    double a_lat_max = 0.2;    // 최대 횡가속도(구심 가속도) 한계 (커브길 감속용)
    double a_dec_mag = 3.0;//0.1;    // 최대 감속도 크기 (양수로 입력, 브레이크 성능)
    double a_max = 3.0;        // 최대 가속도 크기
    double dt = 5.0 / 150.0;          // MPC 제어 주기 generate_mpc.py -> Tf / N
    double min_dist_thres = 10 * v_max * dt;    // 20 * 0.35
    double zero_velocity_thres = std::max(0.01, dt * a_max); //dt * a_dec_mag;

    // 로봇 하드웨어 제원 (실제 차량 스펙에 맞게 수정)
    double wheelbase = 1.0;       // 축간 거리 L (m)
    double delta_max = 1.5;       // 최대 조향각 (rad), generate_mpc.py
    double omega_max = 0.5;       // 제자리 최대 회전 각속도 (rad/s), generate_mpc.py

    // 동적 각도 임계값 계산 (마진 1.5배 적용, 최소 0.1 rad 보장)
    double spin_dtheta_thres = std::max(0.05, omega_max * dt * 1.5);
    // 자전거 모드는 최대 속도에서 최대 조향을 꺾었을 때 변하는 각도가 기준
    double bi_dtheta_thres = std::max(0.05, (v_max * std::tan(delta_max) / wheelbase) * dt * 1.5);

    std::cout << "min_dist_thres: " << min_dist_thres 
              << ", " << "zero_velocity_thres: " << zero_velocity_thres
              << ", " << "bi_dtheta_thres: " << bi_dtheta_thres
              << ", " << "spin_dtheta_thres: " << spin_dtheta_thres << std::endl;

    // get resampled reference trajectory
    if (!mpc_bi.getRefTraj(g_path, v_max, a_lat_max, a_max, a_dec_mag, dt)) return -1;
    
    // 글로벌 궤적 포인터 (각 컨트롤러가 동일한 ref_traj_를 공유)
    const std::vector<ReferenceTraj>& resampled_traj = mpc_bi.getRefTrajectoryData();
    mpc_parallel.setRefTrajectoryData(resampled_traj);
    mpc_spin.setRefTrajectoryData(resampled_traj);

    std::cout << "1. reference trajecotory load" << std::endl;

    // for (int i = 0; i < resampled_traj.size(); ++i)
    //     std::cout << "i : "<< i << ", " 
    //               << "mode : " << resampled_traj[i].mode << ", " 
    //               << "v : " << resampled_traj[i].v << ", " 
    //               << "theta : " << resampled_traj[i].theta << ", "  
    //               << "a : " << resampled_traj[i].a << ", " 
    //               << "delta : " << resampled_traj[i].delta << ", " 
    //               << "delta_dot : " << resampled_traj[i].delta_dot << std::endl;

    // 3. 로봇 초기 위치 (테스트용, g_path의 시작점)
    double current_x[5] = {
        g_path[0].x, 
        g_path[0].y, 
        g_path[0].theta, 
        0.0,               // 초기 시작 속도는 0
        g_path[0].steering // 초기 조향각
    };
    double current_u[2] = {0.0, 0.0};

    std::vector<State> track_path;
    std::vector<std::pair<double, double>> controls;
    int closest_idx = 0;

    std::vector<MpcResultLog> mpc_log;
    std::vector<guidanceLog> guid_log;
    std::cout << "init ref_traj.mode : " << resampled_traj[closest_idx].mode << std::endl;
    bool is_action_applied = false;
    int recovery_cnt = 0;
    int MAX_RECOVERY_CNT = 30;
    
    // 4. 제어 루프
    while (closest_idx < resampled_traj.size() - 1) {

        VehicleMode curr_mode = resampled_traj[closest_idx].mode;
        static VehicleMode prev_mode = curr_mode; // 정적 변수로 이전 모드 기억

        std::cout << "추종 인덱스: " << closest_idx << " | mode: " << curr_mode
                  << " | 현재 상태 X: " << current_x[0] << ", Y: " << current_x[1] 
                  << " , Theta: " << current_x[2] << ", v: " << current_x[3] << ", delta: " << current_x[4] 
                  << " | 제어 입력 a: " << current_u[0] << ", delta_dot: " << current_u[1] << std::endl;

        // if (closest_idx != 0 && curr_mode != resampled_traj[closest_idx - 1].mode) {
        //     std::cout << "mode change!! : " << resampled_traj[closest_idx - 1].mode
        //               << " --> " << curr_mode << ", idx : " << closest_idx << std::endl;
        // }

        bool solve_success = false;     // MPC SQP최적화 연산 성공 여부
        if (curr_mode == VehicleMode::BicycleMode)
        {
            // 도착여부 판단
            if (mpc_bi.isGuidanceFinished(current_x)) break;

            // 각 모드별 인덱스 위치 동기화
            mpc_bi.setClosestIdx(closest_idx);
            // 현재 상태를 솔버에 제약 조건으로 주입
            if (prev_mode == VehicleMode::SpinMode) {
                std::cout << "mode switch : " << prev_mode << " -> " << curr_mode << std::endl;
                current_x[3] = 0.0; // v reset
                current_x[4] = 0.0; // omega -> delta reset
                current_u[0] = 0.0; // a reset
                current_u[1] = 0.0; // omega_dot -> delta_dot reset
            }
            mpc_bi.setInitialState(current_x, current_x);
            // 현재 위치를 넘겨주면 내부에서 타겟 인덱스 탐색 및 윈도우 주입 자동 처리
            closest_idx = mpc_bi.updateSlidingWindow(current_x, curr_mode, 
                    min_dist_thres, zero_velocity_thres,
                    bi_dtheta_thres, spin_dtheta_thres);

            solve_success = mpc_bi.solve();
            if (solve_success) {
                // 다음 스텝의 예측 상태를 현재 위치로 누적
                mpc_bi.getPredictedState(1, current_x);
                mpc_bi.getControlInput(current_u);
            }
        }
        else if (curr_mode == VehicleMode::ParallelMode)
        {
            // 도착여부 판단
            if (mpc_parallel.isGuidanceFinished(current_x)) break;

            // 각 모드별 인덱스 위치 동기화
            mpc_parallel.setClosestIdx(closest_idx);
            // 현재 상태를 솔버에 제약 조건으로 주입
            if (prev_mode == VehicleMode::SpinMode) {
                std::cout << "mode switch : " << prev_mode << " -> " << curr_mode << std::endl;
                current_x[3] = 0.0; // v reset
                current_x[4] = 0.0; // omega -> delta reset
                current_u[0] = 0.0; // a reset
                current_u[1] = 0.0; // omega_dot -> delta_dot reset
            }
            mpc_parallel.setInitialState(current_x, current_x);
            // 현재 위치를 넘겨주면 내부에서 타겟 인덱스 탐색 및 윈도우 주입 자동 처리
            closest_idx = mpc_parallel.updateSlidingWindow(current_x, curr_mode, 
                    min_dist_thres, zero_velocity_thres,
                    bi_dtheta_thres, spin_dtheta_thres);

            solve_success = mpc_parallel.solve();
            if (solve_success) {
                // 다음 스텝의 예측 상태를 현재 위치로 누적
                mpc_parallel.getPredictedState(1, current_x);
                mpc_parallel.getControlInput(current_u);
            }
        }
        else    // spin mode
        {
            // 도착여부 판단
            if (mpc_spin.isGuidanceFinished(current_x)) break;

            // 각 모드별 인덱스 위치 동기화
            mpc_spin.setClosestIdx(closest_idx);
            // 현재 상태를 솔버에 제약 조건으로 주입
            if (prev_mode != VehicleMode::SpinMode) {  // Spin이 아닌 모든 모드에서 넘어올 때
                std::cout << "mode switch : " << prev_mode << " -> " << curr_mode << std::endl;
                current_x[3] = 0.0;     // v = 0.0
                current_x[4] = 0.0;     // delta -> omega = 0.0
                current_u[0] = 0.0;     // a = 0.0
                current_u[1] = 0.0;     // delta_dot -> omega_dot = 0.0
            }
            mpc_spin.setInitialState(current_x, current_x);
            // 현재 위치를 넘겨주면 내부에서 타겟 인덱스 탐색 및 윈도우 주입 자동 처리
            closest_idx = mpc_spin.updateSlidingWindow(current_x, curr_mode, 
                    min_dist_thres, zero_velocity_thres,
                    bi_dtheta_thres, spin_dtheta_thres);
            
            solve_success = mpc_spin.solve();
            
            if (solve_success) {
                // 다음 스텝의 예측 상태를 현재 위치로 누적
                mpc_spin.getPredictedState(1, current_x);
                mpc_spin.getControlInput(current_u);
            }
        }
        prev_mode = curr_mode; 
        
        if (!solve_success){
            std::cerr << "[Warning] MPC 최적화 연산 실패! Recovery FSM 가동" << std::endl;
            recovery_cnt++;
            // A. 부드러운 감속 로직 (Jerk-limited Braking)
            double current_v = current_x[3];
            if (std::abs(current_v) > 0.01) {
                // 1. 타겟 브레이크 가속도 설정
                double target_brake = (current_v > 0) ? -a_dec_mag : a_dec_mag;
                
                // 2. 한 스텝당 변할 수 있는 최대 가속도 폭 제한 (Max Jerk: 5.0 m/s^3 기준)
                double max_delta_u0 = 5.0 * dt; 

                // 3. 현재 입력값(current_u[0])을 타겟을 향해 부드럽게 이동 (Clamping)
                if (target_brake > current_u[0] + max_delta_u0) {
                    current_u[0] += max_delta_u0;
                } else if (target_brake < current_u[0] - max_delta_u0) {
                    current_u[0] -= max_delta_u0;
                } else {
                    current_u[0] = target_brake; // 타겟 도달 시 고정
                }
            } else {
                // 완전 정지 상태
                current_u[0] = 0.0;
                current_x[3] = 0.0;
            }

            // B. 조향각 복원 로직 (Rate-limited Steering Return)
            // 실패 직전의 조향각을 그대로 굳혀서 곡률을 유지한 채 감속하도록 강제
            current_u[1] = 0.0; // delta_dot = 0.0 (핸들 회전 정지)

            // double current_delta = current_x[4];
            // if (std::abs(current_delta) > 0.05) {
            //     // 1. 핸들을 중앙으로 풀기 위한 타겟 각속도 (초당 약 5.7도)
            //     double target_steer_rate = (current_delta > 0) ? -0.1 : 0.1;
                
            //     // 2. 한 스텝당 변할 수 있는 최대 각속도 폭 제한 (Max Steer Accel: 2.0 rad/s^2 기준)
            //     double max_delta_u1 = 2.0 * dt;

            //     // 3. 현재 조향 속도(current_u[1])를 타겟을 향해 부드럽게 이동
            //     if (target_steer_rate > current_u[1] + max_delta_u1) {
            //         current_u[1] += max_delta_u1;
            //     } else if (target_steer_rate < current_u[1] - max_delta_u1) {
            //         current_u[1] -= max_delta_u1;
            //     } else {
            //         current_u[1] = target_steer_rate;
            //     }
            // } else {
            //     current_u[1] = 0.0;
            // }
            
            // 2. Recovery 상태 머신 (is_action_applied 플래그로 상태 전환 제어)
            // if (!is_action_applied) {
            if (recovery_cnt == 1) {
                std::cout << "Recovery Step 1: 솔버 내부 메모리(Warm Start) 강제 초기화" << std::endl;
                // 이전 스텝의 꼬여버린 예측 해를 현재 위치 기준으로 깨끗하게 덮어씌움
                if (curr_mode == VehicleMode::BicycleMode) 
                    mpc_bi.setInitialGuess(current_x, current_u);
                else if (curr_mode == VehicleMode::ParallelMode) 
                    mpc_parallel.setInitialGuess(current_x, current_u);
                else 
                    mpc_spin.setInitialGuess(current_x, current_u);
            } 
            else if(recovery_cnt > MAX_RECOVERY_CNT) {
                std::cout << "Recovery Step 2: 초기화로도 실패. 타겟 인덱스 강제 스킵 (Deadlock 탈출)" << std::endl;
                // 다음 궤적이 다른 모드(Spin)라면 스킵 절대 금지!
                int next_idx = std::min(closest_idx + 1, (int)resampled_traj.size() - 1);
                if (resampled_traj[next_idx].mode == curr_mode) {
                    closest_idx = next_idx; // 같은 모드일 때만 인덱스를 건너뜀
                } else {
                    std::cout << " -> 다음 궤적이 모드 전환점이므로 인덱스를 고정하고 정지를 대기합니다." << std::endl;
                }
                // // 장애물이나 극단적 곡률로 인해 특정 인덱스에서 막혔다면 억지로 한 칸 넘김
                // int next_idx = std::min(closest_idx + 1, (int)resampled_traj.size() - 1);
                recovery_cnt = 0;
            }

            // 루프를 종료(break)하지 않고 다음 제어 주기로 넘어감
            continue; 
        } 
        else {
            // 연산 성공 시 복구 플래그 초기화
            if (recovery_cnt > 0) {
                std::cout << "[Success] 솔버 자가 복구 완료! 정상 주행 복귀." << std::endl;
                recovery_cnt = 0; 
            }
            // is_action_applied = false; 
        }

        // 한 스텝 이동할 때마다 현재 상태를 track_path에 기록
        State current_state;
        current_state.x = current_x[0];
        current_state.y = current_x[1];
        current_state.theta = current_x[2];
        current_state.vehicle = curr_mode;
        track_path.push_back(current_state);

        // current control inputs
        controls.push_back({current_u[0], current_u[1]});

        // save mpc results
        MpcResultLog curr_mpc;
        curr_mpc.x         = current_x[0];
        curr_mpc.y         = current_x[1];
        curr_mpc.theta     = current_x[2];
        curr_mpc.v         = current_x[3];
        curr_mpc.delta     = current_x[4];
        curr_mpc.a         = current_u[0];
        curr_mpc.delta_dot = current_u[1];
        mpc_log.emplace_back(curr_mpc);

        // save guidance log
        guidanceLog tmp;
        tmp.mpc_log = curr_mpc;
        tmp.closest_idx = closest_idx;
        guid_log.emplace_back(tmp);

    }

    std::cout << "경로 추종 시뮬레이션 완료." << std::endl;
    saveMpcResultToBin(mpc_log, "/tmp/mpc_log");

    // evaluate tracking performance
    evaluateGuidance metrics = evaluateGuidanceMetrics(guid_log, resampled_traj);
    std::cout << "========== MPC Tracking Evaluation ==========\n";
    std::cout << "Crosstrack Error (RMSE) : " << metrics.rmse_cte << " m\n";
    std::cout << "Max Crosstrack Error    : " << metrics.max_cte << " m\n";
    std::cout << "Heading Error (RMSE)    : " << metrics.rmse_he * 180.0 / M_PI << " deg\n";
    std::cout << "Max Heading Error       : " << metrics.max_he * 180.0 / M_PI << " deg\n";
    std::cout << "Control Effort (Smooth) : " << metrics.total_control_effort << "\n";
    std::cout << "=============================================\n";

    ///////////////////////////////////////////////
    //////////////////// 시각화 ////////////////////
    ///////////////////////////////////////////////
    mapInfo mapinfo;
    if (!loadMapInfoFromBin(mapinfo, "/tmp/mapinfo")) {
        std::cout << "Map load 실패" << std::endl;
        return -1;
    }

    int cell_size   = mapinfo.cell_size;     // 픽셀 비율 
    double r_length = mapinfo.r_length;       // 로봇 길이
    double r_width  = mapinfo.r_width;        // 로봇 폭
    double resolution = mapinfo.resolution;     // 맵 해상도

    double sx = mapinfo.sx; double sy = mapinfo.sy; 
    double gx = mapinfo.gx; double gy = mapinfo.gy;
    double px_scale = mapinfo.px_scale;
    double origin_x = mapinfo.origin_x;
    double origin_y = mapinfo.origin_y;
    GridMap<int>& gt_map = mapinfo.map;

    visualize_tracking_performance(
        g_path,         // 전역 경로 (회색 선)
        track_path,     // 실제 주행 경로 (파란색 선)
        gt_map, 
        cell_size, 
        r_length, 
        r_width, 
        "MPC_Tracking_Result", 
        resolution, px_scale, origin_x, origin_y
    );


    return 0;
}
    