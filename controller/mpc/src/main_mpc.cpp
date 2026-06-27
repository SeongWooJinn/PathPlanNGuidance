#include "vehicles_mpc.h"
#include "visualize.h"
#include "maps.h"
#include "eval_guidance.h"
#include "recoveryfsm_mpc.h"
#include "file_handler.h"

int main()
{
    // 0. 필요한 전역맵, 전역경로, 로봇 정보 가져오기
    mapInfo mapinfo;
    if (!loadMapInfoFromBin(mapinfo, "/tmp/parkinglot_map")) {
        std::cout << "Map load 실패" << std::endl;
        return -1;
    }
    RobotConfigs robotconfig;
    if (!loadRobotConfigFromBin(robotconfig, "/tmp/robotconfig")) {
        std::cout << "robotconfig load 실패" << std::endl;
        return -1;
    }
    std::vector<State> g_path = loadPathFromBin("/tmp/hybrid_astar_path");

    // 1. 컨트롤러 초기화
    // 초기화 등 acados 내부 오류 발생 시 수동으로 make : cd ~/c_generated_xx -> make shared_lib
    BicycleMode mpc_bi;
    ParallelMode mpc_parallel;
    SpinMode mpc_spin;

    if (!mpc_bi.initialize()) return -1;
    if (!mpc_parallel.initialize()) return -1;
    if (!mpc_spin.initialize()) return -1;

    // 제어 파라미터 세팅 (실제 차량 사양에 맞게 튜닝)
    // nav2_params.yaml controller server
    // 제동거리 고려해서 설정(d = v^2 / 2a)
    double v_max = 0.5;        // 최대 허용 속도 (m/s),generate_mpc의 v limit보다 작게 설정
    double a_lat_max = 0.2;    // 최대 횡가속도(구심 가속도) 한계 (커브길 감속용)
    double a_dec_mag = 3.0;//0.1;    // 최대 감속도 크기 (양수로 입력, 브레이크 성능)
    double a_max = 3.0;        // 최대 가속도 크기
    double dt = 2.0 / 60.0;          // MPC 제어 주기 generate_mpc.py -> Tf / N

    // 로봇 하드웨어 제원 (실제 차량 스펙에 맞게 수정)
    double wheelbase = robotconfig.WB;       // 축간 거리 L (m)
    double delta_max = robotconfig.delta_max;       // [bicycle] 최대 조향각 (rad), generate_mpc.py
    double delta_dot_max = 1.5;   // [bicycle] 최대 조향각속도 (rad/s), generate_mpc.py
    double omega_max = 0.5;       // [spin] 제자리 최대 회전 각속도 (rad/s), generate_mpc.py
    // bicycle의 u[1]=delta_dot, spin의 u[1]=omega_dot
    double omega_dot_max = delta_dot_max;  // [spin] 제자리 최대 회전 각가속도 (rad/s2)

    // 도착허용범위 설정
    double min_dist_thres = 1.5;//15 * v_max * dt;    // 20 * 0.35
    double zero_velocity_thres = std::max(0.01, dt * a_max); //dt * a_dec_mag;
    // 동적 각도 임계값 계산 (마진 1.5배 적용, 최소 0.1 rad 보장)
    double spin_dtheta_thres = 90 * M_PI / 180.0;//std::max(0.2, omega_max * dt * 1.5);
    // 자전거 모드는 최대 속도에서 최대 조향을 꺾었을 때 변하는 각도가 기준
    double bi_dtheta_thres = 90 * M_PI / 180.0; //robotconfig.delta_max; //std::max(0.2, (v_max * std::tan(delta_max) / wheelbase) * dt * 1.5);

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

    // 3. 로봇 초기 위치 (테스트용, g_path의 시작점)
    double current_x[5] = {
        g_path[0].x, 
        g_path[0].y, 
        g_path[0].theta, 
        0.0,               // 초기 시작 속도는 0
        g_path[0].steering // 초기 조향각
    };
    double current_u[2] = {0.0, 0.0};

    // 3.1 [동적 장애물 초기화] 주차장 시나리오 모사
    int num_obs = 10;    // 솔버가 인식 가능한 최대 장애물 개수(파이썬과 동일)
    std::vector<Obstacle> dynamic_obs;
    initDynamicObsInParkingLot(mapinfo, dynamic_obs);
    ObstacleManager obsMng(mapinfo.map, num_obs);

    std::vector<State> track_path;
    std::vector<std::pair<double, double>> controls;
    int closest_idx = 0;

    std::vector<MpcResultLog> current_log;
    std::vector<guidanceLog> guid_log;
    std::cout << "init ref_traj.mode : " << resampled_traj[closest_idx].mode << std::endl;
    int recovery_cnt = 0;
    int MAX_RECOVERY_CNT = 30;
    recoveryFSM fsm(MAX_RECOVERY_CNT, dt, a_dec_mag, omega_dot_max);
    
    VehicleMode prev_mode = resampled_traj[closest_idx].mode;

    // 4. 제어 루프
    while (closest_idx < resampled_traj.size() - 1) {
        VehicleMode curr_mode = resampled_traj[closest_idx].mode;
        prev_mode = curr_mode;

        // std::cout << "추종 인덱스: " << closest_idx << " | mode: " << curr_mode
        //           << " | 현재 상태 X: " << current_x[0] << ", Y: " << current_x[1] 
        //           << " , Theta: " << current_x[2] << ", v: " << current_x[3] << ", delta: " << current_x[4] 
        //           << " | 제어 입력 a: " << current_u[0] << ", delta_dot: " << current_u[1] << std::endl;

        // 람다식으로 즉시 포인터에 할당
        MpcController& active_mode = [&](VehicleMode mode) -> MpcController& {
            if (mode == VehicleMode::BicycleMode) return mpc_bi;
            else if (mode == VehicleMode::ParallelMode) return mpc_parallel;
            else return mpc_spin;
        }(curr_mode);

        bool solve_success = false;     // MPC SQP최적화 연산 성공 여부

        // 0) 도착여부 판단
        if (active_mode.isGuidanceFinished(current_x)) break;
        // 현재 루프의 dt만큼 장애물 위치 이동(실제로는 외부 모듈에서 받음)
        updateObstacles(dynamic_obs, dt);
        std::vector<Obstacle> top_k_obs = obsMng.getTopKObstacles(current_x, dynamic_obs, 10.0, 3.0);
        // 10m 이내에서 가장 위협적인 장애물 n개 추출 (파이썬 세팅 기준)
        // std::vector<Obstacle> top_k_obs = getTopKDynamicObstacles(current_x, dynamic_obs, 10.0, num_obs);
        // 솔버의 파라미터에 유효 장애물 n개의 좌표 주입
        active_mode.setObstacleParameters(top_k_obs, num_obs, dt);
        // for (const auto& obs : top_k_obs) {
        //     if (obs.x > 0 && obs.y > 0)
        //         std::cout << "obs point : " << obs.x << ", " << obs.y << std::endl;
        // }
        // 1) 각 모드별 인덱스 위치 동기화
        active_mode.setClosestIdx(closest_idx);
        // 2) 현재 상태를 솔버에 제약 조건으로 주입
        if (prev_mode != curr_mode) {
            std::cout << "mode switch : " << prev_mode << " -> " << curr_mode << std::endl;
            // prev가 스핀모드였으면 시스템 변수가 다르므로(omega->delta, omega_dot -> delta_dot) 초기화
            // curr이 스핀모드면 제자리 회전이므로 v, a = 0 / omega, omega_dot = 0부터 시작하도록 초기화
            if (prev_mode == VehicleMode::SpinMode || curr_mode == VehicleMode::SpinMode) {
                current_x[3] = 0.0; // v reset
                current_x[4] = 0.0; // omega -> delta reset
                current_u[0] = 0.0; // a reset
                current_u[1] = 0.0; // omega_dot -> delta_dot reset
            }
        }
        active_mode.setInitialState(current_x, current_x);

        // 3) 현재 위치를 넘겨주면 내부에서 타겟 인덱스 탐색 및 윈도우 주입 자동 처리
        closest_idx = active_mode.updateSlidingWindow(current_x, curr_mode, 
                min_dist_thres, zero_velocity_thres,
                bi_dtheta_thres, spin_dtheta_thres);
        // 4) 최적화 계산
        solve_success = active_mode.solve();
        if (solve_success) {
            // 다음 스텝의 예측 상태를 현재 위치로 누적
            active_mode.getPredictedState(1, current_x);
            active_mode.getControlInput(current_u);

            active_mode.printStatus();
        }
        
        // 5) 최적화 계산 실패시 절차대로 회복기동
        if (!solve_success){
            fsm.execute(current_x, current_u, closest_idx, curr_mode, active_mode, resampled_traj);
            // 루프를 종료(break)하지 않고 다음 제어 주기로 넘어감
            continue; 
        } 
        else {
            // 연산 성공 시 복구 플래그 초기화
            fsm.reset();
        }
        prev_mode = curr_mode; 
        
        // realtime_view를 위해 현재 위치와 장애물 저장
        saveCurrentDataToBin(current_x, dynamic_obs, "/tmp/sim_dynamic");
        // 한 스텝 이동할 때마다 현재 상태를 track_path에 기록
        State current_state;
        current_state.x = current_x[0];
        current_state.y = current_x[1];
        current_state.theta = current_x[2];
        current_state.vehicle = curr_mode;
        track_path.push_back(current_state);

        // current control inputs
        controls.push_back({current_u[0], current_u[1]});

        // save current log results
        MpcResultLog curr_state;
        curr_state.x         = current_x[0];
        curr_state.y         = current_x[1];
        curr_state.theta     = current_x[2];
        curr_state.v         = current_x[3];
        curr_state.delta     = current_x[4];
        curr_state.a         = current_u[0];
        curr_state.delta_dot = current_u[1];
        current_log.emplace_back(curr_state);

        // save guidance log
        guid_log.push_back({curr_state, closest_idx});

    }

    std::cout << "경로 추종 시뮬레이션 완료." << std::endl;
    saveMpcResultToBin(current_log, "/tmp/current_log");

    // evaluate tracking performance
    evaluateGuidance metrics = evaluateGuidanceMetrics(guid_log, resampled_traj, dt);
    std::cout << "========== MPC Tracking Evaluation ==========\n";
    std::cout << "Crosstrack Error (RMSE) : " << metrics.rmse_cte << " m\n";
    std::cout << "Max Crosstrack Error    : " << metrics.max_cte << " m\n";
    std::cout << "Heading Error (RMSE)    : " << metrics.rmse_he * 180.0 / M_PI << " deg\n";
    std::cout << "Max Heading Error       : " << metrics.max_he * 180.0 / M_PI << " deg\n";
    std::cout << "Jerk Accel (RMSE)       : " << metrics.rmse_jerk_a << " m/s^3\n";
    std::cout << "Control Effort (Smooth) : " << metrics.total_control_effort << "\n";
    std::cout << "=============================================\n";

    
    ///////////////////////////////////////////////
    //////////////////// 시각화 ////////////////////
    ///////////////////////////////////////////////
    int cell_size   = mapinfo.cell_size;     // 픽셀 비율 
    double r_length = mapinfo.r_length;       // 로봇 길이
    double r_width  = mapinfo.r_width;        // 로봇 폭
    double resolution = mapinfo.resolution;     // 맵 해상도

    double sx = mapinfo.sx; double sy = mapinfo.sy; 
    double gx = mapinfo.gx; double gy = mapinfo.gy;
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
        resolution, origin_x, origin_y
    );


    return 0;
}
    