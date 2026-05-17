#include "vehicles_mpc.h"
#include "visualize.h"
#include "maps.h"

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
    double v_max = 0.5;        // 최대 허용 속도 (m/s), 
    double a_lat_max = 0.2;    // 최대 횡가속도(구심 가속도) 한계 (커브길 감속용)
    double a_dec_mag = 0.1;    // 최대 감속도 크기 (양수로 입력, 브레이크 성능)
    double dt = 0.1;          // MPC 제어 주기 generate_mpc.py -> Tf / N

    // get resampled reference trajectory
    if (!mpc_bi.getRefTraj(g_path, v_max, a_lat_max, a_dec_mag, dt)) return -1;
    
    // 글로벌 궤적 포인터 (각 컨트롤러가 동일한 ref_traj_를 공유)
    const std::vector<ReferenceTraj>& resampled_traj = mpc_bi.getRefTrajectoryData();

    std::cout << "1. reference trajecotory load" << std::endl;
    // std::cout << "reference trajecotory : " << resampled_traj.size() << std::endl;
    
    // 3. 로봇 초기 위치 (테스트용, g_path의 시작점)
    double current_x[5] = {
        g_path[0].x, 
        g_path[0].y, 
        g_path[0].theta, 
        0.0,               // 초기 시작 속도는 0
        g_path[0].steering // 초기 조향각
    };

    std::vector<State> track_path;
    int closest_idx = 0;
    // 4. 제어 루프
    while (closest_idx < resampled_traj.size() - 1) {

        VehicleMode curr_mode = resampled_traj[closest_idx].mode;
        bool solve_success = false;
        if (curr_mode == VehicleMode::BicycleMode)
        {
            // 도착여부 판단
            if (mpc_bi.isGuidanceFinished(current_x)) break;
            // 현재 상태를 솔버에 제약 조건으로 주입
            mpc_bi.setInitialState(current_x, current_x);
            // 현재 위치를 넘겨주면 내부에서 타겟 인덱스 탐색 및 윈도우 주입 자동 처리
            closest_idx = mpc_bi.updateSlidingWindow(current_x);

            solve_success = mpc_bi.solve();
            // 다음 스텝의 예측 상태를 현재 위치로 누적
            if (solve_success) 
                mpc_bi.getPredictedState(1, current_x);
        }
        else if (curr_mode == VehicleMode::ParallelMode)
        {
            // 도착여부 판단
            if (mpc_parallel.isGuidanceFinished(current_x)) break;
            // 현재 상태를 솔버에 제약 조건으로 주입
            mpc_parallel.setInitialState(current_x, current_x);
            // 현재 위치를 넘겨주면 내부에서 타겟 인덱스 탐색 및 윈도우 주입 자동 처리
            closest_idx = mpc_parallel.updateSlidingWindow(current_x);

            solve_success = mpc_parallel.solve();
            // 다음 스텝의 예측 상태를 현재 위치로 누적
            if (solve_success) 
                mpc_parallel.getPredictedState(1, current_x);
        }
        else    // spin mode
        {
            // 도착여부 판단
            if (mpc_spin.isGuidanceFinished(current_x)) break;
            // 현재 상태를 솔버에 제약 조건으로 주입
            mpc_spin.setInitialState(current_x, current_x);
            // 현재 위치를 넘겨주면 내부에서 타겟 인덱스 탐색 및 윈도우 주입 자동 처리
            closest_idx = mpc_spin.updateSlidingWindow(current_x);

            solve_success = mpc_spin.solve();
            // 다음 스텝의 예측 상태를 현재 위치로 누적
            if (solve_success) 
                mpc_spin.getPredictedState(1, current_x);
        }

        if (!solve_success){
            std::cerr << "MPC 최적화 연산 실패!" << std::endl;
            break;
        }

        // 한 스텝 이동할 때마다 현재 상태를 track_path에 기록
        State current_state;
        current_state.x = current_x[0];
        current_state.y = current_x[1];
        current_state.theta = current_x[2];
        track_path.push_back(current_state);

        std::cout << "추종 인덱스: " << closest_idx 
                  << " | 현재 좌표 X: " << current_x[0] << ", Y: " << current_x[1] 
                  << " | 현재 제어 v: " << current_x[3] << ", a: " << current_x[4] << std::endl;

    }

    std::cout << "경로 추종 시뮬레이션 완료." << std::endl;
    
    //////////////////// 시각화 /////////////////////
    // Extended_HAStar main.cpp와 동일한 값들 세팅필요 //
    ///////////////////////////////////////////////
    double cell_size = 10.0;     // 픽셀 비율 
    double r_length = 1.0;       // 로봇 길이
    double r_width = 0.6;        // 로봇 폭
    double resolution = 0.5;     // 맵 해상도

    double map_height = 30.0;
    double map_width = 60.0;
    int rows = static_cast<int>(map_height / resolution);
    int cols = static_cast<int>(map_width / resolution);

    OccMap gt(rows, cols, resolution);
    double sx = 5.0; double sy = 2.0; 
    double gx = 25.0; double gy = 24.0;
    gt.generate_example_map_v3(0.0, sx, sy, gx, gy);
    GridMap<int>& gt_map = gt.getOccMap();

    visualize_tracking_performance(
        g_path,         // 전역 경로 (회색 선)
        track_path,     // 실제 주행 경로 (파란색 선)
        gt_map, 
        cell_size, 
        r_length, 
        r_width, 
        "MPC_Tracking_Result", 
        resolution
    );


    return 0;
}
    