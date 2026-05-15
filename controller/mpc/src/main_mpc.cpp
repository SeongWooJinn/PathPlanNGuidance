#include "vehicles_model.h"


int main()
{
    // 1. 컨트롤러 초기화
    BicycleMode mpc;
    if (!mpc.initialize()) return -1;
    
    // 2. 경로 로드 (파일 로드 및 내부 포맷 변환 자동 수행)
    std::vector<State> g_path = loadPathFromBin("/tmp/hybrid_astar_path");

    // 제어 파라미터 세팅 (실제 차량 사양에 맞게 튜닝)
    // nav2_params.yaml controller server
    double v_max = 0.5;        // 최대 허용 속도 (m/s), 
    double a_lat_max = 3.0;    // 최대 횡가속도 한계 (커브길 감속용)
    double a_dec_mag = 3.0;    // 최대 감속도 크기 (양수로 입력, 브레이크 성능)
    double dt = 0.1;          // MPC 제어 주기 generate_mpc.py -> Tf / N)

    if (!mpc.getRefTraj(g_path, v_max, a_lat_max, a_dec_mag, dt)) return -1;

    // 3. 로봇 초기 위치 (테스트용 가상 시작점)
    double current_x[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    // 4. 제어 루프
    while (!mpc.isGuidanceFinished()) {
        
        // 현재 상태를 솔버에 제약 조건으로 주입
        mpc.setInitialState(current_x, current_x);

        // 현재 위치를 넘겨주면 내부에서 타겟 인덱스 탐색 및 윈도우 주입 자동 처리
        int closest_idx = mpc.updateSlidingWindow(current_x);

        // 제어 연산 수행
        if (mpc.solve()) {
            // 다음 스텝의 예측 상태를 현재 위치로 누적
            mpc.getPredictedState(1, current_x); 
            
            std::cout << "추종 인덱스: " << closest_idx 
                      << " | 현재 좌표 X: " << current_x[0] << ", Y: " << current_x[1] << std::endl;
        } else {
            std::cerr << "MPC 최적화 연산 실패!" << std::endl;
            break;
        }
    }

    std::cout << "경로 추종 시뮬레이션 완료." << std::endl;
    return 0;
}
    