#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H
#define _USE_MATH_DEFINES

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "mpc/include/structs.h"

class BaseController 
{
public:
    virtual ~BaseController() = default;

    // 하위 수준 인터페이스(vehicle classes), 모든 하위 class가 가져야 할 필수 인터페이스
    virtual bool initialize() = 0;      // 메모리, 포인터 등 초기화
    virtual bool solve() = 0;

    // 상위 수준 인터페이스(controller class)
    virtual bool getRefTraj(
        std::vector<State>& global_path, double v_max,
        double a_lat_max, double a_dec_mag, double dt) = 0;
    virtual int updateSlidingWindow(const double* curr) = 0;
    virtual bool isGuidanceFinished() = 0;

    virtual void setInitialGuess(double* x_init, double* u_init) = 0;  // double* x_init처럼 배열(포인터)로, 초기 예상 궤적 세팅
    virtual void setInitialState(double* lbx0, double* ubx0) = 0;   // 현재 로봇의 물리적 상태 제약 세팅
    virtual void setTargetTrajectory(
        const std::vector<ReferenceTraj>& yref, 
        const ReferenceTrajTerminal& yref_e) = 0;
    virtual void getControlInput(double* u_out) = 0;
    virtual void getPredictedState(int step, double* x_pred) = 0;

protected:
    // 각도를 -M_PI ~ M_PI (-180도 ~ 180도) 사이로 정규화하는 헬퍼 함수
    inline double normalizeAngle(double angle) {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

/////////////// velocity planner (velocity propiling) logic ///////////////
    inline void decelerationProfile(
        std::vector<ReferenceTraj>& path, double a_dec_mag) 
    {
        if (path.empty()) return;

        // 1. 최종 목적지에서의 속도는 무조건 0으로 강제 (안전하게 정지)
        path.back().v = 0.0;

        // 2. 경로의 맨 끝에서부터 앞으로 오면서(역순) 역산
        for (int i = path.size() - 2; i >= 0; --i) {
            
            double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;
            double ds = std::hypot(dx, dy);

            // 등가속도 운동 공식 적용(감속!!)
            // v_i = sqrt(v_f^2 + 2 * a * s)
            double v_f_sq = std::pow(path[i+1].v, 2);
            double v_dec_limit = std::sqrt(v_f_sq + 2.0 * a_dec_mag * ds);

            // 현재 지점의 속도는 '곡률 제한 속도'와 '감속 제한 속도' 중 더 작은 값을 선택
            path[i].v = std::min(path[i].v, v_dec_limit);
        }
    }

    inline ReferenceTraj interpolateState(
        const ReferenceTraj& pt1, const ReferenceTraj& pt2, double r)
    {
        ReferenceTraj new_pt;

        // 1. X, Y 좌표는 단순 선형 보간
        new_pt.x = (1.0 - r) * pt1.x + r * pt2.x;
        new_pt.y = (1.0 - r) * pt1.y + r * pt2.y;

        // 2. Theta (헤딩 각도)는 최단 경로 보간
        double diff_theta = normalizeAngle(pt2.theta - pt1.theta);
        new_pt.theta = normalizeAngle(pt1.theta + r * diff_theta);

        // 3. 조향각(steering)도 부드러운 전환을 위해 선형 보간 적용
        new_pt.delta = (1.0 - r) * pt1.delta + r * pt2.delta;

        new_pt.v = (1.0 - r) * pt1.v + r * pt2.v;

        return new_pt;
    }


    inline std::vector<ReferenceTraj> resampleToTimeBasedTrajectory(
        const std::vector<ReferenceTraj>& spatial_path, 
        double dt)
    {
        std::vector<ReferenceTraj> temporal_path;
        if (spatial_path.empty()) return temporal_path;

        // 1. 공간 경로의 각 점까지의 누적 거리(s) 계산
        std::vector<double> s_spatial(spatial_path.size(), 0.0);
        for (size_t i = 1; i < spatial_path.size(); ++i) {
            s_spatial[i] = s_spatial[i-1] + std::hypot(
                spatial_path[i].x - spatial_path[i-1].x, 
                spatial_path[i].y - spatial_path[i-1].y
            );
        }
        double total_distance = s_spatial.back();

        // 2. 초기점(t=0) 세팅
        ReferenceTraj first_pt = interpolateState(spatial_path[0], spatial_path[1], 0.0);
        first_pt.a = 0.0;
        first_pt.delta_dot = 0.0;
        temporal_path.push_back(first_pt);

        double s_target = 0.0; // 로봇이 이동해야 할 누적 목표 거리
        int idx = 0;           // 탐색을 위한 공간 경로 인덱스

        // 3. 시간(dt) 기준으로 전진하며 리샘플링 (목적지에 도달할 때까지)
        while (s_target < total_distance) {
            
            // [A] 이전 스텝의 상태를 가져와서, 다음 dt 동안 얼마나 갈지 계산
            ReferenceTraj prev_pt = temporal_path.back();
            
            // 핵심: s = v * dt (속도가 0이 되어 무한루프에 빠지는 것을 막기 위해 최소 이동거리 0.01m 보장)
            double ds = std::max(prev_pt.v * dt, 0.01); 
            s_target += ds;

            if (s_target >= total_distance) break;

            // [B] s_target이 위치한 공간 경로상의 선분(idx ~ idx+1) 찾기
            while (idx < spatial_path.size() - 1 && s_spatial[idx + 1] < s_target) {
                idx++;
            }

            // [C] 해당 선분 내에서의 보간 비율(r) 계산
            double segment_length = s_spatial[idx + 1] - s_spatial[idx];
            double r = (s_target - s_spatial[idx]) / segment_length;

            // [D] 앞서 만든 함수로 x, y, theta, delta, v 보간!
            ReferenceTraj new_pt = interpolateState(spatial_path[idx], spatial_path[idx + 1], r);

            // [E] 3단계의 핵심: 미분값 추출 (a, delta_dot)
            // 두 점 사이의 시간 간격은 무조건 dt 이므로 단순 시간 나누기 적용
            new_pt.a = (new_pt.v - prev_pt.v) / dt;
            
            // 조향각속도: 각도 랩어라운드(Wrap-around)를 고려하여 차이 계산
            double diff_delta = new_pt.delta - prev_pt.delta;
            new_pt.delta_dot = diff_delta / dt;

            // (선택) 물리적 한계로 a와 delta_dot을 클램핑(Clamping) 처리할 수도 있습니다.
            // new_pt.a = std::clamp(new_pt.a, -MAX_DECEL, MAX_ACCEL);

            temporal_path.push_back(new_pt);
        }

        // 4. 종점 처리 (속도 및 제어 입력 0)
        ReferenceTraj last_pt = temporal_path.back(); // 혹은 spatial_path의 마지막 점 복사
        last_pt.v = 0.0;
        last_pt.a = 0.0;
        last_pt.delta_dot = 0.0;
        temporal_path.push_back(last_pt);

        return temporal_path;
    }
};


#endif