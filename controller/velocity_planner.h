#ifndef VELOCITY_PLANNER_H
#define VELOCITY_PLANNER_H

#include "base_controller.h"


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
        double sign = (path[i].v > 0) ? 1.0 : -1.0;
        double speed_limit = std::min(std::abs(path[i].v), v_dec_limit);
        path[i].v = sign * speed_limit;

        // for (const auto& p : path) 
        // {
        //     std::cout << p.v << std::endl;
        // }
    }
}
inline ReferenceTraj interpolateState(
    const ReferenceTraj& pt1, const ReferenceTraj& pt2, double r)
{
    // pt1 -> pt2
    ReferenceTraj new_pt;
    new_pt.mode = pt2.mode;

    // parallel mode: 헤딩 고정, 조향각 고정, 직선 대각선 이동
    if (new_pt.mode == VehicleMode::ParallelMode) {
        new_pt.x = (1.0 - r) * pt1.x + r * pt2.x;
        new_pt.y = (1.0 - r) * pt1.y + r * pt2.y;
        new_pt.theta = pt1.theta; // 헤딩 유지
        new_pt.delta = pt2.delta; // 평행 이동을 위한 타겟 조향각으로 완전 고정!
        new_pt.v = (1.0 - r) * pt1.v + r * pt2.v;
        return new_pt;
    } 
    // spin mode : x,y 고정, 속도 0, 헤딩만 회전
    else if (new_pt.mode == VehicleMode::SpinMode) {
        new_pt.x = pt1.x;
        new_pt.y = pt1.y;
        new_pt.theta = normalizeAngle(pt1.theta + r * normalizeAngle(pt2.theta - pt1.theta));
        new_pt.delta = pt2.delta;
        new_pt.v = 0.0;
        return new_pt;
    }

    // Bicycle mode
    // 1. X, Y 좌표는 단순 선형 보간
    new_pt.x = (1.0 - r) * pt1.x + r * pt2.x;
    new_pt.y = (1.0 - r) * pt1.y + r * pt2.y;

    // 2. Theta (헤딩 각도)는 최단 경로 보간
    double diff_theta = normalizeAngle(pt2.theta - pt1.theta);
    new_pt.theta = normalizeAngle(pt1.theta + r * diff_theta);

    // 3. 조향각(steering)도 부드러운 전환을 위해 선형 보간 적용
    new_pt.delta = (1.0 - r) * pt1.delta + r * pt2.delta;

    // 4. 속도(v)도 부드러운 전환을 위해 선형 보간 적용
    new_pt.v = (1.0 - r) * pt1.v + r * pt2.v;

    return new_pt;
}

// 3차 에르미트 스플라인(Cubic Hermite Spline)
inline ReferenceTraj hermiteSplineinterpolateState(
    const ReferenceTraj& pt1, const ReferenceTraj& pt2, double r) 
{
    // pt1 -> pt2
    ReferenceTraj new_pt;
    new_pt.obs = 0.0;
    new_pt.mode = pt2.mode;

    // parallel mode: 헤딩 고정, 조향각 고정, 직선 대각선 이동
    if (new_pt.mode == VehicleMode::ParallelMode) {
        new_pt.x = (1.0 - r) * pt1.x + r * pt2.x;
        new_pt.y = (1.0 - r) * pt1.y + r * pt2.y;
        new_pt.theta = pt1.theta; // 헤딩 유지
        new_pt.delta = pt2.delta; // 평행 이동을 위한 타겟 조향각으로 완전 고정!
        new_pt.v = (1.0 - r) * pt1.v + r * pt2.v;
        return new_pt;
    } 
    // spin mode : x,y 고정, 속도 0, 헤딩만 회전
    else if (new_pt.mode == VehicleMode::SpinMode) {
        new_pt.x = pt1.x;
        new_pt.y = pt1.y;
        new_pt.theta = normalizeAngle(pt1.theta + r * normalizeAngle(pt2.theta - pt1.theta));
        new_pt.delta = pt2.delta;
        new_pt.v = 0.0;
        return new_pt;
    }

    // Bicycle mode
    double ds = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);

    // 기어가 바뀌는 구간은 스플라인을 그리지 않고 단순 선형 보간 처리 (꼬임 방지)
    double dir1 = (pt1.v >= 0.0) ? 1.0 : -1.0;
    double dir2 = (pt2.v >= 0.0) ? 1.0 : -1.0;
    
    if (dir1 != dir2) {
        new_pt.x = (1.0 - r) * pt1.x + r * pt2.x;
        new_pt.y = (1.0 - r) * pt1.y + r * pt2.y;
        new_pt.theta = normalizeAngle(pt1.theta + r * normalizeAngle(pt2.theta - pt1.theta));
        new_pt.delta = (1.0 - r) * pt1.delta + r * pt2.delta;
        new_pt.v = (1.0 - r) * pt1.v + r * pt2.v;
        new_pt.mode = pt1.mode;
        return new_pt;
    }

    double t0_x = dir1 * std::cos(pt1.theta) * ds;
    double t0_y = dir1 * std::sin(pt1.theta) * ds;
    double t1_x = dir2 * std::cos(pt2.theta) * ds;
    double t1_y = dir2 * std::sin(pt2.theta) * ds;

    double r2 = r * r;
    double r3 = r2 * r;

    double h00 =  2.0 * r3 - 3.0 * r2 + 1.0;
    double h10 =        r3 - 2.0 * r2 + r;
    double h01 = -2.0 * r3 + 3.0 * r2;
    double h11 =        r3 -       r2;

    new_pt.x = h00 * pt1.x + h10 * t0_x + h01 * pt2.x + h11 * t1_x;
    new_pt.y = h00 * pt1.y + h10 * t0_y + h01 * pt2.y + h11 * t1_y;

    double h00_dot =  6.0 * r2 - 6.0 * r;
    double h10_dot =  3.0 * r2 - 4.0 * r + 1.0;
    double h01_dot = -6.0 * r2 + 6.0 * r;
    double h11_dot =  3.0 * r2 - 2.0 * r;

    double dx_dr = h00_dot * pt1.x + h10_dot * t0_x + h01_dot * pt2.x + h11_dot * t1_x;
    double dy_dr = h00_dot * pt1.y + h10_dot * t0_y + h01_dot * pt2.y + h11_dot * t1_y;

    // 이동 속도가 거의 0일 때는 이전 헤딩을 그대로 유지 (atan2(0,0) 에러 방지)
    double speed = std::hypot(dx_dr, dy_dr);
    if (speed > 1e-3) {
        double current_dir = ((1.0 - r) * pt1.v + r * pt2.v >= 0.0) ? 1.0 : -1.0;
        new_pt.theta = std::atan2(current_dir * dy_dr, current_dir * dx_dr);
    } else {
        new_pt.theta = normalizeAngle(pt1.theta + r * normalizeAngle(pt2.theta - pt1.theta));
    }

    new_pt.delta = (1.0 - r) * pt1.delta + r * pt2.delta;
    new_pt.v = (1.0 - r) * pt1.v + r * pt2.v;

    return new_pt;
}

inline std::vector<ReferenceTraj> resampleTimeBasedTrajectory(
    const std::vector<ReferenceTraj>& spatial_path, 
    double dt)
{
    std::vector<ReferenceTraj> temporal_path;
    if (spatial_path.empty()) return temporal_path;

    // 1. 공간 경로의 각 점까지의 누적 거리(s) 계산
    std::vector<double> s_spatial(spatial_path.size(), 0.0);
    for (size_t i = 1; i < spatial_path.size(); ++i) {
        double dx = spatial_path[i].x - spatial_path[i-1].x;
        double dy = spatial_path[i].y - spatial_path[i-1].y;
        double ds = std::hypot(dx, dy);

        // 방어 로직: 제자리 회전(Spin) 등 물리적 이동 거리가 0일 때 '가상 거리' 부여
        if (ds < 1e-3) {
            // 헤딩 변화량을 거리로 환산 (1라디안 회전을 1m 전진과 동일한 비율로 취급)
            double dtheta = std::abs(normalizeAngle(spatial_path[i].theta - spatial_path[i-1].theta));
            if (dtheta > 1e-3) {
                ds = dtheta * 1.0; 
            } else {
                ds = 0.1; // 단순 모드 변경이나 기어 변속 시 제자리에 대기할 시간(버퍼) 확보
            }
        }
        s_spatial[i] = s_spatial[i-1] + ds;
    }
    // // 1. 공간 경로의 각 점까지의 누적 거리(s) 계산
    // std::vector<double> s_spatial(spatial_path.size(), 0.0);
    // for (size_t i = 1; i < spatial_path.size(); ++i) {
    //     s_spatial[i] = s_spatial[i-1] + std::hypot(
    //         spatial_path[i].x - spatial_path[i-1].x, 
    //         spatial_path[i].y - spatial_path[i-1].y
    //     );
    // }
    double total_distance = s_spatial.back();

    // 2. 초기점(t=0) 세팅
    ReferenceTraj first_pt = hermiteSplineinterpolateState(spatial_path[0], spatial_path[1], 0.0);
    first_pt.a = 0.0;
    first_pt.delta_dot = 0.0;
    temporal_path.push_back(first_pt);

    double s_target = 0.0; // 로봇이 이동해야 할 누적 목표 거리
    int idx = 0;           // 탐색을 위한 공간 경로 인덱스

    // 3. 시간(dt) 기준으로 전진하며 리샘플링 (목적지에 도달할 때까지)
    while (s_target < total_distance) {
        
        // 1. 이전 스텝의 상태를 가져와서, 다음 dt 동안 얼마나 갈지 계산
        ReferenceTraj prev_pt = temporal_path.back();
        
        // 속도가 0이 되어 무한루프에 빠지는 것을 막기 위해 최소 이동거리 0.01m 보장
        double ds = std::max(std::abs(prev_pt.v) * dt, 0.01); 
        s_target += ds;

        if (s_target >= total_distance) break;

        // 2. s_target이 위치한 공간 경로상의 선분(idx ~ idx+1) 찾기
        while (idx < spatial_path.size() - 1 && s_spatial[idx + 1] < s_target) {
            idx++;
        }

        // 3. 해당 선분 내에서의 보간 비율(r) 계산
        double segment_length = s_spatial[idx + 1] - s_spatial[idx];
        double r = (s_target - s_spatial[idx]) / segment_length;

        // 4. x, y, theta, delta, v 보간!
        ReferenceTraj new_pt = hermiteSplineinterpolateState(spatial_path[idx], spatial_path[idx + 1], r);

        // // 모드 다르면 switching time 적용(my_robot.switch_time in hastar)
        // // switching time 동안 정지함
        // double switching_time = 1.0;    // in hastar
        // if (prev_pt.mode != new_pt.mode) {
        //     int buffer_steps = switching_time / dt;
        //     ReferenceTraj align_pt = prev_pt; 
        //     align_pt.v = 0.0;
        //     align_pt.a = 0.0;
        //     align_pt.mode = new_pt.mode;

        //     double start_delta = prev_pt.delta;
        //     double end_delta = new_pt.delta;

        //     // 바퀴만 돌리도록 궤적 생성
        //     for (size_t i = 1; i <= buffer_steps; ++i) {
        //         align_pt.delta = start_delta + (end_delta - start_delta) * static_cast<double>(i / buffer_steps);
        //         align_pt.delta_dot = (end_delta - start_delta) / (buffer_steps * dt);
        //         temporal_path.push_back(align_pt);
        //     }
        //     // 버퍼가 끝난 시점부터 다시 정상 궤적 추종을 이어가기 위해 prev_pt 갱신
        //     prev_pt = temporal_path.back();
        // }
        // 5. 미분값 추출 (a, delta_dot)
        new_pt.a = (new_pt.v - prev_pt.v) / dt;
        // new_pt.a = std::clamp(new_pt.a, -MAX_DECEL, MAX_ACCEL);
        
        // 조향각속도: 각도 랩어라운드(Wrap-around)를 고려하여 차이 계산
        double diff_delta = new_pt.delta - prev_pt.delta;
        new_pt.delta_dot = diff_delta / dt;

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

#endif