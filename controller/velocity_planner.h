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
inline void accelerationProfile(
    std::vector<ReferenceTraj>& path, double a_acc_mag) 
{
    if (path.empty()) return;

    // 261011 수정: 재계획 시 전달된 현재 속도를 유지하여 매번 정지부터 가속하지 않는다.
    // std::vector<double> s_spatial(path.size(), 0.0);

    for(int i = 1; i < path.size(); ++i){
        if (path[i].mode == VehicleMode::SpinMode) {
            path[i].v = 0.0;
            // path[i].v = (path[i-1].v >= 0.0) ? 1e-4 : -1e-4;
            continue;
        }
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        double ds = std::hypot(dx, dy);
        // s_spatial[i] = s_spatial[i-1] + ds;

        // 등가속도 운동 공식 적용, 다음 스텝의 최대 허용 속도
        double max_v_next = std::sqrt(path[i-1].v * path[i-1].v + 2 * a_acc_mag * ds);
        // double max_v_next = std::sqrt(path[0].v * path[0].v + 2 * a_acc_mag * s_spatial[i]);
        double sign = (path[i].v >= 0) ? 1.0 : -1.0;
        if (std::abs(path[i].v) > max_v_next) {
            path[i].v = sign * max_v_next;
        }

    }

}

inline void decelerationProfile(
    std::vector<ReferenceTraj>& path, double a_dec_mag) 
{
    if (path.empty()) return;

    // 1. 최종 목적지에서의 속도는 무조건 0으로 강제 (안전하게 정지)
    path.back().v = 0.0;

    // 2. 경로의 맨 끝에서부터 앞으로 오면서(역순) 역산
    for (int i = path.size() - 2; i >= 0; --i) {
        if (path[i].mode == VehicleMode::SpinMode) {
            path[i].v = 0.0;
            // path[i].v = (path[i-1].v >= 0.0) ? 1e-4 : -1e-4;
            continue;
        }
        double dx = path[i+1].x - path[i].x;
        double dy = path[i+1].y - path[i].y;
        double ds = std::hypot(dx, dy);
        // s_spatial[i] = s_spatial[i-1] + ds;

        // 등가속도 운동 공식 적용, 다음 스텝의 최대 허용 속도
        double v_dec_lim = std::sqrt(path[i+1].v * path[i+1].v + 2 * a_dec_mag * ds);
        double sign = (path[i].v >= 0) ? 1.0 : -1.0;
        if (std::abs(path[i].v) > v_dec_lim) {
            path[i].v = sign * v_dec_lim;
        }

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
        new_pt.delta = pt1.delta;
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

    // 1. parallel mode: 헤딩 고정, 조향각 고정, 직선 대각선 이동
    if (new_pt.mode == VehicleMode::ParallelMode) {
        new_pt.x = (1.0 - r) * pt1.x + r * pt2.x;
        new_pt.y = (1.0 - r) * pt1.y + r * pt2.y;
        new_pt.theta = pt1.theta; // 헤딩 유지
        new_pt.delta = pt2.delta; // 평행 이동을 위한 타겟 조향각으로 완전 고정!
        new_pt.v = (1.0 - r) * pt1.v + r * pt2.v;
        return new_pt;
    } 
    // 2. spin mode : x,y 고정, 속도 0, 헤딩만 회전
    else if (new_pt.mode == VehicleMode::SpinMode) {
        new_pt.x = pt1.x;
        new_pt.y = pt1.y;
        new_pt.theta = normalizeAngle(pt1.theta + r * normalizeAngle(pt2.theta - pt1.theta));
        new_pt.delta = pt1.delta;
        new_pt.v = 0.0;
        return new_pt;
    }

    // 3. Bicycle mode
    double ds = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);

    // 기어가 바뀌는 구간은 스플라인을 그리지 않고 단순 선형 보간 처리 (꼬임 방지)
    double dir1 = (pt1.v >= 0.0) ? 1.0 : -1.0;
    double dir2 = (pt2.v >= 0.0) ? 1.0 : -1.0;
    
    // 두 점이 모두 BicycleMode일 때만 기어 변속 보간을 수행하도록 방어! -> 이게 없으면 spinmode에서 v, delta가 생김
    if (pt1.mode == VehicleMode::BicycleMode && pt2.mode == VehicleMode::BicycleMode && dir1 != dir2) {
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

    // double dx_dr = h00_dot * pt1.x + h10_dot * t0_x + h01_dot * pt2.x + h11_dot * t1_x;
    // double dy_dr = h00_dot * pt1.y + h10_dot * t0_y + h01_dot * pt2.y + h11_dot * t1_y;

    new_pt.theta = normalizeAngle(pt1.theta + r * normalizeAngle(pt2.theta - pt1.theta));

    // 이전 모드(Spin)의 조향각이 새 모드(Bicycle)로 오염되는 것을 완벽 차단
    // if(pt1.mode != pt2.mode) {
    //     new_pt.delta = pt2.delta;
    // }
    // else {
    //     new_pt.delta = (1.0 - r) * pt1.delta + r * pt2.delta;
    // }
    new_pt.delta = (1.0 - r) * pt1.delta + r * pt2.delta;
    new_pt.v = (1.0 - r) * pt1.v + r * pt2.v;

    return new_pt;
}

inline std::vector<ReferenceTraj> resampleTimeBasedTrajectory(
    const std::vector<ReferenceTraj>& spatial_path, 
    double dt, double acceleration = 1.0, double max_velocity = 0.26)
{
    std::vector<ReferenceTraj> temporal_path;
    if (spatial_path.empty()) return temporal_path;

    // 261011 수정: 단일 점과 잘못된 dt를 방어하고 구간별 이동 시간으로 리샘플링한다.
    if (spatial_path.size() == 1 || !std::isfinite(dt) || dt <= 0.0) {
        temporal_path.push_back(spatial_path.front());
        return temporal_path;
    }
    temporal_path.push_back(spatial_path.front());
    for (size_t i = 0; i + 1 < spatial_path.size(); ++i) {
        const auto & start = spatial_path[i];
        const auto & end = spatial_path[i + 1];
        const double distance = std::hypot(end.x - start.x, end.y - start.y);
        const double yaw_change = normalizeAngle(end.theta - start.theta);
        const bool spin = start.mode == VehicleMode::SpinMode;
        // 261011 수정: Spin은 생성 솔버의 양방향 한계 안인 0.3 rad/s로 시간화한다.
        // 261011 수정: 양끝 정지 구간은 최소 이동량 대신 제한된 삼각/사다리꼴 속도 프로파일을 만든다.
        const bool rest_to_rest = !spin && std::abs(start.v) + std::abs(end.v) < 1e-9;
        const double accel = std::max(1e-3, acceleration);
        const double peak = std::min(std::max(1e-3, max_velocity), std::sqrt(accel * distance));
        const double ramp = peak / accel;
        const double cruise = peak > 1e-9 ? std::max(0.0, distance - peak * ramp) / peak : 0.0;
        const double duration = spin ? std::abs(yaw_change) / 0.3 :
            (rest_to_rest ? 2.0 * ramp + cruise :
            2.0 * distance / (std::abs(start.v) + std::abs(end.v)));
        if (duration < 1e-9) continue;
        const int steps = std::max(1, static_cast<int>(std::ceil(duration / dt)));
        if (steps > 100000 || temporal_path.size() + steps > 1000000) {
            return {};  // 261011 수정: 비정상 경로의 무제한 메모리 증가를 거부한다.
        }
        for (int step = 1; step <= steps; ++step) {
            // 261011 수정: 가감속 구간의 위치는 속도 적분 비율로 보간한다.
            const double u = static_cast<double>(step) / steps;
            const double v0 = std::abs(start.v), v1 = std::abs(end.v);
            double r = spin || v0 + v1 < 1e-9 ? u :
                (2.0 * v0 * u + (v1 - v0) * u * u) / (v0 + v1);
            double motion_speed = start.v + u * (end.v - start.v);
            if (rest_to_rest && distance > 1e-9) {
                const double t = u * duration;
                double traveled;
                if (t < ramp) {
                    traveled = 0.5 * accel * t * t;
                    motion_speed = accel * t;
                } else if (t <= ramp + cruise) {
                    traveled = 0.5 * peak * ramp + peak * (t - ramp);
                    motion_speed = peak;
                } else {
                    const double remaining = duration - t;
                    traveled = distance - 0.5 * accel * remaining * remaining;
                    motion_speed = accel * remaining;
                }
                r = std::clamp(traveled / distance, 0.0, 1.0);
                motion_speed *= (start.gear == 1 ? -1.0 : 1.0) * duration / (steps * dt);
            }
            ReferenceTraj point{};
            // 261011 수정: 직선 보간은 원래 충돌 검사된 경로 밖으로 스플라인이 튀는 것을 막는다.
            point.x = spin ? start.x : start.x + r * (end.x - start.x);
            point.y = spin ? start.y : start.y + r * (end.y - start.y);
            point.theta = start.theta + r * yaw_change;
            point.v = spin ? 0.0 : motion_speed;
            // 261011 수정: 곡선의 조향 참조를 0으로 버리지 않고 보간하며 기어 경계의 혼합을 막는다.
            point.delta = spin ? yaw_change / (steps * dt) :
                (start.gear == end.gear && start.mode == end.mode ?
                start.delta + u * (end.delta - start.delta) : start.delta);
            point.mode = start.mode;
            // 261011 수정: 정지점에서도 outgoing 구간의 기어를 유지한다.
            point.gear = start.gear;
            point.a = (point.v - temporal_path.back().v) / dt;
            point.delta_dot = (point.delta - temporal_path.back().delta) / dt;
            temporal_path.push_back(point);
        }
        // 261011 수정: 다음 구간의 모드를 경계점에 보존해 실제 정지 후 전환할 수 있게 한다.
        temporal_path.back().mode = end.mode;
        temporal_path.back().gear = end.gear;  // 261011 수정: 기어 전환 경계 보존.
    }
    ReferenceTraj last = spatial_path.back();
    last.v = last.a = last.delta = last.delta_dot = 0.0;
    temporal_path.push_back(last);
    return temporal_path;
}


#endif
