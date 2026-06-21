#ifndef EVAL_GUIDANCE_H
#define EVAL_GUIDANCE_H

#include "structs.h"
#include "mpc/include/structs_mpc.h"

struct guidanceLog
{
    MpcResultLog curr_log;
    int closest_idx;
};

struct evaluateGuidance
{
    double rmse_cte; // 횡방향 오차의 제곱평균제곱근 (RMSE)
    double max_cte;  // 최대 횡방향 오차
    double rmse_he;    // 헤딩 각도 오차의 RMSE
    double max_he;     // 최대 헤딩 각도 오차
    double rmse_jerk_a;
    double total_control_effort;  // 총 제어 소모량 (급제어/저크 성분 평가)
};

inline evaluateGuidance evaluateGuidanceMetrics(
    const std::vector<guidanceLog>& actual, 
    const std::vector<ReferenceTraj>& ref_traj,
    double dt)
    {
        evaluateGuidance metrics = {0.0, 0.0, 0.0, 0.0, 0.0};

        if (actual.empty() || ref_traj.empty()) {
            std::cout << "[Error] Actual logs or Reference path is empty.\n";
            return metrics;
        }

        double sum_sq_cte = 0.0;
        double sum_sq_he = 0.0;
        double sum_sq_control = 0.0;
        double max_dtheta = std::numeric_limits<double>::min();
        double sum_sq_jerk_a = 0.0;
        double sum_sq_jerk_delta = 0.0;
        
        double prev_a = 0.0;
        for (int i = 0; i < actual.size() - 1; ++i) {
            int nearest_idx = actual[i].closest_idx;
            ReferenceTraj nearest_traj = ref_traj[nearest_idx];
            
            double dx = actual[i].curr_log.x - nearest_traj.x;
            double dy = actual[i].curr_log.y - nearest_traj.y;
            double dist_sq = dx * dx + dy * dy;

            // 횡방향 오차(Crosstrack Error) 계산
            double cte = std::sqrt(dist_sq);
            sum_sq_cte += cte * cte;
            metrics.max_cte = std::max(metrics.max_cte, cte);

            // 헤딩 오차(Heading Error) 계산
            double dtheta = actual[i].curr_log.theta - nearest_traj.theta;
            while (dtheta > M_PI) dtheta -= 2 * M_PI;
            while (dtheta < -M_PI) dtheta += 2 * M_PI;
            sum_sq_he += dtheta * dtheta;
            if (std::abs(dtheta) > max_dtheta){
                max_dtheta = std::abs(dtheta);
            }
            // metrics.max_he = std::max(metrics.max_he, std::abs(dtheta));

            // 제어 소모량(Control Effort) 계산
            // 가속도(a)와 조향각 속도(delta_dot)의 제곱합을 통해 제어의 급격함을 평가
            double effort = (actual[i].curr_log.a * actual[i].curr_log.a) + 
                            (actual[i].curr_log.delta_dot * actual[i].curr_log.delta_dot);
            sum_sq_control += effort;

            // jerk a
            double jerk_a = (actual[i].curr_log.a - prev_a) / dt;
            sum_sq_jerk_a += jerk_a * jerk_a;
            
            prev_a = actual[i].curr_log.a; // 다음 루프를 위해 업데이트

        }

        metrics.max_he = max_dtheta;
        size_t N = actual.size();
        metrics.rmse_cte = std::sqrt(sum_sq_cte / N);
        metrics.rmse_he = std::sqrt(sum_sq_he / N);
        metrics.total_control_effort = sum_sq_control / N; // 평균 제어 에너지 소모율

        metrics.rmse_jerk_a = std::sqrt(sum_sq_jerk_a / N);

        return metrics;
    }

#endif