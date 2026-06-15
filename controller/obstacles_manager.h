#ifndef OBSTACLES_MANAGER_H
#define OBSTACLES_MANAGER_H

#include "structs_mpc.h"
#include <vector>

struct Obstacle
{
    double x, y, radius, vx, vy;
};

inline std::vector<Obstacle> getTopKObstacles(
    const double *current_x, 
    const std::vector<Obstacle>& all_obs,
    int k)                      // k -> 파이썬에서 정의
{
    
    std::vector<std::pair<double, Obstacle>> dist_obs;
    std::vector<Obstacle> top_obs;

    if (all_obs.empty()) return top_obs;

    double cx = current_x[0];
    double cy = current_x[1];

    for (const auto& obs: all_obs) {
        double dx = cx - obs.x;
        double dy = cy - obs.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        // 10m thres
        if (dist <= 10.0)
            dist_obs.push_back({dist, obs});
    }

    // 거리 기준 오름차순 정렬
    std::sort(dist_obs.begin(), dist_obs.end(),
              [](const auto& a, const auto& b) {
                return a.first < b.first;
              });

    for (int i = 0; i < k; ++i) {
        if (i < dist_obs.size()) {
            top_obs.push_back(dist_obs[i].second);
        } else {
            top_obs.push_back({-10000.0, -10000.0, 0.0, 0.0, 0.0});
        }
    }

    return top_obs;
}

inline void updateObstacles(std::vector<Obstacle>& curr_obs, double dt)
{
    for (auto& obs : curr_obs) {
        obs.x += obs.vx * dt;
        obs.y += obs.vy * dt;
    }
}

#endif