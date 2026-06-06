#ifndef EVAL_XHAPATH_H
#define EVAL_XHAPAHT_H
#include "structs.h"


struct evaluatePath
{
    double total_length{0.0};
    double total_yaw_changes{0.0};
    int gear_switch_cnt{0};
    double min_clearance{0.0};
};

inline evaluatePath evaluatePathMetrics(
    const std::vector<State>& path, const GridMap<double>& dist_obs)
    {
        if (path.size() < 2) return {0.0, 0.0, 0, 0.0};

        int rows = dist_obs.rows();
        int cols = dist_obs.cols();
        double res = dist_obs.pixel_scale_;
        evaluatePath metrics;
        int gear_cnt = 0;
        double min_dist = std::numeric_limits<double>::infinity();

        for (int i = 1; i < path.size(); ++i) {
            double dx = path[i].x - path[i-1].x;
            double dy = path[i].y - path[i-1].y;
            double ds = std::hypot(dx, dy);
            metrics.total_length += ds;

            if (path[i-1].gear != path[i].gear)
                gear_cnt += 1; // metrics.gear_switch_cnt++;

            double dtheta = path[i].theta - path[i-1].theta;
            while (dtheta > M_PI) dtheta -= 2 * M_PI;
            while (dtheta < - M_PI) dtheta += 2 * M_PI;
            metrics.total_yaw_changes += std::abs(dtheta);

            int xi = path[i].x / res;
            int yi = path[i].y / res;

            if (xi >= 0 && xi < cols && yi >= 0 && yi < rows) {
                double obs_dist = dist_obs(yi, xi);
                if (obs_dist < min_dist) {
                    min_dist = obs_dist;
                }
            }

        }
        metrics.gear_switch_cnt = gear_cnt;
        metrics.min_clearance = min_dist;

        return metrics;

    }




#endif 