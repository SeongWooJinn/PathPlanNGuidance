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

inline void initDynamicObsInParkingLot(
    mapInfo& mapinfo,
    std::vector<Obstacle>& dynamic_obs)
{
    int rows = mapinfo.map.rows();
    int cols = mapinfo.map.cols();

    double top_left_y = mapinfo.origin_y;
    double top_left_x = mapinfo.origin_x;
    double res = mapinfo.resolution;

    // Obstacle 1 위치: 입구 통로(1.5/6 지점) 좌측 끝에서 왼쪽으로 주행 (-vx)
    double aisle1_y = [&](double y)
                        {
                            return y * res + top_left_y;
                        }((1.5 * rows / 6.0));
    double obs1_x = [&](double x) 
                        {
                            return top_left_x + x * res;
                        }(cols * 0.0); // 맵 좌측 80% 지점
    dynamic_obs.push_back({obs1_x, aisle1_y, 0.8, 0.2, 0.1}); // 반경 0.8m, 속도 -0.3m/s

    // Obstacle 2 위치: 하단 통로(4.5/6 지점) 중간에서 오른쪽으로 주행 (+vx)
    double aisle2_y = [&](double y)
                        {
                            return y * res + top_left_y;
                        }((4.5 * rows / 6.0));
    double obs2_x = [&](double x) 
                        {
                            return top_left_x + x * res;
                        }(cols * 0.9); // 맵 좌측 10% 지점
    dynamic_obs.push_back({obs2_x, aisle2_y, 0.8, -0.15, 0.0}); // 반경 0.8m, 속도 0.2m/s
    
    // Obstacle 3 위치: 상단 주차장(0.5/6 지점)에서 아래쪽(통로)으로 후진하며 튀어나옴 (+vy)
    double parking_spot_y = [&](double y)
                        {
                            return y * res + top_left_y;
                        }((0.5 * rows / 6.0));
    double obs3_x = [&](double x) 
                        {
                            return top_left_x + x * res;
                        }(cols * 0.5); // 맵 정중앙 주차 구역
    dynamic_obs.push_back({obs3_x, parking_spot_y, 0.5, 0.1, 0.1}); // 반경 0.5m, y방향 속도 0.15m/s

    // Obstacle 4
    double aisle4_y = [&](double y)
                        {
                            return y * res + top_left_y;
                        }((3.0 * rows / 6.0));
    // double obs4_x = [&](double x) 
    //                     {
    //                         return top_left_x + x * res;
    //                     }(cols * 0.0); 
    // dynamic_obs.push_back({obs4_x, aisle4_y, 1.5, 0.2, 0.0}); // 반경 0.5m, y방향 속도 0.15m/s
}

inline void saveCurrentDataToBin(
    double* current_x, 
    const std::vector<Obstacle>& top_k_obs, 
    const std::string& filename) 
{
    // ios::binary 플래그를 사용하여 바이너리 쓰기 모드로 파일 열기
    std::string tmp_filename = filename + ".tmp";
    std::ofstream out(tmp_filename, std::ios::binary);
    if (!out) {
        std::cerr << "current, obs 저장 실패: " << filename << std::endl;
        return;
    }
    
    out.write(reinterpret_cast<const char*>(&current_x[0]), sizeof(double));
    out.write(reinterpret_cast<const char*>(&current_x[1]), sizeof(double));
    out.write(reinterpret_cast<const char*>(&current_x[2]), sizeof(double));

    int num_obs = top_k_obs.size();
    out.write(reinterpret_cast<const char*>(&num_obs), sizeof(int));
    for (int i = 0; i < top_k_obs.size(); ++i) {
        out.write(reinterpret_cast<const char*>(&top_k_obs[i].x), sizeof(double));
        out.write(reinterpret_cast<const char*>(&top_k_obs[i].y), sizeof(double));
        out.write(reinterpret_cast<const char*>(&top_k_obs[i].radius), sizeof(double));
    }

    out.close();
    std::rename(tmp_filename.c_str(), filename.c_str());
    // std::cout << "current x, obs가 " << filename << " 에 바이너리로 저장되었습니다." << std::endl;
}
#endif