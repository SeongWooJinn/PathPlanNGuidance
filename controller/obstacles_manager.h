#ifndef OBSTACLES_MANAGER_H
#define OBSTACLES_MANAGER_H

#include "structs_mpc.h"
#include <vector>

struct Obstacle
{
    double x, y, radius, vx, vy;
};

class ObstacleManager
{
public:
    ObstacleManager(const GridMap<int>& map, const int total_num_obs) 
        : map_(map), total_num_obs_(total_num_obs)
    { }

    inline std::vector<Obstacle> getTopKObstacles(
        const double *current_x,
        const std::vector<Obstacle>& all_dynamic_obs,
        double dynamic_obs_dist,
        double static_obs_radius)                      
    {
        std::vector<Obstacle> combined_obs;
        combined_obs.reserve(total_num_obs_); // 메모리 재할당 방지

        dynamic_obs_ = getTopKDynamicObstacles(current_x, all_dynamic_obs, dynamic_obs_dist);
        combined_obs.insert(combined_obs.end(), dynamic_obs_.begin(), dynamic_obs_.end());

        num_static_obs_ = total_num_obs_ - num_dynamic_obs_;
        static_obs_ = getTopKStaticObstacles(current_x, static_obs_radius);
        combined_obs.insert(combined_obs.end(), static_obs_.begin(), static_obs_.end());
        
        // if (static_obs_.size() > 0)
        //     std::cout << "static num : " << static_obs_.size() << std::endl;
        // 남은 빈자리를 모두 유령 장애물로 강제로 꽉꽉 채워 넣습니다.
        while (combined_obs.size() < total_num_obs_) {
            // 주의: radius를 0.0이 아닌 0.1로 주어 혹시 모를 0 나누기(Divide by Zero) 에러 방지
            combined_obs.push_back({-10000.0, -10000.0, 0.1, 0.0, 0.0});
        }
        return combined_obs;
    }
private:
    // double* current_x_;
    GridMap<int> map_;
    std::vector<Obstacle> dynamic_obs_;
    std::vector<Obstacle> static_obs_;
    int total_num_obs_{0};        // generate_mpc 파이썬에서 정의
    int num_dynamic_obs_{0};
    int num_static_obs_{0};

    inline std::vector<Obstacle> getTopKDynamicObstacles(
        const double *current_x, 
        const std::vector<Obstacle>& all_obs,
        double obs_dist)                      
    {
        
        std::vector<std::pair<double, Obstacle>> dist_obs;
        std::vector<Obstacle> top_obs;

        if (all_obs.empty()) return top_obs;

        double cx = current_x[0];
        double cy = current_x[1];

        for (const auto& obs: all_obs) {
            double dx = cx - obs.x;
            double dy = cy - obs.y;
            double dist_sq = dx*dx + dy*dy;
            // double dist = std::sqrt(dx * dx + dy * dy);
            // 10m thres
            if (dist_sq <= obs_dist*obs_dist)
                dist_obs.push_back({dist_sq, obs});
        }

        // 거리 기준 오름차순 정렬
        std::sort(dist_obs.begin(), dist_obs.end(),
                [](const auto& a, const auto& b) {
                    return a.first < b.first;
                });

        for (int i = 0; i < total_num_obs_; ++i) {
            if (i < dist_obs.size()) {
                top_obs.push_back(dist_obs[i].second);
            } 
        }

        num_dynamic_obs_ = top_obs.size();
        // num_static_obs_ = total_num_obs_ - num_dynamic_obs_;
        return top_obs;
    }

    inline std::vector<Obstacle> getTopKStaticObstacles(
        const double *current_x, 
        double static_obs_radius)                      
    {
        
        std::vector<std::pair<double, Obstacle>> dist_static_obs;
        std::vector<Obstacle> top_static_obs;

        if (num_static_obs_ <= 0) return top_static_obs; // 할당량이 없으면 즉시 종료

        int cx_px = map_.WorldXToXi(current_x[0]);
        int cy_px = map_.WorldYToYi(current_x[1]);
        int static_obs_cells = static_cast<int>(static_obs_radius/map_.pixel_scale_);

        for (int dx = -static_obs_cells; dx <= static_obs_cells; ++dx) {
            for (int dy = -static_obs_cells; dy <= static_obs_cells; ++dy) {
                int xi = cx_px + dx;
                int yi = cy_px + dy;
                if(!map_.InRange(xi, yi)) continue;
                if(map_(yi, xi) > 0) {        // 1 -> obs
                    double ox = map_.XiToWorldX(xi);
                    double oy = map_.YiToWorldY(yi);
                    double dist_x = ox - current_x[0];
                    double dist_y = oy - current_x[1];
                    double dist_sq = dist_x*dist_x + dist_y*dist_y;
                    Obstacle static_obs{ox, oy, map_.pixel_scale_ / 2.0, 0.0, 0.0};
                    dist_static_obs.push_back({dist_sq, static_obs});
                }
            }
        }

        // 거리 기준 오름차순 정렬
        std::sort(dist_static_obs.begin(), dist_static_obs.end(),
                [](const auto& a, const auto& b) {
                    return a.first < b.first;
                });
                
        // 공간 필터링 (Non-Maximum Suppression) 추가
        // 벽을 구성하는 점들이 한 곳에 뭉치지 않도록, 최소 1.5m 간격을 두고 대표점만 추출
        double min_separation = 1.5; 
        double min_sep_sq = min_separation * min_separation;

        for (int i = 0; i < dist_static_obs.size(); ++i) {
            if (top_static_obs.size() >= num_static_obs_) break; // 할당량 채우면 종료

            const Obstacle& candidate = dist_static_obs[i].second;
            bool is_too_close = false;

            // 이미 선발된 대표 장애물들과 거리를 비교하여 너무 가까우면 기각
            for (const auto& selected : top_static_obs) {
                double dx = candidate.x - selected.x;
                double dy = candidate.y - selected.y;
                if (dx * dx + dy * dy < min_sep_sq) {
                    is_too_close = true;
                    break;
                }
            }

            // 기존 점들과 충분히 떨어져 있는 새로운 벽의 표면이라면 선발
            if (!is_too_close) {
                top_static_obs.push_back(candidate);
            }
        }

        // 3. 최종 방어선: 부족한 슬롯은 유령 장애물로 채움
        while (top_static_obs.size() < num_static_obs_) {
            top_static_obs.push_back({-10000.0, -10000.0, 0.1, 0.0, 0.0});
        }

        // for (int i = 0; i < num_static_obs_; ++i) {
        //     if (i < dist_static_obs.size()) {
        //         top_static_obs.push_back(dist_static_obs[i].second);
        //     } else {
        //         top_static_obs.push_back({-10000.0, -10000.0, 0.1, 0.0, 0.0});
        //     }
        // }

        return top_static_obs;
    }

};

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
    double obs4_x = [&](double x) 
                        {
                            return top_left_x + x * res;
                        }(cols * 0.0); 
    dynamic_obs.push_back({obs4_x, aisle4_y, 1.5, 0.2, 0.0}); // 반경 0.5m, y방향 속도 0.15m/s
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