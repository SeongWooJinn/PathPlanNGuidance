// #pragma once
#ifndef XHASTAR_H
#define XHASTAR_H
#define _USE_MATH_DEFINES
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <math.h>
#include <memory>
#include "structs.h"
#include "costmap.h"
#include "visualize.h"
#include "astar.h"
#include "vehicles.h"

using pii = std::pair<int, int>;
using pdd = std::pair<double, double>;
using tddd = std::tuple<double, double, double>;
// using int64 = long long;

class HybridAStar {
public:
    HybridAStar() = default;
    HybridAStar(
        const GridMap<int>& occ_map,
        const GridMap<double>& cost_map);
    // [추가로직]
    // 차량 모드들 등록
    void registVehicleMode(std::unique_ptr<IVehicleMode> vehicle);
    // 차량 look up table 접근
    IVehicleMode* getVehicleLUT(VehicleMode mode);

    bool run(double sx, double sy, double stheta, int sgear, VehicleMode smode,
        double gx, double gy, double gtheta);
    // get path
    std::vector<State> reconstructPath();
    
    void setWeights(PlannerWeights weights);
    void setExpotentialCostMap(const GridMap<double>& dist_obs/*const cv::Mat& dist_obs*/); 
    void setSigmoidCostMap(const GridMap<double>& dist_obs); 
    void setVoronoiFieldCostMap(const GridMap<double>& dist_obs); 
    void setNav2CostMap(const GridMap<double>& dist_obs); 
    double getTotalDistance(std::vector<State> path); 
    void visualize_searched_segs(
        double sx, double sy,
        double gx, double gy,
        int cell_size,
        std::string title);
    GridMap<double> getCostMap() const {return cost_map_;}

private:
    // map and scales
    int rows_, cols_;
    GridMap<int> occ_map_;
    GridMap<double> cost_map_;

    int N_THETA_;

    // vehicles
    //std::unique_ptr<IVehicleMode> cur_vehicle_;
    std::vector<std::unique_ptr<IVehicleMode>> all_vehicles_;
    IVehicleMode* analytic_shot_ = nullptr;
    std::vector<IVehicleMode*> vehicles_look_up_;   // unordered map으로 변경해야할수도

    // collision free approximate
    //cv::Mat dist_obs_;
    GridMap<double> dist_obs_;

    // edge cost weight
    // w_obs_ : 장애물과 거리에 대한 비용을 얼마나 크게 설정할것인지
    // w_fov_ : 진행방향과 헤딩각과의 오차에 대한 비용을 얼마나 크게 설정할것인지
    // weighted_a : 가중 a*(휴리스틱 가속화), 비용 가중치에 따라 튜닝 필요
    // double w_obs_ = 2.0;// 6.0;
    // double w_fov_ = 6.0;//6.0
    // double weighted_a = 1.7;//2.0;
    PlannerWeights planner_weights_;

    // Guided Path & Heuristic
    std::unique_ptr<AStar> guide_ptr_;
    std::vector<std::pair<double, double>> guide_path_;
    std::vector<double> cumul_guide_path_;

    // analytic sol
    bool has_analytic_path_ = false;
    IVehicleMode* mode_for_analytic_path_ptr_ = nullptr;
    int analytic_path_check_interval_ = 20;
    double possible_analytic_check_dist_ = 10.0; //5.0;

    // search structures (sparse)
    using PQE = std::tuple<double, int64>; // f, idx -> index만 관리
    std::priority_queue<PQE, std::vector<PQE>, std::greater<PQE>> open_;

    // int64 max_nodes_;
    // std::vector<double> gScore_;  // idx : g로 관리
    // std::vector<bool> closed_;
    // std::vector<PrevInfo> cameFrom_; // idx : PrevInfo world좌표로 관리
    std::unordered_map<int64, double> gScore_;  // idx : g로 관리
    std::unordered_set<int64> closed_;
    std::unordered_map<int64, PrevInfo> cameFrom_; // idx : PrevInfo world좌표로 관리

    int64 last_goal_idx_ = -1;
    double gScoreRSPath_;   // rs_path 포함된 전체 비용

    // precomputed 2D Dijkstra heuristic
    //std::vector<std::vector<double>> h2d_;
    GridMap<double> h2d_;
    bool precomputeValid;

    std::vector<PathSegment> searched_segs_;

    // ----------------- utilities -----------------
    inline int ThetaToIndex(double theta) {
        double t = fmod(theta, 2 * M_PI);
        if (t < 0) t += 2 * M_PI;
        int idx = static_cast<int>(round(t / (2 * M_PI) * N_THETA_)) % N_THETA_;
        return idx;
    }
    inline double IndexToTheta(int idx) {
        return ((double)idx / (double)N_THETA_) * 2.0 * M_PI;
    }
    inline int64 NodeToIndex(const Node& n) {
        // gearIdx: 0->forward, 1->reverse
        // vehicle mode: (0, 1, 2) 3개
        // return (((xi * cols) + yi) * N_THETA_ + thetai) * 2 + gearIdx

        int64 idx = n.yi;
        idx = idx * cols_ + n.xi;
        idx = idx * N_THETA_ + n.thetai;
        idx = idx * 2 + n.gear;
        // vehicle mode -> 0, 1, 2를 가짐
        idx = idx * 3 + static_cast<int>(n.state.vehicle);    // [추가] 260308
        return idx;
    }
    inline Node IndexToNode(int64 idx) {
        Node n;
        n.state.vehicle = static_cast<VehicleMode>(idx % 3);    // [추가] 260308
        idx /= 3;    // [추가]
        n.gear = idx % 2;
        idx /= 2;
        n.thetai = idx % N_THETA_;
        idx /= N_THETA_;
        //n.yi = idx % cols_;
        //n.xi = idx / cols_;
        n.xi = idx % cols_;
        n.yi = idx / cols_;
        return n;
    }

    // b -> a로 회전하기 위한 signed angle
    inline double angDiff(double a, double b) {
        double d = fmod(a - b, 2 * M_PI);
        if (d < -M_PI) d += 2 * M_PI;
        if (d > M_PI) d -= 2 * M_PI;
        return d;
    }
    // -pi ~ pi
    inline double normalizeAngle(double a)
    {
        a = std::fmod(a + M_PI, 2.0 * M_PI);
        if (a < 0) a += 2.0 * M_PI;
        return a - M_PI;
    }
    void getDistFromObs() {
        // Distance Transform을 위해 장애물을 0으로 
        cv::Mat grid_img(rows_, cols_, CV_8UC1);
        for (int y = 0; y < rows_; ++y) {
            for (int x = 0; x < cols_; ++x) {
                grid_img.at<uchar>(y, x) = (occ_map_(y, x) == 1) ? 0 : 255;
            }
        }
        // 각 픽셀에서 가장 가까운 장애물까지의 거리
        cv::Mat dist_px;
        cv::distanceTransform(grid_img, dist_px, cv::DIST_L2, 5);   // return CV_32FC1임

        dist_obs_ = GridMap<double>(rows_, cols_, occ_map_.pixel_scale_);

        // 반드시 float으로 매핑(CV_32FC1 이므로)
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            dist_map(dist_px.ptr<float>(), rows_, cols_);

        // double로 캐스팅한 후 스케일 곱
        dist_obs_.data_ = dist_map.cast<double>() * occ_map_.pixel_scale_;
        // visualizeCostMap(dist_obs_, 10);
    }

    bool collisionFreeApprox(double x, double y, double theta) {

        //// fast check 로봇 길이보다 길면 무조건 안전 -> 무조건 빠른건 아님, 제약이 타이트해지므로
        int yi = occ_map_.WorldYToYi(y);
        int xi = occ_map_.WorldXToXi(x);
        if (!occ_map_.InRange(xi, yi)) return false;
        if (dist_obs_.data_(yi, xi) > analytic_shot_->getRobotLength()) 
            return true;
        
        // 로봇 길이보다 짧을때에만 collision check
        std::vector<std::pair<double, double>> center_candi = analytic_shot_->getVehicleDisk();   // 차량 r은 모드와 무관
        for (const auto& circle : center_candi) {
            double cx = x + circle.first * std::cos(theta);
            double cy = y + circle.first * std::sin(theta);

            int cxi = occ_map_.WorldXToXi(cx);
            int cyi = occ_map_.WorldYToYi(cy);

            if (!occ_map_.InRange(cxi, cyi)) continue;

            if ((circle.second > dist_obs_(cyi, cxi)) ||
                cost_map_(cyi, cxi) >= 253.0) return false;
            // if ((circle.second > dist_obs_(cyi, cxi)) ||
            //     std::isinf(cost_map_(cyi, cxi))) return false;
        }
        return true;    // collision free
    }
    // --------------------- Cost Utils -------------------------
    double edgeCostHybrid_multi_value(
        Node& from, //int64 from_idx,
        Node& to, //int64 to_idx,
        const PathSegment& seg)
    {
        //Node from = IndexToNode(from_idx);
        //Node to = IndexToNode(to_idx);

        double cost = 0.0;

        // 1. for base cost
        // base cost = length + curvature + steering error + reverse + gear switch cost
        //auto target_vehicle = vehicles_look_up_[(static_cast<int>(to.vehicle))];
        auto target_vehicle = vehicles_look_up_[(static_cast<int>(to.state.vehicle))];
        double base_cost = target_vehicle->getEdgeCost(seg, from, to);

        // 2. for obstacle avoidance cost
        double d_max = -1.0f;
        for (auto& p : seg.samples) {
            int xi = occ_map_.WorldXToXi(p.x);
            int yi = occ_map_.WorldYToYi(p.y);

            if (!occ_map_.InRange(xi, yi)) {
                return std::numeric_limits<double>::infinity();
            }

            double cost = cost_map_(yi, xi);
            if (cost >= 253.0) {
                return std::numeric_limits<double>::infinity();
            }
            d_max = std::max(d_max, cost);
        }
        double obs_cost = d_max;    // 0~255.0

        // 3. for perception aware cost
        // 진행방향 psi계산
        double dx = seg.end.x - seg.start.x;
        double dy = seg.end.y - seg.start.y;
        double psi = std::atan2(dy, dx);

        // 헤딩과 진행방향 차이(이탈각 alpha) 계산
        double alpha = std::fabs(angDiff(psi, seg.end.theta));

        //double cur_mode_vel = target_vehicle->getRefVelocity();
        double fov_half = target_vehicle->getSensorFOVHalf();
        double fov_cost = 0.0;
        if (alpha > fov_half) {
            double blind_ang = alpha - fov_half;
            fov_cost = blind_ang * blind_ang;
        }
        else {
            // fov 내부더라도 중앙에서 멀어지면 선형 비용 할당
            fov_cost = alpha;
        }

        // 4. 최종 cost 
        cost += base_cost;
        cost += planner_weights_.w_obs * obs_cost;
        cost += (planner_weights_.w_fov * fov_cost * COST_SCALE);
        return cost;
    }

    /** ---------------- Heuristic Utils ------------------
     * Dijkstra heuristic function is only for distance to closer to goal
     */
    void computeDijkstraHeuristic(int gx, int gy) {
        //h2d_.assign(rows_, std::vector<double>(cols_, std::numeric_limits<double>::infinity()));
        h2d_.data_.setConstant(std::numeric_limits<double>::infinity());
        using T = std::pair<double, pii>;
        std::priority_queue<T, std::vector<T>, std::greater<T>> pq;
        //if (!InGrid(gx, gy) || occ_map_[gy][gx]) return;
        if (!h2d_.InRange(gx, gy) || occ_map_(gy, gx)) return;
        h2d_(gy, gx) = 0.0;
        pq.push({ 0.0, {gx, gy} });
        std::vector<pii> nbrs = { {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1} };
        while (!pq.empty()) {
            auto top = pq.top(); pq.pop();
            double cd = top.first; int cx = top.second.first, cy = top.second.second;
            if (cd > h2d_(cy, cx)) continue;
            for (auto& d : nbrs) {
                int nx = cx + d.first, ny = cy + d.second;
                if (!occ_map_.InRange(nx, ny)) continue;
                if (occ_map_(ny, nx)) continue;
                if (cost_map_(ny, nx) >= 250.0) continue;
                double eu = std::hypot((double)d.first, (double)d.second) * occ_map_.pixel_scale_;
                // cost_map의 페널티를 '미터(m)' 단위로 환산하여 물리적 거리(eu)에 더해줌
                double penalty = (planner_weights_.w_obs * cost_map_(ny, nx)) / COST_SCALE;
                // double penalty = cost_map_(ny, nx) / COST_SCALE;
                double edge_cost = eu + penalty;
                // double edge_cost = eu; // cost at neighbor times dist
                double nd = cd + edge_cost;
                if (nd < h2d_(ny, nx)) {
                    h2d_(ny, nx) = nd;
                    pq.push({ nd, {nx, ny} });
                }
            }
        }
        precomputeValid = true;
    }
    void setGuidePathOptimize(std::vector<std::pair<double, double>>& raw_gp) {
        //guide_path_optimized_.clear();
        //guide_path_optimized_.resize(raw_gp.size());
        cumul_guide_path_.clear();
        cumul_guide_path_.resize(raw_gp.size());

        double cumul_dist = 0.0;
        for (int i = raw_gp.size() - 1; i >= 0; --i) {
            if (i < raw_gp.size() - 1) {
                double dx = raw_gp[i].first - raw_gp[i + 1].first;
                double dy = raw_gp[i].second - raw_gp[i + 1].second;
                cumul_dist += std::hypot(dx, dy);
            }
            cumul_guide_path_[i] = cumul_dist;
        }
    }
    bool isCorridor(const PathSegment& seg, int base_idx, double thres) {
        if (base_idx < 0) return false;

        // 확장한 노드와 가이드 패스간의 수직거리계산
        double cx = seg.end.x; double cy = seg.end.y;   // seg.start?? seg.end???
        double min_dist = std::numeric_limits<double>::infinity();

        int start_idx = std::max(0, base_idx - 10);  // 조금 뒤부터
        int end_idx = std::min(static_cast<int>(guide_path_.size()), base_idx + 15); // 약간 앞까지

        for (int i = start_idx; i < end_idx; ++i) {
            double dx = guide_path_[i].first - cx;
            double dy = guide_path_[i].second - cy;
            double dist_sq = dx * dx + dy * dy;
            if (min_dist > dist_sq) min_dist = dist_sq;
        }
        // 허용 반경을 넘으면 스킵
        //if (std::sqrt(min_dist) > thres) return false;
        if (min_dist > thres * thres) return false;
        return true;
    }
    // guide path heurisic
    double heuristic_guided_path(Node& next_n, Node& goal_n)
    {
        // h = h_dist + h_cross(현재 위치 - 가이드 패스까지 이탈거리)
        // 1. 현재위치에서 가까운 guide path index와 h_cross(이탈 거리)
        double nearest_dist = std::numeric_limits<double>::infinity();
        int curr_nearest_idx = -1;

        if (next_n.nearest_guide_idx == -1) {
            for (int i = 0; i < guide_path_.size(); ++i) {
                double dx = guide_path_[i].first - next_n.state.x;
                double dy = guide_path_[i].second - next_n.state.y;
                double dist_sq = dx * dx + dy * dy;
                if (nearest_dist > dist_sq) {
                    nearest_dist = dist_sq;
                    curr_nearest_idx = i;
                }
            }
        }
        else {
            int start_search = std::max(0, next_n.nearest_guide_idx - 10);  // 조금 뒤부터
            int end_search = std::min(static_cast<int>(guide_path_.size()), next_n.nearest_guide_idx + 20); // 약간 앞까지
            for (int i = start_search; i < end_search; ++i) {
                double dx = guide_path_[i].first - next_n.state.x;
                double dy = guide_path_[i].second - next_n.state.y;
                double dist_sq = dx * dx + dy * dy;
                if (nearest_dist > dist_sq) {
                    nearest_dist = dist_sq;
                    curr_nearest_idx = i;
                }
            }
        }
        // std::cout << "curr_nearest_idx : " << curr_nearest_idx << std::endl;

        double h_cross = std::sqrt(nearest_dist);

        // nearest_guide_idx 갱신
        next_n.nearest_guide_idx = curr_nearest_idx;

        // 2. nearest index에서 goal까지의 가이드 패스 거리 h_dist
        // 미리 저장해놓은 배열에 접근 O(1)
        double h_dd = cumul_guide_path_[curr_nearest_idx];

        // 가이드 패스에서 이탈하지 않도록 x 2.0
        return 2.0 * h_cross + h_dd;
    }
    // dijkstra heuristic
    double heuristic_dij(int nxi, int nyi) {
        if (!precomputeValid) return 0.0;
        if (!occ_map_.InRange(nxi, nyi)) return std::numeric_limits<double>::infinity();
        return h2d_(nyi, nxi) * COST_SCALE;
    }
    double heuristic_guide_kinematic(Node& next_n, Node& goal_n) {

        // =========== minimum heuristic =========== 
        double h = std::numeric_limits<double>::infinity();
        for (const auto& p : vehicles_look_up_) {
            double candi_h = p->getKinematicHeuristic(next_n, goal_n);
            if (candi_h < h)
                h = candi_h;
        }
        h *= COST_SCALE;
        
        // =========== guide path heuristic===========
        double h_g = heuristic_guided_path(next_n, goal_n) * COST_SCALE;
        
        double final_h = 0.0;
        //final_h = std::max(h, h2d_[next_n.yi][next_n.xi]);   // holonomic + non-holonomic heuristic
        final_h = std::max(h, h_g);   // holonomic + guide path heuristic

        // cur_mode != goal_mode -> 반드시 전환이 발생하므로 heuristic에 더해도 무방
        // -----> 어차피 끝은 analytic path여서 bicycle mode이어야함
        if (next_n.state.vehicle != goal_n.state.vehicle)
            final_h += getVehicleLUT(goal_n.state.vehicle)->getSwitchCost() * COST_SCALE;
            // final_h += getVehicleLUT(goal_n.state.vehicle)->getSwitchCost();

        return final_h;
    }
    double heuristic_dij_kinematic(Node& next_n, Node& goal_n) {

        // =========== minimum heuristic =========== 
        double h = std::numeric_limits<double>::infinity();
        for (const auto& p : vehicles_look_up_) {
            double candi_h = p->getKinematicHeuristic(next_n, goal_n);
            if (candi_h < h)
                h = candi_h;
        }
        h *= COST_SCALE;

        // w_obs(1.0)랑 h2d_(1.1)가중치 간의 균형 필요
        double final_h = 0.0;
        final_h = std::max(h, 1.1 * h2d_(next_n.yi, next_n.xi) * COST_SCALE);   // holonomic + non-holonomic heuristic

        // cur_mode != goal_mode -> 반드시 전환이 발생하므로 heuristic에 더해도 무방
        // -----> 어차피 끝은 analytic path여서 bicycle mode이어야함
        if (next_n.state.vehicle != goal_n.state.vehicle)
            final_h += getVehicleLUT(goal_n.state.vehicle)->getSwitchCost() * COST_SCALE;

        return final_h;
    }

    double getRSPathCosts(
        std::vector<State>& rs_path,
        double cur_g)
    {
        int cur_mode = -1;
        for (int i = 0; i < vehicles_look_up_.size(); ++i) {
            if (vehicles_look_up_[i] == mode_for_analytic_path_ptr_)
                cur_mode = i;
        }
        double total_cost = cur_g;
        for (size_t i = 0; i < rs_path.size() - 1; ++i) {
            const State& s1 = rs_path[i];
            const State& s2 = rs_path[i + 1];

            Node from;
            from.state = s1;
            from.xi = occ_map_.WorldXToXi(s1.x); from.yi = occ_map_.WorldYToYi(s1.y);
            from.thetai = ThetaToIndex(s1.theta); from.gear = s1.gear;
            from.state.vehicle = static_cast<VehicleMode>(cur_mode);

            Node to;
            to.state = s2;
            to.xi = occ_map_.WorldXToXi(s2.x); to.yi = occ_map_.WorldYToYi(s2.y);
            to.thetai = ThetaToIndex(s2.theta); to.gear = s2.gear;
            to.state.vehicle = static_cast<VehicleMode>(cur_mode);

            PathSegment rs_seg;
            rs_seg.start = s1;
            rs_seg.end = s2;
            rs_seg.length = std::hypot(s1.x - s2.x, s1.y - s2.y);
            rs_seg.samples = { s2 };

            //total_cost += edgeCostHybrid_multi_value(NodeToIndex(from), NodeToIndex(to), rs_seg);
            total_cost += edgeCostHybrid_multi_value(from, to, rs_seg);
        }
        return total_cost;
    }

};

#endif
