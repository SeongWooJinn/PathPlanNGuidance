#include "xhastar.h"
#include "costmap.h"

HybridAStar::HybridAStar(
    const GridMap<int>& occ_map,
    const GridMap<double>& cost_map)
    : occ_map_(occ_map), cost_map_(cost_map)
{
    rows_ = occ_map_.rows();
    cols_ = occ_map_.cols();
    h2d_ = GridMap<double>(rows_, cols_, occ_map_.pixel_scale_);
    h2d_.data_.setConstant(std::numeric_limits<double>::infinity());

    // for continuous path
    N_THETA_ = 72;
    precomputeValid = false;

    // for collisionFreeCheck
    getDistFromObs();  // return dist_obs_

}

// [추가로직]
// 차량 모드들 등록
void HybridAStar::registVehicleMode(std::unique_ptr<IVehicleMode> vehicle)
{
    // unique_ptr의 소유권 이전 전 필요 정보 저장
    VehicleMode mode = vehicle->getModeType();
    IVehicleMode* vehicle_ptr = vehicle.get();
    int idx = static_cast<int>(mode);

    // analytic path를 위해 bicycle모드 포인터는 저장, analytic path가 여러 모드라면 vector로 shot을 바꿔야함
    if (mode == VehicleMode::BicycleMode) {
        analytic_shot_ = vehicle_ptr;
    }
    // 소유권 이전, vehicle -> nullptr
    all_vehicles_.emplace_back(std::move(vehicle));

    // look up table 등록
    if (vehicles_look_up_.size() <= idx) {
        vehicles_look_up_.resize(idx + 1, nullptr);
    }
    vehicles_look_up_[idx] = vehicle_ptr;
}
// 차량 look up table 접근
IVehicleMode* HybridAStar::getVehicleLUT(VehicleMode mode)
{
    int idx = static_cast<int>(mode);
    if (idx >= 0 && idx < vehicles_look_up_.size())
        return vehicles_look_up_[idx];
    return nullptr;
}

// ---- public run implementation ----
bool HybridAStar::run(double sx, double sy, double stheta, int sgear, VehicleMode smode,
    double gx, double gy, double gtheta)//, int ggear)
{
    if (sgear == 0.0 || sgear == 1.0) {} // 0.0 : forward, 1.0 : reverse
    else { std::cerr << "start gear error \n"; return false; }
    /* if (ggear == 0.0 || ggear == 1.0) {}
        else { std::cerr << "goal gear error \n"; return false; }*/
    stheta = normalizeAngle(stheta); gtheta = normalizeAngle(gtheta);

    // meter & radian to index
    int start_xi = occ_map_.WorldXToXi(sx);
    int start_yi = occ_map_.WorldYToYi(sy);
    int start_ti = ThetaToIndex(stheta);

    int goal_xi = occ_map_.WorldXToXi(gx);
    int goal_yi = occ_map_.WorldYToYi(gy);
    int goal_ti = ThetaToIndex(gtheta);

    std::cout << " ================= VEHICLE / START / GOAL INFORMATION ================= " << std::endl;
    std::cout << "(" << start_xi << ", " << start_yi << ", " << start_ti << ")" << std::endl;
    std::cout << "(" << goal_xi << ", " << goal_yi << ", " << goal_ti << ")" << std::endl;
    //std::cout << "min_turn_radius : " << cur_vehicle_->getMinTurnR() << ")" << std::endl;
    for (const auto& vehicle : all_vehicles_) {
        std::cout << "Vehicle Mode : " << vehicle->getModeType() << ", " <<
            "min_turn_radius : " << vehicle->getMinTurnR() << std::endl;
    }

    //if (!InGrid(start_xi, start_yi) || !InGrid(goal_xi, goal_yi)) return false;
    if (!occ_map_.InRange(start_xi, start_yi) || !occ_map_.InRange(goal_xi, goal_yi)) return false;

    // obstacle cost map, update cost_map_
    switch (planner_weights_.costmap_type)
    {
    case 0:
        setVoronoiFieldCostMap(dist_obs_);
        break;
    case 1:
        setExpotentialCostMap(dist_obs_);
        break;
    case 2:
        setSigmoidCostMap(dist_obs_);
        break;
    default:
        setNav2CostMap(dist_obs_);
        break;
    }

    if (cost_map_(start_yi, start_xi) >= 253.0)
    {
        std::cerr << "StartOnInValidSpace" << std::endl;
        return false;
    }
    // check collision of goal position
    if (!collisionFreeApprox(gx, gy, gtheta)) 
    {
        std::cerr << "[Error] Goal pose (including footprint) is in collision!" << std::endl;
        return false;
    }
    if (cost_map_(goal_yi, goal_xi) >= 253.0)
    {
        std::cerr << "GolaOnInValidSpace" << std::endl;
        return false;
    }
    
    if (!planner_weights_.use_guide_heuristic)
        // Non-holonomic Heuristic
        computeDijkstraHeuristic(goal_xi, goal_yi);  // return dijkstra distance h2d_

    // guide path
    if (planner_weights_.use_guide_heuristic){
        guide_ptr_ = std::make_unique<AStar>(occ_map_, cost_map_);
        //auto guide_ptr_start = std::chrono::system_clock::now();
        bool is_guide_path;
        is_guide_path = guide_ptr_->run(start_xi, start_yi, goal_xi, goal_yi);  // input -> index
        std::cout << "GUIDE PATH SUCCESS : " << is_guide_path << std::endl;

        guide_path_ = guide_ptr_->reconstructPath_resample(gx, gy); // input -> meter
        if (guide_path_.empty()) {
            std::cerr << "[Error] : NO Feasible Path" << std::endl;
            return false;
        }
        if (planner_weights_.is_standalone)
            visualize_guide_path(guide_path_, occ_map_, 10, "Astar_Guide_Path", occ_map_.pixel_scale_);
        else
            visualize_ros2_guide_path(guide_path_, occ_map_, 10, "Astar_Guide_Path_ros2", occ_map_.pixel_scale_);

        setGuidePathOptimize(guide_path_);
    }

    // clear structures
    gScore_.clear(); closed_.clear(); cameFrom_.clear();
    // std::fill(gScore_.begin(), gScore_.end(), std::numeric_limits<double>::infinity());
    // std::fill(closed_.begin(), closed_.end(), false);

    // state idx and start/goal info save for loop
    Node start_n;
    start_n.xi = start_xi; start_n.yi = start_yi; start_n.thetai = start_ti; start_n.gear = sgear;
    start_n.state.x = sx; start_n.state.y = sy; start_n.state.theta = stheta; start_n.state.gear = sgear; start_n.state.steering = 0.0;
    start_n.state.vehicle = smode;     //   260308      
    //start_n.vehicle = smode;     // 초기 vehicle mode
    start_n.g = 0.0;

    Node goal_n;
    goal_n.xi = goal_xi; goal_n.yi = goal_yi; goal_n.thetai = goal_ti;
    goal_n.gear = 0;    // goal gear 무의미
    goal_n.state.x = gx; goal_n.state.y = gy; goal_n.state.theta = gtheta; goal_n.state.steering = 0.0;
    goal_n.state.gear = 0;    // goal gear 무의미
    // [추가로직]
    goal_n.state.vehicle = VehicleMode::BicycleMode;     //  260308(because of rs path)
    //goal_n.vehicle = VehicleMode::ParallelMode; 
    goal_n.g = 0.0;     // path 찾은 후 할당
    goal_n.h = 0.0;     // path 찾은 후 할당

    if (planner_weights_.use_guide_heuristic)
        start_n.h = heuristic_guide_kinematic(start_n, goal_n);      // for guided path heuristic
    else
        start_n.h = heuristic_dij_kinematic(start_n, goal_n);       // just for rs dij heurisitc

    int64 start_idx = NodeToIndex(start_n);
    gScore_[start_idx] = start_n.g;
    open_.push({ start_n.g + start_n.h, start_idx });

    PrevInfo s_info;
    s_info.g = gScore_[start_idx];
    s_info.parent_idx = start_idx;
    s_info.end = start_n.state;
    //s_info.vehicle = start_n.vehicle;  // 260308
    s_info.parent_guide_path_idx = start_n.nearest_guide_idx; // 시작 노드가 찾은 인덱스 저장!
    cameFrom_[start_idx] = s_info;

    // rs용 goal state
    State goal_state;
    goal_state = goal_n.state;

    int iter = 0;
    while (!open_.empty() && iter < 500000) {
        auto [f, cur_idx] = open_.top();
        open_.pop();

        if (closed_.count(cur_idx)) { ++iter; continue; }
        // if (closed_[cur_idx]) { ++iter; continue; }

        State cur_state = cameFrom_[cur_idx].end;
        cur_state.theta = normalizeAngle(cur_state.theta);
        Node cur_n = IndexToNode(cur_idx); // get current node
        cur_n.state = cur_state;
        cur_n.nearest_guide_idx = cameFrom_[cur_idx].parent_guide_path_idx; // [추가] 부모의 인덱스 복원!

        // // loop 터미널 출력은 속도 저하됨
        // std::cout << "iter: " << iter << " -> " << std::endl;
        //    "curr state : " << "x, y, theta, gear, steer, vehiclemode : (" << cur_state.x << ", " << cur_state.y << ", " <<
        //    cur_state.theta << ", " << cur_state.gear << ", " << cur_state.steering << ", " << cur_state.vehicle << ")" << std::endl;

        double dist2goal = std::hypot(cur_state.x - gx, cur_state.y - gy);
        // if (dist2goal < 10.0) {
        //     std::cout << "Near Goal! Iter: " << iter <<  std::endl;
        // }
        // periodically check analyticPath
        bool check_rs = true;
        if (iter % analytic_path_check_interval_ == 0) {
            if (dist2goal <= possible_analytic_check_dist_) {
                //[추가로직]
                auto checker = [this](double x, double y, double theta) -> bool {
                    return collisionFreeApprox(x, y, theta);
                    };
                // analytic_shot_은 analytic path를 사용하기위한 포인터 멤버변수, 여러 analytic이라면 반복문으로 구현해야함
                // 공통 인터페이스 클래스에서 false 반환하므로 analytic 가능한 것들만 보면됨
                if (!analytic_shot_->tryAnalyticExpansion(cur_state, goal_state, checker)) check_rs = false;
                if (check_rs) {
                    std::vector<State> analyticpath = analytic_shot_->getAnalyticPath();
                    has_analytic_path_ = true;
                    mode_for_analytic_path_ptr_ = analytic_shot_;
                    gScoreRSPath_ = getRSPathCosts(analyticpath, gScore_[cur_idx]); // rs_path포함한 전체 경로비용
                    std::cout << "############# Find Analytic Path!! #############" << std::endl;
                    std::cout << "Total cost, Cost before analytic : " << gScoreRSPath_ << ", " << gScore_[cur_idx] << "\n";
                    std::cout << "Iteration : " << iter << std::endl;
                    last_goal_idx_ = cur_idx;   // rs 직전의 노드 인덱스
                    // std::cout << "analytic_path : " << "\n";
                    // for (const auto& p : analyticpath) {
                    //     std::cout << "(" << p.x << ", " << p.y << ", " << p.theta <<
                    //         ", " << p.steering << ", " << p.gear << ")" << std::endl;
                    // }
                    return true;
                }
                // [추가 로직] RS 곡선은 실패했지만, 거리가 매우 가깝고 헤딩이 비슷하다면 강제 성공 처리
                double xy_tolerance = 0.5; // 0.5m 이내
                double yaw_tolerance = 10.0 * M_PI / 180.0; // 10도 이내
                
                if (dist2goal < xy_tolerance && std::fabs(angDiff(cur_state.theta, gtheta)) < yaw_tolerance) {
                    std::cout << "############# Reached within Goal Tolerance! #############" << std::endl;
                    has_analytic_path_ = false; // RS 패스는 없음
                    last_goal_idx_ = cur_idx;
                    return true;
                }
            }
        }

        closed_.insert(cur_idx);
        // closed_[cur_idx] = true;
        // successors: steering_set x directions
        //for (double delta : steering_set()) { // delta -> steering
        for (const auto& mode : all_vehicles_) {
            for (const auto& seg : mode->getSuccessors(cur_state)) {
                if (planner_weights_.use_guide_heuristic) {
                    if (!isCorridor(seg, cur_n.nearest_guide_idx, 5.0)) continue; // for guided path heuristic, thres custom
                }
                bool sample_ok = true;
                for (auto& pp : seg.samples) { // pp -> for bound check in samples
                    int xi_s = occ_map_.WorldXToXi(pp.x), yi_s = occ_map_.WorldYToYi(pp.y);
                    //if (!InGrid(xi_s, yi_s)) { sample_ok = false; break; }
                    if (!occ_map_.InRange(xi_s, yi_s)) { sample_ok = false; break; }
                }
                if (!sample_ok) continue;
                // check collision with obs grids in samples by footprint
                bool collision = false;
                for (auto& pp : seg.samples) { // pp -> for bound check in samples
                    if (!collisionFreeApprox(pp.x, pp.y, pp.theta)) {
                        collision = true; break;
                    }
                }
                //searched_segs_.emplace_back(seg);   // save searched segs 
                // compute next discrete state
                Node next_n;
                next_n.state = seg.end;
                next_n.xi = occ_map_.WorldXToXi(seg.end.x);
                next_n.yi = occ_map_.WorldYToYi(seg.end.y);
                next_n.thetai = ThetaToIndex(seg.end.theta);
                next_n.gear = seg.end.gear;
                //next_n.vehicle = mode->getModeType();   // [추가]k
                next_n.nearest_guide_idx = cur_n.nearest_guide_idx;   // [추가]

                if (!occ_map_.InRange(next_n.xi, next_n.yi)) continue;
                if (next_n.thetai < 0 || next_n.thetai >= N_THETA_) continue;

                int64 next_idx = NodeToIndex(next_n);
                if (closed_.count(next_idx)) continue;
                // if (closed_[next_idx]) continue;

                double tentative_g = gScore_[cur_idx] +
                    edgeCostHybrid_multi_value(cur_n, next_n, seg); //edgeCostHybrid_multi_value(cur_idx, next_idx, seg);
                if (!gScore_.count(next_idx) || tentative_g < gScore_[next_idx]) {
                // if (tentative_g < gScore_[next_idx]) {
                    gScore_[next_idx] = tentative_g;
                    double h = 0.0;
                    if (planner_weights_.use_guide_heuristic)
                        h = heuristic_guide_kinematic(next_n, goal_n); // for guide path heurisic
                    else
                        h = heuristic_dij_kinematic(next_n, goal_n); // reed-shepp & dijkstra
                    next_n.g = gScore_[next_idx];
                    next_n.h = h;

                    PrevInfo info;
                    info.parent_idx = cur_idx;
                    info.g = tentative_g;
                    info.h = h;
                    info.end = seg.end;
                    //info.vehicle = next_n.vehicle;
                    //info.vehicle = next_n.state.vehicle;  // 260308
                    info.parent_guide_path_idx = next_n.nearest_guide_idx;

                    cameFrom_[next_idx] = info;

                    double f_new = tentative_g + planner_weights_.weighted_a * h;     
                    // if (planner_weights_.use_guide_heuristic)
                    //     f_new = tentative_g + planner_weights_.weighted_a * h;  // weight a*, g와 h가 스케일이 안맞을 수도 있으므로 스케일 조정
                    // else
                    //     f_new = tentative_g + h;  
                    open_.push({ f_new, next_idx });
                }
            }
        }
        ++iter;
    } // end
    return false;
}
// get path
std::vector<State> HybridAStar::reconstructPath()
//std::vector<std::pair<State, VehicleMode>> reconstructPath()
{
    std::vector<State> out;
    //std::vector<std::pair<State, VehicleMode>> out;

    if (last_goal_idx_ < 0) { std::cout << "No Hybrid A* path\n"; return out; }
    if (!out.empty()) { std::cout << "parameter error\n"; return out; }

    int64 cur = last_goal_idx_;

    while (true) {
        auto it = cameFrom_.find(cur);
        if (it == cameFrom_.end()) break;

        // previnfo에 vehicle정보 넣기
        const PrevInfo& info = it->second;
        // const PrevInfo& info = cameFrom_[cur];
        out.emplace_back(info.end);
        //out.push_back({ info.end, info.vehicle });

        if (info.parent_idx == cur) break;  // 종료
        cur = info.parent_idx;
    }

    std::reverse(out.begin(), out.end());
    // [수정]
    //if (has_analytic_path_) {
    //    out.insert(out.end(),
    //        mode_for_analytic_path_ptr_->getAnalyticPath().begin() + 1,
    //        mode_for_analytic_path_ptr_->getAnalyticPath().end()); // Reed-Shepp path insert
    //}
    if (has_analytic_path_) {
        auto& analytic_path = mode_for_analytic_path_ptr_->getAnalyticPath();
        if (analytic_path.size() > 1) {
            for (size_t i = 1; i < analytic_path.size(); ++i)
                //out.push_back({ analytic_path[i], VehicleMode::BicycleMode });
                out.push_back(analytic_path[i]);
        }
    }
    return out;
}

void HybridAStar::setWeights(PlannerWeights weights) {
    planner_weights_ = weights;
}
void HybridAStar::setExpotentialCostMap(const GridMap<double>& dist_obs/*const cv::Mat& dist_obs*/) {
    // expotentialCostMap(dist_obs, occ_map_, cost_map_, 10.0, analytic_shot_->getSigmaRobotObs(), "expotential_cost_map_ros2");
    expotentialCostMap(dist_obs, occ_map_, cost_map_, planner_weights_.exp_decay_rate , analytic_shot_->getSigmaRobotObs(), "expotential_cost_map_ros2");
}
void HybridAStar::setSigmoidCostMap(const GridMap<double>& dist_obs) {
    // sigmoidCostMap(dist_obs, occ_map_, cost_map_, 10.0, analytic_shot_->getSigmaRobotObs(),"sigmoid_cost_map_ros2");
    sigmoidCostMap(dist_obs, occ_map_, cost_map_, planner_weights_.sig_inflation_w, analytic_shot_->getSigmaRobotObs(),"sigmoid_cost_map_ros2");
}
void HybridAStar::setVoronoiFieldCostMap(const GridMap<double>& dist_obs) {
    // VoronoiFieldCostMap(dist_obs, occ_map_, cost_map_, analytic_shot_->getSigmaRobotObs(), "voronoifield_cost_map_ros2");
    VoronoiFieldCostMap(dist_obs, occ_map_, cost_map_, analytic_shot_->getSigmaRobotObs(), "voronoifield_cost_map_ros2");
}
void HybridAStar::setNav2CostMap(const GridMap<double>& dist_obs) {
    // Nav2CostMap(dist_obs, occ_map_, cost_map_, analytic_shot_->getSigmaRobotObs(), 10.0, "nav2style_cost_map_ros2");
    Nav2CostMap(dist_obs, occ_map_, cost_map_, analytic_shot_->getSigmaRobotObs(), planner_weights_.nav2_decay_rate, "nav2style_cost_map_ros2");
}
double HybridAStar::getTotalDistance(std::vector<State> path) {
    if (path.size() < 1) return 0.0;

    double dist_sq = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i - 1].x - path[i].x;
        double dy = path[i - 1].y - path[i].y;

        dist_sq += dx * dx + dy * dy;
    }
    return std::sqrt(dist_sq);
}
void HybridAStar::visualize_searched_segs(
    double sx, double sy,
    double gx, double gy,
    int cell_size,
    std::string title)
{
    // Occupancy Grid 시각화용 이미지 (검은=장애물, 흰색=free)
    cv::Mat img(rows_ * cell_size, cols_ * cell_size, CV_8UC3, Color::White);
    for (int y = 0; y < rows_; ++y) {
        for (int x = 0; x < cols_; ++x) {
            if (occ_map_(y, x) == 1) {
                cv::rectangle(img,
                    cv::Point(x * cell_size, y * cell_size),
                    cv::Point((x + 1) * cell_size - 1, (y + 1) * cell_size - 1),
                    Color::Black, cv::FILLED);
            }
        }
    }
    double res = occ_map_.pixel_scale_;
    // 탐색 세그먼트들
    for (const auto& s : searched_segs_) {
        std::vector<State> samples = s.samples;
        for (const auto& ss : samples) {
            double x_sam = ss.x;
            double y_sam = ss.y;
            cv::Point sample_point((x_sam / res) * cell_size, (y_sam / res) * cell_size);
            cv::circle(img, sample_point, cell_size / 10, Color::Gray, cv::FILLED);
        }
    }

    double startx = sx / res, starty = sy / res;
    double goalx = gx / res, goaly = gy / res;
    cv::Point start(startx * cell_size, starty * cell_size);
    cv::Point goal(goalx * cell_size, goaly * cell_size);

    cv::circle(img, start, cell_size / 10, start_color, cv::FILLED);
    cv::circle(img, goal, cell_size / 10, goal_color, cv::FILLED);

    cv::imshow(title, img);
    cv::waitKey(0);
}

