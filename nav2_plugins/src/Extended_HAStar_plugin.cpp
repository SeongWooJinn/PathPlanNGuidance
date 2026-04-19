#include "../include/Extended_HAStar_plugin.hpp"
#include "../include/planner_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace extended_planner
{

void ExtendedHybridAStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    // ------------------ Nav2 Initial Setting ------------------
    tf_ = tf;
    name_ = name;
    costmap_ros_ = costmap_ros;     // costmap_ros_ : 2D Lidar sensor raw data in Nav2
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();

    // ------------------ Common vehicle params setting ------------------
    // 플러그인은 독립적인 '노드'가 아니라 Nav2의 planner_server 노드에 기생하는 형태이기 때문에, 
    // configure 함수로 전달받은 **parent (부모 노드의 포인터)**를 사용해서 파라미터를 선언

    node_ = parent;
    auto node = node_.lock();
    if(!node) throw std::runtime_error("Unable to lock node");

    clock_ = node->get_clock();
    logger_ = node->get_logger();
    // costmap publisher
    costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "custom_costmap", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(logger_, "Configured plugin %s", name_.c_str());

    /////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// 1. Robot Config Parameters Setting ///////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////
    // nav2_util, 이미 선언된 파라미터 충돌 에러를 방지
    declare_parameter_if_not_declared(node, name_ + ".WB", rclcpp::ParameterValue(1.0));
    robotconfigs_.WB = node->get_parameter(name_ + ".WB").as_double();

    declare_parameter_if_not_declared(node, name_ + ".robot_length", rclcpp::ParameterValue(1.0));
    robotconfigs_.robot_length = node->get_parameter(name_ + ".robot_length").as_double();

    declare_parameter_if_not_declared(node, name_ + ".robot_width", rclcpp::ParameterValue(0.6));
    robotconfigs_.robot_width = node->get_parameter(name_ + ".robot_width").as_double();

    declare_parameter_if_not_declared(node, name_ + ".switch_time", rclcpp::ParameterValue(1.0));
    robotconfigs_.switch_time = node->get_parameter(name_ + ".switch_time").as_double();

    declare_parameter_if_not_declared(node, name_ + ".ref_vel", rclcpp::ParameterValue(0.5));
    robotconfigs_.ref_vel = node->get_parameter(name_ + ".ref_vel").as_double();

    declare_parameter_if_not_declared(node, name_ + ".sensor_fov", rclcpp::ParameterValue(2.0 * M_PI / 3.0));
    robotconfigs_.sensor_fov = node->get_parameter(name_ + ".sensor_fov").as_double();

    declare_parameter_if_not_declared(node, name_ + ".delta_max", rclcpp::ParameterValue(M_PI * 30.0 / 180.0));
    robotconfigs_.delta_max = node->get_parameter(name_ + ".delta_max").as_double();

    declare_parameter_if_not_declared(node, name_ + ".alpha", rclcpp::ParameterValue(90.0 * M_PI / 180.0));
    robotconfigs_.alpha = node->get_parameter(name_ + ".alpha").as_double();

    declare_parameter_if_not_declared(node, name_ + ".beta", rclcpp::ParameterValue(M_PI));
    robotconfigs_.beta = node->get_parameter(name_ + ".beta").as_double();

    /////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// 2. Planner Weights Parameters Setting ///////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////
    declare_parameter_if_not_declared(node, name_ + ".is_standalone", rclcpp::ParameterValue(false));
    plannerweights_.is_standalone = node->get_parameter(name_ + ".is_standalone").as_bool();

    declare_parameter_if_not_declared(node, name_ + ".use_guide_heuristic", rclcpp::ParameterValue(true));
    plannerweights_.use_guide_heuristic = node->get_parameter(name_ + ".use_guide_heuristic").as_bool();

    declare_parameter_if_not_declared(node, name_ + ".costmap_type", rclcpp::ParameterValue(0));
    plannerweights_.costmap_type = node->get_parameter(name_ + ".costmap_type").as_int();

    declare_parameter_if_not_declared(node, name_ + ".nav2_decay_rate", rclcpp::ParameterValue(10.0));
    plannerweights_.nav2_decay_rate = node->get_parameter(name_ + ".nav2_decay_rate").as_double();
    declare_parameter_if_not_declared(node, name_ + ".exp_decay_rate", rclcpp::ParameterValue(10.0));
    plannerweights_.exp_decay_rate = node->get_parameter(name_ + ".exp_decay_rate").as_double();
    declare_parameter_if_not_declared(node, name_ + ".sig_inflation_w", rclcpp::ParameterValue(10.0));
    plannerweights_.sig_inflation_w = node->get_parameter(name_ + ".sig_inflation_w").as_double();

    declare_parameter_if_not_declared(node, name_ + ".w_obs", rclcpp::ParameterValue(2.0));
    plannerweights_.w_obs = node->get_parameter(name_ + ".w_obs").as_double();
    declare_parameter_if_not_declared(node, name_ + ".w_fov", rclcpp::ParameterValue(6.0));
    plannerweights_.w_fov = node->get_parameter(name_ + ".w_fov").as_double();
    declare_parameter_if_not_declared(node, name_ + ".weighted_a", rclcpp::ParameterValue(1.7));
    plannerweights_.weighted_a = node->get_parameter(name_ + ".weighted_a").as_double();

    /////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// 3. Vehicle Weights Parameters Setting ///////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////
    declare_parameter_if_not_declared(node, name_ + ".w_curv", rclcpp::ParameterValue(1.0));
    vehicleweights_.w_curv = node->get_parameter(name_ + ".w_curv").as_double();
    declare_parameter_if_not_declared(node, name_ + ".w_steer", rclcpp::ParameterValue(1.0));
    vehicleweights_.w_steer = node->get_parameter(name_ + ".w_steer").as_double();
    declare_parameter_if_not_declared(node, name_ + ".reverse_penalty", rclcpp::ParameterValue(1.5));
    vehicleweights_.reverse_penalty = node->get_parameter(name_ + ".reverse_penalty").as_double();
    declare_parameter_if_not_declared(node, name_ + ".gear_shift_penalty", rclcpp::ParameterValue(1.0));
    vehicleweights_.gear_shift_penalty = node->get_parameter(name_ + ".gear_shift_penalty").as_double();

    declare_parameter_if_not_declared(node, name_ + ".bicycle_switch_penalty", rclcpp::ParameterValue(0.1));
    vehicleweights_.bicycle_switch_penalty = node->get_parameter(name_ + ".bicycle_switch_penalty").as_double();
    declare_parameter_if_not_declared(node, name_ + ".parallel_switch_penalty", rclcpp::ParameterValue(0.1));
    vehicleweights_.parallel_switch_penalty = node->get_parameter(name_ + ".parallel_switch_penalty").as_double();
    declare_parameter_if_not_declared(node, name_ + ".holonomic_switch_penalty", rclcpp::ParameterValue(0.1));
    vehicleweights_.holonomic_switch_penalty = node->get_parameter(name_ + ".holonomic_switch_penalty").as_double();
  
}

void ExtendedHybridAStarPlanner::activate()
{
    // dynamic param set add
    RCLCPP_INFO(logger_, "Activating...");
}
void ExtendedHybridAStarPlanner::deactivate()
{
    RCLCPP_INFO(logger_, "Deactivating...");
}
void ExtendedHybridAStarPlanner::cleanup()
{
    RCLCPP_INFO(logger_, "Cleaning up...");
    // memory clean
    planner_.reset();
}

nav_msgs::msg::Path ExtendedHybridAStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker)
{
    RCLCPP_INFO(logger_, "=== 하이브리드 A* 탐색 함수 진입 성공! ===");

    // 1. Nav2 Costmap 데이터들을 알고리즘의 맵 형식으로 변환
    unsigned char* costs = costmap_->getCharMap();    // nav2 costmap(2dlidar raw data)
    unsigned int cols = costmap_->getSizeInCellsX();  // cols
    unsigned int rows = costmap_->getSizeInCellsY();  // rows
    double resolution = costmap_->getResolution();
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();

    // RCLCPP_INFO(rclcpp::get_logger("ExtendedHybridAStar"), "1. Costmap 크기: rows=%d, cols=%d", rows, cols);

    // Create Map for algorithm
    GridMap<int> occ_map(rows, cols, resolution, origin_x, origin_y);
    GridMap<double> cost_map(rows, cols, resolution, origin_x, origin_y);
    // cost_map.data_.setConstant(1.0);
    cost_map.data_.setConstant(255.0);  // basic cost : UNKNOWN SPACE COST


    // Nav2 type costmap -> occ_map
    for (int y = 0; y < rows; ++y){
        for (int x = 0; x < cols; ++x) {
            int index = y * cols + x;
            unsigned char nav2_cost = costs[index];
            // 254 : lethal, 255 : unknown spaces
            if (nav2_cost == 254 || nav2_cost == 255) {
            // if (nav2_cost == 254) {
                occ_map(y, x) = 1;
            } else {
                occ_map(y, x) = 0;
            }

        }
    }

    // vehicle mode 생성
    // std::move시, bicycle_에는 nullptr이 남으므로 두번째 createPlan시에는 사라짐, creatPlan에서 직접 만들어줘야함
    auto bicycle = std::make_unique<BicycleMode>();
    bicycle->setModeType(VehicleMode::BicycleMode);
    bicycle->setVehicleProperties(robotconfigs_.WB, robotconfigs_.delta_max, robotconfigs_.robot_length, robotconfigs_.robot_width, 
                                  robotconfigs_.switch_time, robotconfigs_.ref_vel, robotconfigs_.sensor_fov);
    bicycle->setMapResolution(resolution);
    bicycle->setWeights(vehicleweights_);

    auto crab = std::make_unique<ParallelMode>();
    crab->setModeType(VehicleMode::ParallelMode);
    crab->setVehicleProperties(robotconfigs_.WB, robotconfigs_.delta_max, robotconfigs_.robot_length, robotconfigs_.robot_width, 
                               robotconfigs_.switch_time, robotconfigs_.ref_vel, robotconfigs_.sensor_fov);
    crab->setMapResolution(resolution);
    crab->setWeights(vehicleweights_);

    auto holo = std::make_unique<HolonomicMode>();
    holo->setModeType(VehicleMode::HolonomicMode);
    holo->setVehicleProperties(robotconfigs_.WB, robotconfigs_.delta_max, robotconfigs_.robot_length, robotconfigs_.robot_width, 
                                  robotconfigs_.switch_time, robotconfigs_.ref_vel, robotconfigs_.sensor_fov);
    holo->setMapResolution(resolution);
    holo->setWeights(vehicleweights_);

    // 2. planner initialize
    planner_ = std::make_unique<HybridAStar>(occ_map, cost_map); 
    planner_->setWeights(plannerweights_);
    // vehicle mode 등록
    planner_->registVehicleMode(std::move(bicycle));
    planner_->registVehicleMode(std::move(crab));
    planner_->registVehicleMode(std::move(holo));

    // 3. Setting for path planning 
    double sx = start.pose.position.x;
    double sy = start.pose.position.y;
    double stheta = tf2::getYaw(start.pose.orientation);    // 쿼터니언을 yaw로 변환!
    int sgear = 0.0; 
    VehicleMode smode = VehicleMode::BicycleMode;

    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;
    double gtheta = tf2::getYaw(goal.pose.orientation);    // 쿼터니언을 yaw로 변환!

    if (!occ_map.InRange(occ_map.WorldXToXi(sx), occ_map.WorldYToYi(sy))) 
    {
        throw nav2_core::StartOutsideMapBounds(
                "Start Coordinates of(" + std::to_string(sx) + ", " +
                std::to_string(sy) + ") was outside bounds");
    }

    if (!occ_map.InRange(occ_map.WorldXToXi(gx), occ_map.WorldYToYi(gy))) 
    {
        throw nav2_core::GoalOutsideMapBounds(
                "Goal Coordinates of(" + std::to_string(gx) + ", " +
                std::to_string(gy) + ") was outside bounds");
    }

    if (occ_map(occ_map.WorldYToYi(sy),occ_map.WorldXToXi(sx))) 
    {
        throw nav2_core::StartOccupied(
            "Start Coordinates of(" + std::to_string(sx) + ", " +
            std::to_string(sy) + ") was occupied");
    }

    if (occ_map(occ_map.WorldYToYi(gy),occ_map.WorldXToXi(gx))) 
    {
        throw nav2_core::GoalOccupied(
            "Goal Coordinates of(" + std::to_string(gx) + ", " +
            std::to_string(gy) + ") was occupied");
    }
    std::vector<State> raw_path;
    if(!planner_->run(sx, sy, stheta, sgear, smode, gx, gy, gtheta)) {
        throw nav2_core::NoValidPathCouldBeFound("Failed to create plan");
    }

    RCLCPP_INFO(logger_, "Find Path!");
    raw_path = planner_->reconstructPath(); // try-catch exception handle
    visualize_ros2_hybridastar_path(raw_path, occ_map, 10, robotconfigs_.robot_length, 
                        robotconfigs_.robot_width, "Global_Hybrid_A*_path_ros2", resolution);

    // 4. 탐색된 경로를 nav_msgs::msg::Path 형태로 변환
    nav_msgs::msg::Path path;
    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;

    for (const auto& state : raw_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        
        // 위치(x, y) 입력
        pose.pose.position.x = state.x; 
        pose.pose.position.y = state.y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, state.theta); // Roll(0), Pitch(0), Yaw(state.theta) 적용
        pose.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(pose);
    }

    RCLCPP_INFO(logger_, "Finished Path Planning");
    customCostmapPublisher(planner_->getCostMap(), resolution, origin_x, origin_y); // costmap pub
    return path;
}

void ExtendedHybridAStarPlanner::customCostmapPublisher(
    const GridMap<double>& cost_map, double resolution, double origin_x, double origin_y)
{
    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.frame_id = global_frame_;
    msg->header.stamp = clock_->now();

    msg->info.resolution = resolution;
    msg->info.width = cost_map.cols();
    msg->info.height = cost_map.rows();
    
    // 지도의 원점(왼쪽 아래 구석) 좌표 설정
    msg->info.origin.position.x = origin_x;
    msg->info.origin.position.y = origin_y;
    msg->info.origin.position.z = 0.0;
    // 쿼터니언 회전 (기본값)
    msg->info.origin.orientation.x = 0.0;
    msg->info.origin.orientation.y = 0.0;
    msg->info.origin.orientation.z = 0.0;
    msg->info.origin.orientation.w = 1.0;

    // GridMap 데이터를 OccupancyGrid(0~100)로 변환 (255는 -1로 처리)
    for (size_t y = 0; y < cost_map.rows(); ++y){
        for (size_t x = 0; x < cost_map.cols(); ++x){
            double val = cost_map(y, x);
            if (val == 255.0) msg->data.push_back(-1);
            else if (val >= 253.0) msg->data.push_back(100); // 치명적 장애물
            else msg->data.push_back(static_cast<int8_t>((val / 252.0) * 100));
        }
    }
    // for (double val : cost_map.data_) {
    //     if (val == 255.0) msg->data.push_back(-1);
    //     else if (val >= 253.0 || val <= 254.0) msg->data.push_back(100); // 치명적 장애물
    //     else msg->data.push_back(static_cast<int8_t>((val / 252.0) * 100));
    // }
    costmap_pub_->publish(std::move(msg));
}



}  // namespace extended_planner

// 플러그인 등록 매크로 (필수!)
PLUGINLIB_EXPORT_CLASS(extended_planner::ExtendedHybridAStarPlanner, nav2_core::GlobalPlanner)