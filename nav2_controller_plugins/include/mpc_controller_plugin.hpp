#ifndef MPC_CONTROLLER_PLUGIN_H
#define MPC_CONTROLLER_PLUGIN_H


#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include <angles/angles.h>

#include "vehicles_mpc.h"
#include "eval_guidance.h"
#include "recoveryfsm_mpc.h"
#include "file_handler.h"

namespace mpc_planner
{

class MpcController : public nav2_core::Controller
{
public:
    MpcController() = default;
    ~MpcController() override = default;

    // 플러그인 라이프사이클 관리 함수들
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    // Nav2 필수 오버라이드 인터페이스
    // 1. set Global plan
    void setPlan(const nav_msgs::msg::Path& _plan) override;

    // 2. set command
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker* goal_checker) override;

    // 3. 제한 속도 동적 변경
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

    // publisher cost map to rviz2
    void customLocalCostmapPublisher(
        const GridMap<double>& cost_map, 
        double resolution, 
        double origin_x, 
        double origin_y);

private:
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;
    // Clock
    rclcpp::Clock::SharedPtr clock_;
    // Logger
    rclcpp::Logger logger_{rclcpp::get_logger("MpcController")};
    // for nav2 controller cost map
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    // The frame of the costmap
    std::string frame_;
    // plugin name
    std::string name_;
    // parent node weak ptr
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    // Global path
    nav_msgs::msg::Path Global_path_ros_;
    std::vector<State> Global_path_;
    std::vector<ReferenceTraj> resampled_traj_;
    // objects
    std::unique_ptr<BicycleMode> mpc_bi_;
    std::unique_ptr<ParallelMode> mpc_parallel_;
    std::unique_ptr<SpinMode> mpc_spin_;
    // control variables
    int prev_idx_ = -1;
    int closest_idx_ = -1;
    double prev_steering_ = 0.0;
    VehicleMode prev_mode_ = VehicleMode::BicycleMode;
    std::unique_ptr<recoveryFSM> fsm_;

    // params
    double v_max_ = 0.5;            // 최대 허용 속도 (m/s),generate_mpc의 v limit보다 작게 설정
    double a_lat_max_ = 0.2;        // 최대 횡가속도(구심 가속도) 한계 (커브길 감속용)
    double a_dec_mag_ = 3.0;//0.1;  // 최대 감속도 크기 (양수로 입력, 브레이크 성능)
    double a_max_ = 3.0;            // 최대 가속도 크기
    int N_        = 60;
    double Tf_    = 2.0;
    double dt_; // = 2.0 / 60.0;        // MPC 제어 주기 generate_mpc.py -> Tf / N

    // 로봇 하드웨어 제원 (실제 차량 스펙에 맞게 수정)
    double delta_dot_max_ = 1.5;             // [bicycle] 최대 조향각속도 (rad/s), generate_mpc.py
    double omega_max_ = 0.5;                 // [spin] 제자리 최대 회전 각속도 (rad/s), generate_mpc.py
    double omega_dot_max_;  // = delta_dot_max_;  // [spin] 제자리 최대 회전 각가속도 (rad/s2)

    // 도착허용범위 설정
    double min_dist_thres_ = 1.5;//15 * v_max * dt;    // 20 * 0.35
    double zero_velocity_thres_ = 0.2; //std::max(0.01, dt * a_max); 
    // 동적 각도 임계값 계산 (마진 1.5배 적용, 최소 0.1 rad 보장)
    double spin_dtheta_thres_ = 180 * M_PI / 180.0; //std::max(0.2, omega_max * dt * 1.5); // 
    // 자전거 모드는 최대 속도에서 최대 조향을 꺾었을 때 변하는 각도가 기준
    double bi_dtheta_thres_ = 180 * M_PI / 180.0;

};

}// namespace mpc_planner


#endif