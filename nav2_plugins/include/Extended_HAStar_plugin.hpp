#ifndef EXTENDED_HYBRID_ASTAR_PLUGIN_HPP_
#define EXTENDED_HYBRID_ASTAR_PLUGIN_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

#include "../../Extended_HAStar/include/xhastar.h" 
// // #include "../../Extended_HAStar/include/maps.h" // maps.h는 ros2연동시 미사용

namespace extended_planner
{

class ExtendedHybridAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  ExtendedHybridAStarPlanner() = default;
  ~ExtendedHybridAStarPlanner() = default;

  // 플러그인 라이프사이클 관리 함수들
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // 가장 핵심이 되는 경로 생성 함수
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  // publisher cost map to rviz2
  void customCostmapPublisher(const GridMap<double>& cost_map, 
    double resolution, double origin_x, double origin_y);

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("ExtendedHybridAStarPlanner")};

  // Original map made by 2D Lidar, Static Layer, Obstacle Layer, Inflation Layer, ...
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Global Custom Costmap from costmap_ros_
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;
  
  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Planner based on ExtendedHybridAstar
  std::unique_ptr<HybridAStar> planner_;

  // Publisher cost map to rviz2 for just visualizing
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  // Weights
  PlannerWeights plannerweights_;
  VehicleWeights vehicleweights_;

  // Configs
  RobotConfigs robotconfigs_;

    // // vehicles physical params
    // double WB_;
    // double robot_length_;
    // double robot_width_;
    // double switch_time_;
    // double ref_vel_;
    // double sensor_fov_;

    // double delta_max_; 
    // double alpha_;
    // double beta_;

};

}  // namespace extended_planner

#endif  // EXTENDED_HYBRID_ASTAR_PLUGIN_HPP_