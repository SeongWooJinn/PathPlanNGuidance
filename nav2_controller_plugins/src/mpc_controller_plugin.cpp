#include "mpc_controller_plugin.hpp"
// 260905 추가: 경로 좌표계 검사와 로봇 자세 TF 변환 공통 함수.
#include "path_frame_utils.hpp"
// 261011 수정: 기존 local_costmap의 footprint 충돌 검사와 목표 허용오차를 사용한다.
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include <mutex>
// #include "planner_exceptions.hpp"

using nav2_util::declare_parameter_if_not_declared;


namespace mpc_planner
{

void MpcController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    // ------------------ Nav2 Initial Setting ------------------
    tf_ = tf;
    name_ = name;
    costmap_ros_ = costmap_ros;     // costmap_ros_ : 2D Lidar sensor raw data in Nav2
    frame_ = costmap_ros_->getGlobalFrameID();

    // ------------------ Common vehicle params setting ------------------
    // 플러그인은 독립적인 '노드'가 아니라 Nav2의 planner_server 노드에 기생하는 형태이기 때문에, 
    // configure 함수로 전달받은 **parent (부모 노드의 포인터)**를 사용해서 파라미터를 선언
    node_ = parent;
    auto node = node_.lock();
    if(!node) throw std::runtime_error("Unable to lock node");

    clock_ = node->get_clock();
    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configured plugin %s", name_.c_str());

    declare_parameter_if_not_declared(node, name_ + ".v_max_", rclcpp::ParameterValue(0.5));
    v_max_ = node->get_parameter(name_ + ".v_max_").as_double();
    // 261011 수정: 백분율 속도 제한은 고정된 기본 속도에 적용한다.
    configured_v_max_ = v_max_;

    declare_parameter_if_not_declared(node, name_ + ".a_lat_max_", rclcpp::ParameterValue(0.2));
    a_lat_max_ = node->get_parameter(name_ + ".a_lat_max_").as_double();

    declare_parameter_if_not_declared(node, name_ + ".a_dec_mag_", rclcpp::ParameterValue(3.0));
    a_dec_mag_ = node->get_parameter(name_ + ".a_dec_mag_").as_double();

    declare_parameter_if_not_declared(node, name_ + ".a_max_", rclcpp::ParameterValue(3.0));
    a_max_ = node->get_parameter(name_ + ".a_max_").as_double();

    declare_parameter_if_not_declared(node, name_ + ".N_", rclcpp::ParameterValue(60));
    N_ = node->get_parameter(name_ + ".N_").as_int();

    declare_parameter_if_not_declared(node, name_ + ".Tf_", rclcpp::ParameterValue(2.0));
    Tf_ = node->get_parameter(name_ + ".Tf_").as_double();
    dt_ = Tf_ / N_;
    // 261011 수정: YAML만 바꿔 생성된 acados 솔버와 다른 시간축을 쓰는 설정을 거부한다.
    if (N_ != BICYCLE_MODEL_N || std::abs(Tf_ - 2.0) > 1e-6 || v_max_ <= 0.0) {
        throw std::runtime_error("MPC requires generated N=60, Tf=2.0 and positive v_max_");
    }

    declare_parameter_if_not_declared(node, name_ + ".delta_dot_max_", rclcpp::ParameterValue(1.5));
    delta_dot_max_ = node->get_parameter(name_ + ".delta_dot_max_").as_double();

    declare_parameter_if_not_declared(node, name_ + ".omega_max_", rclcpp::ParameterValue(0.5));
    omega_max_ = node->get_parameter(name_ + ".omega_max_").as_double();
    omega_dot_max_ = delta_dot_max_;

    declare_parameter_if_not_declared(node, name_ + ".min_dist_thres_", rclcpp::ParameterValue(1.5));
    min_dist_thres_ = node->get_parameter(name_ + ".min_dist_thres_").as_double();

    declare_parameter_if_not_declared(node, name_ + ".zero_velocity_thres_", rclcpp::ParameterValue(0.2));
    zero_velocity_thres_ = node->get_parameter(name_ + ".zero_velocity_thres_").as_double();

    declare_parameter_if_not_declared(node, name_ + ".spin_dtheta_thres_", rclcpp::ParameterValue(M_PI));
    spin_dtheta_thres_ = node->get_parameter(name_ + ".spin_dtheta_thres_").as_double();

    declare_parameter_if_not_declared(node, name_ + ".bi_dtheta_thres_", rclcpp::ParameterValue(M_PI));
    bi_dtheta_thres_ = node->get_parameter(name_ + ".bi_dtheta_thres_").as_double();

    // ------------------ MPC Solver Initialization ------------------
    mpc_bi_ = std::make_unique<BicycleMode>();
    mpc_parallel_ = std::make_unique<ParallelMode>();
    mpc_spin_ = std::make_unique<SpinMode>();

    // 261011 수정: 솔버 초기화 실패를 숨기지 않고 lifecycle 구성 실패로 전달한다.
    if (!mpc_bi_->initialize() || !mpc_parallel_->initialize() || !mpc_spin_->initialize()) {
        throw std::runtime_error("MPC solver initialization failed");
    }
    // 261011 수정: 기존 생성 솔버를 재생성하지 않고 실행 시 경로 위치 추종 비용을 보정한다.
    mpc_bi_->configureTrackingWeights();

    int recovery_cnt = 0;
    int MAX_RECOVERY_CNT = 30;
    fsm_ = std::make_unique<recoveryFSM>(MAX_RECOVERY_CNT, dt_, a_dec_mag_, omega_dot_max_);
}

void MpcController::cleanup()
{
    RCLCPP_INFO(logger_, "Cleaning up...");
    // memory clean
    mpc_bi_.reset();
    mpc_parallel_.reset();
    mpc_spin_.reset();
}
void MpcController::activate()
{
    // dynamic param set add
    RCLCPP_INFO(logger_, "Activating...");
}
void MpcController::deactivate()
{
    RCLCPP_INFO(logger_, "Deactivating...");
}


// Nav2 필수 오버라이드 인터페이스
// 1. set Global plan
// extended_hastar creatPlan 반환값 가져오기
void MpcController::setPlan(const nav_msgs::msg::Path& _plan)
{
    RCLCPP_INFO(logger_, "=== setPlan 함수 진입 성공! ===");
    RCLCPP_INFO(logger_, "Received plan with %zu poses", _plan.poses.size());

    // 260905 추가: 새 경로를 저장하기 전에 frame_id와 개별 pose의 좌표계 일관성을 확인한다.
    if (!_plan.poses.empty()) {
        validatePathFrame(_plan);
    }
    
    // 261011 수정: 같은 목표의 재계획은 최종 정렬 상태를 유지하고 새 목표에서는 해제한다.
    if (!Global_path_ros_.poses.empty() && !_plan.poses.empty()) {
        const auto & old_goal = Global_path_ros_.poses.back().pose;
        const auto & new_goal = _plan.poses.back().pose;
        if (std::hypot(old_goal.position.x - new_goal.position.x,
            old_goal.position.y - new_goal.position.y) > 1e-3 ||
            std::abs(angles::shortest_angular_distance(tf2::getYaw(old_goal.orientation),
                tf2::getYaw(new_goal.orientation))) > 1e-3) final_rotation_ = false;
    }
    Global_path_ros_ = _plan;
    Global_path_.clear();

    // 261011 수정: 실패한 새 경로가 이전 참조 궤적을 재사용하지 않도록 초기화한다.
    resampled_traj_.clear();
    if (_plan.poses.empty()) throw nav2_core::InvalidPath("MPC received an empty path");

    for (size_t i = 0; i < _plan.poses.size(); ++i)
    {
        State s;
        s.x = _plan.poses[i].pose.position.x;
        s.y = _plan.poses[i].pose.position.y;
        s.theta = tf2::getYaw(_plan.poses[i].pose.orientation);

        s.steering = 0.0;   // mpc 최적화 계산에서 추론

        // 1. gear
        if (i < _plan.poses.size() - 1) {
            double dx = _plan.poses[i+1].pose.position.x - s.x;
            double dy = _plan.poses[i+1].pose.position.y - s.y;
            double dir = std::atan2(dy, dx);

            double angle_diff = std::abs(angles::shortest_angular_distance(s.theta, dir));
            // 261011 수정: 제자리 회전은 이동 방향이 없으므로 이전 기어를 상속한다.
            s.gear = std::hypot(dx, dy) < 1e-6 ?
                (Global_path_.empty() ? 0 : Global_path_.back().gear) :
                ((angle_diff > M_PI / 2) ? 1 : 0);
        }
        else if (i == _plan.poses.size() - 1) {
            // 261011 수정: 한 점짜리 경로에서도 빈 벡터 back()을 호출하지 않는다.
            s.gear = Global_path_.empty() ? 0 : Global_path_.back().gear;
        }
        else {
            s.gear = 0;
        }

        s.vehicle = VehicleMode::BicycleMode;           // default mode
        // 2. vehicle mode
        if (i < _plan.poses.size() - 1) {
            double dx = _plan.poses[i+1].pose.position.x - s.x;
            double dy = _plan.poses[i+1].pose.position.y - s.y;
            double next_theta = tf2::getYaw(_plan.poses[i+1].pose.orientation);
            double dist = std::hypot(dx, dy);
            double angle_diff = std::abs(angles::shortest_angular_distance(s.theta,next_theta));
            // 261011 수정: 경로 곡률과 전후진 부호에서 Bicycle의 참조 조향각을 계산한다.
            if (dist > 1e-5) {
                const double signed_distance = (s.gear == 1 ? -dist : dist);
                s.steering = std::clamp(std::atan(0.11 *
                    angles::shortest_angular_distance(s.theta, next_theta) / signed_distance), -0.7, 0.7);
            }
            
            // 다음 점과 거리 차이가 없고 일정 각도차 이상이면 스핀모드
            if (dist < 1e-2 && angle_diff > M_PI * 1.0 / 180.0)
                s.vehicle = VehicleMode::SpinMode;
            // 다음 점과 각도 차가 거의 없으면 크랩모드
            // 261011 수정: Waffle의 직선 전진·후진도 BicycleMode이며 크랩으로 재분류하지 않는다.
        }
        Global_path_.push_back(s);
    }


    // get resampled reference trajectory
    // 261011 수정: 재계획 궤적은 직전 명령이 아닌 측정 속도로 시작한다.
    mpc_bi_->setInitialReferenceSpeed(measured_v_);
    // 261011 수정: 제한 중 재계획해도 원래 속도 프로파일을 보존하여 제한 해제가 정상 동작한다.
    if (!mpc_bi_->getRefTraj(Global_path_, configured_v_max_, a_lat_max_, a_max_, a_dec_mag_, dt_)) {
        Global_path_.clear();
        throw nav2_core::InvalidPath("MPC reference trajectory generation failed");
    }
    
    // 글로벌 궤적 포인터 (각 컨트롤러가 동일한 ref_traj_를 공유)
    resampled_traj_ = mpc_bi_->getRefTrajectoryData();
    unlimited_traj_ = resampled_traj_;  // 261011 수정: 동적 제한 이전 프로파일 보존.
    for (auto & point : resampled_traj_) point.v = std::clamp(point.v, -v_max_, v_max_);
    mpc_bi_->setRefTrajectoryData(resampled_traj_);  // 261011 수정: 새 경로에도 현재 제한을 적용한다.
    mpc_parallel_->setRefTrajectoryData(resampled_traj_);
    mpc_spin_->setRefTrajectoryData(resampled_traj_);

    RCLCPP_INFO(logger_, "Refernece plan set with %zu points", resampled_traj_.size());
    RCLCPP_INFO(logger_, "=== Complete Reference trajectory load! ===");

    prev_idx_     = -1;
    closest_idx_ = 0;
    // 261011 수정: 실제 직전 모드는 재계획으로 덮어쓰지 않아 정지 전 전환을 감지한다.
    // 261011 수정: 새 경로의 초기 인덱스는 다음 제어 주기의 실제 자세에서 결정한다.
    initialize_plan_index_ = true;
    solve_failure_count_ = 0;
    fsm_->resetDeadLockSkip();
}

// 2. set command
// 261011 수정: 측정 상태를 사용하며 모드 전환·목표 정렬·실패 복구를 반환 전에 처리한다.
geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker)
{
    geometry_msgs::msg::TwistStamped command;
    command.header.frame_id = costmap_ros_->getBaseFrameID();
    command.header.stamp = clock_->now();
    measured_v_ = velocity.linear.x;
    if (resampled_traj_.empty() || Global_path_ros_.poses.empty()) {
        throw nav2_core::InvalidPath("MPC has no valid reference trajectory");
    }
    // 261011 수정: 이전 TF 보정과 timestamp 보존을 유지한다.
    const auto tracking = transformPoseToPathFrame(
        pose, Global_path_ros_.header.frame_id, *tf_,
        tf2::durationFromSec(costmap_ros_->getTransformTolerance()));
    double yaw = tf2::getYaw(tracking.pose.orientation);
    // 261011 수정: yaw를 참조와 같은 2pi 구간으로 맞춰 경계에서 큰 오차를 막는다.
    yaw = resampled_traj_[std::max(0, closest_idx_)].theta +
        angles::shortest_angular_distance(resampled_traj_[std::max(0, closest_idx_)].theta, yaw);
    const auto & goal = Global_path_ros_.poses.back().pose;
    const double distance = std::hypot(goal.position.x - tracking.pose.position.x,
        goal.position.y - tracking.pose.position.y);
    const double yaw_error = angles::shortest_angular_distance(yaw, tf2::getYaw(goal.orientation));
    double xy_tolerance = 0.25, yaw_tolerance = 0.25;
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (goal_checker && goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
        if (pose_tolerance.position.x > 0.0) xy_tolerance = pose_tolerance.position.x;
        yaw_tolerance = std::abs(tf2::getYaw(pose_tolerance.orientation));
    }
    // 261011 수정: Nav2와 같은 허용오차로 위치 도착 후 최종 yaw를 정렬한다.
    // 261011 수정: AMCL 보정으로 허용오차 밖으로 밀리면 위치 추종을 재개한다.
    // 방향 정렬은 허용오차보다 안쪽에서 시작해 회전 중 보정 여유를 확보한다.
    if (distance > xy_tolerance) final_rotation_ = false;
    if (distance <= std::min(0.15, xy_tolerance * 0.6)) final_rotation_ = true;
    if (final_rotation_) {
        if (std::abs(velocity.linear.x) > 0.02) return command;
        if (std::abs(yaw_error) > yaw_tolerance) {
            command.twist.angular.z = std::clamp(1.5 * yaw_error, -0.35, 0.5);
            checkCommandCollision(pose, command.twist);
        }
        return command;
    }
    // 261011 수정: 첫 제어 주기에 새 경로의 첫 구간에서만 최근접점을 찾는다.
    if (initialize_plan_index_) {
        double best = std::numeric_limits<double>::max();
        closest_idx_ = 0;
        for (size_t i = 0; i < resampled_traj_.size(); ++i) {
            if (resampled_traj_[i].mode != resampled_traj_.front().mode) break;
            // 261011 수정: 초기 최근접점도 첫 기어 구간을 벗어나지 않는다.
            if (resampled_traj_[i].gear != resampled_traj_.front().gear) break;
            const double d = std::hypot(tracking.pose.position.x - resampled_traj_[i].x,
                tracking.pose.position.y - resampled_traj_[i].y);
            if (d < best) { best = d; closest_idx_ = static_cast<int>(i); }
        }
        initialize_plan_index_ = false;
    }
    VehicleMode mode = resampled_traj_[closest_idx_].mode;
    ::MpcController & active = mode == VehicleMode::SpinMode ?
        static_cast<::MpcController &>(*mpc_spin_) : static_cast<::MpcController &>(*mpc_bi_);
    // 261011 수정: 관측한 각속도로 Bicycle 조향 상태를 추정하고 가짜 이전 속도 대입을 제거한다.
    const double steering = std::abs(measured_v_) > 0.02 ?
        std::clamp(std::atan(velocity.angular.z * 0.11 / measured_v_), -0.7, 0.7) : 0.0;
    double state[5] = {tracking.pose.position.x, tracking.pose.position.y, yaw,
        mode == VehicleMode::SpinMode ? 0.0 : measured_v_,
        mode == VehicleMode::SpinMode ? velocity.angular.z : steering};
    double input[2] = {0.0, 0.0};
    // 261011 수정: 실제 정지 전에는 Spin 모델 또는 반대 방향으로 전환하지 않는다.
    const bool mode_change = mode != prev_mode_;
    if ((mode == VehicleMode::SpinMode && std::abs(measured_v_) > 0.02) ||
        (mode_change && (std::abs(measured_v_) > 0.02 || std::abs(velocity.angular.z) > 0.05))) {
        return command;
    }
    active.setClosestIdx(closest_idx_);
    active.setInitialState(state, state);
    if (mode_change) active.setInitialGuess(state, input);
    closest_idx_ = active.updateSlidingWindow(state, mode, min_dist_thres_, 0.02,
        bi_dtheta_thres_, spin_dtheta_thres_);
    fsm_->updateDeadLockCnt(closest_idx_, prev_idx_);
    // 261011 수정: 성공/실패 반환보다 먼저 데드락을 검사해 Nav2 재계획/복구를 요청한다.
    if (fsm_->isDeadLockForReplan()) {
        throw nav2_core::FailedToMakeProgress("MPC reference index made no progress");
    }
    if (fsm_->isDeadLock()) {
        fsm_->executeDeadLockSkip(closest_idx_, mode, resampled_traj_);
        return command;
    }
    if (!active.solve()) {
        // 261011 수정: 가속도/조향각속도를 cmd_vel로 보내지 않고 정지 후 연속 실패를 알린다.
        if (++solve_failure_count_ >= 3) {
            throw nav2_core::NoValidControl("MPC solver failed for three consecutive cycles");
        }
        active.setInitialGuess(state, input);
        return command;
    }
    solve_failure_count_ = 0;
    active.getPredictedState(1, state);
    command.twist.linear.x = mode == VehicleMode::SpinMode ? 0.0 :
        std::clamp(state[3], -v_max_, v_max_);
    command.twist.angular.z = mode == VehicleMode::SpinMode ?
        std::clamp(state[4], -0.35, 0.5) :
        std::clamp(command.twist.linear.x * std::tan(state[4]) / 0.11, -omega_max_, omega_max_);
    // 261011 수정: 생성된 궤적이 기어를 바꿀 때 관측 속도가 0이 된 뒤 새 방향을 허용한다.
    if (command.twist.linear.x * measured_v_ < 0.0 && std::abs(measured_v_) > 0.02) {
        command.twist.linear.x = command.twist.angular.z = 0.0;
    }
    if (!std::isfinite(command.twist.linear.x) || !std::isfinite(command.twist.angular.z)) {
        throw nav2_core::NoValidControl("MPC produced non-finite velocity");
    }
    checkCommandCollision(pose, command.twist);
    prev_mode_ = mode;
    prev_idx_ = closest_idx_;
    prev_opt_v_ = command.twist.linear.x;
    prev_steering_ = state[4];
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "[MPC] idx=%d v=%.3f w=%.3f",
        closest_idx_, command.twist.linear.x, command.twist.angular.z);
    return command;
}

// 261011 수정: local_costmap 좌표계에서 실제 발행 명령의 1초 예측 footprint를 검사한다.
void MpcController::checkCommandCollision(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & command)
{
    const auto local = transformPoseToPathFrame(pose, costmap_ros_->getGlobalFrameID(), *tf_,
        tf2::durationFromSec(costmap_ros_->getTransformTolerance()));
    auto * map = costmap_ros_->getCostmap();
    const auto footprint = costmap_ros_->getRobotFootprint();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
    double x = local.pose.position.x, y = local.pose.position.y;
    double theta = tf2::getYaw(local.pose.orientation);
    for (int i = 0; i <= 20; ++i) {
        unsigned int mx, my;
        const double cost = checker.footprintCostAtPose(x, y, theta, footprint);
        if (!map->worldToMap(x, y, mx, my) ||
            map->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE ||
            cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            throw nav2_core::NoValidControl("MPC command footprint collides with local costmap");
        }
        x += command.linear.x * std::cos(theta + command.angular.z * 0.025) * 0.05;
        y += command.linear.x * std::sin(theta + command.angular.z * 0.025) * 0.05;
        theta += command.angular.z * 0.05;
    }
}

// 261011 수정: 속도 제한을 기본 속도에서 계산하고 기존 참조에도 즉시 반영한다.
void MpcController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
    const double requested = speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT ?
        configured_v_max_ : (percentage ? configured_v_max_ * speed_limit / 100.0 : speed_limit);
    if (!std::isfinite(requested) || requested < 0.0) return;
    v_max_ = std::min(configured_v_max_, requested);
    resampled_traj_ = unlimited_traj_;  // 261011 수정: 제한을 누적하지 않고 기준 궤적에서 재계산.
    for (auto & point : resampled_traj_) {
        point.v = std::clamp(point.v, -v_max_, v_max_);
    }
    mpc_bi_->setRefTrajectoryData(resampled_traj_);
    mpc_spin_->setRefTrajectoryData(resampled_traj_);
}

// publisher cost map to rviz2
void MpcController::customLocalCostmapPublisher(
    const GridMap<double>& cost_map, 
    double resolution, 
    double origin_x, 
    double origin_y)
{

}
    

}

// Nav2 Pluginlib에 클래스를 등록하는 매크로 (필수!)
PLUGINLIB_EXPORT_CLASS(mpc_planner::MpcController, nav2_core::Controller)
