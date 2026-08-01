#include "mpc_controller_plugin.hpp"


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
    global_frame_ = costmap_ros_->getGlobalFrameID();

    // ------------------ Common vehicle params setting ------------------
    // 플러그인은 독립적인 '노드'가 아니라 Nav2의 planner_server 노드에 기생하는 형태이기 때문에, 
    // configure 함수로 전달받은 **parent (부모 노드의 포인터)**를 사용해서 파라미터를 선언
    node_ = parent;
    auto node = node_.lock();
    if(!node) throw std::runtime_error("Unable to lock node");

    clock_ = node->get_clock();
    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configured plugin %s", name_.c_str());

    declare_parameter_if_not_declared(node, name_ + ".v_max", rclcpp::ParameterValue(0.5));
    v_max_ = node->get_parameter(name_ + ".v_max").as_double();

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

    if (!mpc_bi_->initialize()) RCLCPP_ERROR(logger_, "Bicycle MPC Init Failed");
    if (!mpc_parallel_->initialize()) RCLCPP_ERROR(logger_, "Parallel MPC Init Failed");
    if (!mpc_spin_->initialize()) RCLCPP_ERROR(logger_, "Spin MPC Init Failed");

    int recovery_cnt = 0;
    int MAX_RECOVERY_CNT = 30;
    fsm_ = std::make_unique<recoveryFSM>(MAX_RECOVERY_CNT, dt_, a_dec_mag_, omega_dot_max_);
}

void MpcController::cleanup()
{
    RCLCPP_INFO(logger_, "Cleaning up...");
    // memory clean
    mpc_bi.reset();
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
    Global_path_ros_ = _plan;
    Global_path_.clear();

    if (_plan.poses.empty()) return;

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
            s.gear = (angle_diff > M_PI / 2) ? 1 : 0;
        }
        else if (i == _plan.poses.size() - 1) {
            s.gear = Global_path_.back().gear;
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
            
            // 다음 점과 거리 차이가 없고 일정 각도차 이상이면 스핀모드
            if (dist < 1e-2 && angle_diff > M_PI * 1.0 / 180.0)
                s.vehicle = VehicleMode::SpinMode;
            // 다음 점과 각도 차가 거의 없으면 크랩모드
            else if (angle_diff < 1e-3)
                s.vehicle = VehicleMode::ParallelMode;
        }
        Global_path_.push_back(s);
    }

    RCLCPP_DEBUG(logger_, "Global plan set with %zu points", Global_path_.size());

    // get resampled reference trajectory
    if (!mpc_bi.getRefTraj(Global_path_, v_max_, a_lat_max_, a_max_, a_dec_mag_, dt_)) return -1;
    
    // 글로벌 궤적 포인터 (각 컨트롤러가 동일한 ref_traj_를 공유)
    resampled_traj_ = mpc_bi.getRefTrajectoryData();
    mpc_parallel.setRefTrajectoryData(resampled_traj_);
    mpc_spin.setRefTrajectoryData(resampled_traj_);

    RCLCPP_INFO(logger_, "=== Complete Reference trajecotory load! ===");

    prev_idx     = -1;
    closest_idx_ = 0;
    prev_mode_   = resampled_traj_[closest_idx_].mode;
}

// 2. set command
geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker)
{
    RCLCPP_INFO(logger_, "=== MPC Controller 함수 진입 성공! ===");

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    if (Global_path_.empty()) {
        RCLCPP_INFO(logger_, "=== Global path empty!! ===");
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    double current_v = velocity.linear.x;
    double current_w = velocity.angular.z;

    // gear 추정
    int current_gear = (current_v < -0.01) ? 1 : 0; 
    // 조향각(Steering) 추정
    double current_steering = prev_steering_;

    VehicleMode current_mode = resampled_traj_[closest_idx_].mode;

    double current_x[5] = {
        pose.pose.position.x,
        pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation),
        current_v,
        current_steering
    };
    double current_u[2] {
        current_v,
        current_w
    };

    State curr_state;
    curr_state.x = current_x[0];
    curr_state.y = current_x[1];
    curr_state.theta = current_x[2];
    curr_state.gear = current_gear;
    curr_state.steering = current_x[4];
    curr_state.vehicle = current_mode; 

    //////////////////////////////////////
    ///////// start run guidance /////////
    //////////////////////////////////////
    RCLCPP_INFO(logger_, "=== MPC Controller 제어 명령 생성 시작! ===");

    // 람다식으로 즉시 포인터에 할당
    auto& active_mode = [&](VehicleMode mode) -> auto& {
        if (mode == VehicleMode::BicycleMode) return *mpc_bi_;
        else if (mode == VehicleMode::ParallelMode) return *mpc_parallel_;
        else return *mpc_spin_;
    }(current_mode);

    bool solve_success = false;     // MPC SQP최적화 연산 성공 여부

    // 0) 도착여부 판단
    if (active_mode.isGuidanceFinished(current_x)) {
        RCLCPP_INFO(logger_, "=== 도착 완료!, 추종 종료 ===");
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel; 
    }
    // 1) 각 모드별 인덱스 위치 동기화
    active_mode.setClosestIdx(closest_idx_);

    // 2) 현재 상태를 솔버에 제약 조건으로 주입
    if (prev_mode_ != current_mode) {
        std::cout << "mode switch : " << prev_mode_ << " -> " << current_mode << std::endl;
        // prev가 스핀모드였으면 시스템 변수가 다르므로(omega->delta, omega_dot -> delta_dot) 초기화
        // curr이 스핀모드면 제자리 회전이므로 v, a = 0 / omega, omega_dot = 0부터 시작하도록 초기화
        if (prev_mode_ == VehicleMode::SpinMode || current_mode == VehicleMode::SpinMode) {
            current_x[3] = 0.0; // v reset
            current_x[4] = 0.0; // omega -> delta reset
            current_u[0] = 0.0; // a reset
            current_u[1] = 0.0; // omega_dot -> delta_dot reset
        }
    }
    active_mode.setInitialState(current_x, current_x);

    // 3) 현재 위치를 넘겨주면 내부에서 타겟 인덱스 탐색 및 윈도우 주입 자동 처리
    closest_idx_ = active_mode.updateSlidingWindow(current_x, current_mode, 
            min_dist_thres_, zero_velocity_thres_,
            bi_dtheta_thres_, spin_dtheta_thres_);
    // 인덱스가 업데이트 되었는지 체크
    fsm_->updateDeadLockCnt(closest_idx_, prev_idx_);

    // 4) 최적화 계산
    solve_success = active_mode.solve();
    if (solve_success) {
        // 다음 스텝의 예측 상태를 현재 위치로 누적
        active_mode.getPredictedState(1, current_x);
        active_mode.getControlInput(current_u);
        active_mode.printStatus();
        prev_mode = current_mode;      // 성공했을때 prev_mode 업데이트
        prev_steering_ = current_x[4];
        // 연산 성공 시 회복 카운터 초기화
        fsm_->resetSolveFailure();
        cmd_vel.twist.linear.x = current_u[0];
        cmd_vel.twist.angular.z = current_u[1];
        return cmd_vel;
    }
    
    // 5) 최적화 계산 실패시 절차대로 회복기동
    if (!solve_success){
        fsm_->executeSolveFailure(current_x, current_u, closest_idx_, current_mode, active_mode, resampled_traj_);
        // 루프를 종료(break)하지 않고 다음 제어 주기로 넘어감
        // continue; 
        cmd_vel.twist.linear.x = current_u[0];
        cmd_vel.twist.angular.z = current_u[1];
        prev_steering_ = current_x[4];
        return cmd_vel;

    }  
    
    // 6) 인덱스가 업데이트 실패 시, 데드락 탈출을 위한 인덱스 skip
    if (fsm_->isDeadLockForReplan()) {
        RCLCPP_ERROR(logger_, "[FSM] 완전 고립. 전역경로계획 재생성 호출!");
        fsm_->resetDeadLockSkip();
        throw nav2_core::PlannerException("Deadlock detected, trigger global replanning");
    }
    else if (fsm_->isDeadLock()) {
        fsm_->executeDeadLockSkip(closest_idx_, current_mode, resampled_traj_);
        // 인덱스를 건너뛰었으므로 이번 주기는 0을 주고 대기 (다음 주기에 변경된 인덱스로 최적화 진행)
        RCLCPP_WARN(logger_, "[FSM] 인덱스 스킵 수행. 1 Cycle 정지 대기.");
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        prev_steering_ = current_x[4];
        return cmd_vel;
    }
}

// 3. 제한 속도 동적 변경
void MpcController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
    // speed_limit이 0.0 근처로 들어오거나 특정 상수(NO_SPEED_LIMIT)로 들어오면 속도 제한 해제
    if (speed_limit == 0.0 || speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
        v_max_ = v_max_;
    }
    else if (percentage) {
        // 퍼센티지 제한 (예: 50.0 이면 원래 속도의 절반인 0.25 m/s)
        v_max_ *= (speed_limit / 100.0);
    }
    else {
        // 절대 속도 제한 (예: 무조건 0.2 m/s 이하로)
        v_max_ = speed_limit;
    }

    RCLCPP_INFO(logger_, "Speed limit dynamically updated! Current v_max: %.2f", v_max_);
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