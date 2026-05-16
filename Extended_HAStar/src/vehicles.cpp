#include "vehicles.h"

///////////////////////////////////////////////
///////////// 1. Abstract Class ///////////////
///////////////////////////////////////////////
bool IVehicleMode::tryAnalyticExpansion(
    const State&, const State&,
    std::function<bool(double, double, double)>)
{
    // 공통 : false 반환, BicycleMode등은 오버라이드 필요
    return false;
}

// Get/Set 가상함수
std::vector<double> IVehicleMode::getActionSet()
{
    double delta_steer = 2 * delta_max_ / (N_STEER_ - 1);
    std::vector<double> action_set;
    for (int i = 0; i < N_STEER_; ++i) {
        action_set.push_back(-delta_max_ + i * delta_steer);
    }
    return action_set;
}
std::vector<PathSegment> IVehicleMode::getSuccessors(const State& cur_state)
{
    std::vector<PathSegment> successors;
    for (const auto& actions : getActionSet()) {
        for (int dir : {+1, -1}) {  // dir -> forward : +1, reverse : -1
            PathSegment seg = propagate(cur_state, dir, actions);
            successors.emplace_back(seg);
        }
    }
    return successors;
}
VehicleMode IVehicleMode::getModeType() const { return curr_mode_type_; }
std::vector<std::pair<double, double>> IVehicleMode::getVehicleDisk()
{
    //double body_r = sigma_robot_and_obs_;   // max(robot length, width)
    //return{
    //    {robot_length_ / 3.0, body_r},
    //    {0.0, body_r},
    //    {-robot_length_ / 3.0, body_r}
    //};

    // A Multi-Heuristic Search-based Motion Planning for Automated Parking 논문 인용
    int num_disk = 3;
    double body_r = sqrt((robot_length_ / num_disk) * (robot_length_ / num_disk)
        + (robot_width_ / 2.0) * (robot_width_ / 2.0));
    double dist_btw_disk = 2.0 * sqrt(body_r * body_r - (robot_width_ / 2.0) * (robot_width_ / 2.0));
    std::vector<std::pair<double, double>> res;
    for (int i = 0; i < num_disk; ++i) {
        // res.push_back({ -body_r + i * dist_btw_disk, body_r });
        res.push_back({ -body_r + i * dist_btw_disk, body_r + sigma_robot_and_obs_ });  // margin
    }
    return res;
}
double IVehicleMode::getMinTurnR() const { return min_turn_radius_; }
const std::vector<State>& IVehicleMode::getAnalyticPath() const { return state_path_; }  // 상수 참조 반환
double IVehicleMode::getSigmaRobotObs() const { return sigma_robot_and_obs_; }
double IVehicleMode::getSwitchCost() { return mode_switching_time_ * vehicle_ref_vel_; }
double IVehicleMode::getRefVelocity() const { return vehicle_ref_vel_; }
double IVehicleMode::getSensorFOVHalf() const { return front_sensor_fov_ / 2.0; }
double IVehicleMode::getRobotLength() const { return robot_length_; }

void IVehicleMode::setModeType(VehicleMode mode) { curr_mode_type_ = mode; }
void IVehicleMode::setMapResolution(double res) 
{ 
    map_resolution_ = res; 
    // 충돌 검사 간격은 해상도의 절반
    sample_ds_ = map_resolution_ * 0.5;
    // 가지 길이는 로봇 길이와 비슷하거나 살짝 길게
    step_len_ = robot_length_ * 2.0;
}

void IVehicleMode::setWeights(VehicleWeights weight)
{
    vehicle_weights_ = weight;
}
//////////////////////////////////////////////////
///////////// 2. BicycleMode Class ///////////////
//////////////////////////////////////////////////
BicycleMode::BicycleMode() { curr_mode_type_ = VehicleMode::BicycleMode; }
//virtual VehicleMode getModeType() const override {
//    return VehicleMode::BicycleMode;
//}

void BicycleMode::setAnalyticPathSpace() {
    state_space_ = std::make_shared<ob::ReedsSheppStateSpace>(min_turn_radius_);
    //state_space_ = std::make_shared<ob::DubinsStateSpace>(min_turn_radius_);
    s = state_space_->allocState();
    g = state_space_->allocState();
}

void BicycleMode::setVehicleProperties(
    double WB, double dmax, double length,
    double width, double switch_time, double ref_vel, double fov, int nsteer)
{
    WB_ = WB; delta_max_ = dmax; robot_length_ = length; robot_width_ = width;
    sigma_robot_and_obs_ = std::max(robot_length_, robot_width_);// + robot_width_ / 2.0; // margin
    min_turn_radius_ = 2 * WB_ / std::tan(delta_max_);
    mode_switching_time_ = switch_time;
    vehicle_ref_vel_ = ref_vel;
    front_sensor_fov_ = fov;
    N_STEER_ = nsteer;
    setAnalyticPathSpace();
}

PathSegment BicycleMode::propagate(const State& s, const int direction, const double action)
{
    PathSegment seg;
    seg.samples.reserve(10);        // 공간 미리 확보
    seg.start = s;
    State cur = s;
    cur.steering = action; 
    cur.gear = (direction > 0) ? 0 : 1;
    cur.vehicle = getModeType();

    double L = step_len_ * (direction >= 0 ? 1.0 : -1.0); // signed step
    int steps = std::max(1, (int)ceil(fabs(step_len_ / sample_ds_)));
    seg.samples.reserve(steps);

    double step = L / steps;
    // get possible x,y,theta in steps
    for (int i = 0; i < steps; i++) {
        double dtheta = (step / WB_) * std::tan(action);
        double mid_theta = cur.theta + dtheta / 2.0;
        cur.x += step * cos(mid_theta);
        cur.y += step * sin(mid_theta);
        cur.theta = normalizeAngle(cur.theta + dtheta);
        seg.samples.emplace_back(cur);
    }
    // get last x,y,theta of last sample
    seg.end = cur;
    seg.length = fabs(step_len_);
    return seg;
}

double BicycleMode::getEdgeCost(const PathSegment& seg, Node& from, Node& to)
{
    // for length 길이는 정규화(나눗셈) 하지 않고 실제 미터(m)
    double length = seg.length;
    // for curvature 
    double dcurv = std::fabs(angleDiff(seg.end.theta, seg.start.theta)); // seg.start가 cur값, seg.end가 next 후보이므로
    // for steering error
    double prev_steering = seg.start.steering;
    double curr_steering = seg.end.steering;
    double dsteer = std::fabs(curr_steering - prev_steering);

    // scaling
    //length /= step_len_;
    //dcurv /= M_PI;
    dcurv /= (1 / min_turn_radius_);
    dsteer /= delta_max_;

    // calculate cost
    double cost = 0.0;
    cost += length;
    cost += vehicle_weights_.w_curv * dcurv;// *dcurv;
    cost += vehicle_weights_.w_steer * dsteer;// *dsteer;

    // for mode switching penalties
    //if (from.vehicle != to.vehicle)
    if (from.state.vehicle != to.state.vehicle)
        cost += this->getSwitchCost();//vehicle_ref_vel_ * mode_switching_time_;
    // for reverse penalties
    if (seg.end.gear == 1)
        cost += vehicle_weights_.reverse_penalty;
    // for gear shift penalties
    if (from.gear != to.gear)
        cost += vehicle_weights_.gear_shift_penalty;

    return cost * COST_SCALE;
}

double BicycleMode::getKinematicHeuristic(const Node& next, const Node& goal)
{
    // ob::State* s = state_space_->allocState();
    // ob::State* g = state_space_->allocState();
    
    // s->as<ob::SE2StateSpace::StateType>()->setXY(next.xi, next.yi);
    // s->as<ob::SE2StateSpace::StateType>()->setYaw(normalizeAngle(next.thetai));

    // g->as<ob::SE2StateSpace::StateType>()->setXY(goal.xi, goal.yi);
    // g->as<ob::SE2StateSpace::StateType>()->setYaw(normalizeAngle(goal.thetai));
    s->as<ob::SE2StateSpace::StateType>()->setXY(next.state.x, next.state.y);
    s->as<ob::SE2StateSpace::StateType>()->setYaw(normalizeAngle(next.state.theta));

    g->as<ob::SE2StateSpace::StateType>()->setXY(goal.state.x, goal.state.y);
    g->as<ob::SE2StateSpace::StateType>()->setYaw(normalizeAngle(goal.state.theta));

    double h_rs = state_space_->distance(s, g);
    // state_space_->freeState(s);
    // state_space_->freeState(g);

    return h_rs;
    
    // 임시
    //return std::hypot(next.state.x - goal.state.x, next.state.y - goal.state.y);
}

bool BicycleMode::tryAnalyticExpansion(
    const State& start, const State& goal,
    std::function<bool(double, double, double)> collisionChecker)
{
    //path.clear();
    std::vector<State> temp_path;
    ob::State* s = state_space_->allocState();
    ob::State* g = state_space_->allocState();
    auto* ss = s->as<ob::SE2StateSpace::StateType>();
    auto* gs = g->as<ob::SE2StateSpace::StateType>();

    ss->setX(start.x);
    ss->setY(start.y);
    ss->setYaw(normalizeAngle(start.theta));

    gs->setX(goal.x);
    gs->setY(goal.y);
    gs->setYaw(normalizeAngle(goal.theta));
    // RS 거리 계산
    double L = state_space_->distance(s, g);
    if (L < 1e-6) {
        std::cout << "L : " << L << "\n";
        //path.emplace_back(start);
        temp_path.emplace_back(start);
        return true;
    }
    //double step = std::min(0.2, min_turn_radius_ * 0.2); // step은 해상도보다 무조건 작아야함
    double step = map_resolution_; //1.0; // step은 해상도보다 무조건 작아야함
    int N = ceil(L / step);
    //std::cout << "RS analytic solution Step size : " << N << "\n";

    State prev = start;
    double total_len = 0.0;
    double reverse_len = 0.0;
    for (int i = 0; i <= N; ++i) {
        double t = double(i) / N;
        ob::State* q = state_space_->allocState();
        state_space_->interpolate(s, g, t, q);

        auto* qs = q->as<ob::SE2StateSpace::StateType>();
        State st;
        st.x = qs->getX();
        st.y = qs->getY();
        st.theta = normalizeAngle(qs->getYaw());
        st.steering = (i == 0) ? prev.steering : estimateSteer(prev, st);
        st.gear = (i == 0) ? prev.gear : estimateGear(prev, st);
        st.vehicle = getModeType();
        if (!collisionChecker(st.x, st.y, st.theta)) {
            state_space_->freeState(q);
            state_space_->freeState(s);
            state_space_->freeState(g);
            return false;
        }
        //// 후진 경로 체크
        //double seg_dist = std::hypotf(st.x - prev.x, st.y - prev.y);
        //total_len += seg_dist;
        //if (st.gear == 1) { reverse_len += seg_dist; }

        temp_path.emplace_back(st);
        prev = st;
        state_space_->freeState(q);
    }
    // 후진 경로 x% 이상이면 실패
    //if (total_len > 0 && reverse_len / total_len > 0.2) { return false; }

    state_path_ = std::move(temp_path);
    state_space_->freeState(s);
    state_space_->freeState(g);
    return true;
}
double BicycleMode::getSwitchCost() { return vehicle_weights_.bicycle_switch_penalty * mode_switching_time_ * vehicle_ref_vel_; }


///////////////////////////////////////////////////
///////////// 3. ParallelMode Class ///////////////
///////////////////////////////////////////////////
// 모든 바퀴가 같은 각도(alpha)로 정렬되어 헤딩방향을 고정한채 이동하는 모드
ParallelMode::ParallelMode() { curr_mode_type_ = VehicleMode::ParallelMode; }

void ParallelMode::setVehicleProperties(
    double WB, double dmax, double length,
    double width, double switch_time, double ref_vel, double fov, int nsteer)
{
    WB_ = WB; delta_max_ = dmax; robot_length_ = length; robot_width_ = width;
    sigma_robot_and_obs_ = std::max(robot_length_, robot_width_);// + robot_width_ / 2.0; // margin
    // 곡률반경이 없으므로 min_turn_radius_는 무한대, 곡률 고려한다면 수정 가능
    min_turn_radius_ = std::numeric_limits<double>::infinity();
    mode_switching_time_ = switch_time;
    vehicle_ref_vel_ = ref_vel;
    front_sensor_fov_ = fov;
    N_STEER_ = nsteer;
}

PathSegment ParallelMode::propagate(const State& s, const int direction, const double action)
{
    PathSegment seg;
    seg.samples.reserve(10);        // 공간 미리 확보
    seg.start = s;
    State cur = s;
    cur.steering = action; 
    cur.gear = (direction > 0) ? 0 : 1;
    cur.vehicle = getModeType();

    double L = step_len_ * (direction >= 0 ? 1.0 : -1.0); // signed step
    int steps = std::max(1, (int)ceil(fabs(step_len_ / sample_ds_)));
    seg.samples.reserve(steps);

    double step = L / steps;
    // get possible x,y,theta in steps
    for (int i = 0; i < steps; i++) {
        // parallel mode는 theta(heading)의 변화 없음
        cur.x += step * cos(seg.start.theta + cur.steering);
        cur.y += step * sin(seg.start.theta + cur.steering);
        cur.theta = seg.start.theta; // normalizeAngle(cur.theta + dtheta);
        seg.samples.emplace_back(cur);
    }
    // get last x,y,theta of last sample
    seg.end = cur;
    seg.length = fabs(step_len_);
    return seg;
}

double ParallelMode::getEdgeCost(const PathSegment& seg, Node& from, Node& to)
{
    // parallel mode에서 dcurv = 0

    // for length 길이는 정규화(나눗셈) 하지 않고 실제 미터(m)
    double length = seg.length;
    // for curvature 
    //double dcurv = std::fabs(angleDiff(seg.end.theta, seg.start.theta)); 
    // for steering error
    double prev_steering = seg.start.steering;
    double curr_steering = seg.end.steering;
    double dsteer = std::fabs(curr_steering - prev_steering);   // parallel에서는 조향각 아닌 바퀴 회전각

    // scaling
    //length /= step_len_;
    //dcurv /= M_PI;
    //dcurv /= (1 / min_turn_radius_);
    dsteer /= delta_max_;

    // calculate cost
    double cost = 0.0;
    cost += length;
    //cost += w_curv_ * dcurv;// *dcurv;
    cost += vehicle_weights_.w_curv * dsteer;// *dsteer;

    // for mode switching penalties
    //if (from.vehicle != to.vehicle)
    if (from.state.vehicle != to.state.vehicle)
        cost += this->getSwitchCost();//vehicle_ref_vel_ * mode_switching_time_;
    // for reverse penalties
    if (seg.end.gear == 1)
        cost += vehicle_weights_.reverse_penalty;
    // for gear shift penalties
    if (from.gear != to.gear)
        cost += vehicle_weights_.gear_shift_penalty;

    return cost * COST_SCALE;
}

double ParallelMode::getKinematicHeuristic(const Node& next, const Node& goal)
{
    // theta는 무의미, 유클리디언 거리 반환

    double dx = next.state.x - goal.state.x;
    double dy = next.state.y - goal.state.y;
    double h_rs = std::hypotf(dx, dy);

    return h_rs;
}

double ParallelMode::getSwitchCost() { return vehicle_weights_.parallel_switch_penalty * mode_switching_time_ * vehicle_ref_vel_; }

///////////////////////////////////////////////////
///////////// 4. SpinMode Class ///////////////
///////////////////////////////////////////////////
// 위치는 고정한채 제자리 회전하여 헤딩만 바꾸는 모드
// yaw 각속도 => ref_vel

SpinMode::SpinMode() { curr_mode_type_ = VehicleMode::SpinMode; }


void SpinMode::setVehicleProperties(
    double WB, double dmax, double length,
    double width, double switch_time, double ref_vel, double fov, int nsteer)
{
    WB_ = WB; delta_max_ = dmax; robot_length_ = length; robot_width_ = width;
    sigma_robot_and_obs_ = std::max(robot_length_, robot_width_);// + robot_width_ / 2.0; // margin
    min_turn_radius_ = 0.0;     // 회전만 하므로
    mode_switching_time_ = switch_time;
    vehicle_ref_vel_ = ref_vel;
    front_sensor_fov_ = fov;
    N_STEER_ = nsteer;
}

PathSegment SpinMode::propagate(const State& s, const int direction, const double action)
{
    PathSegment seg;
    seg.samples.reserve(10);        // 공간 미리 확보
    seg.start = s;
    State cur = s;
    cur.steering = action; 
    cur.gear = (direction > 0) ? 0 : 1;
    cur.vehicle = getModeType();

    int steps = 5;
    seg.samples.reserve(steps);
    
    double step = action / steps;
    // get possible x,y,theta in steps
    for (int i = 0; i < steps; i++) {
        // Spin mode는 거리 변화 없음
        //double dtheta =
        cur.x = seg.start.x;  //+= step * cos(seg.start.theta + cur.steering);
        cur.y = seg.start.y;  //+= step * sin(seg.start.theta + cur.steering);
        //cur.theta = normalizeAngle(seg.start.theta + step * i); // normalizeAngle(cur.theta + dtheta);
        cur.theta = normalizeAngle(cur.theta + step); // normalizeAngle(cur.theta + dtheta);
        seg.samples.emplace_back(cur);
    }
    // get last x,y,theta of last sample
    seg.end = cur;
    seg.length = 0.0;
    return seg;
}

double SpinMode::getEdgeCost(const PathSegment& seg, Node& from, Node& to)
{
    //Spin 길이는 0

    // for length
    //double length = seg.length;     
    // for curvature 
    double dcurv = std::fabs(angleDiff(seg.end.theta, seg.start.theta));
    // for steering error
    double prev_steering = seg.start.steering;
    double curr_steering = seg.end.steering;
    double dsteer = std::fabs(curr_steering - prev_steering);

    // scaling
    //length /= step_len_;
    dcurv /= M_PI;
    //dcurv /= (1 / min_turn_radius_);
    dsteer /= delta_max_;

    // calculate cost
    double cost = 0.0;
    //cost += length;
    cost += vehicle_weights_.w_curv * dcurv;// *dcurv;
    cost += vehicle_weights_.w_steer * dsteer;// *dsteer;

    // for mode switching penalties
    //if (from.vehicle != to.vehicle)
    if (from.state.vehicle != to.state.vehicle)
        cost += this->getSwitchCost();//vehicle_ref_vel_ * mode_switching_time_;
    // for reverse penalties
    if (seg.end.gear == 1)
        cost += vehicle_weights_.reverse_penalty;
    // for gear shift penalties
    if (from.gear != to.gear)
        cost += vehicle_weights_.gear_shift_penalty;

    return cost * COST_SCALE;
}

double SpinMode::getKinematicHeuristic(const Node& next, const Node& goal)
{
    // goal과의 delta theta에 따라 거리가 같더라도 cost 다름
    double dx = next.state.x - goal.state.x;
    double dy = next.state.y - goal.state.y;
    double dist = std::hypotf(dx, dy);

    double dtheta = std::fabs(angleDiff(goal.state.theta, next.state.theta));
    // w_rot_ = cost of 1 rad / cost of 1 m (m/rad, 미터 단위로 맞춰주기 위해)
    double h_rs = dist + dtheta;
    return h_rs;

}

double SpinMode::getSwitchCost() { return vehicle_weights_.spin_switch_penalty * mode_switching_time_ * vehicle_ref_vel_; }


