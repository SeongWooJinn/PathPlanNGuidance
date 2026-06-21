// #pragma once
#ifndef VEHICLES_H
#define VEHICLES_H
#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/State.h>
#include "structs.h"

namespace ob = ompl::base;

///////////////////////////////////////////////
///////////// 1. Abstract Class ///////////////
///////////////////////////////////////////////
class IVehicleMode
{
public:
    virtual ~IVehicleMode() = default;
    // 순수 가상함수
    virtual PathSegment propagate(const State&, const int, const double) = 0;    // segments candidates
    virtual double getEdgeCost(const PathSegment&, Node&, Node&) = 0;            // cost of vehicle
    virtual double getKinematicHeuristic(const Node&, const Node&) = 0;          // heristic function(rs, dubins, L2)
    virtual void setVehicleProperties(double, double, double, double, double, double, double, int) = 0;
    virtual bool tryAnalyticExpansion(const State&, const State&, std::function<bool(double, double, double)>);

    virtual std::vector<double> getActionSet();
    virtual std::vector<PathSegment> getSuccessors(const State& cur_state);
    // Get/Set 가상함수
    virtual VehicleMode getModeType() const;
    virtual std::vector<std::pair<double, double>> getVehicleDisk();
    virtual double getMinTurnR() const;
    virtual const std::vector<State>& getAnalyticPath() const;  // 상수 참조 반환
    virtual double getSigmaRobotObs() const;
    virtual double getSwitchCost();
    virtual double getRefVelocity() const;
    virtual double getSensorFOVHalf() const;
    virtual double getRobotLength() const;

    virtual void setModeType(VehicleMode mode);
    virtual void setMapResolution(double res);
    virtual void setWeights(VehicleWeights weight);

protected:
    // vehicle params
    VehicleMode curr_mode_type_;
    double WB_;
    double delta_max_;
    double robot_length_, robot_width_;
    double sigma_robot_and_obs_;
    double min_turn_radius_;
    double mode_switching_time_;
    double vehicle_ref_vel_;
    double front_sensor_fov_;
    int N_STEER_;
    double step_len_;
    double sample_ds_;
    bool has_space_path_ = false;
    ob::StateSpacePtr state_space_;
    std::vector<State> state_path_;
    // Energy comsumption as mode or J/m & J/rad

    // cost weights
    VehicleWeights vehicle_weights_;

    double map_resolution_;

    inline double angleDiff(double a, double b) {
        double diff = std::fmod(a - b, 2 * M_PI);
        if (diff > M_PI) diff -= 2 * M_PI;
        if (diff < -M_PI) diff += 2 * M_PI;
        return diff;
    }
    inline double normalizeAngle(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

};

//////////////////////////////////////////////////
///////////// 2. BicycleMode Class ///////////////
//////////////////////////////////////////////////
class BicycleMode : public IVehicleMode
{
public:
    BicycleMode();

    void setAnalyticPathSpace();
    void setVehicleProperties(double WB, double dmax, double length,
        double width, double switch_time, double ref_vel, double fov, int nsteer) override;
    PathSegment propagate(const State& s, const int direction, const double action) override;
    double getEdgeCost(const PathSegment& seg, Node& from, Node& to) override;
    double getKinematicHeuristic(const Node& next, const Node& goal) override;
    bool tryAnalyticExpansion(const State& start, const State& goal,
        std::function<bool(double, double, double)> collisionChecker) override;
    double getSwitchCost() override;

private:

    ob::State* s = nullptr; //state_space_->allocState();
    ob::State* g = nullptr; //state_space_->allocState();

    inline int estimateGear(const State& prev, const State& curr)
    {
        double dx = curr.x - prev.x;
        double dy = curr.y - prev.y;
        double heading = curr.theta;

        double proj = dx * cos(heading) + dy * sin(heading);
        //return (proj >= 0.0) ? +1 : -1;     
        return (proj >= 0.0) ? 0 : 1;     // 0 forward, 1 reverse
    }
    inline double estimateSteer(const State& prev, const State& curr)
    {
        double dx = curr.x - prev.x;
        double dy = curr.y - prev.y;
        double ds = std::hypotf(dx, dy);
        double dtheta = normalizeAngle(curr.theta - prev.theta);
        double kappa = dtheta / ds;
        return atan(WB_ * kappa);
    }
};

///////////////////////////////////////////////////
///////////// 3. ParallelMode Class ///////////////
///////////////////////////////////////////////////
// 모든 바퀴가 같은 각도(alpha)로 정렬되어 헤딩방향을 고정한채 이동하는 모드
class ParallelMode : public IVehicleMode
{
public:
    ParallelMode();

    void setVehicleProperties(
        double WB, double dmax, double length,
        double width, double switch_time, double ref_vel, double fov, int nsteer = 5) override;
    virtual PathSegment propagate(const State& s, const int direction, const double action) override;
    double getEdgeCost(const PathSegment& seg, Node& from, Node& to) override;
    double getKinematicHeuristic(const Node& next, const Node& goal) override;
    double getSwitchCost() override;

private:

};

///////////////////////////////////////////////////
///////////// 4. SpinMode Class ///////////////
///////////////////////////////////////////////////
// 위치는 고정한채 제자리 회전하여 헤딩만 바꾸는 모드
// yaw 각속도 => ref_vel
class SpinMode : public IVehicleMode
{
public:
    SpinMode();
    void setVehicleProperties(
        double WB, double dmax, double length,
        double width, double switch_time, double ref_vel, double fov, int nsteer = 5) override;
    virtual PathSegment propagate(const State& s, const int direction, const double action) override;

    double getEdgeCost(const PathSegment& seg, Node& from, Node& to) override;
    double getKinematicHeuristic(const Node& next, const Node& goal) override;
    double getSwitchCost() override;

private:
    double w_rot_ = vehicle_weights_.w_curv * step_len_ / M_PI;

};

#endif
