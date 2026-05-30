#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H
#define _USE_MATH_DEFINES

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "mpc/include/structs_mpc.h"

const double GOAL_TOLERANCE = 0.01;

class BaseController 
{
public:
    virtual ~BaseController() = default;

    // 하위 수준 인터페이스(vehicle classes), 모든 하위 class가 가져야 할 필수 인터페이스
    virtual bool initialize() = 0;      // 메모리, 포인터 등 초기화
    virtual bool solve() = 0;

    // 상위 수준 인터페이스(controller class)
    virtual bool getRefTraj(
        std::vector<State>& global_path, double v_max,
        double a_lat_max, double a_acc_mag, double a_dec_mag, double dt) = 0;
    virtual int updateSlidingWindow(const double* curr, VehicleMode& curr_mode, 
        double min_d, double zero_v,
        double bi_dtheta, double spin_dtheta) = 0;
    virtual bool isGuidanceFinished(const double* curr) = 0;

    virtual void setInitialGuess(double* x_init, double* u_init) = 0;  // double* x_init처럼 배열(포인터)로, 초기 예상 궤적 세팅
    virtual void setInitialState(double* lbx0, double* ubx0) = 0;   // 현재 로봇의 물리적 상태 제약 세팅
    virtual void setTargetTrajectory(
        const std::vector<ReferenceTraj>& yref, 
        const ReferenceTrajTerminal& yref_e) = 0;
    virtual void getControlInput(double* u_out) = 0;
    virtual void getPredictedState(int step, double* x_pred) = 0;

protected:
    // 각도를 -M_PI ~ M_PI (-180도 ~ 180도) 사이로 정규화하는 헬퍼 함수
    inline double normalizeAngle(double angle) {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

};


#endif