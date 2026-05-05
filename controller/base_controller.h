#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H
#define _USE_MATH_DEFINES

#include "structs.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>

class BaseController 
{
public:
    virtual ~BaseController() = default;

    // 모든 제어기가 가져야 할 필수 인터페이스
    virtual bool initialize() = 0;      // 메모리, 포인터 등 초기화
    virtual bool solve() = 0;

    virtual void setInitialGuess(double* x_init, double* u_init) = 0;  // double* x_init처럼 배열(포인터)로, 초기 예상 궤적 세팅
    virtual void setInitialState(double* lbx0, double* ubx0) = 0;   // 현재 로봇의 물리적 상태 제약 세팅
    virtual void setTargetTrajectory(
        const std::vector<ReferenceTraj>& yref, 
        const ReferenceTrajTerminal& yref_e) = 0;
    virtual void getControlInput(double* u_out) = 0;
    virtual void getPredictedState(int step, double* x_pred) = 0;

};


#endif