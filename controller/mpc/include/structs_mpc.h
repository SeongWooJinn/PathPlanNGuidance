#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>
#include <iostream>
#include <fstream>
#include "structs.h"

struct ReferenceTraj {
    double x, y, theta, v, delta; // 상태 목표
    double a, delta_dot;          // 제어 입력 목표
    double obs;                   // 추가 변수 
    VehicleMode mode;             // hastar 현재 추종 궤적의 모드 전환을 위해
};

// 종점용 (5차원: 상태 5)
struct ReferenceTrajTerminal {
    double x, y, theta, v, delta; 
};


#endif