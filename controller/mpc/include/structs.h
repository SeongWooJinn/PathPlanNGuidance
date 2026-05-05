#ifndef STRUCTS_H
#define STRUCTS_H

struct ReferenceTraj {
    double x, y, theta, v, delta; // 상태 목표
    double a, delta_dot;          // 제어 입력 목표
    double obs;                   // 추가 변수 
};

// 2. 종점용 (5차원: 상태 5)
struct ReferenceTrajTerminal {
    double x, y, theta, v, delta; 
};



#endif