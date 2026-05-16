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
};

// 종점용 (5차원: 상태 5)
struct ReferenceTrajTerminal {
    double x, y, theta, v, delta; 
};


// // 바이너리 파일 읽기 함수
// inline std::vector<State> loadPathFromBin(const std::string& filename) {
//     std::vector<State> path;
//     // 파일을 끝(ate)에서 열어서 파일의 전체 크기를 먼저 파악
//     std::ifstream in(filename, std::ios::binary | std::ios::ate);
//     if (in) {
//         size_t fileSize = in.tellg();
//         in.seekg(0, std::ios::beg); // 다시 처음으로 이동
        
//         size_t numElements = fileSize / sizeof(State);
//         path.resize(numElements);
        
//         // 메모리에 단숨에 읽어오기
//         in.read(reinterpret_cast<char*>(path.data()), fileSize);
//         in.close();
//         std::cout << "성공적으로 " << numElements << "개의 경로점을 불러왔습니다." << std::endl;
//     } else {
//         std::cerr << "경로 파일을 찾을 수 없습니다: " << filename << std::endl;
//     }
//     return path;
// }


#endif