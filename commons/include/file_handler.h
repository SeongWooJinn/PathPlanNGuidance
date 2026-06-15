#ifndef FILE_HANDLER_H
#define FILE_HANDLER_H

#include "structs.h"

///////////////////////////////////////////////////
///////// .bin save/load helper funcions //////////
///////////////////////////////////////////////////
// .bin save function
inline void savePathToBin(const std::vector<State>& path, const std::string& filename) {
    // ios::binary 플래그를 사용하여 바이너리 쓰기 모드로 파일 열기
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        std::cerr << "경로 파일 저장 실패: " << filename << std::endl;
        return;
    }
    // vector의 메모리 시작 주소부터 전체 크기만큼 단숨에 쓰기
    out.write(reinterpret_cast<const char*>(path.data()), path.size() * sizeof(State));
    out.close();
    std::cout << "Hybrid A* 경로가 " << filename << " 에 바이너리로 저장되었습니다." << std::endl;
}

// 바이너리 파일 읽기 함수
inline std::vector<State> loadPathFromBin(const std::string& filename) {
    std::vector<State> path;
    // 파일을 끝(ate)에서 열어서 파일의 전체 크기를 먼저 파악
    std::ifstream in(filename, std::ios::binary | std::ios::ate);
    if (in) {
        size_t fileSize = in.tellg();
        in.seekg(0, std::ios::beg); // 다시 처음으로 이동
        
        size_t numElements = fileSize / sizeof(State);
        path.resize(numElements);
        
        // 메모리에 단숨에 읽어오기
        in.read(reinterpret_cast<char*>(path.data()), fileSize);
        in.close();
        std::cout << "성공적으로 " << numElements << "개의 경로점을 불러왔습니다." << std::endl;
    } else {
        std::cerr << "경로 파일을 찾을 수 없습니다: " << filename << std::endl;
    }
    return path;
}

// For plot in controller packages
inline void saveMapInfoToBin(const mapInfo& log_data, const std::string& filename) {
    // ios::binary 플래그를 사용하여 바이너리 쓰기 모드로 파일 열기
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        std::cerr << "MapInfo 파일 저장 실패: " << filename << std::endl;
        return;
    }

    out.write(reinterpret_cast<const char*>(&log_data.cell_size), sizeof(int));
    out.write(reinterpret_cast<const char*>(&log_data.r_length), sizeof(double));
    out.write(reinterpret_cast<const char*>(&log_data.r_width), sizeof(double));
    out.write(reinterpret_cast<const char*>(&log_data.resolution), sizeof(double));
    out.write(reinterpret_cast<const char*>(&log_data.sx), sizeof(int));
    out.write(reinterpret_cast<const char*>(&log_data.sy), sizeof(int));
    out.write(reinterpret_cast<const char*>(&log_data.gx), sizeof(int));
    out.write(reinterpret_cast<const char*>(&log_data.gy), sizeof(int));

    int rows = log_data.map.rows();
    int cols = log_data.map.cols();
    out.write(reinterpret_cast<const char*>(&rows), sizeof(int));
    out.write(reinterpret_cast<const char*>(&cols), sizeof(int));

    // double px_scale = log_data.map.pixel_scale_;
    double origin_x = log_data.map.origin_x_;
    double origin_y = log_data.map.origin_y_;
    // out.write(reinterpret_cast<const char*>(&px_scale), sizeof(double));
    out.write(reinterpret_cast<const char*>(&origin_x), sizeof(double));
    out.write(reinterpret_cast<const char*>(&origin_y), sizeof(double));

    int num_elements = rows * cols;
    if (rows > 0 && cols > 0) {
        out.write(reinterpret_cast<const char*>(log_data.map.data_.data()), num_elements * sizeof(int));
    }
    
    out.close();
    std::cout << "MapInfo가 " << filename << " 에 바이너리로 저장되었습니다." << std::endl;

}
inline bool loadMapInfoFromBin(mapInfo& log_data, const std::string& filename) {
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        std::cerr << "MapInfo 결과 파일 열기 실패: " << filename << std::endl;
        return false;
    }

    // 파일에서 구조체 크기만큼 읽어서 log_data 메모리에 덮어쓰기
    in.read(reinterpret_cast<char*>(&log_data.cell_size), sizeof(int));
    in.read(reinterpret_cast<char*>(&log_data.r_length), sizeof(double));
    in.read(reinterpret_cast<char*>(&log_data.r_width), sizeof(double));
    in.read(reinterpret_cast<char*>(&log_data.resolution), sizeof(double));
    in.read(reinterpret_cast<char*>(&log_data.sx), sizeof(int));
    in.read(reinterpret_cast<char*>(&log_data.sy), sizeof(int));
    in.read(reinterpret_cast<char*>(&log_data.gx), sizeof(int));
    in.read(reinterpret_cast<char*>(&log_data.gy), sizeof(int));

    int rows, cols;
    in.read(reinterpret_cast<char*>(&rows), sizeof(int));
    in.read(reinterpret_cast<char*>(&cols), sizeof(int));

    // double px_scale, origin_x, origin_y;
    // in.read(reinterpret_cast<char*>(&log_data.px_scale), sizeof(double));
    in.read(reinterpret_cast<char*>(&log_data.origin_x), sizeof(double));
    in.read(reinterpret_cast<char*>(&log_data.origin_y), sizeof(double));
    log_data.map.pixel_scale_ = log_data.resolution;
    log_data.map.origin_x_ = log_data.origin_x;
    log_data.map.origin_y_ = log_data.origin_y;
    
    // [C] Eigen Matrix 메모리 재할당 및 실제 데이터 복원
    if (rows > 0 && cols > 0) {
        // resize()를 호출하여 읽어들일 크기만큼 메모리를 동적 할당합니다.
        log_data.map.data_.resize(rows, cols);
        
        int num_elements = rows * cols;
        // 할당된 메모리 공간에 바이너리 데이터를 덮어씌웁니다.
        in.read(reinterpret_cast<char*>(log_data.map.data_.data()), num_elements * sizeof(int));
    }
    in.close();
    std::cout << "MapInfo 결과 파일 열기 성공: " << filename << std::endl;
    return true;
}

// MPC 결과 바이너리 저장 함수
inline void saveMpcResultToBin(const std::vector<MpcResultLog>& log_data, const std::string& filename) {
    // ios::binary 플래그를 사용하여 바이너리 쓰기 모드로 파일 열기
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        std::cerr << "MPC 결과 파일 저장 실패: " << filename << std::endl;
        return;
    }
    
    // vector의 메모리 시작 주소부터 전체 크기만큼 단숨에 쓰기
    out.write(reinterpret_cast<const char*>(log_data.data()), log_data.size() * sizeof(MpcResultLog));
    out.close();
    std::cout << "MPC 결과가 " << filename << " 에 바이너리로 저장되었습니다." << std::endl;
}

#endif