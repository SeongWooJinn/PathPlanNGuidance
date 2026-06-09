// #pragma once
#ifndef STRUCT_H
#define STRUCT_H
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

// using int64 = long long;

constexpr double COST_SCALE = 100.0;    // cost & heuristic scale factor for matching costmap normalization(0~255)

// 현재 vehicle mode를 표시, 순서대로 0~2
// 일부만 적용 시 bicycle은 필수, parallel/spin 순서 변경 필요
enum class VehicleMode : int
{
    // 무조건 0부터 시작
    BicycleMode = 0,
    SpinMode = 1,
    ParallelMode = 2,
    // ParallelMode = 1,
    // SpinMode = 2,
    COUNT       // 단지 enum class 요소 개수를 파악하기 위한 요소
};

//// State : 차량의 물리적 표현을 위한 정보(연속 관점, control)
struct State {
    double x, y, theta, steering;
    int gear; // 0 forward, 1 reverse
    VehicleMode vehicle;    // vehicle mode
};

// A* 계산 정보(graph 관점, discrete)
struct Node {
    int xi, yi, thetai, gear;
    State state;
    double g, h;
    int nearest_guide_idx = -1;
};

// previnfo : tree edge 정보, State end로 들어올때 어떤 모션으로 들어왔는지 정보
struct PrevInfo {
    int64 parent_idx;
    double g,h;              // optional store
    State end;
    int parent_guide_path_idx = -1;
};

// start->end까지 충돌/비용 정보
struct PathSegment {
    State start;
    State end;
    double length;
    std::vector<State> samples; // for collision check
};

// grid map struct
template <typename T>
struct GridMap
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data_;
    double pixel_scale_;
    double origin_x_;
    double origin_y_;


    // constructor
    GridMap() = default;
    GridMap(int rows, int cols, double px = 1.0, double ox = 0.0, double oy = 0.0)
        : pixel_scale_(px), origin_x_(ox), origin_y_(oy)
    {
        data_.resize(rows, cols);
        data_.setZero();
    }

    // member func
    inline int rows() const { return data_.rows(); }
    inline int cols() const { return data_.cols(); }
    inline bool InRange(int x, int y) const {
        return x >= 0 && y >= 0 && x < cols() && y < rows();
    }
    
    // Map Origin : bottom-left, x : right + / y : top + in ROS
    // origin is zero in standalone file
    // meter -> index
    inline int WorldXToXi(double x) { return static_cast<int>(floor( (x - origin_x_) / pixel_scale_)); }
    inline int WorldYToYi(double y) { return static_cast<int>(floor( (y - origin_y_) / pixel_scale_)); }  // (origin_y_ - y) in top-left
    // index -> meter
    inline double XiToWorldX(int xi) { return origin_x_ + (xi * pixel_scale_) + (pixel_scale_ / 2.0); }
    inline double YiToWorldY(int yi) { return origin_y_ + (yi * pixel_scale_) + (pixel_scale_ / 2.0); }  // (yi * pixel_scale_) + (pixel_scale_ / 2.0) - origin_y_ in top-left


    // 데이터 접근 연산자 오버로딩 
    inline T& operator()(int y, int x) { return data_(y, x); }
    inline const T& operator()(int y, int x) const { return data_(y, x); }
};

// ROBOT CONFIG
struct RobotConfigs
{
    double WB = 1.0;
    double robot_length = 1.0;
    double robot_width = 0.6; // footprint (m)
    double switch_time = 1.0;
    double ref_vel = 0.5;
    double sensor_fov = 2.0 * M_PI / 3.0;
    double delta_max = M_PI * 30.0 / 180.0; // rad
    double alpha = 90.0 * M_PI / 180.0;    // actionset범위 조절 가능 
    double beta = M_PI;     // 180도 회전 [-PI/2, PI/2] 
};

// plannerweights parameters for ros2 & standalone exe
struct PlannerWeights {
    // 1. System & Mode Settings
    bool is_standalone = true;         // for save result .png
    bool use_guide_heuristic = false;    // guide_heuristic or dij_rs_heuristic

    // 2. Cost Map Settings
    int costmap_type = 0; // 0: Voronoi, 1: Exponential, 2: Sigmoid, 3: Nav2
    double nav2_decay_rate = 1.0;      
    double exp_decay_rate = 1.0;
    double sig_inflation_w = 1.0;

    // 3. Hybrid A* Search Weights
    double w_obs = 3.0;// 6.0;
    double w_fov = 6.0;//6.0
    double weighted_a = 1.5;//2.0;

};

// VehicleWeights parameters for ros2 & standalone exe
struct VehicleWeights {

    // 4. Vehicle Kinematics Weights
    double w_curv = 1.0; //2.0;
    double w_steer = 1.0; //4.0;
    double reverse_penalty = 3.0;//1.5;
    double gear_shift_penalty = 1.0; // 2.0;

    // 5. Mode Switch Penalties
    double bicycle_switch_penalty = 1.0;
    double parallel_switch_penalty = 1.0;
    double spin_switch_penalty = 1.0;
};

// 메모리 패딩을 제거하여 C++ 구조체와 Python numpy dtype 구조를 1:1로 강제 매핑
#pragma pack(push, 1)
struct MpcResultLog {
    double x, y, theta, v, delta;
    double a, delta_dot;
};
#pragma pack(pop)

// For map share to controller packages
struct mapInfo
{
    int cell_size;     // 픽셀 비율 
    double r_length;       // 로봇 길이
    double r_width;        // 로봇 폭
    double resolution;     // 맵 해상도
    double sx, sy; 
    double gx, gy;
    double origin_x, origin_y;
    GridMap<int> map;
};

// enum class 연산자 오버로딩
inline std::ostream& operator<<(std::ostream& os, VehicleMode v) {
    switch (v) {
    case VehicleMode::BicycleMode: os << "BicycleMode"; break;
    case VehicleMode::ParallelMode: os << "ParallelMode"; break;
    case VehicleMode::SpinMode: os << "SpinMode"; break;
    }
    return os;
}

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