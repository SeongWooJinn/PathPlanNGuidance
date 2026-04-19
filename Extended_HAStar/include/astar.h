// #pragma once
#ifndef ASTAR_H
#define ASTAR_H
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <unordered_map>
#include <chrono>
#include "structs.h"

using pdi = std::pair<double, int>; // cost, index
using pii = std::pair<int, int>;

struct Vertex
{
    int id_;
    int x, y;
};

class AStar
{
private:
    int rows_, cols_;
    //double pixel_scale_x_, pixel_scale_y_;
    GridMap<int> occ_map_;
    GridMap<double> cost_map_;

    std::vector<double> gScore_;
    std::vector<bool> closed_;              // 방문 확인용
    std::unordered_map<int, int> cameFrom_; // 경로 확인용
    std::vector<Vertex> Vertexs_;
    std::priority_queue<pdi, std::vector<pdi>, std::greater<pdi>> pq;

    inline int CoordToIndex(const int& x, const int& y) { return y * cols_ + x; }
    //inline bool InRange(int x, int y) { return (x >= 0 && y >= 0 && x < cols_ && y < rows_); }
    // inline double heuristic(Vertex& u, Vertex& v)
    // {
    //     double dx = static_cast<double>(u.x - v.x);
    //     double dy = static_cast<double>(u.y - v.y);
    //     return std::hypot(dx, dy);
    // }
    // inline double euclidist(Vertex& u, Vertex& v) {
    //     double dx = u.x - v.x;
    //     double dy = u.y - v.y;
    //     return std::hypot(dx, dy);
    // }
    inline double heuristic(double from_x_m, double to_x_m, double from_y_m, double to_y_m)
    {
        return std::hypot(from_x_m - to_x_m, from_y_m - to_y_m);
    }
    inline double euclidist(double from_x_m, double to_x_m, double from_y_m, double to_y_m)
    {
        return std::hypot(from_x_m - to_x_m, from_y_m - to_y_m);
    }

public:
    AStar(const GridMap<int>& occ_map, const GridMap<double>& cost_map);
    bool run(int sx, int sy, int gx, int gy);
    double edgeCost(int from, int to);
    std::vector<std::pair<double, double>> reconstructPath_resample(double gx, double gy);
    std::vector<pii> reconstructPath(int gx, int gy);
    double getScore(int x, int y);

};

#endif