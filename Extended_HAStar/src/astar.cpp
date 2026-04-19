#include "astar.h"

AStar::AStar(
    const GridMap<int>& occ_map,
    const GridMap<double>& cost_map)
    : occ_map_(occ_map), cost_map_(cost_map)
    //AStar(const std::vector<std::vector<int>>& occ_map,
    //    const std::vector<std::vector<double>>& cost_map)
    //    : occ_map_(occ_map), cost_map_(cost_map)
{
    rows_ = occ_map_.data_.rows();
    cols_ = occ_map_.data_.cols();
    //rows_ = static_cast<int>(occ_map_.size()); std::cout << rows_ << "\n";
    //cols_ = static_cast<int>(occ_map_[0].size()); std::cout << cols_ << "\n";
    int n = rows_ * cols_; // grid size
    gScore_.assign(n, std::numeric_limits<double>::infinity());
    closed_.assign(n, false);
    Vertexs_.resize(n);
    for (int y = 0; y < rows_; ++y) {
        for (int x = 0; x < cols_; ++x) {
            Vertexs_[CoordToIndex(x, y)].id_ = CoordToIndex(x, y);
            Vertexs_[CoordToIndex(x, y)].x = x;
            Vertexs_[CoordToIndex(x, y)].y = y;
        }
    }
}

bool AStar::run(int sx, int sy, int gx, int gy) {
    if (!occ_map_.InRange(sx, sy) || !occ_map_.InRange(gx, gy)) {
        std::cout << "[sx, sy] : " << sx << ", " << sy << "\n";
        std::cout << "[gx, gy] : " << gx << ", " << gy << "\n";
        std::cerr << "Start/Goal out of range\n";
        return false;
    }
    if (occ_map_(sy, sx)) {
        std::cerr << "Start Vertex is Obstacle Vertex\n";
        return false;
    }
    if (occ_map_(gy, gx)) {
        std::cerr << "Goal Vertex is Obstacle Vertex\n";
        return false;
    }
    // coord to index
    int start = CoordToIndex(sx, sy);
    int goal = CoordToIndex(gx, gy);

    // initialize
    cameFrom_.clear();
    std::fill(gScore_.begin(), gScore_.end(), std::numeric_limits<double>::infinity());
    std::fill(closed_.begin(), closed_.end(), false);
    while (!pq.empty()) pq.pop();

    std::vector<pii> directions = {
        {-1,-1},{-1,0},{-1,1},
        {0,-1},        {0,1},
        {1,-1}, {1,0}, {1,1}
    };

    pq.push({ 0, start });
    gScore_[start] = 0;

    while (!pq.empty()) {
        double dist = pq.top().first;
        int curr = pq.top().second;
        pq.pop();

        if (curr == goal) return true;
        if (closed_[curr]) continue;
        closed_[curr] = true;

        for (const auto& [dx, dy] : directions) {
            int nx = Vertexs_[curr].x + dx;
            int ny = Vertexs_[curr].y + dy;
            int next = CoordToIndex(nx, ny);
            if (!occ_map_.InRange(nx, ny)) continue;
            if (closed_[next]) continue;
            if (occ_map_(ny, nx)) continue;
            //double g_new = gScore_[curr] + cost_map_[ny][nx];
            double g_new = gScore_[curr] + edgeCost(curr, next);
            if (gScore_[next] > g_new) {
                gScore_[next] = g_new;
                cameFrom_[next] = curr;

                double next_x = occ_map_.XiToWorldX(Vertexs_[next].x);
                double next_y = occ_map_.YiToWorldY(Vertexs_[next].y);
                double goal_x = occ_map_.XiToWorldX(Vertexs_[goal].x);
                double goal_y = occ_map_.YiToWorldY(Vertexs_[goal].y);
                
                double f_new = gScore_[next] + heuristic(next_x, goal_x, next_y, goal_y);
                pq.push({ f_new, next });
            }
        }

    }
    return false;
}

double AStar::edgeCost(int from, int to) {

    int from_x = from % cols_; int from_y = from / cols_;
    int to_x = to % cols_; int to_y = to / cols_;

    //if (occ_map_[to_y][to_x])  // 1 : obstacles, 0 : free
    //    return std::numeric_limits<double>::infinity();

    double from_x_m = occ_map_.XiToWorldX(from_x);
    double from_y_m = occ_map_.YiToWorldY(from_y);
    double to_x_m = occ_map_.XiToWorldX(to_x);
    double to_y_m = occ_map_.YiToWorldY(to_y);

    double obs_cost = 1.0 * cost_map_(to_y, to_x);  // 도착노드의 cost를 반환함(to)
    double move_cost = euclidist(from_x_m, to_x_m, from_y_m, to_y_m);
    return obs_cost + move_cost;
}

std::vector<std::pair<double, double>> 
AStar::reconstructPath_resample(double gx, double gy) {

    int gxi = occ_map_.WorldXToXi(gx);
    int gyi = occ_map_.WorldYToYi(gy);
    if (!occ_map_.InRange(gxi, gyi)) {
        std::cout << "Out of range astar\n";
        return {};
    }
    int goal = CoordToIndex(gxi, gyi);
    if (gScore_[goal] == std::numeric_limits<double>::infinity()) return {};

    // int pixel_scale = 1.0;
    std::vector<std::pair<double, double>> path;
    int cur = goal;
    // path.emplace_back(Vertexs_[cur].x, Vertexs_[cur].y);
    path.emplace_back(gx, gy);
    while (cameFrom_.count(cur)) {
        cur = cameFrom_.at(cur);
        // 정수인 경로의 인덱스를 노드의 중앙으로 이동 (pixel_scale = 1.0)
        // path.emplace_back(Vertexs_[cur].x * pixel_scale + pixel_scale * 0.5,
        //     Vertexs_[cur].y * pixel_scale + pixel_scale * 0.5);

        // 인덱스 -> 미터 변경
        double x_m = occ_map_.XiToWorldX(Vertexs_[cur].x);
        double y_m = occ_map_.YiToWorldY(Vertexs_[cur].y);
        path.emplace_back(x_m, y_m);
    }
    reverse(path.begin(), path.end());
    return path;
}

std::vector<pii> AStar::reconstructPath(int gx, int gy) {
    if (!occ_map_.InRange(gx, gy)) return {};
    int goal = CoordToIndex(gx, gy);
    if (gScore_[goal] == std::numeric_limits<double>::infinity()) return {};

    std::vector<pii> path;
    int cur = goal;
    path.emplace_back(Vertexs_[cur].x, Vertexs_[cur].y);
    while (cameFrom_.count(cur)) {
        cur = cameFrom_.at(cur);
        path.emplace_back(Vertexs_[cur].x, Vertexs_[cur].y);
    }
    reverse(path.begin(), path.end());
    return path;
}
double AStar::getScore(int x, int y) {
    if (!occ_map_.InRange(x, y)) return std::numeric_limits<double>::infinity();
    return gScore_[CoordToIndex(x, y)];
}
