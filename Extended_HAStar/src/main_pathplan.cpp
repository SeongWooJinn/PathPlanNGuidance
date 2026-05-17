
#include "xhastar.h"
#include "maps.h"

int main() {
    // x : col index, y : row index, gear 0 : forward / 1 : reverse
    
    // ------------------ Initial Setting ------------------
    bool isreplanned = false;
    //int rows = 30, cols = 60;
    //double sx = 5.0; double sy = 25.0; double stheta = 0.5 * M_PI; int sgear = 0.0; VehicleMode smode = VehicleMode::BicycleMode;
    //double gx = 40.0; double gy = 3.0; double gtheta = 0.0 * M_PI;

    // [meter unit]
    double map_height = 30.0;
    double map_width = 60.0;
    double resolution = 0.2; //1.0;
    double sx = 5.0; double sy = 2.0; double stheta = 1.0 * M_PI; int sgear = 0.0; VehicleMode smode = VehicleMode::BicycleMode;
    double gx = 25.0; double gy = 24.0; double gtheta = 1.0 * M_PI;
    //double gx = 35.0; double gy = 8.0; double gtheta = 1.0 * M_PI;
    //double gx = 59.0; double gy = 29.0; double gtheta = 1.0 * M_PI;

    // [index]
    int rows = static_cast<int>(map_height / resolution);
    int cols = static_cast<int>(map_width / resolution);

    int rnd_obs = rows * cols * 0.0;
    double init_cost = 1.0;

    // ------------------ Common vehicle params setting ------------------
    RobotConfigs my_robot;

    // ROBOT1 : MY ROBOT
    my_robot.WB = 1.0;
    my_robot.robot_length = 1.0;
    my_robot.robot_width = 0.6; // footprint (m)
    my_robot.switch_time = 1.0;
    my_robot.ref_vel = 0.5;
    my_robot.sensor_fov = 2.0 * M_PI / 3.0;
    my_robot.delta_max = M_PI * 30.0 / 180.0; // rad
    // my_robot.alpha = 90.0 * M_PI / 180.0;    // actionset범위 조절 가능 
    // my_robot.beta = M_PI;     // 180도 회전 [-PI/2, PI/2]  
    my_robot.alpha = 20.0 * M_PI / 180.0;    // actionset범위 조절 가능 
    my_robot.beta = M_PI;     // 180도 회전 [-PI/2, PI/2]  

    // // ROBOT2 : TURTLEBOT3 WAFFLE
    // double WB = 0.14;
    // double robot_length = 0.28;
    // double robot_width = 0.306;
    // double switch_time = 0.1;
    // double ref_vel = 0.2;
    // double sensor_fov = 2 * M_PI;
    // double delta_max = 75.0 * M_PI / 180.0;    // TURTLEBOT3 WAFFLE
    // double alpha = M_PI / 2.0;    // TURTLEBOT3 WAFFLE
    // double beta = M_PI / 2.0;   // TURTLEBOT3 WAFFLE
      
    // ------------------ Weights setting ------------------
    PlannerWeights planner_w;

    // 1. System & Mode Settings
    planner_w.is_standalone = true;         // for save result .png
    planner_w.use_guide_heuristic = false;    // guide_heuristic or dij_rs_heuristic

    // 2. Cost Map Settings
    planner_w.costmap_type = 0; // 0: Voronoi, 1: Exponential, 2: Sigmoid, 3: Nav2
    planner_w.nav2_decay_rate = 1.0;      
    planner_w.exp_decay_rate = 1.0;
    planner_w.sig_inflation_w = 1.0;

    // 3. Hybrid A* Search Weights
    planner_w.w_obs = 3.0;// 6.0;
    planner_w.w_fov = 6.0;//6.0
    planner_w.weighted_a = 1.5;//2.0;

    VehicleWeights vehicle_w;
    // 4. Vehicle Kinematics Weights
    vehicle_w.w_curv = 1.0; //2.0;
    vehicle_w.w_steer = 1.0; //4.0;
    vehicle_w.reverse_penalty = 3.0;//1.5;
    vehicle_w.gear_shift_penalty = 1.0; // 2.0;

    // 5. Mode Switch Penalties
    vehicle_w.bicycle_switch_penalty = 1.0;
    vehicle_w.parallel_switch_penalty = 1.0;
    vehicle_w.spin_switch_penalty = 1.0;
    
    // Bicycle mode params setting
    double min_turn_R_bicycle = (2 * my_robot.WB) / std::tan(my_robot.delta_max);   // 곡률반경
    auto bicycle = std::make_unique<BicycleMode>();
    bicycle->setModeType(VehicleMode::BicycleMode);
    bicycle->setVehicleProperties(my_robot.WB, my_robot.delta_max, my_robot.robot_length, my_robot.robot_width,
                                  my_robot.switch_time, my_robot.ref_vel, my_robot.sensor_fov);
    bicycle->setMapResolution(resolution);
    bicycle->setWeights(vehicle_w);

    // Parallel mode params setting
    auto crab = std::make_unique<ParallelMode>();
    crab->setModeType(VehicleMode::ParallelMode);
    crab->setVehicleProperties(my_robot.WB, my_robot.alpha, my_robot.robot_length, my_robot.robot_width, 
                               my_robot.switch_time, my_robot.ref_vel, my_robot.sensor_fov);
    crab->setMapResolution(resolution);
    crab->setWeights(vehicle_w);

    // Spin mode params setting
    auto spin = std::make_unique<SpinMode>();
    spin->setModeType(VehicleMode::SpinMode);
    spin->setVehicleProperties(my_robot.WB, my_robot.beta, my_robot.robot_length, my_robot.robot_width, 
                               my_robot.switch_time, my_robot.ref_vel, my_robot.sensor_fov);
    spin->setMapResolution(resolution);
    spin->setWeights(vehicle_w);

    // ------------------ Example map setting ------------------
    // OccMap gt(rows, cols);
    OccMap gt(rows, cols, resolution);
    gt.generate_example_map_v3(rnd_obs, sx, sy, gx, gy);
    // visualize_map(gt.getOccMap(), 10, sx, sy, gx, gy, "Occ Map", resolution);

    // ------------------ Cost map setting ------------------
    GridMap<double> cost_w(rows, cols, resolution);
    GridMap<int>& gt_map = gt.getOccMap();
    for (int x = 0; x < cols; ++x) {
        for (int y = 0; y < rows; ++y) {
            if (gt_map(y, x) == 1)
                cost_w(y, x) = std::numeric_limits<double>::infinity();
            else
                cost_w(y, x) = init_cost;
        }
    }

    // ------------------ Run ------------------
    if (!isreplanned) {
        HybridAStar hastar(gt_map, cost_w);
        hastar.setWeights(planner_w);
        
        // vehicle mode 등록
        hastar.registVehicleMode(std::move(bicycle));
        hastar.registVehicleMode(std::move(crab));
        // hastar.registVehicleMode(std::move(spin));

        // Global Hybrid AStar Path
        //std::vector<std::pair<State, VehicleMode>> g_path;
        std::vector<State> g_path;
        std::vector<std::tuple<double, double, double>> g_path_vis, g_path_smoothing_vis;

        auto start = std::chrono::system_clock::now();
        if (hastar.run(sx, sy, stheta, sgear, smode, gx, gy, gtheta)) {
            auto end = std::chrono::system_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start);
            std::cout << "Time for Path Search : " << elapsed.count() << " sec" << std::endl;
            g_path = hastar.reconstructPath();
            std::cout << "Total Distance : " << hastar.getTotalDistance(g_path) << std::endl;
            visualize_hybridastar_path(g_path, gt_map, 10, my_robot.robot_length, my_robot.robot_width, "Global Hybrid A* path in GT map", resolution);
            // hastar.visualize_searched_segs(sx, sy, gx, gy, 10, "State Expansions");
        }

        std::cout << "Global path of Hybrid A* (No Smoothing)\n";
        std::cout << "x, y, theta, gear, steer, vehicle mode\n";
        for (const auto& p : g_path)
            std::cout << p.x << ", " << p.y << ", " << p.theta
            << ", " << p.gear << ", " << p.steering << ", " << p.vehicle << std::endl;

        savePathToBin(g_path, "/tmp/hybrid_astar_path");
    }
    // std::vector<State> load_path = loadPathFromBin("/tmp/hybrid_astar_path");
    // std::cout << "load path successfully" << std::endl;
    // for (const auto& p : load_path) {
    //     std::cout << p.x << ", " << p.y << ", " << p.theta
    //     << ", " << p.gear << ", " << p.steering << ", " << p.vehicle << std::endl;
    // }

}