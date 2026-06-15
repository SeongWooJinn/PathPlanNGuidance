// #pragma once
#ifndef VISUALIZE_H
#define VISUALIZE_H
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include "structs.h"
// #include <opencv2/opencv.hpp>


// constexpr int legend_x0 = 0;
// constexpr int legend_y0 = 0;		// 시작 위치
// constexpr int step = 25;    // 줄 간격

namespace Color {
	const cv::Scalar Black(0, 0, 0);
	const cv::Scalar Green(0, 255, 0);
	const cv::Scalar Red(0, 0, 255);
	const cv::Scalar Orange(0, 165, 255);
	const cv::Scalar DarkOrange(0, 140, 255);
	const cv::Scalar Gold(0, 215, 255);
	const cv::Scalar Blue(255, 0, 0);
	const cv::Scalar LightBlue(255, 200, 100);
	const cv::Scalar Mint(170, 255, 170);
	const cv::Scalar Navy(128, 0, 0);
	const cv::Scalar SkyBlue(235, 206, 135);
	const cv::Scalar Yellow(0, 255, 255);
	const cv::Scalar Magenta(255, 0, 255);
	const cv::Scalar Cyan(255, 255, 0);
	const cv::Scalar Gray(127, 127, 127);
	const cv::Scalar DarkGray(64, 64, 64);
	const cv::Scalar Charcoal(30, 30, 30);
	const cv::Scalar White(255, 255, 255);
}

const cv::Scalar start_color = Color::Red;
const cv::Scalar goal_color = Color::Blue;


/////////////////////////////////////////////////////////
///////////////// STANDALONE VISUALIZE //////////////////
/////////////////////////////////////////////////////////

inline void drawVehicle(
	cv::Mat& img,
	int x, int y, double theta,
	double length, double width,
	cv::Scalar color);

void visualize_hybridastar_path(
	//const std::vector<std::pair<State, VehicleMode>>& path,
	const std::vector<State>& path,
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size, double r_length, double r_width,
	std::string title, double resolution);

void visualize_map(
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size,
	double sc, double sr, double gc, double gr,
	std::string title, double resolution);

void visualize_guide_path(
	const std::vector<std::pair<double, double>>& path,
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size,
	std::string title, double resolution);

void visualize_tracking_performance(
    const std::vector<State>& global_path,
    const std::vector<State>& tracked_path, // MPC가 실제로 주행한 궤적 추가
    GridMap<int>& occ_map,
    int cell_size, double r_length, double r_width,
    std::string title, double resolution,
	double origin_x, double origin_y);
///////////////////////////////////////////////////
///////////////// ROS2 VISUALIZE //////////////////
///////////////////////////////////////////////////

inline int convert_ltTolb(int rows, int y) {
	int y_lb = (rows - 1) - y;
	return y_lb;
} 
inline void drawVehicle_ros2(
	cv::Mat& img,
	int x, int y, double theta,
	double length, double width,
	cv::Scalar color);

void visualize_ros2_hybridastar_path(
	//const std::vector<std::pair<State, VehicleMode>>& path,
	const std::vector<State>& path,
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size, double r_length, double r_width,
	std::string title, double resolution);

void visualize_ros2_map(
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size,
	double sc, double sr, double gc, double gr,
	std::string title, double resolution);

void visualize_ros2_guide_path(
	const std::vector<std::pair<double, double>>& path,
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size,
	std::string title, double resolution);
#endif