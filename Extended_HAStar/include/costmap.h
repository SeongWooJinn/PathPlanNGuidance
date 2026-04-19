// #pragma once
#ifndef COSTMAP_H
#define COSTMAP_H
#include <iostream>
#include <vector>
#include <filesystem>
#include "structs.h"

namespace fs = std::filesystem;

// All added 'inline' tag, dont need to seperate .cpp / .h
inline void getDelaunayTriangle(int, int, int, const cv::Rect&, cv::Subdiv2D&, std::vector<cv::Vec6f>&);
inline void drawDelaunayTriangle(int, int, int, const cv::Rect, const std::vector<cv::Vec6f>);
inline void getVoronoiField(int, int, int, cv::Subdiv2D&);
inline void drawVoronoiField(int, int, int, const std::vector<std::vector<cv::Point2f>>, const std::vector<cv::Point2f>);
inline void visualizeCostMap(const GridMap<double>&, int, const std::string&);

// "Practical Search Techniques in Path Planning for Autonomous Driving" 논문의 rho_v 계산
// 이 논문의 local optimization은 경로 후처리 작업임
inline void VoronoiFieldCostMap(
	//const cv::Mat& dist_meter,//int rows_, int cols_, double px_scale,
	const GridMap<double>& dist_meter,
	const GridMap<int>& occ_map_,
	GridMap<double>& cost_map_,
	double sigma,
	const std::string& title)
{
	// 장애물 점들 불러오기
	int rows = occ_map_.data_.rows();
	int cols = occ_map_.data_.cols();
	cv::Rect rect(0, 0, cols, rows);
	cv::Subdiv2D subdiv(rect);

	std::vector<cv::Point2f> points;
	for (int y = 0; y < rows; ++y) {
		for (int x = 0; x < cols; ++x) {
			// obs 점 삽입
			if (occ_map_(y,x) == 1) {
				cv::Point2f fp(static_cast<float>(x), static_cast<float>(y));
				points.emplace_back(fp);
				subdiv.insert(fp);
			}
		}
	}
	// 맵 전체에서 최대 거리(가장 넓은 공간의 중심) 찾기
	double min_val, max_dist_val;
	cv::Mat dist_meter_mat(rows, cols, CV_64FC1, const_cast<double*>(dist_meter.data_.data()));
	cv::minMaxLoc(dist_meter_mat, &min_val, &max_dist_val);
	std::cout << "max_dist_val : " << max_dist_val << "\n";

	// 예외 처리: 장애물이 하나도 없거나 꽉 찬 경우
	if (max_dist_val <= 0) max_dist_val = 1.0;

	// Voronoi Field Cost 계산
	// 거리가 클수록(중심일수록) 비용최소, 작을수록(장애물 근처) 비용증가
	for (int y = 0; y < rows; ++y) {
		for (int x = 0; x < cols; ++x) {
			// d_v 계산
			// 1. x,y와 가까운 셀(facets)의 인덱스 찾기(nearestIdx)
			cv::Point2f p(x, y);
			cv::Point2f facetcenter;
			int nearestIdx = subdiv.findNearest(p, &facetcenter);

			// 2. 중심점에 해당하는 보로노이 셀의 꼭지점 좌표들 계산
			std::vector<int> idx = { nearestIdx };	// x,y와 가까운 셀의 인덱스
			std::vector<std::vector<cv::Point2f>> facetlist;	// idx에 해당하는 셀 꼭지점 list
			std::vector<cv::Point2f> facetcenters;	// 셀의 중심점(facetcenter와 동일, vector형)
			subdiv.getVoronoiFacetList(idx, facetlist, facetcenters);
			std::vector<cv::Point2f> corner_pts = facetlist[0];

			// 3. x,y와 셀의 변 사이중 최소값 찾기(d_v)
			double d_v = std::numeric_limits<double>::infinity();
			for (int i = 0; i < corner_pts.size(); ++i) {
				int next_i = (i + 1) % corner_pts.size();
				cv::Point2f a = corner_pts[i];
				cv::Point2f b = corner_pts[next_i];
				cv::Point2f ba_vec = b - a;	// b - a
				cv::Point2f pa_vec = p - a;	// p - a

				double dist_sq = ba_vec.x * ba_vec.x + ba_vec.y * ba_vec.y;
				if (dist_sq == 0) {
					d_v = std::min(d_v, cv::norm(p - a));
					continue;
				}
				double t = std::max(0.0, std::min(1.0, pa_vec.dot(ba_vec) / dist_sq));
				cv::Point2f proj = a + t * ba_vec;
				double min_dist = cv::norm(p - proj);

				if (min_dist < d_v)
					d_v = min_dist;
			}
			// pixel -> meter
			d_v *= occ_map_.pixel_scale_;
			// rho_v 계산
			double dist = dist_meter(y, x);
			// if (dist <= sigma) { // sigma 거리 이내
			// 	cost_map_(y,x) = std::numeric_limits<double>::infinity();
			// 	continue;
			// }
			if (occ_map_(y, x) == 1) {
				cost_map_(y, x) = 254.0;	// LETHAL_OBSTACLE
			}
			else if (dist <= sigma) {
				cost_map_(y, x) = 253.0;	// INSCRIBED_OBSTACLE
			}
			if (dist <= max_dist_val) {
				double cost_decay = 1.0 / (1.0 + dist);
				double voronoi_decay = (d_v / (dist + d_v));
				double potential_cost = std::pow((dist - max_dist_val) / max_dist_val, 2);
				// cost_map_(y,x) = cost_decay * voronoi_decay * potential_cost;// +1.0;
				cost_map_(y,x) = 252.0 * cost_decay * voronoi_decay * potential_cost;	// INFRATION_LAYER
			}
			else
				cost_map_(y,x) = 0.0;
		}
	}

	// getVoronoiField(rows, cols, 10, subdiv); // 보로노이 필드 가시화
	visualizeCostMap(cost_map_, 10, title); // cost map 가시화
}

inline void expotentialCostMap(
	//const cv::Mat& dist_meter,//int rows_, int cols_, double px_scale,
	const GridMap<double>& dist_meter,
	const GridMap<int>& occ_map_,
	GridMap<double>& cost_map_,
	double beta, double sigma,
	const std::string& title)
{
	// beta : 비용의 감쇄 속도
	// sigma = (sigma < 1e-6) ? 1.0 : sigma;
	for (int y = 0; y < dist_meter.data_.rows(); ++y) {
		for (int x = 0; x < dist_meter.data_.cols(); ++x) {

			double dist = dist_meter.data_(y, x);
			// if (dist <= sigma) {
			// 	cost_map_(y,x) = std::numeric_limits<double>::infinity();
			// 	continue;
			// }
			if (occ_map_(y, x) == 1) {
				cost_map_(y, x) = 254.0;	// LETHAL_OBSTACLE
			}
			else if (dist <= sigma) {
				cost_map_(y, x) = 253.0;	// INSCRIBED_OBSTACLE
			}
			double clearance = dist - sigma;
			cost_map_(y, x) = 252.0 * std::exp(-beta * (clearance / sigma));	// INFRATION_LAYER
			// cost_map_(y, x) = (occ_map_(y, x) == 1.0) ? std::numeric_limits<double>::infinity()
			// 	: alpha * std::exp(-beta * (clearance / sigma));
		}
	}
	visualizeCostMap(cost_map_, 10, title); // cost map 가시화
}

inline void sigmoidCostMap(
	//const cv::Mat& dist_meter,//int rows_, int cols_, double px_scale,
	const GridMap<double>& dist_meter,
	const GridMap<int>& occ_map_,
	GridMap<double>& cost_map_,
	double k, double sigma,
	const std::string& title)
{
	// k : 비용의 가파름 정도
	for (int y = 0; y < dist_meter.data_.rows(); ++y) {
		for (int x = 0; x < dist_meter.data_.cols(); ++x) {

			double dist = dist_meter(y, x);
			// if (dist <= sigma) { // sigma 거리 이내
			// 	cost_map_(y,x) = std::numeric_limits<double>::infinity();
			// }
			if (occ_map_(y, x) == 1) {
				cost_map_(y, x) = 254.0;	// LETHAL_OBSTACLE
			}
			else if (dist <= sigma) {
				cost_map_(y, x) = 253.0;	// INSCRIBED_OBSTACLE
			}
			else {
				double clearance = dist - sigma;
				// cost_map_(y,x) = (occ_map_(y,x) == 1.0) ? std::numeric_limits<double>::infinity()
				// 	: 1.0 / (1.0 + std::exp(k * clearance));
				cost_map_(y,x) = 252.0 * (2.0 / (1.0 + std::exp(k * clearance)));	// INFRATION_LAYER
				
			}
		}
	}
	visualizeCostMap(cost_map_, 10, title); // cost map 가시화
}

inline void Nav2CostMap(
	//const cv::Mat& dist_meter,//int rows_, int cols_, double px_scale,
	const GridMap<double>& dist_meter,
	const GridMap<int>& occ_map_,
	GridMap<double>& cost_map_,
	double sigma,
	// double weight,
	double inflation_w,
	const std::string& title)
{
	// if (inscribed_w >= inflation_w) throw;
	// double inscribed_radius = inscribed_w * sigma;
	double inflation_radius = inflation_w * sigma;

	for (int y = 0; y < dist_meter.data_.rows(); ++y) {
		for (int x = 0; x < dist_meter.data_.cols(); ++x) {

			double dist = dist_meter(y, x);
			// if (dist <= sigma) {
				// cost_map_(y, x) = std::numeric_limits<double>::infinity();
				// continue;
			// // }
			// else if (dist < inscribed_radius) {
			// 	// 로봇이 벽에 끼었을 때 탈출할 수 있도록 "지나치게 비싼" 비용은 피함
			// 	double ratio = (inscribed_radius - dist) / inscribed_radius;
			// 	cost_map_(y, x) = weight * (1.0 + ratio);
			// }
			if (occ_map_(y, x) == 1) {
				cost_map_(y, x) = 254.0;	// LETHAL_OBSTACLE
			}
			// else if (dist <= inscribed_radius) {
			else if (dist <= sigma) {
				cost_map_(y, x) = 253.0;	// INSCRIBED_OBSTACLE
			}
			else if (dist < inflation_radius) {
				// 거리가 멀어질수록 비용이 부드럽게 1.0으로 떨어짐
				// double ratio = (inflation_radius - dist) / (inflation_radius - inscribed_radius);
				double ratio = (inflation_radius - dist) / (inflation_radius - sigma);

				// 거리의 제곱에 비례하여 비용 감소 (가까울수록 급격히 증가)
				// cost_map_(y, x) = (weight * ratio * ratio) + 1.0;
				cost_map_(y, x) = (251.0 * ratio * ratio) + 1.0;	// INFRATION_LAYER
			}
			else{
				cost_map_(y, x) = 0.0;		// FREE_SPACE
			}
			//std::cout << "cost_map : " << cost_map_[y][x] << "\n";
		}
	}
	visualizeCostMap(cost_map_, 10, title); // cost map 가시화
}

// ============== [Helper functions] ============== 
inline void getDelaunayTriangle(
	int rows, int cols, int cell_size,
	const cv::Rect& rect, cv::Subdiv2D& subdiv,
	std::vector<cv::Vec6f>& triangleList)
{
	// 전체 delaunay triangle 가져오기 예시
	subdiv.getTriangleList(triangleList);
	drawDelaunayTriangle(rows, cols, cell_size, rect, triangleList);
}
inline void drawDelaunayTriangle(
	int rows, int cols, int cell_size,
	const cv::Rect rect,
	const std::vector<cv::Vec6f> triangleList)
{
	cv::Mat img = cv::Mat::zeros(rows * cell_size, cols * cell_size, CV_8UC3);
	for (size_t i = 0; i < triangleList.size(); i++) {
		cv::Vec6f t = triangleList[i];
		cv::Point pt1(cvRound(t[0]), cvRound(t[1])), pt2(cvRound(t[2]), cvRound(t[3])), pt3(cvRound(t[4]), cvRound(t[5]));

		// 삼각형의 모든 정점이 사각형 영역 안에 있는지 확인 후 그리기
		if (rect.contains(pt1) && rect.contains(pt2) && rect.contains(pt3)) {
			cv::line(img, pt1 * cell_size, pt2 * cell_size, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
			cv::line(img, pt2 * cell_size, pt3 * cell_size, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
			cv::line(img, pt3 * cell_size, pt1 * cell_size, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
		}
	}

	imshow("Delaunay Triangulation", img);
	cv::waitKey(10);
}

inline void getVoronoiField(
	int rows, int cols, int cell_size,
	cv::Subdiv2D& subdiv)
{
	// 전체 voronoi set 가져오기 예시
	std::vector<std::vector<cv::Point2f>> facets;	// 각 셀의 집합
	std::vector<cv::Point2f> centers;	// 각셀의 중심점
	std::vector<int> indices;			// 셀 별 고유 인덱스
	subdiv.getVoronoiFacetList(indices, facets, centers);
	drawVoronoiField(rows, cols, cell_size, facets, centers);
}
inline void drawVoronoiField(
	int rows, int cols, int cell_size,
	const std::vector<std::vector<cv::Point2f>> facets,
	const std::vector<cv::Point2f> centers)
{
	// voronoi 결과 시각화 및 데이터 접근
	cv::Mat img = cv::Mat::zeros(rows * cell_size, cols * cell_size, CV_8UC3);
	for (size_t i = 0; i < facets.size(); i++) {
		// OpenCV의 fillConvexPoly 등을 사용하기 위해 Point2f를 Point로 변환
		std::vector<cv::Point> ifacet;
		for (size_t j = 0; j < facets[i].size(); j++) {
			ifacet.push_back(facets[i][j] * cell_size);
		}

		// 랜덤 색상으로 각 셀 채우기
		cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
		cv::fillConvexPoly(img, ifacet, color, cv::LINE_AA);

		// 셀의 경계선 그리기
		polylines(img, ifacet, true, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

		// 중심점 표시
		circle(img, centers[i] * cell_size, 2, cv::Scalar(0, 0, 0), -1);
	}

	// cv::imshow("Voronoi Field", img);
	// cv::waitKey(10);
	cv::imwrite("/home/uj/Voronoi Field.png", img);
}
inline void visualizeCostMap(
	//const std::vector<std::vector<double>>& cost_map,
	const GridMap<double>& cost_map,
	int cell_size, const std::string& title) 
{
	int rows = cost_map.data_.rows();
	int cols = cost_map.data_.cols();

	// 1. float 형식의 Mat 생성 (계산의 정밀도를 위해)
	cv::Mat map_mat(rows, cols, CV_32FC1);
	double max_finite_val = 0.0;

	// 먼저 무한대가 아닌 값들 중 최댓값을 찾기
	for (int x = 0; x < cols; ++x) {
		for (int y = 0; y < rows; ++y) {
			double val = cost_map(y, x);
			if (!std::isinf(val) && val > max_finite_val)
				max_finite_val = val;
		}
	}
	//for (const auto& row : cost_map.data_) {
	//	for (double val : row) {
	//		if (!std::isinf(val) && val > max_finite_val) max_finite_val = val;
	//	}
	//}

	// 데이터 복사 및 infinity 처리
	for (int y = 0; y < rows; ++y) {
		for (int x = 0; x < cols; ++x) {
			double val = cost_map(y,x);
			if (std::isinf(val)) {
				// 장애물(무한대)은 최댓값보다 조금 더 높은 값으로 설정
				map_mat.at<float>(y, x) = static_cast<float>(max_finite_val * 1.2);
			}
			else {
				map_mat.at<float>(y, x) = static_cast<float>(val);
			}
		}
	}
	cv::Mat resized_mat;
	// cell_size를 반영하여 출력 크기 결정
	cv::resize(map_mat, resized_mat, cv::Size(), cell_size, cell_size, cv::INTER_NEAREST);

	// 2. 0~255 범위로 정규화 (CV_8U 타입으로 변환)
	cv::Mat norm_mat;
	cv::normalize(resized_mat, norm_mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	// 3. 컬러맵 적용 (JET 맵: 낮은 값=파란색, 높은 값=빨간색)
	cv::Mat color_mat;
	cv::applyColorMap(norm_mat, color_mat, cv::COLORMAP_JET);
	//cv::ColormapTypes::

	// 결과 출력
	// cv::imshow("Voronoi Cost Map Visualization", color_mat);
	// cv::waitKey(10);
	fs::path dir("/home/uj");
	fs::path name = title + ".png";
	fs::path path_img = dir / name;
	cv::imwrite(path_img.string(), color_mat);
}

#endif