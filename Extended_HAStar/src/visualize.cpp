#include "visualize.h"

namespace fs = std::filesystem;

/////////////////////////////////////////////////////////
///////////////// STANDALONE VISUALIZE //////////////////
/////////////////////////////////////////////////////////
inline void drawVehicle(
	cv::Mat& img,
	int x, int y, double theta,
	double length, double width,
	cv::Scalar color)
{
	std::vector<cv::Point> tri;
	// 앞쪽
	cv::Point p0(
		x + 0.5 * length * std::cos(theta),
		//y - 0.5 * length * std::sin(theta)
		y + 0.5 * length * std::sin(theta)  // 아래가 +y
	);

	// 뒤쪽 중심
	double bx = x - 0.5 * length * std::cos(theta);
	//double by = y + 0.5 * length * std::sin(theta);
	double by = y - 0.5 * length * std::sin(theta);

	// 좌우
	cv::Point p1(
		bx + 0.5 * width * std::cos(theta + CV_PI / 2),
		//by - 0.5 * width * std::sin(theta + CV_PI / 2)
		by + 0.5 * width * std::sin(theta + CV_PI / 2)  // 아래가 +y
	);

	cv::Point p2(
		bx + 0.5 * width * std::cos(theta - CV_PI / 2),
		//by - 0.5 * width * std::sin(theta - CV_PI / 2)
		by + 0.5 * width * std::sin(theta - CV_PI / 2)  // 아래가 +y
	);

	// 로봇의 완전한 끝(0.5)이 아니라, 약간 중심쪽(-0.1)으로 당겨서 V자 홈을 만듭니다.
	cv::Point p_inner(
		x - 0.1 * length * std::cos(theta),
		y - 0.1 * length * std::sin(theta)
	);

	tri.push_back(p0);
	tri.push_back(p1);
	tri.push_back(p_inner);
	tri.push_back(p2);

	cv::polylines(img, tri, true, color, 1);
	//cv::fillConvexPoly(img, tri, color);
}

void visualize_hybridastar_path(
	//const std::vector<std::pair<State, VehicleMode>>& path,
	const std::vector<State>& path,
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size, double r_length, double r_width,
	std::string title, double resolution)
{
	int legend_x0 = 0;// occ_map.origin_x_;
	int legend_y0 = 0;// occ_map.origin_y_; // 시작 위치
	int step = 25;        // 줄 간격

	//cv::Scalar start_color = Color::Yellow;
	//cv::Scalar goal_color = Color::Magenta;

	int rows = occ_map.data_.rows();
	int cols = occ_map.data_.cols();

	// Occupancy Grid 시각화용 이미지 (검은=장애물, 흰색=free)
	// 인덱스이므로 해상도 무관
	cv::Mat img(rows * cell_size, cols * cell_size, CV_8UC3, Color::White);
	for (int y = 0; y < rows; ++y) {//
		for (int x = 0; x < cols; ++x) {
			if (occ_map(y,x) == 1) {
				cv::rectangle(img,
					cv::Point(x * cell_size, y * cell_size),
					cv::Point((x + 1) * cell_size - 1, (y + 1) * cell_size - 1),
					Color::Black, cv::FILLED);
			}
		}
	}

	// 경로 시각화 (빨간 선)
	cv::Scalar mode_color;
	for (size_t i = 1; i < path.size(); ++i) {

		int p1x = occ_map.WorldXToXi(path[i - 1].x);
		int p1y = occ_map.WorldYToYi(path[i - 1].y);
		double p1t = path[i - 1].theta;
		VehicleMode p1mode = path[i - 1].vehicle;

		int p2x = occ_map.WorldXToXi(path[i].x);
		int p2y = occ_map.WorldYToYi(path[i].y);
		double p2t = path[i].theta;
		VehicleMode p2mode = path[i].vehicle;

		// p1의 vehiclemode에 따라 색상 결정
		if (p1mode == VehicleMode::BicycleMode)
			mode_color = Color::Red;
		else if (p1mode == VehicleMode::ParallelMode)
			mode_color = Color::Green;
		else
			mode_color = Color::Blue;

		// // line draw
		// cv::Point p1(p1x * cell_size, p1y * cell_size);
		// cv::Point p2(p2x * cell_size, p2y * cell_size);
		// cv::line(img, p1, p2, mode_color, 2);

		drawVehicle(img,
			p1x * cell_size,
			p1y * cell_size,
			p1t,
			(r_length / resolution) * cell_size,
			(r_width / resolution) * cell_size,
			mode_color);
	}
	// // Start point triangle
	// drawVehicle(img,
	// 	occ_map.WorldXToXi(path[0].x) * cell_size,
	// 	occ_map.WorldYToYi(path[0].y) * cell_size,
	// 	path[0].theta,
	// 	(r_length / resolution) * cell_size,
	// 	(r_width / resolution) * cell_size,
	// 	mode_color);
	// goal point triangle
	drawVehicle(img,
		occ_map.WorldXToXi(path[path.size() - 1].x) * cell_size,
		occ_map.WorldYToYi(path[path.size() - 1].y) * cell_size,
		path[path.size() - 1].theta,
		(r_length / resolution) * cell_size,
		(r_width / resolution) * cell_size,
		mode_color);

	// int startx = occ_map.WorldXToXi(path[0].x);
	// int starty = occ_map.WorldYToYi(path[0].y);
	// int goalx = occ_map.WorldXToXi(path[path.size() - 1].x);
	// int goaly = occ_map.WorldYToYi(path[path.size() - 1].y);
	// // double startx = (path[0].x / resolution), starty = (path[0].y / resolution);
	// // double goalx = (path[path.size() - 1].x / resolution), goaly = (path[path.size() - 1].y / resolution);
	// cv::Point start(startx * cell_size, starty * cell_size);
	// cv::Point goal(goalx * cell_size, goaly * cell_size);

	// cv::circle(img, start, cell_size / 10, start_color, cv::FILLED);
	// cv::circle(img, goal, cell_size / 10, goal_color, cv::FILLED);

	// // legend
	// cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10), 1, start_color, cv::FILLED);
	// cv::putText(img, "S", cv::Point(legend_x0 + 30, legend_y0 + 15),
	// 	cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	// cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10 + step), 1, goal_color, cv::FILLED);
	// cv::putText(img, "G", cv::Point(legend_x0 + 30, legend_y0 + 15 + step),
	// 	cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

	// cv::imshow(title, img);
	// cv::waitKey(0);
	fs::path dir("/home/uj");
	fs::path name = title + ".png";
	fs::path path_img = dir / name;
	cv::imwrite(path_img.string(), img);
}

void visualize_map(
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size,
	double sc, double sr, double gc, double gr,
	std::string title, double resolution)
{
	int legend_x0 = 0;// occ_map.origin_x_;
	int legend_y0 = 0;// occ_map.origin_y_; // 시작 위치
	int step = 25;        // 줄 간격

	int rows = occ_map.data_.rows();
	int cols = occ_map.data_.cols();

	// Occupancy Grid 시각화용 이미지 (검은=장애물, 흰색=free)
	cv::Mat img(rows * cell_size, cols * cell_size, CV_8UC3, Color::White);
	for (int y = 0; y < rows; ++y) {
		for (int x = 0; x < cols; ++x) {
			if (occ_map(y,x) == 1) {
				cv::rectangle(img,
					cv::Point(x * cell_size, y * cell_size),
					cv::Point((x + 1) * cell_size - 1, (y + 1) * cell_size - 1),
					Color::Black, cv::FILLED);
			}
		}
	}

	cv::Point start(occ_map.WorldXToXi(sc) * cell_size + cell_size / 2,
		occ_map.WorldYToYi(sr) * cell_size + cell_size / 2);
	cv::Point goal(occ_map.WorldXToXi(gc)* cell_size + cell_size / 2,
		occ_map.WorldYToYi(gr) * cell_size + cell_size / 2);

	cv::circle(img, start, cell_size / 10, start_color, cv::FILLED);
	cv::circle(img, goal, cell_size / 10, goal_color, cv::FILLED);

	// legend
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10), 8, start_color, cv::FILLED);
	cv::putText(img, "Start", cv::Point(legend_x0 + 30, legend_y0 + 15),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10 + step), 8, goal_color, cv::FILLED);
	cv::putText(img, "Goal", cv::Point(legend_x0 + 30, legend_y0 + 15 + step),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

	// cv::imshow(title, img);
	// cv::waitKey(10);
	fs::path dir("/home/uj");
	fs::path name = title + ".png";
	fs::path path_img = dir / name;
	cv::imwrite(path_img.string(), img);
}

void visualize_guide_path(
	const std::vector<std::pair<double, double>>& path,
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size,
	std::string title, double resolution)
{
	int legend_x0 = 0;// occ_map.origin_x_;
	int legend_y0 = 0;// occ_map.origin_y_; // 시작 위치
	int step = 25;        // 줄 간격

	//cv::Scalar start_color = Color::Yellow;
	//cv::Scalar goal_color = Color::Magenta;

	int rows = occ_map.data_.rows();
	int cols = occ_map.data_.cols();

	// Occupancy Grid 시각화용 이미지 (검은=장애물, 흰색=free)
	cv::Mat img(rows * cell_size, cols * cell_size, CV_8UC3, Color::White);
	for (int y = 0; y < rows; ++y) {
		for (int x = 0; x < cols; ++x) {
			if (occ_map(y,x) == 1) {
				cv::rectangle(img,
					cv::Point(x * cell_size, y * cell_size),
					cv::Point((x + 1) * cell_size - 1, (y + 1) * cell_size - 1),
					Color::Black, cv::FILLED);
			}
		}
	}

	// 경로 시각화 (빨간 선)
	for (size_t i = 1; i < path.size(); ++i) {
		// double xi = (path[i - 1].first / resolution);
		// double yi = (path[i - 1].second / resolution);
		// double xii = (path[i].first / resolution);
		// double yii = (path[i].second / resolution);
		int xi = occ_map.WorldXToXi(path[i - 1].first);
		int yi = occ_map.WorldYToYi(path[i - 1].second);
		int xii = occ_map.WorldXToXi(path[i].first);
		int yii = occ_map.WorldYToYi(path[i].second);

		// line draw
		cv::Point p1(xi * cell_size, yi * cell_size);
		cv::Point p2(xii * cell_size, yii * cell_size);
		cv::line(img, p1, p2, Color::Cyan, 2);

	}

	// double startx = (path[0].first / resolution), starty = (path[0].second / resolution);
	// double goalx = (path[path.size() - 1].first / resolution), goaly = (path[path.size() - 1].second / resolution);
	int startx = occ_map.WorldXToXi(path[0].first), starty = occ_map.WorldYToYi(path[0].second);
	int goalx = occ_map.WorldXToXi(path[path.size() - 1].first), goaly = occ_map.WorldYToYi(path[path.size() - 1].second);
	cv::Point start(startx * cell_size, starty * cell_size);
	cv::Point goal(goalx * cell_size, goaly * cell_size);

	cv::circle(img, start, cell_size / 10, start_color, cv::FILLED);
	cv::circle(img, goal, cell_size / 10, goal_color, cv::FILLED);

	// legend
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10), 8, start_color, cv::FILLED);
	cv::putText(img, "Start", cv::Point(legend_x0 + 30, legend_y0 + 15),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10 + step), 8, goal_color, cv::FILLED);
	cv::putText(img, "Goal", cv::Point(legend_x0 + 30, legend_y0 + 15 + step),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

	// cv::imshow(title, img);
	// cv::waitKey(10);
	fs::path dir("/home/uj");
	fs::path name = title + ".png";
	fs::path path_img = dir / name;
	cv::imwrite(path_img.string(), img);
}

void visualize_tracking_performance(
    const std::vector<State>& global_path,
    const std::vector<State>& tracked_path, // MPC가 실제로 주행한 궤적 추가
    GridMap<int>& occ_map,
    int cell_size, double r_length, double r_width,
    std::string title, double resolution,
	double origin_x, double origin_y)
{
    int rows = occ_map.data_.rows();
    int cols = occ_map.data_.cols();

    // 1. Occupancy Grid 맵 그리기 (기존 로직 동일)
    cv::Mat img(rows * cell_size, cols * cell_size, CV_8UC3, Color::White);
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (occ_map(y,x) == 1) {
                cv::rectangle(img,
                    cv::Point(x * cell_size, y * cell_size),
                    cv::Point((x + 1) * cell_size - 1, (y + 1) * cell_size - 1),
                    Color::Black, cv::FILLED);
            }
        }
    }

    // 2. 옅은 색상으로 전역 경로(Reference) 먼저 그리기 (밑바탕)
    for (size_t i = 1; i < global_path.size(); ++i) {
        int p1x = [&](double p) {
				return static_cast<int>(floor( (p - origin_x) / resolution));
			}(global_path[i - 1].x );
        int p1y = [&](double p) {
				return static_cast<int>(floor( (p - origin_y) / resolution));
			}(global_path[i - 1].y );
        int p2x = [&](double p) {
				return static_cast<int>(floor( (p - origin_x) / resolution));
			}(global_path[i].x );
        int p2y = [&](double p) {
				return static_cast<int>(floor( (p - origin_y) / resolution));
			}(global_path[i].y );
        
        cv::Point p1(p1x * cell_size, p1y * cell_size);
        cv::Point p2(p2x * cell_size, p2y * cell_size);
        
        // 옅은 회색 점선 느낌으로 그리기
        cv::line(img, p1, p2, cv::Scalar(200, 200, 200), 2);
    }

    // 3. 진한 색상으로 실제 추종 경로(Tracked) 그리기 (차량 폴리곤 포함)
	cv::Scalar mode_color;
    for (size_t i = 1; i < tracked_path.size(); ++i) {
        int p1x = [&](double p) {
				return static_cast<int>(floor( (p - origin_x) / resolution));
			}(tracked_path[i - 1].x );
        int p1y = [&](double p) {
				return static_cast<int>(floor( (p - origin_y) / resolution));
			}(tracked_path[i - 1].y );
        double p1t = tracked_path[i - 1].theta;
		VehicleMode p1mode = tracked_path[i-1].vehicle;

		// p1의 vehiclemode에 따라 색상 결정
		if (p1mode == VehicleMode::BicycleMode)
			mode_color = Color::Red;
		else if (p1mode == VehicleMode::ParallelMode)
			mode_color = Color::Green;
		else
			mode_color = Color::Blue;

        // // 실제 이동한 궤적 선 (파란색 등 눈에 띄는 색상)
        // int p2x = [&](double p) {
		// 		return static_cast<int>(floor( (p - origin_x) / px_scale));
		// 	}(tracked_path[i].x);
        // int p2y = [&](double p) {
		// 		return static_cast<int>(floor( (p - origin_y) / px_scale));
		// 	}(tracked_path[i].y);
        // cv::line(img, 
        //          cv::Point(p1x * cell_size, p1y * cell_size), 
        //          cv::Point(p2x * cell_size, p2y * cell_size), 
        //          mode_color, 3);

        // 로봇의 실제 헤딩을 반영한 박스 그리기
        drawVehicle(img,
            p1x * cell_size,
            p1y * cell_size,
            p1t,
            (r_length / resolution) * cell_size,
            (r_width / resolution) * cell_size,
            mode_color); // 차량 모드에 따라 색상 분기 가능
    }
	// goal point triangle
	int gx = [&](double p) {
			return static_cast<int>(floor( (p - origin_x) / resolution));
		}(tracked_path[tracked_path.size() - 1].x);
	int gy = [&](double p) {
			return static_cast<int>(floor( (p - origin_y) / resolution));
		}(tracked_path[tracked_path.size() - 1].y);
	drawVehicle(img,
		gx * cell_size,
		gy * cell_size,
		tracked_path[tracked_path.size() - 1].theta,
		(r_length / resolution) * cell_size,
		(r_width / resolution) * cell_size,
		mode_color);


    // 4. 이미지 저장
    fs::path dir("/home/uj");
    fs::path name = title + "_tracking.png";
    fs::path path_img = dir / name;
    cv::imwrite(path_img.string(), img);
    std::cout << "추종 결과 시각화 완료: " << path_img.string() << std::endl;
}

///////////////////////////////////////////////////
///////////////// ROS2 VISUALIZE //////////////////
///////////////////////////////////////////////////
inline void drawVehicle_ros2(
	cv::Mat& img,
	int x, int y, double theta,
	double length, double width,
	cv::Scalar color)
{
	std::vector<cv::Point> tri;
	// 앞쪽
	cv::Point p0(
		x + 0.5 * length * std::cos(theta),
		y - 0.5 * length * std::sin(theta)
		// y + 0.5 * length * std::sin(theta)  // 아래가 +y
	);

	// 뒤쪽 중심
	double bx = x - 0.5 * length * std::cos(theta);
	double by = y + 0.5 * length * std::sin(theta);
	// double by = y - 0.5 * length * std::sin(theta);

	// 좌우
	cv::Point p1(
		bx + 0.5 * width * std::cos(theta + CV_PI / 2),
		by - 0.5 * width * std::sin(theta + CV_PI / 2)
		// // by + 0.5 * width * std::sin(theta + CV_PI / 2)  // 아래가 +y
	);

	cv::Point p2(
		bx + 0.5 * width * std::cos(theta - CV_PI / 2),
		by - 0.5 * width * std::sin(theta - CV_PI / 2)
		// by + 0.5 * width * std::sin(theta - CV_PI / 2)  // 아래가 +y
	);
	// 로봇의 완전한 끝(0.5)이 아니라, 약간 중심쪽(-0.1)으로 당겨서 V자 홈을 만듭니다.
	cv::Point p_inner(
		x - 0.1 * length * std::cos(theta),
		y - 0.1 * length * std::sin(theta)
	);
	tri.push_back(p0);
	tri.push_back(p1);
	tri.push_back(p_inner);
	tri.push_back(p2);

	cv::polylines(img, tri, true, color, 1);
	//cv::fillConvexPoly(img, tri, color);
}

void visualize_ros2_hybridastar_path(
	//const std::vector<std::pair<State, VehicleMode>>& path,
	const std::vector<State>& path,
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size, double r_length, double r_width,
	std::string title, double resolution)
{
	int legend_x0 = 0;// occ_map.origin_x_;
	int legend_y0 = 0;// occ_map.origin_y_; // 시작 위치
	int step = 25;        // 줄 간격

	int rows = occ_map.data_.rows();
	int cols = occ_map.data_.cols();

	// Occupancy Grid 시각화용 이미지 (검은=장애물, 흰색=free)
	// 인덱스이므로 해상도 무관
	cv::Mat img(rows * cell_size, cols * cell_size, CV_8UC3, Color::White);
	for (int y = 0; y < rows; ++y) {//
		for (int x = 0; x < cols; ++x) {
			int y_lb = convert_ltTolb(rows, y);
			if (occ_map(y,x) == 1) {
				cv::rectangle(img,
					cv::Point(x * cell_size, y_lb * cell_size),
					cv::Point((x + 1) * cell_size - 1, (y_lb + 1) * cell_size - 1),
					Color::Black, cv::FILLED);
			}
		}
	}

	// 경로 시각화 (빨간 선)
	cv::Scalar mode_color;
	for (size_t i = 1; i < path.size(); ++i) {

		int p1x = occ_map.WorldXToXi(path[i - 1].x);
		int p1y = occ_map.WorldYToYi(path[i - 1].y);
		int p1y_lb = convert_ltTolb(rows, p1y);
		double p1t = path[i - 1].theta;
		VehicleMode p1mode = path[i - 1].vehicle;

		int p2x = occ_map.WorldXToXi(path[i].x);
		int p2y = occ_map.WorldYToYi(path[i].y);
		int p2y_lb = convert_ltTolb(rows, p2y);
		double p2t = path[i].theta;
		VehicleMode p2mode = path[i].vehicle;

		// p1의 vehiclemode에 따라 색상 결정
		if (p1mode == VehicleMode::BicycleMode)
			mode_color = Color::Red;
		else if (p1mode == VehicleMode::ParallelMode)
			mode_color = Color::Green;
		else
			mode_color = Color::Blue;

		// line draw
		cv::Point p1(p1x * cell_size, p1y_lb * cell_size);
		cv::Point p2(p2x * cell_size, p2y_lb * cell_size);
		cv::line(img, p1, p2, mode_color, 2);

		// if (i == 1 || p1mode != p2mode) {
		// 	drawVehicle_ros2(img,
		// 		p1x * cell_size,
		// 		p1y_lb * cell_size,
		// 		p1t,
		// 		(r_length / resolution) * (static_cast<int>(cell_size / 2)),
		// 		(r_width / resolution) * (static_cast<int>(cell_size / 2)),
		// 		mode_color);

		// }
		drawVehicle_ros2(img,
			p1x * cell_size,
			p1y_lb * cell_size,
			p1t,
			(r_length / resolution) * (static_cast<int>(cell_size / 2)),
			(r_width / resolution) * (static_cast<int>(cell_size / 2)),
			mode_color);
	}
	// goal point triangle
	int pgx = occ_map.WorldXToXi(path[path.size() - 1].x);
	int pgy = occ_map.WorldYToYi(path[path.size() - 1].y);
	int pgy_lb = convert_ltTolb(rows, pgy);
	double pgt = path[path.size() - 1].theta;
	drawVehicle_ros2(img,
		pgx * cell_size, pgy_lb * cell_size, pgt,
		(r_length / resolution) * (static_cast<int>(cell_size / 2)),
		(r_width / resolution) * (static_cast<int>(cell_size / 2)),
		mode_color);

	int startx = occ_map.WorldXToXi(path[0].x);
	int starty = occ_map.WorldYToYi(path[0].y);
	int starty_lb = convert_ltTolb(rows, starty);

	int goalx = occ_map.WorldXToXi(path[path.size() - 1].x);
	int goaly = occ_map.WorldYToYi(path[path.size() - 1].y);
	int goaly_lb = convert_ltTolb(rows, goaly);

	cv::Point start(startx * cell_size, starty_lb * cell_size);
	cv::Point goal(goalx * cell_size, goaly_lb * cell_size);

	cv::circle(img, start, cell_size, start_color, cv::FILLED);
	cv::circle(img, goal, cell_size, goal_color, cv::FILLED);

	// legend
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10), 1, start_color, cv::FILLED);
	cv::putText(img, "S", cv::Point(legend_x0 + 30, legend_y0 + 15),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10 + step), 1, goal_color, cv::FILLED);
	cv::putText(img, "G", cv::Point(legend_x0 + 30, legend_y0 + 15 + step),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

	// cv::imshow(title, img);
	// cv::waitKey(0);
	fs::path dir("/home/uj");
	fs::path name = title + ".png";
	fs::path path_img = dir / name;
	cv::imwrite(path_img.string(), img);
}

void visualize_ros2_map(
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size,
	double sc, double sr, double gc, double gr,
	std::string title, double resolution)
{
	int legend_x0 = 0;// occ_map.origin_x_;
	int legend_y0 = 0;// occ_map.origin_y_; // 시작 위치
	int step = 25;        // 줄 간격

	int rows = occ_map.data_.rows();
	int cols = occ_map.data_.cols();

	// Occupancy Grid 시각화용 이미지 (검은=장애물, 흰색=free)
	// 인덱스이므로 해상도 무관
	cv::Mat img(rows * cell_size, cols * cell_size, CV_8UC3, Color::White);
	for (int y = 0; y < rows; ++y) {//
		for (int x = 0; x < cols; ++x) {
			int y_lb = convert_ltTolb(rows, y);
			if (occ_map(y,x) == 1) {
				cv::rectangle(img,
					cv::Point(x * cell_size, y_lb * cell_size),
					cv::Point((x + 1) * cell_size - 1, (y_lb + 1) * cell_size - 1),
					Color::Black, cv::FILLED);
			}
		}
	}

	int startx = occ_map.WorldXToXi(sc);
	int starty = occ_map.WorldYToYi(sr);
	int starty_lb = convert_ltTolb(rows, starty);

	int goalx = occ_map.WorldXToXi(gc);
	int goaly = occ_map.WorldYToYi(gr);
	int goaly_lb = convert_ltTolb(rows, goaly);

	cv::Point start(startx * cell_size + cell_size / 2, starty_lb * cell_size + cell_size / 2);
	cv::Point goal(goalx * cell_size + cell_size / 2, goaly_lb * cell_size + cell_size / 2);

	cv::circle(img, start, cell_size, start_color, cv::FILLED);
	cv::circle(img, goal, cell_size, goal_color, cv::FILLED);

	// legend
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10), 8, start_color, cv::FILLED);
	cv::putText(img, "Start", cv::Point(legend_x0 + 30, legend_y0 + 15),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10 + step), 8, goal_color, cv::FILLED);
	cv::putText(img, "Goal", cv::Point(legend_x0 + 30, legend_y0 + 15 + step),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

	cv::imshow(title, img);
	// cv::waitKey(3000);
	cv::waitKey(3);
}

void visualize_ros2_guide_path(
	const std::vector<std::pair<double, double>>& path,
	//const std::vector<std::vector<int>>& occ_map,
	GridMap<int>& occ_map,
	int cell_size,
	std::string title, double resolution)
{
	int legend_x0 = 0;// occ_map.origin_x_;
	int legend_y0 = 0;// occ_map.origin_y_; // 시작 위치
	int step = 25;        // 줄 간격

	//cv::Scalar start_color = Color::Yellow;
	//cv::Scalar goal_color = Color::Magenta;

	int rows = occ_map.data_.rows();
	int cols = occ_map.data_.cols();

	// Occupancy Grid 시각화용 이미지 (검은=장애물, 흰색=free)
	// 인덱스이므로 해상도 무관
	cv::Mat img(rows * cell_size, cols * cell_size, CV_8UC3, Color::White);
	for (int y = 0; y < rows; ++y) {//
		for (int x = 0; x < cols; ++x) {
			int y_lb = convert_ltTolb(rows, y);
			if (occ_map(y,x) == 1) {
				cv::rectangle(img,
					cv::Point(x * cell_size, y_lb * cell_size),
					cv::Point((x + 1) * cell_size - 1, (y_lb + 1) * cell_size - 1),
					Color::Black, cv::FILLED);
			}
		}
	}

	// 경로 시각화 (빨간 선)
	for (size_t i = 1; i < path.size(); ++i) {
		int xi = occ_map.WorldXToXi(path[i - 1].first);
		int yi = occ_map.WorldYToYi(path[i - 1].second);
		int yi_lb = convert_ltTolb(rows, yi);
		int xii = occ_map.WorldXToXi(path[i].first);
		int yii = occ_map.WorldYToYi(path[i].second);
		int yii_lb = convert_ltTolb(rows, yii);

		// line draw
		cv::Point p1(xi * cell_size, yi_lb * cell_size);
		cv::Point p2(xii * cell_size, yii_lb * cell_size);
		cv::line(img, p1, p2, Color::Cyan, 2);

	}

	// double startx = (path[0].first / resolution), starty = (path[0].second / resolution);
	// double goalx = (path[path.size() - 1].first / resolution), goaly = (path[path.size() - 1].second / resolution);

	int startx = occ_map.WorldXToXi(path[0].first);
	int starty = occ_map.WorldYToYi(path[0].second);
	int starty_lb = convert_ltTolb(rows, starty);

	int goalx = occ_map.WorldXToXi(path[path.size() - 1].first);
	int goaly = occ_map.WorldYToYi(path[path.size() - 1].second);
	int goaly_lb = convert_ltTolb(rows, goaly);

	cv::Point start(startx * cell_size, starty_lb * cell_size);
	cv::Point goal(goalx * cell_size, goaly_lb * cell_size);

	cv::circle(img, start, cell_size, start_color, cv::FILLED);
	cv::circle(img, goal, cell_size, goal_color, cv::FILLED);

	// legend
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10), 8, start_color, cv::FILLED);
	cv::putText(img, "Start", cv::Point(legend_x0 + 30, legend_y0 + 15),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
	cv::circle(img, cv::Point(legend_x0 + 10, legend_y0 + 10 + step), 8, goal_color, cv::FILLED);
	cv::putText(img, "Goal", cv::Point(legend_x0 + 30, legend_y0 + 15 + step),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

	// cv::imshow(title, img);
	// cv::waitKey(10);
	fs::path dir("/home/uj");
	fs::path name = title + ".png";
	fs::path path_img = dir / name;
	cv::imwrite(path_img.string(), img);
}