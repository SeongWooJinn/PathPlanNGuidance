#include "maps.h"

OccMap::OccMap() {}
OccMap::OccMap(const int& rows, const int& cols, double resolution)
	//: rows_{ rows }, cols_{ cols }, occ_map_(rows_, std::vector<int>(cols_, 0)) { }
	: rows_{ rows }, cols_{ cols }, occ_map_(rows, cols, resolution) { }

void OccMap::init_map() {
	occ_map_.data_.setZero();
	//std::vector<std::vector<int>> map(rows_, std::vector<int>(cols_, 0));
	//occ_map_ = map;
}
// All r,c -> index
void OccMap::generate_Random_Map(int num_rnd_obs, const int& sc_, const int& sr_, const int& gc_, const int& gr_) {
	init_map();
	try
	{
		std::mt19937 rng(0);
		std::uniform_int_distribution<int> uix(0, cols_ - 1);
		std::uniform_int_distribution<int> uiy(0, rows_ - 1);

		for (int i = 0; i < num_rnd_obs; ++i) {
			int r = uiy(rng);
			int c = uix(rng);
			if (occ_map_(r,c) == 1) continue;
			setObstacles(r, c);
			if (!isPossible(sr_, sc_, gr_, gc_, r, c)) throw;
		}

	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << "\n";
	}
}

void OccMap::generate_example_map(int num_rnd_obs, const int& sc_, const int& sr_, const int& gc_, const int& gr_) {
	init_map();
	try
	{
		// Block 1
		meterToIndex(5.0, 9.0, 8.0, 14.0);
		for (int r = r_start_; r < r_end_; ++r) {
			for (int c = c_start_; c < c_end_; ++c) {
				if (!isBound(r, c)) throw std::out_of_range("Obstacle cell is Out of Range");
				setObstacles(r, c);
				if (!isPossible(sr_, sc_, gr_, gc_, r, c)) throw;
			}
		}
		// Block 2
		meterToIndex(12.0, 16.0, 2.0, 6.0);
		for (int r = r_start_; r < r_end_; ++r) {
			for (int c = c_start_; c < c_end_; ++c) {
				if (!isBound(r, c)) throw std::out_of_range("Obstacle cell is Out of Range");
				setObstacles(r, c);
				if (!isPossible(sr_, sc_, gr_, gc_, r, c)) throw;
			}
		}
		// Block 3
		meterToIndex(20.0, 25.0, 17.0, 53.0);
		for (int r = r_start_; r < r_end_; ++r) {
			for (int c = c_start_; c < c_end_; ++c) {
				if (!isBound(r, c)) throw std::out_of_range("Obstacle cell is Out of Range");
				setObstacles(r, c);
				if (!isPossible(sr_, sc_, gr_, gc_, r, c)) throw;
			}
		}
		// Block 3
		meterToIndex(0.0, 10.0, 20.0, 30.0);
		for (int r = r_start_; r < r_end_; ++r) {
			for (int c = c_start_; c < c_end_; ++c) {
				if (!isBound(r, c)) throw std::out_of_range("Obstacle cell is Out of Range");
				setObstacles(r, c);
				if (!isPossible(sr_, sc_, gr_, gc_, r, c)) throw;
			}
		}
		// vertical wall 1
		meterToIndex(18.0, 28.0, 3.0, 0.0);
		for (int r = r_start_; r < r_end_; ++r) {
			if (r < 0 || r >= rows_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r, c_start_);
			if (!isPossible(sr_, sc_, gr_, gc_, r, c_start_)) throw;
		}
		// vertical wall 1
		meterToIndex(16.0, 23.0, 6.0, 0.0);
		for (int r = r_start_; r < r_end_; ++r) {
			if (r < 0 || r >= rows_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r, c_start_);
			if (!isPossible(sr_, sc_, gr_, gc_, r, c_start_)) throw;
		}
		// horizontal wall 1
		meterToIndex(21.0, 0.0, 0.0, 4.0);
		for (int c = c_start_; c < c_end_; ++c) {
			if (c < 0 || c >= cols_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(21, c);
			if (!isPossible(sr_, sc_, gr_, gc_, r_start_, c)) throw;
		}
		// horizontal wall 1
		meterToIndex(9.0, 0.0, 0.0, 18.0);
		for (int c = c_start_; c < c_end_; ++c) {
			if (c < 0 || c >= cols_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r_start_, c);
			if (!isPossible(sr_, sc_, gr_, gc_, r_start_, c)) throw;
		}
		getRandomObs(num_rnd_obs, sr_, sc_, gr_, gc_);

	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << "\n";
	}
}

void OccMap::generate_example_map_v2(int num_rnd_obs, const int& sc_, const int& sr_, const int& gc_, const int& gr_) {
	init_map();
	try
	{
		// vertical wall 1
		for (int r = 18; r < rows_; ++r) {
			if (r < 0 || r >= rows_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r, 3);
			if (!isPossible(sr_, sc_, gr_, gc_, r, 3)) throw;
		}
		// vertical wall 1
		for (int r = 0; r < 15; ++r) {
			if (r < 0 || r >= rows_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r, 30);
			if (!isPossible(sr_, sc_, gr_, gc_, r, 30)) throw;
		}
		// horizontal wall 1
		for (int c = 20; c < cols_; ++c) {
			if (c < 0 || c >= cols_) throw std::out_of_range("Obstacle cell is Out of Range");
			if (!isPossible(sr_, sc_, gr_, gc_, 21, c)) throw;
		}
		// horizontal wall 1
		for (int c = 0; c < 18; ++c) {
			if (c < 0 || c >= cols_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(9, c);
			if (!isPossible(sr_, sc_, gr_, gc_, 9, c)) throw;
		}
		getRandomObs(num_rnd_obs, sr_, sc_, gr_, gc_);

	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << "\n";
	}
}

void OccMap::generate_example_map_v3(int num_rnd_obs, const int& sc_, const int& sr_, const int& gc_, const int& gr_) {
	init_map();
	try
	{
		meterToIndex(18.0, rows_, 3.0, 0.0);
		for (int r = r_start_; r < rows_; ++r) {
			if (r < 0 || r >= rows_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r, c_start_);
			if (!isPossible(sr_, sc_, gr_, gc_, r, c_start_)) throw;
		}
		meterToIndex(18.0, 0.0, 3.0, 12.0);
		for (int c = c_start_; c < c_end_; ++c) {
			if (c < 0 || c >= cols_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r_start_, c);
			if (!isPossible(sr_, sc_, gr_, gc_, r_start_, c)) throw;
		}

		// ㄴ
		// ㄴ
		meterToIndex(0.0, 15.0, 30.0, 0.0);
		for (int r = r_start_; r < r_end_; ++r) {
			if (r < 0 || r >= rows_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r, c_start_);
			if (!isPossible(sr_, sc_, gr_, gc_, r, c_start_)) throw;
		}
		meterToIndex(15.0, 0.0, 30.0, 45.0);
		for (int c = c_start_; c < c_end_; ++c) {
			if (c < 0 || c >= cols_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r_start_, c);
			if (!isPossible(sr_, sc_, gr_, gc_, r_start_, c)) throw;
		}
		meterToIndex(5.0, 0.0, 30.0, 45.0);
		for (int c = c_start_; c < c_end_; ++c) {
			if (c < 0 || c >= cols_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r_start_, c);
			if (!isPossible(sr_, sc_, gr_, gc_, r_start_, c)) throw;
		}

		// 『
		meterToIndex(21.0, 0.0, 20.0, cols_);
		for (int c = c_start_; c < c_end_; ++c) {
			if (c < 0 || c >= cols_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r_start_, c);
			if (!isPossible(sr_, sc_, gr_, gc_, r_start_, c)) throw;
		}
		meterToIndex(21.0, 25.0, 20.0, 0.0);
		for (int r = r_start_; r < r_end_; ++r) {
			if (r < 0 || r >= rows_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r, c_start_);
			if (!isPossible(sr_, sc_, gr_, gc_, r, c_start_)) throw;
		}

		// 」
		meterToIndex(9.0, 0.0, 0.0, 19.0);
		for (int c = c_start_; c < c_end_; ++c) {
			if (c < 0 || c >= cols_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r_start_, c);
			if (!isPossible(sr_, sc_, gr_, gc_, r_start_, c)) throw;
		}
		meterToIndex(6.0, 9.0, 18.0, 0.0);
		for (int r = r_start_; r < r_end_; ++r) {
			if (r < 0 || r >= rows_) throw std::out_of_range("Obstacle cell is Out of Range");
			setObstacles(r, c_start_);
			if (!isPossible(sr_, sc_, gr_, gc_, r, c_start_)) throw;
		}
		getRandomObs(num_rnd_obs, sr_, sc_, gr_, gc_);

	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << "\n";
	}
}
GridMap<int>& OccMap::getOccMap() {
	return occ_map_;
}
void OccMap::setOccMap(const GridMap<int>& _map) {
	occ_map_ = _map;
}
//std::vector<std::vector<int>>& OccMap::getOccMap() {
//	return occ_map_;
//}
//void OccMap::setOccMap(const std::vector<std::vector<int>>& _map) {
//	occ_map_ = _map;
//}

void OccMap::setObstacles(int r, int c) {
	if (!isBound(r, c)) {
		std::cerr << r << ", " << c << " out of bound\n" << std::endl;
		return;
	}
	// if (occ_map_(r,c) == 1) {
	// 	std::cerr << r << ", " << c << " is already free space\n" << std::endl;
	// 	return;
	// }
	occ_map_(r,c) = 1;
}

void OccMap::removeObstacles(int r, int c) {
	if (!isBound(r, c)) {
		std::cerr << r << ", " << c << " out of bound\n" << std::endl;
		return;
	}
	if (occ_map_(r,c) == 0) {
		std::cerr << r << ", " << c << " is not obstacle\n" << std::endl;
		return;
	}
	occ_map_(r,c) = 0;
}
int OccMap::getRow() const {
	return rows_;
}
int OccMap::getCol() const {
	return cols_;
}
bool OccMap::isPossible(const int& sr_, const int& sc_, const int& gr_, const int& gc_, int r, int c)
{
	if ((r == sr_ && c == sc_) || (r == gr_ && c == gc_)) {
		std::cout << "Position Error\n"
			<< "start (r, c) : " << "(" << sr_ << ", " << sc_ << ")\n"
			<< "goal (r, c) : " << "(" << gr_ << ", " << gc_ << ")\n"
			<< "obstacle (r, c) : " << "(" << r << ", " << c << ")\n";
		return false;
	}
	return true;
}
void OccMap::getRandomObs(int num_rnd_obs, const int& sr_, const int& sc_, const int& gr_, const int& gc_)
{
	if (num_rnd_obs != 0) {
		// random obstacles
		std::mt19937 rng(1);
		std::uniform_int_distribution<int> uix(0, cols_ - 1);
		std::uniform_int_distribution<int> uiy(0, rows_ - 1);

		for (int i = 0; i < num_rnd_obs; ++i) {
			int r = uiy(rng);
			int c = uix(rng);
			if (occ_map_(r,c) == 1) continue;
			if ((r == sr_ && c == sc_) || (r == gr_ && c == gc_)) continue;
			setObstacles(r, c);
			if (!isPossible(sr_, sc_, gr_, gc_, r, c))
				throw std::invalid_argument("Obstacle cell is same with start or goal cell");
		}
	}
}
//
//////////// SLAM Class
//SLAM::SLAM(const OccMap& gt_map, const int& view_range) :
//	gt_map_(gt_map), view_range_(view_range) {
//
//	slam_map_ = gt_map_;
//	slam_map_.init_map();  // init slam occ map : free spaces
//}
//
//std::vector<std::pair<int, int>> SLAM::rescan(int cr, int cc) {
//	//std::cout << "12\n";
//	//std::cout << "(" << cx << ", " << cy << ")\n";
//	std::vector<std::pair<int, int>> changedGrids;
//	int center = view_range_ / 2;
//	for (int r = -center; r <= center; ++r) {
//		for (int c = -center; c <= center; ++c) {
//			int nr = cr + r;
//			int nc = cc + c;
//			if (!isBound(nr, nc)) continue;
//			int gt_val = gt_map_.getOccMap()[nr][nc];
//			int slam_val = slam_map_.getOccMap()[nr][nc];
//			std::cout << "gt : " << gt_val << ", " << "slam : " << slam_val << "\n";
//			if (gt_val != slam_val) {
//				slam_map_.getOccMap()[nr][nc] = gt_val;
//				changedGrids.push_back({ nr, nc });
//			}
//		}
//	}
//	return changedGrids;
//}
//
////const OccMap& SLAM::getSlamMap() const {
////	return slam_map_;
////}
//OccMap& SLAM::getSlamMap() {
//	return slam_map_;
//}

//int main()
//{
//	int rows = 30;
//	int cols = 30;
//	int rnd_obs = 100;
//	int sx = 0; int sy = 0;
//	int gx = 9; int gy = 9;
//	OccMap map(rows, cols);
//	map.generate_example_map(rnd_obs, sx, sy, gx, gy);
//	auto ori = map.getOccMap();
//
//	int view_rng = 3;
//	SLAM slam(map, view_rng);
//	/*slam.rescan(sx, sy);
//	auto local = slam.getSlamMap().getOccMap();*/
//	for (int i = 0; i <= 15; ++i) {
//		for (int j = 0; j <= 15; ++j) {
//			slam.rescan(i, j);
//		}
//	}
//	auto local = slam.getSlamMap().getOccMap();
//	// Occupancy Grid 시각화용 이미지 (검은=장애물, 흰색=free)
//	int cell_size = 10;
//	cv::Mat oriimg(rows * cell_size, cols * cell_size, CV_8UC3, cv::Scalar(255, 255, 255));
//	cv::Mat slamimg(rows * cell_size, cols * cell_size, CV_8UC3, cv::Scalar(255, 255, 255));
//	for (int y = 0; y < rows; ++y) {
//		for (int x = 0; x < cols; ++x) {
//			if (ori[y][x] == 1) {
//				cv::rectangle(oriimg,
//					cv::Point(x * cell_size, y * cell_size),
//					cv::Point((x + 1) * cell_size - 1, (y + 1) * cell_size - 1),
//					cv::Scalar(0, 0, 0), cv::FILLED);
//			}
//
//			if (local[y][x] == 1) {
//				cv::rectangle(slamimg,
//					cv::Point(x * cell_size, y * cell_size),
//					cv::Point((x + 1) * cell_size - 1, (y + 1) * cell_size - 1),
//					cv::Scalar(0, 0, 0), cv::FILLED);
//			}
//		}
//	}
//
//	int left = (sy - view_rng) * cell_size;
//	int top = (sx - view_rng) * cell_size;
//	int width = (2 * view_rng + 1) * cell_size;
//	int height = width;
//	cv::rectangle(slamimg, cv::Rect(left, top, width, height), cv::Scalar(255, 0, 255), 2);
//
//	cv::imshow("OriginalOccMap", oriimg);
//	cv::imshow("SlamOccMap", slamimg);
//	cv::waitKey();
//}
