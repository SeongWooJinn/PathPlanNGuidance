// #pragma once
#ifndef MAPS_H
#define MAPS_H
#include <iostream>
#include <vector>
#include <exception>
#include <random>
#include "structs.h"

class OccMap
{
public:
	OccMap();
	OccMap(const int& rows, const int& cols, double resolution);
	void init_map();
	void generate_Random_Map(int num_rnd_obs, const int& sc_, const int& sr_, const int& gc_, const int& gr_);
	void generate_example_map(int num_rnd_obs, const int& sc_, const int& sr_, const int& gc_, const int& gr_);
	void generate_example_map_v2(int num_rnd_obs, const int& sc_, const int& sr_, const int& gc_, const int& gr_);
	void generate_example_map_v3(int num_rnd_obs, const int& sc_, const int& sr_, const int& gc_, const int& gr_);
	//std::vector<std::vector<int>>& getOccMap();
	//void setOccMap(const std::vector<std::vector<int>>& _map);
	GridMap<int>& getOccMap();
	void setOccMap(const GridMap<int>& _map);
	void setObstacles(int r, int c);
	void removeObstacles(int r, int c);
	int getRow() const;
	int getCol() const;
	bool isPossible(const int& sr_, const int& sc_, const int& gr_, const int& gc_, int r, int c);
	void getRandomObs(int num_rnd_obs, const int& sr_, const int& sc_, const int& gr_, const int& gc_);


private:

	int rows_, cols_;
	//std::vector<std::vector<int>> occ_map_;
	GridMap<int> occ_map_;
	int r_start_, r_end_;
	int c_start_, c_end_;

	inline bool isBound(int r, int c) {
		return r >= 0 && r < rows_ && c >= 0 && c < cols_;
	}
	inline void meterToIndex(double rs, double re, double cs, double ce)
	{
		// r_start_ = occ_map_.WorldYToYi(rs);
		// r_end_ = occ_map_.WorldYToYi(re);
		// c_start_ = occ_map_.WorldXToXi(cs);
		// c_end_ = occ_map_.WorldXToXi(ce);

		r_start_ = std::max(0, occ_map_.WorldYToYi(rs));
		r_end_ = std::min(rows_, occ_map_.WorldYToYi(re));
		c_start_ = std::max(0, occ_map_.WorldXToXi(cs));
		c_end_ = std::min(cols_, occ_map_.WorldXToXi(ce));
	}
};

#endif