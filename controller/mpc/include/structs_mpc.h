#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>
#include <iostream>
#include <fstream>
#include "structs.h"

struct ReferenceTraj {
    double x, y, theta, v, delta; // 상태 목표
    double a, delta_dot;          // 제어 입력 목표
    double obs;                   // 추가 변수 
    VehicleMode mode;             // hastar 현재 추종 궤적의 모드 전환을 위해
};

// 종점용 (5차원: 상태 5)
struct ReferenceTrajTerminal {
    double x, y, theta, v, delta; 
};

struct RealTimePerformance
{
    double sum_time_tot_{0.0};
    double sum_time_tot_sq_{0.0};

    double sum_time_qp_{0.0};
    double sum_time_qp_sq_{0.0};

    double sum_time_lin_{0.0};
    double sum_time_lin_sq_{0.0};

    int cnt_{0};

    void update(double tot, double qp, double lin) {
        tot *= 1000.0;
        qp *= 1000.0;
        lin *= 1000.0;

        sum_time_tot_ += tot;
        sum_time_qp_ += qp;
        sum_time_lin_ += lin;

        sum_time_tot_sq_ += tot * tot;
        sum_time_qp_sq_ += qp * qp;
        sum_time_lin_sq_ += lin * lin;

        cnt_++;
    }

    double getMean(double sum) const {
        return (cnt_ > 0) ? (sum / cnt_) : 0.0;
    }

    double getRMS(double sum_sq) const {
        return (cnt_ > 0) ? std::sqrt(sum_sq / cnt_) : 0.0;
    }
    
    double getStdDev(double sum, double sum_sq) const {
        if (cnt_ <= 1) return 0.0;
        double mean = sum / cnt_;
        double variance = (sum_sq / cnt_) - (mean * mean);
        return (variance > 0.0) ? std::sqrt(variance) : 0.0;
    }

    void printMetrics() {

        double mean_tot = getMean(sum_time_tot_);
        double mean_qp = getMean(sum_time_qp_);
        double mean_lin = getMean(sum_time_lin_);

        double rms_tot = getRMS(sum_time_tot_sq_);
        double rms_qp = getRMS(sum_time_qp_sq_);
        double rms_lin = getRMS(sum_time_lin_sq_);

        double std_tot = getStdDev(sum_time_tot_, sum_time_tot_sq_);
        double std_qp = getStdDev(sum_time_qp_, sum_time_qp_sq_);
        double std_lin = getStdDev(sum_time_lin_, sum_time_lin_sq_);

        std::cout << "============== REAL TIME METRICS ==============" << std::endl;
        printf(" [Total Time] Mean: %6.3f ms | RMS: %6.3f ms | StdDev(Jitter): %6.3f ms\n", 
            mean_tot, rms_tot, std_tot);
            
        printf(" [QP Solver]  Mean: %6.3f ms | RMS: %6.3f ms | StdDev(Jitter): %6.3f ms\n", 
            mean_qp, rms_qp, std_qp);
            
        printf(" [Linearize]  Mean: %6.3f ms | RMS: %6.3f ms | StdDev(Jitter): %6.3f ms\n", 
            mean_lin, rms_lin, std_lin);

    }
};


#endif