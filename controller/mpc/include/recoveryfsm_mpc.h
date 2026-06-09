#ifndef RECOVERYFSM_MPC_H
#define RECOVERYFSM_MPC_H

#include "structs.h"
#include "vehicles_mpc.h"


class recoveryFSM
{
private:
    int recovery_cnt_ = 0;
    int max_recovery_cnt_ = 33;
    double dt_;
    double a_dec_mag_;
    double omega_dot_max_;


public:
    recoveryFSM(int max_cnt, double dt, double a_dec_mag, double omega_dot_max) 
        : max_recovery_cnt_(max_cnt), dt_(dt), a_dec_mag_(a_dec_mag), omega_dot_max_(omega_dot_max)
    {}

    void reset() {
        if(recovery_cnt_ > 0) 
            recovery_cnt_ = 0;
    }

    void execute(double* current_x, double* current_u, int& closest_idx,
                VehicleMode curr_mode, MpcController& active_mode,
                const std::vector<ReferenceTraj>& resampled_traj)
    {
        std::cerr << "[Warning] MPC 최적화 연산 실패! Recovery FSM 가동" << std::endl;
        recovery_cnt_++;
        //////////////////////////
        // A. 부드러운 감속 로직
        //////////////////////////
        double current_v = current_x[3];
        if (std::abs(current_v) > 0.05) {
            // 1. 타겟 브레이크 가속도 설정
            double target_brake = (current_v > 0) ? -a_dec_mag_ : a_dec_mag_;
            
            // 2. 한 스텝당 변할 수 있는 최대 가속도 폭 제한 (Max Jerk: 5.0 m/s^3 기준)
            double max_delta_u0 = 5.0 * dt_; 

            // 3. 현재 입력값(current_u[0])을 타겟을 향해 부드럽게 이동 (Clamping)
            if (target_brake > current_u[0] + max_delta_u0) {
                current_u[0] += max_delta_u0;
            } else if (target_brake < current_u[0] - max_delta_u0) {
                current_u[0] -= max_delta_u0;
            } else {
                current_u[0] = target_brake; // 타겟 도달 시 고정
            }
        } else {
            // 완전 정지 상태
            current_u[0] = 0.0;     // a
            current_x[3] = 0.0;     // v
        }

        //////////////////////////
        // B. 조향각 유지 or 각속도 감속 로직
        //////////////////////////
        // 1. Bicycle or Parallel mode
        if (curr_mode != VehicleMode::SpinMode) {
            // 실패 직전의 조향각을 그대로 유지한 채 감속하도록 강제
            current_u[1] = 0.0; // delta_dot = 0.0 (핸들 회전 정지)
        }
        // 2. Spin mode
        else {
            // current_x[4] -> omega, current_u[1] -> delta omega
            double omega = current_x[4];
            if (std::abs(omega) > 0.05) {     // rad/s
                double target_omega_dot = (omega >= 0.0) ? -omega_dot_max_ : omega_dot_max_;  // 타겟 각가속도
                double max_delta_u1 = 5.0 * dt_;         // 각가속도 변화량(jerk)
                if (target_omega_dot > current_u[1] + max_delta_u1) {
                    current_u[1] += max_delta_u1;
                }
                else if(target_omega_dot < current_u[1] - max_delta_u1) {
                    current_u[1] -= max_delta_u1;
                }
                else {
                    current_u[1] = target_omega_dot; // 타겟 도달 시 고정
                }
            }
            else {
                current_u[1] = 0.0;
                current_x[4] = 0.0;
            }
        }

        // 2. Recovery 상태 머신 (is_action_applied 플래그로 상태 전환 제어)
        if (recovery_cnt_ == 1) {
            std::cout << "Recovery Step 1: 솔버 내부 메모리(Warm Start) 강제 초기화" << std::endl;
            // 이전 스텝의 꼬여버린 예측 해를 현재 위치 기준으로 깨끗하게 덮어씌움
            active_mode.setInitialGuess(current_x, current_u);
        } 
        else if(recovery_cnt_ > max_recovery_cnt_) {
            std::cout << "Recovery Step 2: 초기화로도 실패. 타겟 인덱스 강제 스킵 (Deadlock 탈출)" << std::endl;
            // 다음 궤적이 다른 모드(Spin)라면 스킵 절대 금지!
            int next_idx = std::min(closest_idx + 1, (int)resampled_traj.size() - 1);
            if (resampled_traj[next_idx].mode == curr_mode) {
                closest_idx = next_idx; // 같은 모드일 때만 인덱스를 건너뜀
            } else {
                std::cout << " -> 다음 궤적이 모드 전환점이므로 인덱스 고정, 정지 대기" << std::endl;
            }
            // // 장애물이나 극단적 곡률로 인해 특정 인덱스에서 막혔다면 억지로 한 칸 넘김
            reset();
        }
        
    }
};




#endif