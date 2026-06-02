import numpy as np
import matplotlib.pyplot as plt
import pandas as pd  # pandas 라이브러리 추가

def main():
    # 1. C++ 구조체와 완벽히 동일한 데이터 타입(8바이트 double 7개) 정의
    dt = np.dtype([
        ('x', 'f8'), ('y', 'f8'), ('theta', 'f8'), 
        ('v', 'f8'), ('delta', 'f8'), 
        ('a', 'f8'), ('delta_dot', 'f8')
    ])

    # 2. 바이너리 파일 로드
    try:
        data = np.fromfile("/tmp/mpc_log", dtype=dt)
    except FileNotFoundError:
        print("Error: /tmp/mpc_log 파일을 찾을 수 없습니다.")
        return

    # ============================================================
    # [추가된 부분] 데이터를 CSV 또는 TXT 파일로 저장
    # ============================================================
    # 데이터를 Pandas DataFrame으로 변환
    df = pd.DataFrame(data)
    
    # 1) CSV 형식으로 저장 (콤마(,)로 구분)
    csv_filename = "mpc_log_data.csv"
    df.to_csv(csv_filename, index=False)
    print(f"데이터가 {csv_filename} 파일로 저장되었습니다.")

    # 2) TXT 형식으로 저장 (탭(\t)으로 구분)
    txt_filename = "mpc_log_data.txt"
    df.to_csv(txt_filename, index=False, sep='\t')
    print(f"데이터가 {txt_filename} 파일로 저장되었습니다.")
    # ============================================================

    # 시간 축 생성 (가정: dt = 0.033초, 필요시 수정)
    time_step = 5.0 / 150.0 
    t = np.arange(len(data)) * time_step

    # 3. 그래프 그리기 세팅
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle('MPC Tracking Results', fontsize=16, fontweight='bold')

    # (1) X-Y 궤적 (2D 평면)
    ax1 = plt.subplot(2, 3, (1, 4))
    rows = 30
    cols = 60
    # Numpy 배열은 (행, 열) 즉 (Y높이, X너비) 순서
    # extent=[좌, 우, 하, 상]
    im1 = np.zeros((rows, cols))
    ax1.imshow(im1, cmap='Greys', extent=[0, cols, rows, 0], alpha=0.1) # 맵을 연한 회색으로 표시
    ax1.plot(data['x'], data['y'], 'b-', linewidth=2, label='Robot Trajectory')
    
    ax1.scatter(data['x'][0], data['y'][0], c='g', s=100, label='Start')
    ax1.scatter(data['x'][-1], data['y'][-1], c='r', s=100, label='End')
    
    ax1.set_xlim(0, cols)    # X축은 0부터 cols까지
    ax1.set_ylim(rows, 0)    # Y축은 rows에서 0으로 내려가도록 설정 

    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('X-Y Path')
    ax1.grid(True)
    ax1.legend()
    # ax1.axis('equal') # 비율 맞추기
    ax1.set_aspect('equal') # 비율 맞추기

    # (2) 속도 (v)
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(t, data['v'], 'k-', linewidth=2)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity [m/s]')
    ax2.set_title('Velocity (v)')
    ax2.grid(True)

    # (3) 가속도 (a) - 제어 입력
    ax3 = plt.subplot(2, 3, 5)
    ax3.plot(t, data['a'], 'r-', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Acceleration [m/s^2]')
    ax3.set_title('Control Input: Accel (a)')
    ax3.grid(True)

    # (4) 조향각 (delta)
    ax4 = plt.subplot(2, 3, 3)
    ax4.plot(t, np.rad2deg(data['delta']), 'g-', linewidth=2) # 보기 쉽게 degree 변환
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Steering Angle [deg]')
    ax4.set_title('Steering Angle (delta)')
    ax4.grid(True)

    # (5) 조향 각속도 (delta_dot) - 제어 입력
    ax5 = plt.subplot(2, 3, 6)
    ax5.plot(t, np.rad2deg(data['delta_dot']), 'm-', linewidth=2)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Steering Rate [deg/s]')
    ax5.set_title('Control Input: Steering Rate (delta_dot)')
    ax5.grid(True)

    plt.tight_layout()
    plt.subplots_adjust(top=0.9) # 타이틀 안 겹치게
    plt.show()

if __name__ == '__main__':
    main()
# import numpy as np
# import matplotlib.pyplot as plt

# def main():
#     # 1. C++ 구조체와 완벽히 동일한 데이터 타입(8바이트 double 7개) 정의
#     dt = np.dtype([
#         ('x', 'f8'), ('y', 'f8'), ('theta', 'f8'), 
#         ('v', 'f8'), ('delta', 'f8'), 
#         ('a', 'f8'), ('delta_dot', 'f8')
#     ])

#     # 2. 바이너리 파일 로드
#     try:
#         data = np.fromfile("/tmp/mpc_log", dtype=dt)
#     except FileNotFoundError:
#         print("Error: /tmp/mpc_log 파일을 찾을 수 없습니다.")
#         return

#     # 시간 축 생성 (가정: dt = 0.033초, 필요시 수정)
#     time_step = 5.0 / 150.0 
#     t = np.arange(len(data)) * time_step

#     # 3. 그래프 그리기 세팅
#     fig = plt.figure(figsize=(15, 10))
#     fig.suptitle('MPC Tracking Results', fontsize=16, fontweight='bold')

#     # (1) X-Y 궤적 (2D 평면)
#     ax1 = plt.subplot(2, 3, (1, 4))
#     rows = 30
#     cols = 60
#     # Numpy 배열은 (행, 열) 즉 (Y높이, X너비) 순서
#     # extent=[좌, 우, 하, 상]
#     im1 = np.zeros((rows, cols))
#     ax1.imshow(im1, cmap='Greys', extent=[0, cols, rows, 0], alpha=0.1) # 맵을 연한 회색으로 표시
#     ax1.plot(data['x'], data['y'], 'b-', linewidth=2, label='Robot Trajectory')
    
#     ax1.scatter(data['x'][0], data['y'][0], c='g', s=100, label='Start')
#     ax1.scatter(data['x'][-1], data['y'][-1], c='r', s=100, label='End')
    
#     ax1.set_xlim(0, cols)    # X축은 0부터 cols까지
#     ax1.set_ylim(rows, 0)    # Y축은 rows에서 0으로 내려가도록 설정 

#     ax1.set_xlabel('X [m]')
#     ax1.set_ylabel('Y [m]')
#     ax1.set_title('X-Y Path')
#     ax1.grid(True)
#     ax1.legend()
#     # ax1.axis('equal') # 비율 맞추기
#     ax1.set_aspect('equal') # 비율 맞추기

#     # (2) 속도 (v)
#     ax2 = plt.subplot(2, 3, 2)
#     ax2.plot(t, data['v'], 'k-', linewidth=2)
#     ax2.set_xlabel('Time [s]')
#     ax2.set_ylabel('Velocity [m/s]')
#     ax2.set_title('Velocity (v)')
#     ax2.grid(True)
#     # for t, v in zip(t, data['v']):
#     #     print(f'time: {t:.4f} / v: {v:.4f}')

#     # (3) 가속도 (a) - 제어 입력
#     ax3 = plt.subplot(2, 3, 5)
#     ax3.plot(t, data['a'], 'r-', linewidth=2)
#     ax3.set_xlabel('Time [s]')
#     ax3.set_ylabel('Acceleration [m/s^2]')
#     ax3.set_title('Control Input: Accel (a)')
#     ax3.grid(True)

#     # (4) 조향각 (delta)
#     ax4 = plt.subplot(2, 3, 3)
#     ax4.plot(t, np.rad2deg(data['delta']), 'g-', linewidth=2) # 보기 쉽게 degree 변환
#     ax4.set_xlabel('Time [s]')
#     ax4.set_ylabel('Steering Angle [deg]')
#     ax4.set_title('Steering Angle (delta)')
#     ax4.grid(True)

#     # (5) 조향 각속도 (delta_dot) - 제어 입력
#     ax5 = plt.subplot(2, 3, 6)
#     ax5.plot(t, np.rad2deg(data['delta_dot']), 'm-', linewidth=2)
#     ax5.set_xlabel('Time [s]')
#     ax5.set_ylabel('Steering Rate [deg/s]')
#     ax5.set_title('Control Input: Steering Rate (delta_dot)')
#     ax5.grid(True)

#     plt.tight_layout()
#     plt.subplots_adjust(top=0.9) # 타이틀 안 겹치게
#     plt.show()

# if __name__ == '__main__':
#     main()