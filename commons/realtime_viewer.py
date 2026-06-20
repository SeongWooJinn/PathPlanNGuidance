import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle 
import struct
import time
import os

# ==========================================================
# 1. 전역 맵(GridMap) 바이너리 로더
# ==========================================================
def load_map_info(filename):
    print(f"[{filename}] 맵 파일 로드를 대기 중...")
    while not os.path.exists(filename):
        time.sleep(0.5)
        
    with open(filename, "rb") as f:
        # C++ saveMapInfoToBin 순서에 맞춰 정확히 바이트 추출
        cell_size  = struct.unpack('i', f.read(4))[0]
        r_length   = struct.unpack('d', f.read(8))[0]
        r_width    = struct.unpack('d', f.read(8))[0]
        resolution = struct.unpack('d', f.read(8))[0]
        sx         = struct.unpack('d', f.read(8))[0]
        sy         = struct.unpack('d', f.read(8))[0]
        gx         = struct.unpack('d', f.read(8))[0]
        gy         = struct.unpack('d', f.read(8))[0]
        rows       = struct.unpack('i', f.read(4))[0]
        cols       = struct.unpack('i', f.read(4))[0]
        # px_scale   = struct.unpack('d', f.read(8))[0]
        origin_x   = struct.unpack('d', f.read(8))[0]
        origin_y   = struct.unpack('d', f.read(8))[0]
        
        # GridMap 데이터 (int형 1D 배열) 읽기 및 2D 형태(rows x cols)로 복원
        map_data = np.frombuffer(f.read(rows * cols * 4), dtype=np.int32)
        grid = map_data.reshape((rows, cols))
        
        print(f"맵 로드 완료: {cols}x{rows}, 해상도: {resolution}m, 원점: ({origin_x}, {origin_y})")
        return grid, resolution, origin_x, origin_y, rows, cols
    
# ==========================================================
# 2.5 전역 경로(Global Path) 바이너리 로더 및 시각화
# ==========================================================
def load_global_path(filename):
    print(f"[{filename}] 전역 경로 파일 로드 대기 중...")
    while not os.path.exists(filename):
        time.sleep(0.5)

    path_x = []
    path_y = []

    # 🚨 [매우 중요] C++에서 sizeof(State)를 출력해보고 그 값을 여기에 적어주세요!
    # 예: double 4개(32) + int 1개(4) + 패딩(4) = 40바이트로 가정
    STATE_BYTE_SIZE = 40 

    with open(filename, "rb") as f:
        data = f.read()
        num_elements = len(data) // STATE_BYTE_SIZE
        
        for i in range(num_elements):
            offset = i * STATE_BYTE_SIZE
            
            # 구조체의 맨 앞이 double x, double y 라고 가정하고 첫 16바이트만 읽음
            x, y = struct.unpack_from('2d', data, offset)
            
            # 유효한 좌표만 저장 (쓰레기값 필터링)
            if x > -5000 and y > -5000:
                path_x.append(x)
                path_y.append(y)

    print(f"전역 경로 로드 완료: {num_elements}개의 웨이포인트")
    return path_x, path_y

# ==========================================================
# 2. 시각화 창 설정 및 맵 렌더링
# ==========================================================
map_file = "/tmp/mapinfo"
grid, resolution, origin_x, origin_y, rows, cols = load_map_info(map_file)

fig, ax = plt.subplots(figsize=(10, 10))
ax.set_aspect('equal')
ax.set_title("MPC Real-Time Tracking & Obstacle Avoidance")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")

# 🌟 핵심: 픽셀 배열을 실제 물리적(Meter) 범위로 매핑
extent = [
    origin_x, origin_x + cols * resolution,  # X축 범위 (min, max)
    origin_y, origin_y + rows * resolution   # Y축 범위 (min, max)
]

# origin='lower'를 통해 배열의 [0,0] 인덱스를 좌측 하단으로 맞춤 (ROS 좌표계 호환)
ax.imshow(grid, cmap='Greys', origin='lower', extent=extent, alpha=0.5)

# 축 범위를 맵 크기에 맞게 고정
# ax.set_xlim(extent[0], extent[1])
# ax.set_ylim(extent[2], extent[3])
ax.set_xlim(extent[0], extent[1])
ax.set_ylim(extent[3], extent[2])

global_path_file = "/tmp/hybrid_astar_path" 
gx, gy = load_global_path(global_path_file)

# 🌟 전역 경로를 초록색 실선으로 맵 위에 고정 렌더링 (Z-order를 낮춰 배경처럼 깔림)
ax.plot(gx, gy, color='#00FF00', linewidth=1.5, alpha=0.5, zorder=2, label='Global Path (Ref)')

# ==========================================================
# 3. 동적 플롯 객체 초기화 (로봇, 장애물, 목표경로 등)
# ==========================================================# 
# 궤적 누적을 위한 리스트 선언
history_x = []
history_y = []
# 궤적 라인 (파란색 점선, 투명도 0.5) 추가
trajectory_line, = ax.plot([], [], 'b--', linewidth=1.5, alpha=0.5, label='Trajectory')

robot_scat, = ax.plot([], [], 'bo', markersize=10, label='Robot')

# 방향 벡터를 나타내는 화살표(quiver)로 변경
# scale=1, scale_units='xy'로 설정하면 화살표 길이가 데이터의 물리적 단위(Meter)와 일치합니다.
heading_arrow = ax.quiver(0, 0, 0, 0, color='red', scale=1.0, scale_units='xy', angles='xy', width=0.008, label='Heading')

# obs_scat = ax.scatter([], [], c='orange', s=200, alpha=0.8, edgecolors='red', label='Dyn Obstacles')
# 🌟 수정: scatter 삭제하고, 물리 단위(m)로 그려지는 Circle 리스트 생성
MAX_OBS = 10 # 넉넉하게 10개 준비
obs_circles = []
for _ in range(MAX_OBS):
    # 초기 위치 (0,0), 반경 0으로 안 보이게 세팅
    circle = Circle((0, 0), 0.0, facecolor='orange', edgecolor='red', alpha=0.6)
    ax.add_patch(circle)
    obs_circles.append(circle)

ax.legend(loc='upper right')

# ==========================================================
# 4. 실시간 시뮬레이션 스트림 수신 대기
# ==========================================================
stream_file = "/tmp/sim_dynamic"
print(f"[{stream_file}] 제어 스트림 연결 대기 중...")
while not os.path.exists(stream_file):
    time.sleep(0.5)

f_stream = open(stream_file, "rb")

# ==========================================================
# 5. 애니메이션 루프
# ==========================================================
def update(frame):
    try:
        # 매 프레임마다 새롭게 만들어진 파일을 열어서 한 번에 모두 읽음
        with open(stream_file, "rb") as f:
            data = f.read()
    except FileNotFoundError:
        # C++가 rename 하는 아주 짧은 찰나에 파일을 읽으려 하면 무시하고 다음 프레임 대기
        return robot_scat, heading_arrow, obs_scat

    # 데이터 최소 크기 검증: 로봇 상태(24) + 장애물 개수(4) = 28바이트
    if len(data) < 28:
        return robot_scat, heading_arrow, obs_scat

    # 1) 로봇 상태 읽기 (0~24 바이트)
    rx, ry, rtheta = struct.unpack_from('d d d', data, 0)
    if len(history_x) > 0 and np.hypot(rx - history_x[-1], ry - history_y[-1]) > 10.0:
        # 이전 좌표와 현재 좌표의 거리가 10m 이상 차이나면 시뮬레이션 재시작으로 간주하고 궤적 초기화
        history_x.clear()
        history_y.clear()
    # 로봇의 현재 좌표를 궤적 리스트에 누적
    history_x.append(rx)
    history_y.append(ry)

    # 2) 장애물 개수 읽기 (24~28 바이트)
    num_obs = struct.unpack_from('i', data, 24)[0]
    
    # 3) 동적 장애물 데이터 읽기
    ox, oy, oradius = [], [], []
    offset = 28
    
    # 안전 장치: 실제 들어온 데이터가 예상 크기보다 크거나 같을 때만 파싱
    if num_obs > 0 and len(data) >= 28 + num_obs * 24:
        for i in range(num_obs):
            obs_x, obs_y, obs_r = struct.unpack_from('d d d', data, offset)
            if obs_x > -5000: # 유령 장애물 무시
                ox.append(obs_x)
                oy.append(obs_y)
                oradius.append(obs_r)
            offset += 24
            
    # 4) 화면 갱신
    trajectory_line.set_data(history_x, history_y)
    robot_scat.set_data([rx], [ry])
    heading_arrow.set_offsets(np.c_[rx, ry])
    heading_arrow.set_UVC(np.cos(rtheta) * 2.0, np.sin(rtheta) * 2.0)
    # heading_arrow.set_data([rx, rx + np.cos(rtheta)*2], [ry, ry + np.sin(rtheta)*2])

    # 🌟 수정: 실제 반경(obs_r)을 사용하여 Circle 업데이트
    for i, circle in enumerate(obs_circles):
        if i < len(ox):
            # 장애물이 존재하면 중심점과 반경을 업데이트하고 표시
            circle.center = (ox[i], oy[i])
            circle.radius = oradius[i]
            circle.set_visible(True)
        else:
            # 남는 Circle은 숨김
            circle.set_visible(False)

    return trajectory_line, robot_scat, heading_arrow, *obs_circles

# 애니메이션 실행 (약 30fps)
ani = animation.FuncAnimation(fig, update, interval=33, blit=True, cache_frame_data=False)
plt.show()

# 창을 닫으면 파일 정리
f_stream.close()