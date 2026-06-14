# generate_mpc_code.py
from acados_template import AcadosOcp, AcadosOcpSolver
import casadi as ca
import numpy as np
from bicycle_model import export_bicycle_model
from parallel_model import export_parallel_model
from spin_model import export_spin_model

def generate_mpc(model):
    ocp = AcadosOcp()
    ocp.model = model

    # 모델 이름별로 폴더를 따로 만들도록 
    ocp.code_export_directory = f'c_generated_{model.name}'

    # 예측 호라이즌 설정 (예: 1초 앞을 0.05초 간격으로 20번 쪼개서 예측)
    # N = 150 #50
    # Tf = 5.0 #5.0
    N = 60 #50
    Tf = 2.0 #5.0
    # ocp.dims.N = N
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    # ===============================================
    # 파라미터 (Parameters): 실시간으로 변하는 외부 입력값
    # ===============================================
    # x_obs, y_obs (가장 가까운 장애물의 좌표)
    num_obs = 3                 # real time num obs
    p = ca.SX.sym('p', 2 * num_obs) 
    ocp.model.p = p             # 모델에 파라미터 등록
    # [x1, y1, x2, y2, x3, y3] 유령 장애물
    ocp.parameter_values = np.array([-10000.0, -10000.0] * num_obs) 

    # ===============================================
    # 목적 함수 (Cost Function) 세팅 (NONLINEAR_LS 방식)
    # ===============================================
    ocp.cost.cost_type_0 = 'NONLINEAR_LS'
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS' # 종점(Terminal) 코스트

    nx = model.x.shape[0]
    nu = model.u.shape[0]
    ny = nx + nu + 1            # x(5) + u(2) + 장애물항(1) = 8 차원
    ny_e = nx                   # 종점은 제어입력과 장애물항 생략 가능

    # 민코프스키 하이퍼 타원 파라미터 세팅
    # 차량의 절반 길이/폭 + 안전 마진
    a_radius = 0.8  # 차량 전후 방향 반경
    b_radius = 0.5  # 차량 좌우 방향 반경
    epsilon = 1e-4
    
    obs_penalty = 0.0
    for i in range(num_obs):
        x_obs = p[2*i]
        y_obs = p[2*i + 1]

        # dx, dy
        dx = model.x[0] - x_obs
        dy = model.x[1] - y_obs
        theta = model.x[2]

        # global frame -> robot frame
        dx_rot = dx * ca.cos(theta) + dy * ca.sin(theta)
        dy_rot = -dx * ca.sin(theta) + dy * ca.cos(theta)

        # 4차 하이퍼 타원 (Minkowski Ellipse) 방정식
        # E <= 1 이면 로봇 영역 내부 침범을 의미
        E = (dx_rot / a_radius)**4 + (dy_rot / b_radius)**4
        
        # 타원에 가까워질수록 페널티가 기하급수적으로 증가
        obs_penalty += 1.0 / (E + epsilon)

    # 잔차(Residual) 공식 작성: 이것들의 제곱합이 최소화됨
    # x_obs = p[0]
    # y_obs = p[1]
    # epsilon = 1e-6
    # obs_penalty = 1.0 / ca.sqrt((model.x[0] - x_obs)**2 + (model.x[1] - y_obs)**2 + epsilon)

    # y = [x, y, theta, v, delta, a, delta_dot, obs_penalty]
    ocp.model.cost_y_expr_0 = ca.vertcat(model.x, model.u, obs_penalty)
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u, obs_penalty)
    ocp.model.cost_y_expr_e = model.x

    # 가중치 행렬 (W: Weight) - Q, R, W_obs가 합쳐진 대각 행렬
    W = np.diag([10.0, 10.0, 5.0, 1.0, 1.0,  # Q (상태 추종 가중치)
                 0.1, 0.05,                   # R (제어 부드러움 가중치)
                #  1.0, 5.0,                   # R (제어 부드러움 가중치)
                 100.0])                     # W_obs (장애물 회피 척력 가중치)
    ocp.cost.W_0 = W
    ocp.cost.W = W
    ocp.cost.W_e = np.diag([10.0, 10.0, 5.0, 1.0, 1.0]) # 종점 가중치

    # 참조 궤적 초기화 (나중에 C++에서 덮어씀)
    ocp.cost.yref_0 = np.zeros(ny)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # 제약 조건 (Constraints): 로봇의 물리적 한계
    # idxbu : 제어 입력 index
    # reference : nav2_params.yaml -> controller_server
    a_limit = 4.0           # 실제 a max(in cpp code)보다 크게 설정
    delta_dot_limit = 2.0   # 실제 delta_dot max(in cpp code)보다 크게 설정
    ocp.constraints.lbu = np.array([-a_limit, -delta_dot_limit]) # a, delta_dot(or omega_dot) 최소
    ocp.constraints.ubu = np.array([ a_limit,  delta_dot_limit]) # a, delta_dot(or omega_dot) 최대
    ocp.constraints.idxbu = np.array([0, 1])     # 0th idx : a, 1st idx : delta_dot (omega_dot)   

    v_limit = 0.6   # 실제 v(c++) max보다 크게 설정

    # idxbx : 상태 변수 index
    # reference : nav2_params.yaml -> ExtendedHybridAStar
    if (model.name == 'bicycle_model'):
        # 상태 제약: v(인덱스 3), delta(인덱스 4)
        # ocp.constraints.lbx = np.array([-v_limit, -1.5]) 
        # ocp.constraints.ubx = np.array([ v_limit,  1.5])
        ocp.constraints.lbx = np.array([-v_limit, -0.7])    # 40 deg, (c++) max보다 크게 설정
        ocp.constraints.ubx = np.array([ v_limit,  0.7])
        ocp.constraints.idxbx = np.array([3, 4]) 
        
    elif (model.name == 'parallel_model'):
        # 상태 제약: v(인덱스 3), delta(인덱스 4, parallel에서는 alpha)
        ocp.constraints.lbx = np.array([-v_limit, -1.57]) 
        ocp.constraints.ubx = np.array([ v_limit,  1.57])
        ocp.constraints.idxbx = np.array([3, 4]) 
        
    elif (model.name == 'spin_model'):
        # 상태 제약: v(인덱스 3), omega(인덱스 4, spin에서는 각속도)
        # 🌟 제자리 회전 모드에서는 물리적으로 선속도 v가 무조건 0이어야 함!
        ocp.constraints.lbx = np.array([ 0.0, -0.35]) 
        ocp.constraints.ubx = np.array([ 0.0,  0.50]) 
        # ocp.constraints.lbx = np.array([ 0.0, -1.35]) 
        # ocp.constraints.ubx = np.array([ 0.0,  1.50]) 
        ocp.constraints.idxbx = np.array([3, 4])


    # 초기 상태 세팅
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # 솔버 설정 및 코드 생성
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.levenberg_marquardt = 1e-4
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    # HPIPM 내부적으로도 대각성분에 아주 작은 값을 더하는 옵션이 있습니다.
    ocp.solver_options.hpipm_mode = 'ROBUST'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'    # 실시간 제어에 매우 빠름, RTI option(1회 연산)
    # ocp.solver_options.nlp_solver_type = 'SQP'          # Full SQP(반복 연산)
    # ocp.solver_options.nlp_solver_max_iter = 100         # Full SQP일때 솔버가 최대 n번까지 반복해서 정답을 찾도록 허용
    
    AcadosOcpSolver(ocp, json_file=f'acados_ocp_{model.name}.json')
    print("성공적으로 C 코드가 생성되었습니다!")

if __name__ == '__main__':

    ####### 만약 .so 등이 덜 생성되었으면 안되면 해당 폴더 터미널에서 make shared_lib 수동 입력 ######
    
    # models = [export_bicycle_model(), export_parallel_model(), export_spin_model()]
    # for m in models:
    #     generate_mpc(m)
    generate_mpc(export_bicycle_model())
    generate_mpc(export_parallel_model())
    generate_mpc(export_spin_model())
