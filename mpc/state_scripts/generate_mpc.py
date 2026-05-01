# generate_mpc_code.py
from acados_template import AcadosOcp, AcadosOcpSolver
import casadi as ca
import numpy as np
from bicycle_model import export_bicycle_model

def generate_mpc():
    ocp = AcadosOcp()
    model = export_bicycle_model()
    ocp.model = model

    # 예측 호라이즌 설정 (예: 1초 앞을 0.05초 간격으로 20번 쪼개서 예측)
    N = 20
    Tf = 1.0
    ocp.dims.N = N

    # ===============================================
    # 파라미터 (Parameters): 실시간으로 변하는 외부 입력값
    # ===============================================
    # x_obs, y_obs (가장 가까운 장애물의 좌표)
    p = ca.SX.sym('p', 2) 
    ocp.model.p = p # 모델에 파라미터 등록

    # ===============================================
    # 목적 함수 (Cost Function) 세팅 (NONLINEAR_LS 방식)
    # ===============================================
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS' # 종점(Terminal) 코스트

    nx = model.x.size()[1]
    nu = model.u.size()[1]
    ny = nx + nu + 1 # x(5) + u(2) + 장애물항(1) = 8 차원
    ny_e = nx # 종점은 제어입력과 장애물항 생략 가능

    # 잔차(Residual) 공식 작성: 이것들의 제곱합이 최소화됨
    x_obs = p[0]
    y_obs = p[1]
    epsilon = 1e-6
    obs_penalty = 1.0 / ca.sqrt((model.x[0] - x_obs)**2 + (model.x[1] - y_obs)**2 + epsilon)

    # y = [x, y, theta, v, delta, a, delta_dot, obs_penalty]
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u, obs_penalty)
    ocp.model.cost_y_expr_e = model.x

    # 가중치 행렬 (W: Weight) - Q, R, W_obs가 합쳐진 대각 행렬
    W = np.diag([10.0, 10.0, 5.0, 1.0, 1.0,  # Q (상태 추종 가중치)
                 0.1, 0.5,                   # R (제어 부드러움 가중치)
                 100.0])                     # W_obs (장애물 회피 척력 가중치)
    ocp.cost.W = W
    ocp.cost.W_e = np.diag([10.0, 10.0, 5.0, 1.0, 1.0]) # 종점 가중치

    # 참조 궤적 초기화 (나중에 C++에서 덮어씀)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # ===============================================
    # 제약 조건 (Constraints): 로봇의 물리적 한계
    # ===============================================
    # idxbu : 제어 입력 index
    ocp.constraints.lbu = np.array([-2.5, -1.0]) # a 최소, delta_dot 최소
    ocp.constraints.ubu = np.array([ 2.5,  1.0]) # a 최대, delta_dot 최대
    ocp.constraints.idxbu = np.array([0, 1])     # 0 idx : a, 1 idx : delta_dot

    # 상태 제약 (예: 최대 조향각 제한 -30도 ~ 30도)
    # idxbx : 상태 변수 index
    ocp.constraints.lbx = np.array([-0.523]) 
    ocp.constraints.ubx = np.array([ 0.523])
    ocp.constraints.idxbx = np.array([4]) # delta(4번째 인덱스)

    # 초기 상태 세팅
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # 솔버 설정 및 코드 생성
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI' # 실시간 제어에 매우 빠름

    AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    print("성공적으로 C 코드가 생성되었습니다!")

if __name__ == '__main__':
    generate_mpc()