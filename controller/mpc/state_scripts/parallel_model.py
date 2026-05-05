from acados_template import AcadosModel
import casadi as ca

def export_parallel_model() -> AcadosModel:
    model_name = 'parallel_model'

    # 1. 상태 변수 (State Variables)
    # x, y, theta(헤딩), v(속도), delta(현재 조향각)
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    theta = ca.SX.sym('theta')
    v = ca.SX.sym('v')
    delta = ca.SX.sym('delta')
    sym_x = ca.vertcat(x, y, theta, v, delta)

    # 2. 제어 입력 변수 (Control Inputs)
    # a(가속도), delta_dot(조향각속도 - 핸들을 꺾는 속도)
    a = ca.SX.sym('a')
    delta_dot = ca.SX.sym('delta_dot')
    sym_u = ca.vertcat(a, delta_dot)

    # 3. 상태 변수의 변화율 (State Derivatives)
    x_dot = ca.SX.sym('x_dot')
    y_dot = ca.SX.sym('y_dot')
    theta_dot = ca.SX.sym('theta_dot')
    v_dot = ca.SX.sym('v_dot')
    delta_dot_state = ca.SX.sym('delta_dot_state')
    sym_xdot = ca.vertcat(x_dot, y_dot, theta_dot, v_dot, delta_dot_state)

    # 4. 물리적 제원
    WB = 0.11 # 휠베이스 (RobotConfig의 WB 파라미터와 나중에 연동)

    # 5. 시스템 동역학 방정식 (ODE)
    # f_expl: 명시적 상태 변화율 방정식
    f_expl = ca.vertcat(
        v * ca.cos(theta + delta),       # x_dot=dx/dt
        v * ca.sin(theta + delta),       # y_dot=dy/dt
        0.0,                             # theta_dot=dtheta/dt
        a,                               # v_dot=dv/dt
        delta_dot                        # delta_dot_state=d(delta)/dt
    )
    
    # f_impl: 암시적 형태 (f(x_dot, x, u) = 0)
    f_impl = sym_xdot - f_expl

    # 6. AcadosModel 객체 구성
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = sym_xdot
    model.u = sym_u
    model.name = model_name

    return model

# def main():
#     model = export_parallel_model()
#     print(f'model = {model.f_expl_expr}')

# if __name__ == "__main__":
#     main()