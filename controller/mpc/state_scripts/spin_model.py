from acados_template import AcadosModel
import casadi as ca

def export_spin_model() -> AcadosModel:
    model_name = 'spin_model'

    # 1. 상태 변수 (State Variables) - 차원을 5개로 맞춤
    # x, y, theta(헤딩), v(속도=0.0), omega(각속도)
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    theta = ca.SX.sym('theta')
    v = ca.SX.sym('v')              # dummy state
    omega = ca.SX.sym('omega')
    sym_x = ca.vertcat(x, y, theta, v, omega)

    # 2. 제어 입력 변수 (Control Inputs) - 차원을 2개로 맞춤
    # a(가속도=0.0), omega_dot(각가속도)
    a = ca.SX.sym('a')              # dummy control input
    omega_dot = ca.SX.sym('omega_dot')
    sym_u = ca.vertcat(a, omega_dot)

    # 3. 상태 변수의 변화율 (State Derivatives)
    x_dot = ca.SX.sym('x_dot')
    y_dot = ca.SX.sym('y_dot')
    theta_dot = ca.SX.sym('theta_dot')
    v_dot = ca.SX.sym('v_dot')
    omega_dot_state = ca.SX.sym('omega_dot_state')
    sym_xdot = ca.vertcat(x_dot, y_dot, theta_dot, v_dot, omega_dot_state) 

    # 4. 물리적 제원
    WB = 0.11 # 휠베이스 (RobotConfig의 WB 파라미터와 나중에 연동)

    # 5. 시스템 동역학 방정식 (ODE)
    # f_expl: 명시적 상태 변화율 방정식
    f_expl = ca.vertcat(
        0.0,                # x_dot=dx/dt
        0.0,                # y_dot=dy/dt
        omega,              # theta_dot=dtheta/dt=omega
        0.0,                # v_dot=dv/dt
        omega_dot           # omega_dot_state=d(omega)/dt
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
#     model = export_spin_model()
#     print(f'model = {model.f_expl_expr}')

# if __name__ == "__main__":
#     main()