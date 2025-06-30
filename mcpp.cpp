#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <iomanip>

// 使用Eigen库处理矩阵运算
using namespace casadi;
using namespace std;
using namespace Eigen;

// 离散动力学函数
MX f_d(const MX& x, const MX& u) {
    // 模型参数
    double m = 1.0, g = 9.81, Ixx = 0.1, Iyy = 0.1, Izz = 0.1;
    double kT = 0.1, ktau = 0.01;
    
    // 计算推力和力矩
    MX T = kT * u(0);
    MX tau_phi = ktau * u(1);
    MX tau_theta = ktau * u(2);
    MX tau_psi = ktau * u(3);
    
    // 初始化状态导数
    MX dxdt = MX::zeros(12, 1);
    
    // 计算状态导数 (简化的四旋翼模型)
    dxdt(0) = x(3);       // dot_px = vx
    dxdt(1) = x(4);       // dot_py = vy
    dxdt(2) = x(5);       // dot_pz = vz
    dxdt(3) = T/m * (sin(x(8)) * sin(x(6)) + cos(x(8)) * sin(x(7)) * cos(x(6)));
    dxdt(4) = T/m * (-sin(x(8)) * cos(x(6)) + cos(x(8)) * sin(x(7)) * sin(x(6)));
    dxdt(5) = g - T/m * (cos(x(7)) * cos(x(6)));
    dxdt(6) = x(9);       // dot_phi = wx
    dxdt(7) = x(10);      // dot_theta = wy
    dxdt(8) = x(11);      // dot_psi = wz
    dxdt(9) = tau_phi / Ixx;  // dot_wx
    dxdt(10) = tau_theta / Iyy; // dot_wy
    dxdt(11) = tau_psi / Izz;  // dot_wz
    
    // 欧拉离散化
    double dt = 0.1;
    MX x_next = x + dt * dxdt;
    return x_next;
}

int main() {
    // 参数设置
    double dt = 0.1;      // 时间步长 (s)
    double t_f = 5.0;     // 目标时刻 (s)
    int N = static_cast<int>(t_f / dt);  // 时间步数
    
    // 初始状态、输入和目标状态
    VectorXd x0(12); x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    VectorXd u0(4); u0 << 0.5, 0, 0, 0;
    VectorXd x_desired(12); x_desired << 10, 5, -3, 0, 0, 0, 0.1, 0.2, 0.3, 0, 0, 0;
    
    // 创建优化问题
    Opti opti;
    
    // 决策变量: 控制序列和状态序列
    MX U = opti.variable(4, N);       // 4个输入, N步
    MX X = opti.variable(12, N+1);     // 12个状态, N+1步(包括x0)
    
    // 参数
    MX x_init = opti.parameter(12, 1);
    MX u_init = opti.parameter(4, 1);
    MX x_target = opti.parameter(12, 1);
    
    // 初始条件约束
    opti.subject_to(X.col(0) == x_init);
    
    // 设置参数初始值
    opti.set_value(x_init, x0);
    opti.set_value(x_target, x_desired);
    opti.set_value(u_init, u0);
    
    // 动力学约束
    for (int k = 0; k < N; ++k) {
        MX x_k = X.col(k);
        MX u_k = U.col(k);
        MX x_next = f_d(x_k, u_k);
        opti.subject_to(X.col(k+1) == x_next);
    }
    
    // 控制输入约束
    for (int k = 0; k < N; ++k) {
        opti.subject_to(opti.bounded(0.0, U(0, k), 1.0));  // 油门约束
        opti.subject_to(opti.bounded(-1.0, U(1, k), 1.0)); // 杆操作约束
        opti.subject_to(opti.bounded(-1.0, U(2, k), 1.0)); // 杆操作约束
        opti.subject_to(opti.bounded(-1.0, U(3, k), 1.0)); // 杆操作约束
    }
    
    // 目标函数权重矩阵
    MatrixXd Q_f = MatrixXd::Zero(12, 12);
    Q_f(0,0) = Q_f(1,1) = Q_f(2,2) = 10.0;
    Q_f(6,6) = Q_f(7,7) = Q_f(8,8) = 5.0;
    
    MatrixXd R = MatrixXd::Zero(4, 4);
    R(0,0) = R(1,1) = R(2,2) = R(3,3) = 0.1;
    
    // 构建目标函数
    MX terminal_error = X.col(N) - x_target;
    MX J = mtimes(mtimes(terminal_error.T(), Q_f), terminal_error);
    
    for (int k = 0; k < N; ++k) {
        MX du;
        if (k == 0) {
            du = U.col(k) - u_init;
        } else {
            du = U.col(k) - U.col(k-1);
        }
        J += mtimes(mtimes(du.T(), R), du);
    }
    
    // 设置目标函数并求解
    opti.minimize(J);
    opti.solver("ipopt"); // 使用IPOPT求解器
    
    try {
        opti.solve();
        
        // 获取最优控制序列
        MatrixXd u_opt = opti.value(U);
        cout << "Optimal control sequence:" << endl;
        for (int j = 0; j < N; ++j) {
            cout << "Step " << j << ": [";
            for (int i = 0; i < 4; ++i) {
                cout << setw(6) << setprecision(3) << u_opt(i, j);
                if (i < 3) cout << ", ";
            }
            cout << "]" << endl;
        }
    } catch (const CasadiException& e) {
        cout << "Error: " << e.what() << endl;
        cout << "Status: " << opti.stats()["return_status"] << endl;
    }
    
    return 0;
}