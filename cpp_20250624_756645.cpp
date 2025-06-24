#include <casadi/casadi.hpp>
#include <JSBSim/FGFDMExec.h>
#include <JSBSim/initialization/FGInitialCondition.h>
#include <JSBSim/models/FGAircraft.h>
#include <vector>
#include <iostream>

using namespace casadi;
using namespace JSBSim;

// JSBSim 接口包装器
class JSBSimInterface {
public:
    JSBSimInterface(const std::string& aircraft_name, const std::string& engine_name) {
        fdmex = new FGFDMExec();
        fdmex->SetDebugLevel(0);
        
        if (!fdmex->LoadModel("aircraft", aircraft_name, "engine", engine_name, "systems")) {
            std::cerr << "Failed to load JSBSim model" << std::endl;
            exit(1);
        }
        
        ic = fdmex->GetIC();
        aircraft = fdmex->GetAircraft();
        
        // 初始化状态维度
        state_dim = 12; // 位置(3), 姿态(4), 速度(3), 角速度(3)
        control_dim = 4; // 油门, 副翼, 升降舵, 方向舵
    }
    
    ~JSBSimInterface() {
        delete fdmex;
    }
    
    // 获取状态导数
    std::vector<double> get_state_derivative(const std::vector<double>& state, 
                                           const std::vector<double>& controls) {
        // 设置当前状态
        set_jsbsim_state(state);
        
        // 设置控制输入
        set_jsbsim_controls(controls);
        
        // 运行一个时间步
        fdmex->RunIC();
        bool result = fdmex->Run();
        if (!result) {
            std::cerr << "JSBSim run failed" << std::endl;
            exit(1);
        }
        
        // 获取状态导数
        return get_current_state_derivatives();
    }
    
    // 获取状态维度
    int get_state_dim() const { return state_dim; }
    
    // 获取控制维度
    int get_control_dim() const { return control_dim; }
    
private:
    FGFDMExec* fdmex;
    FGInitialCondition* ic;
    FGAircraft* aircraft;
    int state_dim;
    int control_dim;
    
    // 设置 JSBSim 状态
    void set_jsbsim_state(const std::vector<double>& state) {
        // 位置 (NED坐标系)
        ic->SetPositionNED(state[0], state[1], -state[2]); // JSBSim使用高度为正
        
        // 姿态 (四元数)
        ic->SetQuaternion(state[3], state[4], state[5], state[6]);
        
        // 速度 (机体坐标系)
        ic->SetUBody(state[7]);
        ic->SetVBody(state[8]);
        ic->SetWBody(state[9]);
        
        // 角速度 (机体坐标系)
        ic->SetPQR(state[10], state[11], state[12]);
    }
    
    // 设置 JSBSim 控制输入
    void set_jsbsim_controls(const std::vector<double>& controls) {
        // 油门 [0,1]
        fdmex->GetPropertyManager()->SetDoubleValue("fcs/throttle-cmd-norm", controls[0]);
        
        // 副翼 [-1,1]
        fdmex->GetPropertyManager()->SetDoubleValue("fcs/aileron-cmd-norm", controls[1]);
        
        // 升降舵 [-1,1]
        fdmex->GetPropertyManager()->SetDoubleValue("fcs/elevator-cmd-norm", controls[2]);
        
        // 方向舵 [-1,1]
        fdmex->GetPropertyManager()->SetDoubleValue("fcs/rudder-cmd-norm", controls[3]);
    }
    
    // 获取当前状态导数
    std::vector<double> get_current_state_derivatives() {
        std::vector<double> derivatives(state_dim);
        
        // 位置导数 (转换为NED坐标系)
        derivatives[0] = aircraft->GetVel(1);  // North速度
        derivatives[1] = aircraft->GetVel(2);  // East速度
        derivatives[2] = -aircraft->GetVel(3); // Down速度 (转换为高度变化率)
        
        // 四元数导数
        FGQuaternion quat = aircraft->GetQuaternion();
        FGColumnVector3 pqr = aircraft->GetPQR();
        FGQuaternion quat_der = quat.GetQDot(pqr);
        
        derivatives[3] = quat_der(1);
        derivatives[4] = quat_der(2);
        derivatives[5] = quat_der(3);
        derivatives[6] = quat_der(4);
        
        // 速度导数 (机体坐标系)
        derivatives[7] = aircraft->GetUVWdot(1);
        derivatives[8] = aircraft->GetUVWdot(2);
        derivatives[9] = aircraft->GetUVWdot(3);
        
        // 角速度导数 (机体坐标系)
        derivatives[10] = aircraft->GetPQRdot(1);
        derivatives[11] = aircraft->GetPQRdot(2);
        derivatives[12] = aircraft->GetPQRdot(3);
        
        return derivatives;
    }
};

// 创建轨迹优化求解器
Function create_trajectory_optimizer(JSBSimInterface& jsbsim, int N, double dt) {
    // 状态和控制维度
    int nx = jsbsim.get_state_dim();
    int nu = jsbsim.get_control_dim();
    
    // 优化变量
    MX X = MX::sym("X", nx, N+1);  // 状态轨迹
    MX U = MX::sym("U", nu, N);    // 控制轨迹
    
    // 参数：初始状态和目标状态
    MX x0 = MX::sym("x0", nx);
    MX xf = MX::sym("xf", nx);
    
    // 构建约束和成本函数
    MX cost = 0;
    std::vector<MX> g;
    
    // 初始状态约束
    g.push_back(X(Slice(), 0) - x0);
    
    // 动力学约束
    for (int k = 0; k < N; ++k) {
        // 当前状态和控制
        MX x_k = X(Slice(), k);
        MX u_k = U(Slice(), k);
        
        // 使用 RK4 积分
        // k1 = f(x_k, u_k)
        MX k1 = MX::zeros(nx);
        for (int i = 0; i < nx; ++i) {
            k1(i) = external("jsbsim_deriv", "get_derivative", 
                           {x_k, u_k}, {"state", "controls"}, {"derivative"})[i];
        }
        
        // k2 = f(x_k + dt/2*k1, u_k)
        MX k2 = MX::zeros(nx);
        for (int i = 0; i < nx; ++i) {
            k2(i) = external("jsbsim_deriv", "get_derivative", 
                            {x_k + dt/2*k1, u_k}, {"state", "controls"}, {"derivative"})[i];
        }
        
        // k3 = f(x_k + dt/2*k2, u_k)
        MX k3 = MX::zeros(nx);
        for (int i = 0; i < nx; ++i) {
            k3(i) = external("jsbsim_deriv", "get_derivative", 
                            {x_k + dt/2*k2, u_k}, {"state", "controls"}, {"derivative"})[i];
        }
        
        // k4 = f(x_k + dt*k3, u_k)
        MX k4 = MX::zeros(nx);
        for (int i = 0; i < nx; ++i) {
            k4(i) = external("jsbsim_deriv", "get_derivative", 
                            {x_k + dt*k3, u_k}, {"state", "controls"}, {"derivative"})[i];
        }
        
        MX x_next = x_k + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        
        // 添加动力学约束
        g.push_back(X(Slice(), k+1) - x_next);
        
        // 控制成本（平滑性）
        if (k > 0) {
            cost += 0.1*dot(U(Slice(), k) - U(Slice(), k-1));
        }
    }
    
    // 终端成本
    MX position_error = X(Slice(0, 3), N) - xf(Slice(0, 3));
    MX attitude_error = X(Slice(3, 7), N) - xf(Slice(3, 7));
    cost += 100*dot(position_error) + 50*dot(attitude_error);
    
    // 控制限制
    for (int k = 0; k < N; ++k) {
        g.push_back(U(0, k));  // 油门 [0,1]
        g.push_back(1 - U(0, k));
        g.push_back(U(Slice(1,4), k));  // 操纵面 [-1,1]
        g.push_back(1 - U(Slice(1,4), k));
    }
    
    // 创建 NLP 问题
    MXDict nlp;
    nlp["x"] = vertcat(reshape(X, nx*(N+1), 1), reshape(U, nu*N, 1));
    nlp["f"] = cost;
    nlp["g"] = vertcat(g);
    nlp["p"] = vertcat(x0, xf);
    
    // 求解器选项
    Dict opts;
    opts["ipopt.print_level"] = 5;
    opts["print_time"] = 1;
    opts["expand"] = 1;
    
    // 创建求解器
    return nlpsol("solver", "ipopt", nlp, opts);
}

// JSBSim 导数计算函数
std::vector<double> jsbsim_deriv(const std::vector<double>& state, 
                                const std::vector<double>& controls) {
    static JSBSimInterface jsbsim("c172x", "piston");
    return jsbsim.get_state_derivative(state, controls);
}

int main() {
    // 初始化 JSBSim 接口
    JSBSimInterface jsbsim("c172x", "piston");
    
    // 轨迹参数
    int N = 30;       // 控制步数
    double dt = 0.2;   // 时间步长
    
    // 创建求解器
    Function solver = create_trajectory_optimizer(jsbsim, N, dt);
    
    // 初始状态和目标状态
    std::vector<double> x0(jsbsim.get_state_dim(), 0.0);
    x0[0] = 0.0;    // North 位置
    x0[1] = 0.0;    // East 位置
    x0[2] = 1000.0; // 高度 (Down为负，所以这里表示海拔1000米)
    x0[3] = 1.0;    // 四元数 w (无旋转)
    
    std::vector<double> xf(jsbsim.get_state_dim(), 0.0);
    xf[0] = 5000.0;  // 目标 North 位置
    xf[1] = 2000.0;  // 目标 East 位置
    xf[2] = 1200.0;  // 目标高度
    
    // 目标姿态 (俯仰角5度)
    double pitch = 5.0*M_PI/180.0;
    xf[3] = cos(pitch/2);
    xf[6] = sin(pitch/2);
    
    // 初始猜测
    std::vector<double> X_guess(jsbsim.get_state_dim()*(N+1), 0.0);
    std::vector<double> U_guess(jsbsim.get_control_dim()*N, 0.5);
    U_guess[0] = 0.7;  // 初始油门猜测
    
    // 线性插值初始状态猜测
    for (int k = 0; k <= N; ++k) {
        double alpha = static_cast<double>(k)/N;
        for (int i = 0; i < jsbsim.get_state_dim(); ++i) {
            X_guess[k*jsbsim.get_state_dim() + i] = (1-alpha)*x0[i] + alpha*xf[i];
        }
    }
    
    // 构建参数和初始猜测
    DMDict arg;
    arg["p"] = vertcat(x0, xf);
    arg["x0"] = vertcat(X_guess, U_guess);
    
    // 约束边界
    int num_constraints = jsbsim.get_state_dim()*(N+1) + 6*N;
    arg["lbg"] = std::vector<double>(num_constraints, 0.0);
    arg["ubg"] = std::vector<double>(num_constraints, 0.0);
    
    // 控制限制
    for (int k = 0; k < N; ++k) {
        // 动力学约束是等式约束 (lbg = ubg = 0)
        // 控制限制是不等式约束
        
        // 油门 [0,1]
        arg["lbg"][jsbsim.get_state_dim()*(N+1) + 4*k] = 0.0;
        arg["ubg"][jsbsim.get_state_dim()*(N+1) + 4*k] = 1.0;
        
        // 副翼 [-1,1]
        arg["lbg"][jsbsim.get_state_dim()*(N+1) + 4*k + 1] = -1.0;
        arg["ubg"][jsbsim.get_state_dim()*(N+1) + 4*k + 1] = 1.0;
        
        // 升降舵 [-1,1]
        arg["lbg"][jsbsim.get_state_dim()*(N+1) + 4*k + 2] = -1.0;
        arg["ubg"][jsbsim.get_state_dim()*(N+1) + 4*k + 2] = 1.0;
        
        // 方向舵 [-1,1]
        arg["lbg"][jsbsim.get_state_dim()*(N+1) + 4*k + 3] = -1.0;
        arg["ubg"][jsbsim.get_state_dim()*(N+1) + 4*k + 3] = 1.0;
    }
    
    // 注册外部函数
    ExternalFunction::register_function("jsbsim_deriv", jsbsim_deriv);
    
    // 求解问题
    DMDict res = solver(arg);
    
    // 提取结果
    std::vector<double> X_opt = res["x"](Slice(0, jsbsim.get_state_dim()*(N+1)));
    std::vector<double> U_opt = res["x"](Slice(jsbsim.get_state_dim()*(N+1), 
                                             jsbsim.get_state_dim()*(N+1)+jsbsim.get_control_dim()*N));
    
    std::cout << "Optimization complete." << std::endl;
    std::cout << "Final controls: ";
    for (int k = 0; k < jsbsim.get_control_dim(); ++k) {
        std::cout << U_opt[(N-1)*jsbsim.get_control_dim() + k] << " ";
    }
    std::cout << std::endl;
    
    return 0;
}