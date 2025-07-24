#include <torch/torch.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>

// 自定义 clamp 函数
template <typename T>
T clamp(const T& value, const T& min_val, const T& max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 神经网络控制器 - 支持自定义隐藏层结构
struct AircraftControllerImpl : torch::nn::Module {
    AircraftControllerImpl(int state_dim, int action_dim,const std::vector<int>& hidden_num)
    {

        // 输入层
        auto input_layer = torch::nn::Linear(state_dim, hidden_num[0]);
        layers->push_back(input_layer);
        register_module("layer0", input_layer);

        // 隐藏层
        for (size_t i = 1; i < hidden_num.size(); i++) {
            auto hidden_layer = torch::nn::Linear(hidden_num[i - 1], hidden_num[i]);
            layers->push_back(hidden_layer);
            register_module("layer" + std::to_string(i), hidden_layer);
        }

        // 输出层
        output_layer = register_module("output", torch::nn::Linear(hidden_num.back(), action_dim));
    }

    torch::Tensor forward(torch::Tensor x) {
        // 通过所有隐藏层（使用ReLU激活）
        for (size_t i = 0; i < layers->size(); i++) {
            x = torch::relu(layers[i]->as<torch::nn::Linear>()->forward(x));
        }

        // 输出层（使用Tanh激活）
        x = torch::tanh(output_layer->forward(x));
        return x;
    }

    // 序列化支持
    void save(torch::serialize::OutputArchive& archive) const {
        // 保存隐藏层
        for (size_t i = 0; i < layers->size(); i++) {
            auto layer = layers->ptr(i)->as<torch::nn::Linear>();
            archive.write("layer" + std::to_string(i) + ".weight", layer->weight);
            archive.write("layer" + std::to_string(i) + ".bias", layer->bias);
        }

        // 保存输出层
        archive.write("output.weight", output_layer->weight);
        archive.write("output.bias", output_layer->bias);
    }

    void load(torch::serialize::InputArchive& archive) {
        // 加载隐藏层
        for (size_t i = 0; i < layers->size(); i++) {
            auto layer = layers[i]->as<torch::nn::Linear>();
            archive.read("layer" + std::to_string(i) + ".weight", layer->weight);
            archive.read("layer" + std::to_string(i) + ".bias", layer->bias);
        }

        // 加载输出层
        archive.read("output.weight", output_layer->weight);
        archive.read("output.bias", output_layer->bias);
    }

    torch::nn::ModuleList layers;  // 所有隐藏层
    torch::nn::Linear output_layer{ nullptr };  // 输出层
};

TORCH_MODULE(AircraftController);

// 飞机状态结构体
struct AircraftState {
    double longitude;  // 经度 (度)
    double latitude;   // 纬度 (度)
    double altitude;   // 高度 (米)
    double pitch;      // 俯仰角 (弧度)
    double roll;       // 滚转角 (弧度)
    double yaw;        // 偏航角 (弧度)
    double u;          // 前向速度 (m/s)
    double v;          // 横向速度 (m/s)
    double w;          // 垂直速度 (m/s)

    // 转换为张量
    torch::Tensor toTensor() const {
        torch::Tensor tensor = torch::zeros({ 9 }, torch::kFloat32);
        tensor[0] = longitude;
        tensor[1] = latitude;
        tensor[2] = altitude;
        tensor[3] = pitch;
        tensor[4] = roll;
        tensor[5] = yaw;
        tensor[6] = u;
        tensor[7] = v;
        tensor[8] = w;
        return tensor;
    }

    void print(const std::string& label = "State") const {
        std::cout << label << ": ";
        std::cout << "Lon: " << longitude << "°, ";
        std::cout << "Lat: " << latitude << "°, ";
        std::cout << "Alt: " << altitude << "m, ";
        std::cout << "Pitch: " << degrees(pitch) << "°, ";
        std::cout << "Roll: " << degrees(roll) << "°, ";
        std::cout << "Yaw: " << degrees(yaw) << "°, ";
        std::cout << "U: " << u << "m/s, ";
        std::cout << "V: " << v << "m/s, ";
        std::cout << "W: " << w << "m/s\n";
    }

private:
    double degrees(double rad) const { return rad * 180.0 / M_PI; }
};

// 目标状态结构体
struct TargetState {
    double longitude;  // 目标经度 (度)
    double latitude;   // 目标纬度 (度)
    double altitude;   // 目标高度 (米)

    // 转换为张量
    torch::Tensor toTensor() const {
        torch::Tensor tensor = torch::zeros({ 3 }, torch::kFloat32);
        tensor[0] = longitude;
        tensor[1] = latitude;
        tensor[2] = altitude;
        return tensor;
    }

    void print() const {
        std::cout << "Target: ";
        std::cout << "Lon: " << longitude << "°, ";
        std::cout << "Lat: " << latitude << "°, ";
        std::cout << "Alt: " << altitude << "m\n";
    }
};

// 控制输出结构体
struct ControlOutput {
    double throttle;   // 油门 [0, 1]
    double aileron;    // 副翼 [-1, 1]
    double elevator;   // 升降舵 [-1, 1]
    double rudder;     // 方向舵 [-1, 1]

    static ControlOutput fromTensor(const torch::Tensor& tensor) {
        ControlOutput ctrl;
        auto data = tensor.data_ptr<float>();
        ctrl.throttle = (data[0] + 1.0) * 0.5;
        ctrl.aileron = data[1];
        ctrl.elevator = data[2];
        ctrl.rudder = data[3];
        return ctrl;
    }

    void print() const {
        std::cout << "Control: ";
        std::cout << "Throttle: " << throttle * 100 << "%, ";
        std::cout << "Aileron: " << aileron * 100 << "%, ";
        std::cout << "Elevator: " << elevator * 100 << "%, ";
        std::cout << "Rudder: " << rudder * 100 << "%\n";
    }
};

// 飞机动力学模型
class AircraftDynamics {
public:
    AircraftDynamics() : earth_radius(6371000.0), deg2rad(M_PI / 180.0) {}

    void update(AircraftState& state, const ControlOutput& ctrl, double dt = 0.1) {
        // 姿态变化率
        double pitch_rate = 0.5 * ctrl.elevator - 0.1 * state.pitch;
        double roll_rate = 1.0 * ctrl.aileron - 0.2 * state.roll;
        double yaw_rate = 0.3 * ctrl.rudder - 0.1 * state.yaw;

        // 更新姿态
        state.pitch += pitch_rate * dt;
        state.roll += roll_rate * dt;
        state.yaw += yaw_rate * dt;

        // 限制姿态角度
        state.pitch = clamp(state.pitch, -M_PI / 3.0, M_PI / 3.0);
        state.roll = clamp(state.roll, -M_PI / 2.0, M_PI / 2.0);

        // 速度变化
        double thrust = 20.0 * ctrl.throttle;
        double drag = 0.1 * state.u * std::abs(state.u);
        double lift = 0.2 * std::abs(state.u) * state.pitch;

        double du = (thrust - drag) * dt;
        double dv = (0.5 * ctrl.rudder - 0.1 * state.v) * dt;
        double dw = (lift - 9.8) * dt;

        state.u += du;
        state.v += dv;
        state.w += dw;

        // 坐标系转换
        double cosP = std::cos(state.pitch), sinP = std::sin(state.pitch);
        double cosR = std::cos(state.roll), sinR = std::sin(state.roll);
        double cosY = std::cos(state.yaw), sinY = std::sin(state.yaw);

        double vn = state.u * cosY * cosP +
            state.v * (cosY * sinP * sinR - sinY * cosR) +
            state.w * (cosY * sinP * cosR + sinY * sinR);

        double ve = state.u * sinY * cosP +
            state.v * (sinY * sinP * sinR + cosY * cosR) +
            state.w * (sinY * sinP * cosR - cosY * sinR);

        double vd = state.u * (-sinP) +
            state.v * cosP * sinR +
            state.w * cosP * cosR;

        // 更新位置
        double lat_rad = state.latitude * deg2rad;
        double dlat = (ve / (earth_radius + state.altitude)) * (180.0 / M_PI) * dt;
        double dlon = (vn / ((earth_radius + state.altitude) * std::max(0.0001, std::cos(lat_rad)))) * (180.0 / M_PI) * dt;
        double dh = -vd * dt;

        state.longitude += dlon;
        state.latitude += dlat;
        state.altitude += dh;
    }

private:
    double earth_radius;
    double deg2rad;
};

// 计算状态向量
torch::Tensor compute_state_vector(const AircraftState& current, const TargetState& target) {
    return torch::cat({ current.toTensor(), target.toTensor() });
}

// 主控制循环
void control_loop(AircraftController& controller, AircraftDynamics& dynamics) {
    AircraftState current_state{ 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0 };
    TargetState target_state{ 0.1, 0.1, 1000.0 };

    current_state.print("Initial State");
    target_state.print();

    const int max_steps = 500;
    for (int step = 0; step < max_steps; ++step) {
        // 计算状态向量
        auto state_vector = compute_state_vector(current_state, target_state);

        // 神经网络计算控制输出
        auto action = controller->forward(state_vector);

        // 转换为控制指令
        ControlOutput ctrl = ControlOutput::fromTensor(action);

        // 应用控制指令
        dynamics.update(current_state, ctrl);

        // 每50步打印一次状态
        if (step % 50 == 0) {
            std::cout << "\nStep: " << step << std::endl;
            current_state.print("Current State");
            ctrl.print();

            // 计算到目标的距离
            double dlon = (target_state.longitude - current_state.longitude) * 111319.0;
            double dlat = (target_state.latitude - current_state.latitude) * 111319.0;
            double dalt = target_state.altitude - current_state.altitude;
            double distance = std::sqrt(dlon * dlon + dlat * dlat + dalt * dalt);
            std::cout << "Distance to target: " << distance << " meters\n";
        }

        // 检查是否到达目标
        if (std::abs(target_state.longitude - current_state.longitude) < 0.0001 &&
            std::abs(target_state.latitude - current_state.latitude) < 0.0001 &&
            std::abs(target_state.altitude - current_state.altitude) < 10.0) {
            std::cout << "\nReached target at step " << step << "!\n";
            current_state.print("Final State");
            break;
        }
    }
}

int main() {
    // 定义隐藏层配置
    std::vector<int> hidden_num = { 16, 32, 64, 128, 64, 16 };

    // 创建控制器（使用自定义隐藏层结构）
    AircraftController controller(12, 4, hidden_num);
    AircraftDynamics dynamics;

    // 尝试加载预训练模型
    try {
        torch::load(controller, "aircraft_controller.pt");
        std::cout << "Loaded pretrained model.\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading model: " << e.what() << "\n";
        std::cout << "Using untrained controller.\n";
    }

    // 打印网络结构信息
    std::cout << "Neural Network Architecture:\n";
    std::cout << "Input layer: 12 neurons\n";
    for (size_t i = 0; i < hidden_num.size(); i++) {
        std::cout << "Hidden layer " << (i + 1) << ": " << hidden_num[i] << " neurons\n";
    }
    std::cout << "Output layer: 4 neurons\n";

    // 运行控制循环
    control_loop(controller, dynamics);
    torch::save(controller, "aircraft_controller.pt");

    return 0;
}