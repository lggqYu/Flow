#include <torch/torch.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <deque>

// 经验回放缓冲区
class ReplayBuffer {
public:
    ReplayBuffer(int capacity) : capacity(capacity) {}

    void add(torch::Tensor state, torch::Tensor action, torch::Tensor reward,
        torch::Tensor next_state, torch::Tensor done) {
        if (buffer.size() >= capacity) {
            buffer.pop_front();
        }
        buffer.push_back({ state, action, reward, next_state, done });
    }

    std::vector<torch::Tensor> sample(int batch_size) {
        std::vector<int> indices(batch_size);
        std::uniform_int_distribution<int> dist(0, buffer.size() - 1);
        for (int i = 0; i < batch_size; ++i) {
            indices[i] = dist(gen);
        }

        std::vector<torch::Tensor> states, actions, rewards, next_states, dones;
        for (int idx : indices) {
            auto& transition = buffer[idx];
            states.push_back(transition[0]);
            actions.push_back(transition[1]);
            rewards.push_back(transition[2]);
            next_states.push_back(transition[3]);
            dones.push_back(transition[4]);
        }

        return {
            torch::stack(states),
            torch::stack(actions),
            torch::stack(rewards),
            torch::stack(next_states),
            torch::stack(dones)
        };
    }

    int size() const { return buffer.size(); }

private:
    int capacity;
    std::deque<std::vector<torch::Tensor>> buffer;
    std::mt19937 gen{ std::random_device{}() };
};

// 演员网络 (策略网络)
struct Actor : torch::nn::Module {
    Actor(int state_dim = 9, int action_dim = 4, int hidden_size = 256)
        : fc1(register_module("fc1", torch::nn::Linear(state_dim, hidden_size))),
        fc2(register_module("fc2", torch::nn::Linear(hidden_size, hidden_size))),
        fc3(register_module("fc3", torch::nn::Linear(hidden_size, action_dim))) {
    }

    torch::Tensor forward(torch::Tensor x) {
        x = torch::relu(fc1->forward(x));
        x = torch::relu(fc2->forward(x));
        x = torch::tanh(fc3->forward(x)); // 输出在[-1,1]范围
        return x;
    }

    torch::nn::Linear fc1, fc2, fc3;
};

// 评论家网络 (Q值网络)
struct Critic : torch::nn::Module {
    Critic(int state_dim = 9, int action_dim = 4, int hidden_size = 256)
        : fc1(register_module("fc1", torch::nn::Linear(state_dim + action_dim, hidden_size))),
        fc2(register_module("fc2", torch::nn::Linear(hidden_size, hidden_size))),
        fc3(register_module("fc3", torch::nn::Linear(hidden_size, 1))) {
    }

    torch::Tensor forward(torch::Tensor state, torch::Tensor action) {
        auto x = torch::cat({ state, action }, 1);
        x = torch::relu(fc1->forward(x));
        x = torch::relu(fc2->forward(x));
        x = fc3->forward(x);
        return x;
    }

    torch::nn::Linear fc1, fc2, fc3;
};

// DDPG 智能体
class DDPGAgent {
public:
    DDPGAgent(int state_dim, int action_dim, float gamma = 0.99, float tau = 0.005)
        : gamma(gamma), tau(tau),
        actor(state_dim, action_dim),
        critic(state_dim, action_dim),
        target_actor(state_dim, action_dim),
        target_critic(state_dim, action_dim),
        replay_buffer(100000),
        actor_optimizer(actor->parameters(), torch::optim::AdamOptions(1e-4)),
        critic_optimizer(critic->parameters(), torch::optim::AdamOptions(1e-3)) {

        // 初始化目标网络
        target_actor->copy_weights(actor);
        target_critic->copy_weights(critic);

        // 设置目标网络为评估模式
        target_actor->eval();
        target_critic->eval();
    }

    torch::Tensor select_action(torch::Tensor state, bool add_noise = true) {
        if (add_noise) {
            // 训练时添加探索噪声
            auto action = actor->forward(state).detach();
            auto noise = torch::randn_like(action) * 0.1;
            return torch::clamp(action + noise, -1.0, 1.0);
        }
        else {
            // 测试时不添加噪声
            return actor->forward(state).detach();
        }
    }

    void update(int batch_size) {
        if (replay_buffer.size() < batch_size) return;

        // 从经验回放中采样
        auto batch = replay_buffer.sample(batch_size);
        auto states = batch[0];
        auto actions = batch[1];
        auto rewards = batch[2];
        auto next_states = batch[3];
        auto dones = batch[4];

        // 更新评论家网络
        auto next_actions = target_actor->forward(next_states);
        auto target_q = target_critic->forward(next_states, next_actions).detach();
        auto target_value = rewards + gamma * target_q * (1 - dones);

        auto current_q = critic->forward(states, actions);
        auto critic_loss = torch::mse_loss(current_q, target_value);

        critic_optimizer.zero_grad();
        critic_loss.backward();
        critic_optimizer.step();

        // 更新演员网络
        auto actor_actions = actor->forward(states);
        auto actor_loss = -critic->forward(states, actor_actions).mean();

        actor_optimizer.zero_grad();
        actor_loss.backward();
        actor_optimizer.step();

        // 软更新目标网络
        update_target_network(target_actor, actor, tau);
        update_target_network(target_critic, critic, tau);
    }

    void save_model(const std::string& path) {
        torch::save(actor, path + "_actor.pt");
        torch::save(critic, path + "_critic.pt");
    }

    void load_model(const std::string& path) {
        torch::load(actor, path + "_actor.pt");
        torch::load(critic, path + "_critic.pt");
    }

private:
    void update_target_network(torch::nn::Module& target, torch::nn::Module& source, float tau) {
        auto target_params = target.named_parameters();
        auto source_params = source.named_parameters();

        for (auto& target_param : target_params) {
            auto& name = target_param.key();
            auto& target_data = target_param.value().data();
            auto& source_data = source_params[name].data();
            target_data = tau * source_data + (1.0f - tau) * target_data;
        }
    }

    float gamma;  // 折扣因子
    float tau;    // 目标网络软更新系数

    torch::nn::ModuleHolder<Actor> actor;
    torch::nn::ModuleHolder<Critic> critic;
    torch::nn::ModuleHolder<Actor> target_actor;
    torch::nn::ModuleHolder<Critic> target_critic;

    ReplayBuffer replay_buffer;
    torch::optim::Adam actor_optimizer;
    torch::optim::Adam critic_optimizer;
};

// 飞机动力学模型
class AircraftEnvironment {
public:
    AircraftEnvironment(const std::vector<double>& start_pos,
        const std::vector<double>& target_pos)
        : target(torch::tensor(target_pos, torch::kFloat32)) {
        // 初始化状态: [经度, 纬度, 高度, 俯仰, 滚转, 偏航, U, V, W]
        state = torch::zeros({ 9 }, torch::kFloat32);
        for (int i = 0; i < 3; ++i) {
            state[i] = start_pos[i];
        }

        // 地球参数
        R = 6371000.0; // 地球半径(米)
        deg2rad = M_PI / 180.0;
    }

    void reset(const std::vector<double>& start_pos) {
        for (int i = 0; i < 3; ++i) {
            state[i] = start_pos[i];
        }
        // 重置其他状态为0
        for (int i = 3; i < 9; ++i) {
            state[i] = 0.0;
        }
    }

    void step(const torch::Tensor& action) {
        // 解包状态
        float lon = state[0].item<float>();
        float lat = state[1].item<float>();
        float alt = state[2].item<float>();
        float pitch = state[3].item<float>();
        float roll = state[4].item<float>();
        float yaw = state[5].item<float>();
        float U = state[6].item<float>();
        float V = state[7].item<float>();
        float W = state[8].item<float>();

        // 解包动作
        float throttle = (action[0].item<float>() + 1.0f) * 0.5f; // [0,1]
        float aileron = action[1].item<float>();   // [-1,1]
        float elevator = action[2].item<float>();  // [-1,1]
        float rudder = action[3].item<float>();    // [-1,1]

        // 简化的飞机动力学方程
        // 姿态变化率
        float pitch_rate = 0.5f * elevator - 0.1f * pitch;
        float roll_rate = 1.0f * aileron - 0.2f * roll;
        float yaw_rate = 0.3f * rudder - 0.1f * yaw;

        // 更新姿态
        pitch += pitch_rate * dt;
        roll += roll_rate * dt;
        yaw += yaw_rate * dt;

        // 限制姿态角度
        pitch = std::clamp(pitch, -M_PI / 3, M_PI / 3);
        roll = std::clamp(roll, -M_PI / 2, M_PI / 2);

        // 速度变化
        float thrust = 20.0f * throttle;
        float drag = 0.1f * U * std::abs(U);
        float lift = 0.2f * std::abs(U) * pitch;

        float dU = (thrust - drag) * dt;
        float dV = (0.5f * rudder - 0.1f * V) * dt;
        float dW = (lift - 9.8f) * dt;

        U += dU;
        V += dV;
        W += dW;

        // 机体坐标系转地面坐标系
        float cosP = std::cos(pitch), sinP = std::sin(pitch);
        float cosR = std::cos(roll), sinR = std::sin(roll);
        float cosY = std::cos(yaw), sinY = std::sin(yaw);

        float Vn = U * cosY * cosP + V * (cosY * sinP * sinR - sinY * cosR) + W * (cosY * sinP * cosR + sinY * sinR);
        float Ve = U * sinY * cosP + V * (sinY * sinP * sinR + cosY * cosR) + W * (sinY * sinP * cosR - cosY * sinR);
        float Vd = U * (-sinP) + V * cosP * sinR + W * cosP * cosR;

        // 更新位置
        float lat_rad = lat * deg2rad;
        float dlat = (Ve / (R + alt)) * (180.0f / M_PI) * dt;
        float dlon = (Vn / ((R + alt) * std::cos(lat_rad))) * (180.0f / M_PI) * dt;
        float dh = -Vd * dt;

        lon += dlon;
        lat += dlat;
        alt += dh;

        // 更新状态
        state[0] = lon;
        state[1] = lat;
        state[2] = alt;
        state[3] = pitch;
        state[4] = roll;
        state[5] = yaw;
        state[6] = U;
        state[7] = V;
        state[8] = W;
    }

    // 计算奖励函数
    torch::Tensor compute_reward() {
        auto current_pos = state.slice(0, 0, 3);
        auto distance = torch::norm(current_pos - target);

        // 基础奖励：负距离
        float reward = -distance.item<float>();

        // 到达目标奖励
        if (distance.item<float>() < 0.001) {
            reward += 100.0f;
        }

        // 姿态惩罚（避免极端姿态）
        float pitch_penalty = -0.1f * std::abs(state[3].item<float>());
        float roll_penalty = -0.1f * std::abs(state[4].item<float>());

        // 速度限制
        float speed = torch::norm(state.slice(0, 6, 9)).item<float>();
        float speed_penalty = (speed > 100.0f) ? -0.5f : 0.0f;

        return torch::tensor(reward + pitch_penalty + roll_penalty + speed_penalty);
    }

    torch::Tensor get_state() const { return state.clone(); }
    torch::Tensor is_done() const {
        auto distance = torch::norm(state.slice(0, 0, 3) - target);
        return torch::tensor(distance.item<float>() < 0.001 ? 1.0f : 0.0f);
    }

private:
    torch::Tensor state;
    torch::Tensor target;
    float R = 6371000.0;      // 地球半径(米)
    float deg2rad = M_PI / 180.0;
    float dt = 0.1f;          // 时间步长
};

// 训练函数
void train_ddpg(DDPGAgent& agent, AircraftEnvironment& env, int episodes = 1000) {
    const int batch_size = 64;
    const int max_steps = 500;
    const int start_training = 1000;

    for (int episode = 0; episode < episodes; ++episode) {
        // 重置环境
        env.reset({ 0.0, 0.0, 1000.0 });

        float total_reward = 0.0f;
        bool done = false;

        for (int step = 0; step < max_steps && !done; ++step) {
            // 获取当前状态
            auto state = env.get_state();

            // 选择动作
            auto action = agent.select_action(state.unsqueeze(0)).squeeze(0);

            // 执行动作
            env.step(action);

            // 获取下一个状态和奖励
            auto next_state = env.get_state();
            auto reward = env.compute_reward();
            done = env.is_done().item<float>() > 0.5;
            auto done_tensor = done ? torch::tensor(1.0f) : torch::tensor(0.0f);

            // 存储经验
            agent.replay_buffer.add(state, action, reward, next_state, done_tensor);

            total_reward += reward.item<float>();

            // 更新智能体
            if (agent.replay_buffer.size() > start_training) {
                agent.update(batch_size);
            }
        }

        // 输出训练进度
        if (episode % 50 == 0) {
            std::cout << "Episode: " << episode
                << " | Total Reward: " << total_reward
                << " | Buffer Size: " << agent.replay_buffer.size()
                << std::endl;
        }

        // 定期保存模型
        if (episode % 200 == 0) {
            agent.save_model("aircraft_ddpg");
        }
    }
}

// 测试函数
void test_agent(DDPGAgent& agent, AircraftEnvironment& env) {
    env.reset({ 0.0, 0.0, 1000.0 });

    for (int step = 0; step < 500; ++step) {
        auto state = env.get_state();
        auto action = agent.select_action(state.unsqueeze(0), false).squeeze(0);
        env.step(action);

        auto distance = torch::norm(env.get_state().slice(0, 0, 3) - env.target);
        std::cout << "Step: " << step << " | Distance: " << distance.item<float>() << std::endl;

        if (distance.item<float>() < 0.001) {
            std::cout << "Reached target!" << std::endl;
            break;
        }
    }
}

int main() {
    // 创建DDPG智能体和飞机环境
    DDPGAgent agent(9, 4);
    AircraftEnvironment env({ 0.0, 0.0, 1000.0 }, { 0.1, 0.1, 1000.0 });

    // 训练智能体
    train_ddpg(agent, env, 2000);

    // 保存最终模型
    agent.save_model("aircraft_ddpg_final");

    // 测试训练好的智能体
    test_agent(agent, env);

    std::cout << "Training and testing completed." << std::endl;
    return 0;
}