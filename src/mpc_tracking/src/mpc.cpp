// mpc_tracking/mpc.cpp
#include "mpc_tracking/mpc.h"
#include <iostream>

Mpc::Mpc() {
    N_ = 10;    // 预测步长
    dt_ = 0.05; // 时间步长（更适合机械臂）
    
    // 设置UR10e的速度限制（单位：m/s, rad/s）
    u_max_ << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5;  // 线速度 + 角速度
    u_min_ = -u_max_;
    
    // 设置权重（6自由度）
    std::vector<double> weights = {10, 10, 10, 5, 5, 5,   // Q: 位置+姿态
                                   1, 1, 1, 0.5, 0.5, 0.5}; // R: 速度+角速度
    
    Q_ = DM::zeros(6,6);
    R_ = DM::zeros(6,6);
    
    setWeights(weights);
    feature_weight_ = 0.1;  // 视觉特征误差权重
    
    kinematic_equation_ = setKinematicEquation();
}

Mpc::~Mpc() {}

void Mpc::setWeights(std::vector<double> weights) {
    // 设置状态误差权重Q (6x6)
    for (int i = 0; i < 6; i++) {
        Q_(i, i) = weights[i];
    }
    
    // 设置控制权重R (6x6)
    for (int i = 0; i < 6; i++) {
        R_(i, i) = weights[i + 6];
    }
}

void Mpc::setFeatureWeights(double weight) {
    feature_weight_ = weight;
}

//一阶积分器运动学模型
Function Mpc::setKinematicEquation() {
    // 6自由度状态：位置(x,y,z) + 欧拉角(rx,ry,rz)
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX z = MX::sym("z");
    MX rx = MX::sym("rx");
    MX ry = MX::sym("ry");
    MX rz = MX::sym("rz");
    MX state_vars = MX::vertcat({x, y, z, rx, ry, rz});

    // 6自由度控制：线速度(vx,vy,vz) + 角速度(wx,wy,wz)
    MX vx = MX::sym("vx");
    MX vy = MX::sym("vy");
    MX vz = MX::sym("vz");
    MX wx = MX::sym("wx");
    MX wy = MX::sym("wy");
    MX wz = MX::sym("wz");
    MX control_vars = MX::vertcat({vx, vy, vz, wx, wy, wz});
    
    // 简化运动学：位姿变化率 = 速度
    // 对于视觉伺服，通常是小范围运动，这个简化是合理的
    MX rhs = MX::vertcat({vx, vy, vz, wx, wy, wz});
    
    return Function("robot_kinematic_equation", {state_vars, control_vars}, {rhs});
}//xk+1 = xk + f(xk,uk)*dt

bool Mpc::solve(Eigen::Vector6d current_states, Eigen::MatrixXd desired_states) {
    Opti opti = Opti();
    Slice all;
    MX cost = 0;

    // 定义优化变量：6维状态 × (N+1)步，6维控制 × N步
    X = opti.variable(6, N_ + 1);  // [x,y,z,rx,ry,rz] × 时间步
    U = opti.variable(6, N_);      // [vx,vy,vz,wx,wy,wz] × 时间步

    // 提取变量切片便于使用
    MX x = X(0, all); MX y = X(1, all); MX z = X(2, all);
    MX rx = X(3, all); MX ry = X(4, all); MX rz = X(5, all);
    
    MX vx = U(0, all); MX vy = U(1, all); MX vz = U(2, all);
    MX wx = U(3, all); MX wy = U(4, all); MX wz = U(5, all);

    // 设置参数：当前状态和参考轨迹
    MX X_ref = opti.parameter(6, N_ + 1);
    MX X_cur = opti.parameter(6);
    
    // 转换当前状态
    DM current_dm(6,1);
    for (int i = 0; i < 6; i++) {
        current_dm(i) = current_states(i);
    }
    opti.set_value(X_cur, current_dm);
    
    // 转换参考轨迹
    std::vector<double> ref_vec(desired_states.data(), 
                               desired_states.data() + desired_states.size());
    DM ref_dm(ref_vec);
    X_ref = MX::reshape(ref_dm, 6, N_ + 1);
    opti.set_value(X_ref, ref_dm);

    // 构建代价函数
    for (int i = 0; i < N_; ++i) {
        // 状态误差代价
        MX X_err = X(all, i) - X_ref(all, i);
        cost += MX::mtimes({X_err.T(), Q_, X_err});
        
        // 控制代价
        MX U_i = U(all, i);
        cost += MX::mtimes({U_i.T(), R_, U_i});
        
        // 可选：添加控制变化率惩罚（使控制更平滑）
        if (i > 0) {
            MX U_prev = U(all, i-1);
            MX U_diff = U_i - U_prev;
            cost += 0.1 * MX::mtimes({U_diff.T(), U_diff});  // 平滑项
        }
    }
    
    // 终端代价
    MX X_err_final = X(all, N_) - X_ref(all, N_);
    cost += MX::mtimes({X_err_final.T(), Q_, X_err_final});

    opti.minimize(cost);

    // 系统动力学约束
    for (int i = 0; i < N_; ++i) {
        std::vector<MX> input(2);
        input[0] = X(all, i);  // 当前状态
        input[1] = U(all, i);  // 当前控制
        
        // 离散化：x_{k+1} = x_k + f(x_k, u_k) * dt
        MX X_next = kinematic_equation_(input)[0] * dt_ + X(all, i);
        opti.subject_to(X_next == X(all, i + 1));
    }

    // 初始条件约束
    opti.subject_to(X(all, 0) == X_cur);

    // 控制量约束
    opti.subject_to(u_min_(0) <= vx <= u_max_(0));
    opti.subject_to(u_min_(1) <= vy <= u_max_(1));
    opti.subject_to(u_min_(2) <= vz <= u_max_(2));
    opti.subject_to(u_min_(3) <= wx <= u_max_(3));
    opti.subject_to(u_min_(4) <= wy <= u_max_(4));
    opti.subject_to(u_min_(5) <= wz <= u_max_(5));

    // 可选：姿态角约束（避免奇异性）
    opti.subject_to(-M_PI + 0.1 <= ry <= M_PI - 0.1);  // 限制pitch角

    // 求解器配置
    casadi::Dict solver_opts;
    solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 50;  // 减少迭代次数以提高实时性
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.tol"] = 1e-4;
    
    opti.solver("ipopt", solver_opts);

    try {
        solution_ = std::make_unique<casadi::OptiSol>(opti.solve());
        return true;
    } catch (std::exception& e) {
        std::cout << "MPC求解失败: " << e.what() << std::endl;
        return false;
    }
}

std::vector<double> Mpc::getFirstU() {
    std::vector<double> control(6);
    
    if (solution_) {
        DM U_val = solution_->value(U);
        for (int i = 0; i < 6; i++) {
            control[i] = static_cast<double>(U_val(i, 0));
        }
    } else {
        // 求解失败时返回零控制
        std::fill(control.begin(), control.end(), 0.0);
    }
    
    return control;
}

std::vector<double> Mpc::getPredictX() {
    std::vector<double> trajectory;
    
    if (solution_) {
        DM X_val = solution_->value(X);
        for (int i = 0; i <= N_; ++i) {
            for (int j = 0; j < 6; ++j) {
                trajectory.push_back(static_cast<double>(X_val(j, i)));
            }
        }
    }
    
    return trajectory;
}