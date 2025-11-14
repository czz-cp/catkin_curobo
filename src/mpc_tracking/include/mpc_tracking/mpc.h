// mpc_tracking/mpc.h
#ifndef MPC_H
#define MPC_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>

using namespace casadi;

class Mpc {
public:
    Mpc();
    ~Mpc();
    
    // 修改为6自由度：位置(x,y,z) + 姿态(rx,ry,rz)
    bool solve(Eigen::Vector6d current_states, Eigen::MatrixXd desired_states);
    
    // 修改为6维控制量：线速度(vx,vy,vz) + 角速度(wx,wy,wz)
    std::vector<double> getFirstU();
    
    // 获取预测轨迹
    std::vector<double> getPredictX();
    
    // 设置权重（6自由度）
    void setWeights(std::vector<double> weights);
    
    // 设置视觉伺服特征误差权重
    void setFeatureWeights(double weight);

private:
    int N_;         // 预测步长
    double dt_;     // 时间步长
    
    // 6自由度控制限制
    Eigen::Vector6d u_max_, u_min_;
    Eigen::Vector6d w_max_, w_min_;  // 角速度限制
    
    DM Q_, R_;      // 权重矩阵
    double feature_weight_;  // 视觉特征误差权重
    
    Function kinematic_equation_;  // 运动学方程
    
    // 优化变量和结果
    MX X, U;
    std::unique_ptr<casadi::OptiSol> solution_;
    
    // 创建6自由度运动学方程
    Function setKinematicEquation();
};

#endif