#ifndef NMPC_CTR_H_
#define NMPC_CTR_H_

#include <casadi/casadi.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>

class NMPC
{
private:
    /* data */
    // MPC参数
    int m_predict_step; //一次采样间隔预测的步数
    float m_sample_time; //采样时间
    int m_predict_stage; //存储第几次调用nmpc
    int n_states; //共有n个状态-----3
    int n_controls; //共有n个控制输入-----2
    std::vector<Eigen::Matrix<float,3,1>> m_predict_trajectory; //预测轨迹 
    std::vector<float> m_initial_guess; //初始猜测解
    Eigen::Matrix<float,3,1> m_goal_states;
    //符号定义
    casadi::Function m_solver; //求解器
    casadi::Function m_predict_fun; //预测函数
    std::map<std::string, casadi::DM> m_res; //求解结果
    std::map<std::string, casadi::DM> m_args; //求解参数
    //求解得到的控制量
    Eigen::Matrix<float, 2, 1> m_control_command; 
    
 
public:
    NMPC(int predict_step, float sample_time);
    ~NMPC();
    //定义求解器
    void set_my_nmpc_solver();
    //获取目标点
    void set_goal_states(Eigen::Matrix<float,3,1> goal_states);
    //优化求解
    void opti_solution(Eigen::Matrix<float,3,1> current_states);
    //获取最优控制向量
    Eigen::Matrix<float,2,1> get_controls();
    //获取预测轨迹
    std::vector<Eigen::Matrix<float,3,1>> get_predict_trajectory();

};



#endif