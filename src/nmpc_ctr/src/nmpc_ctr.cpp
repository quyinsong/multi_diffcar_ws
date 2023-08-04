#include "nmpc_ctr.h"

NMPC::NMPC(int predict_step, float sample_time)
{
    m_predict_step = predict_step;
    m_sample_time = sample_time;
    m_predict_stage = -1;

    n_states = 3;
    n_controls = 2;

    m_goal_states<<0,0,0;

    for(int j=0;j<m_predict_step;j++)
    {
        m_initial_guess.push_back(0);
        m_initial_guess.push_back(0);
    }

    //设置求解器
    set_my_nmpc_solver();

}

NMPC::~NMPC()
{

}

//创建求解器
void NMPC::set_my_nmpc_solver()
{
    //模型建立
    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");
    casadi::SX theta = casadi::SX::sym("theta");

    casadi::SX v = casadi::SX::sym("v");
    casadi::SX omega = casadi::SX::sym("omega"); //推力

    casadi::SX states = casadi::SX::vertcat({x,y,theta});
    casadi::SX controls = casadi::SX::vertcat({v,omega});

    // std::cout<<"n_states: "<< n_states<< endl;
    // std::cout<<"n_controls: "<< n_controls<< endl;

    //运动学模型参数

    //运动学模型
    casadi::SX rhs = casadi::SX::vertcat({
        v*casadi::SX::cos(theta),
        v*casadi::SX::sin(theta),
        omega
    });

    //定义模型函数
    casadi::Function m_f = casadi::Function("f", {states, controls}, {rhs});

    // 求解问题符号表示
    casadi::SX U = casadi::SX::sym("U",n_controls,m_predict_step); //待求解的控制变量
    casadi::SX X = casadi::SX::sym("X",n_states,m_predict_step+1); //系统状态
    
    // //当前运动状态
    casadi::SX current_states = casadi::SX::sym("current_sattes",n_states);

    //优化参数（需要给出当前运动状态和预测视野内的参考轨迹,先测试定点控制,则参考轨迹可设置为期望终点）
    casadi::SX opt_para = casadi::SX::sym("opt_para",2*n_states);

    //优化变量（需要求解的控制序列）
    //这里的 U 需要转置一下
    casadi::SX opt_var = casadi::SX::reshape(U.T(),-1,1);

    //根据上述模型函数向前预测无人艇运动状态
    X(casadi::Slice(),0) = opt_para(casadi::Slice(0,3,1)); //状态初始值

    for(int i=0;i<m_predict_step;i++)
    {
        std::vector<casadi::SX> input_X;
        casadi::SX X_current = X(casadi::Slice(),i);
        casadi::SX U_current = U(casadi::Slice(),i);
        input_X.push_back(X_current);
        input_X.push_back(U_current);
        X(casadi::Slice(),i+1) = m_f(input_X).at(0)*m_sample_time+X_current;
    }

    //控制序列与输出的关系函数（预测函数）
    m_predict_fun = casadi::Function("m_predict_fun",{casadi::SX::reshape(U,-1,1),opt_para},{X});

    //惩罚矩阵
    casadi::SX m_Q = casadi::SX::zeros(3,3);
    casadi::SX m_R = casadi::SX::zeros(2,2);
    m_Q(0,0) = 0.5;
    m_Q(1,1) = 0.5;
    m_Q(2,2) = 0.1;
    m_R(0,0) = 0.5;
    m_R(1,1) = 0.05;
    
    //计算代价函数
    casadi::SX cost_fun = casadi::SX::sym("cost_fun");
    cost_fun = 0;

    int trajectory_points_index = 0;

    for(int k=0;k<m_predict_step;k++)
    {
        casadi::SX states_err = X(casadi::Slice(),k)-opt_para(casadi::Slice(3,6,1));
        casadi::SX controls_err = U(casadi::Slice(),k);
        cost_fun = cost_fun+casadi::SX::mtimes({states_err.T(),m_Q,states_err})+
                            casadi::SX::mtimes({controls_err.T(),m_R,controls_err});
    }

    //构建求解器(暂时不考虑约束)
    casadi::SXDict nlp_prob = {
        {"f", cost_fun},
        {"x", opt_var},
        {"p",opt_para}
    };

    std::string solver_name = "ipopt";
    casadi::Dict nlp_opts;
    nlp_opts["expand"] = true;
    nlp_opts["ipopt.max_iter"] = 5000;
    nlp_opts["ipopt.print_level"] = 0;
    nlp_opts["print_time"] = 0;
    nlp_opts["ipopt.acceptable_tol"] =  1e-6;
    nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;

    m_solver = nlpsol("nlpsol", solver_name, nlp_prob, nlp_opts);

}

//设定目标点
void NMPC::set_goal_states(Eigen::Matrix<float, 3, 1> goal_states)
{
    m_goal_states = goal_states;
}

void NMPC::opti_solution(Eigen::Matrix<float,3,1> current_states)
{
    //设置控制约束
    std::vector<float> lbx;
    std::vector<float> ubx;
    std::vector<float> parameters;
    for (int k = 0; k < m_predict_step; k++)
    {
        lbx.push_back(-0.6);

        ubx.push_back(0.6);
    }
    for (int k = 0; k < m_predict_step; k++)
    {
        lbx.push_back(-M_PI/4);

        ubx.push_back(M_PI/4);
    }
    for(int j=0;j<3;j++)
    {
        parameters.push_back(current_states[j]);
    }
    for(int j=0;j<3;j++)
    {
        parameters.push_back(m_goal_states[j]);
    }
    
    //求解参数设置
    m_args["lbx"] = lbx;
    m_args["ubx"] = ubx;
    m_args["x0"] = m_initial_guess;
    m_args["p"] = parameters;
    //求解
    m_res = m_solver(m_args);

    //获取优化变量
    std::vector<float> res_control_all(m_res.at("x"));

    std::vector<float> res_control_v, res_control_omega;

    res_control_v.assign(res_control_all.begin(), res_control_all.begin() + m_predict_step);
    res_control_omega.assign(res_control_all.begin() + m_predict_step, res_control_all.begin() + 2 * m_predict_step);

    //存储下一时刻最初优化猜测解
    std::vector<float> initial_guess;
    for(int j=0;j<m_predict_step-1;j++)
    {
        initial_guess.push_back(res_control_v.at(j));
    }
    initial_guess.push_back(res_control_v.at(m_predict_step-1));
    for(int j=0;j<m_predict_step-1;j++)
    {
        initial_guess.push_back(res_control_omega.at(j));
    }
    initial_guess.push_back(res_control_omega.at(m_predict_step-1));
    m_initial_guess = initial_guess;

    // 采用求解得到的控制序列的第一组作为当前控制量
    m_control_command << res_control_v.front(), res_control_omega.front();

    //预测轨迹
    // std::vector<casadi::SX> input_X;
    // input_X.push_back(res_control_all);
    // input_X.push_back(parameters);
    // m_predict_trajectory = m_predict_fun(input_X).at(0);
    // m_predict_trajectory = predict_trajectory;
    // std::cout<<"predict_trajectory: "<< predict_trajectory(1,5)<<endl; 

    //手动计算预测轨迹
    std::vector<Eigen::Matrix<float,3,1>> predict_X;

    predict_X.push_back(current_states);
    
    for(int j=0;j<m_predict_step;j++)
    {
        Eigen::Matrix<float,3,1> Next_X;
        Next_X<< m_sample_time*res_control_v.at(j)*std::cos(predict_X.at(j)[2])+predict_X.at(j)[0]
                ,m_sample_time*res_control_v.at(j)*std::sin(predict_X.at(j)[2])+predict_X.at(j)[1]
                ,m_sample_time*res_control_omega.at(j)+predict_X.at(j)[2];
        predict_X.push_back(Next_X);

    }
    
    m_predict_trajectory = predict_X;

}

Eigen::Matrix<float,2,1> NMPC::get_controls()
{

    return m_control_command;
}

std::vector<Eigen::Matrix<float,3,1>> NMPC::get_predict_trajectory()
{
    return m_predict_trajectory;
}