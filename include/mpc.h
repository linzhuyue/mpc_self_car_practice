//
// Created by yue on 15/3/2022.
//

#ifndef MPC_MPC_H
#define MPC_MPC_H
#include <vector>
#include "../third_party/Eigen-3.3/Eigen/Core"
class mpc{
public:
    mpc();
    virtual ~mpc();
    //根据给定状态和轨迹的系数，返回控制变量(舵角度和速度)
    std::vector<double> Solve(Eigen::VectorXd state,Eigen::VectorXd coeffs);
    std::vector<double> x_pred_vals;
    std::vector<double> y_pred_vals;
    double curr_time=0;
    double object_value_out=0;
};
#endif //MPC_MPC_H
