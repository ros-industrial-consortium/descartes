#ifndef JACO3_IK_H_INCLUDED
#define JACO3_IK_H_INCLUDED

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>

namespace jaco3_kinematics {

    const double PI = M_PI;
    const double d1 = 0.28481;
    const double d2 = 0.01176;
    const double d3 = 0.42076;
    const double d4 = 0.01276;
    const double d5 = 0.31436;
    const double d6 = 0.0;
    const double d7 = 0.16546;
    const double d3_sq = pow(d3, 2);
    const double d4_sq = pow(d4, 2);
    const double d5_sq = pow(d5, 2);
    const double norm_d4d5 = sqrt(d4_sq + d5_sq);

    // @param q     The 7 joint values
    // @param H_0_i The 4x4 homogeneous transformation describing transformation from base frame i, row-major
    void forward_kinematics(const double* q, Eigen::Matrix4d& H_0_1, Eigen::Matrix4d& H_0_2,
                            Eigen::Matrix4d& H_0_3, Eigen::Matrix4d& H_0_4, Eigen::Matrix4d& H_0_5,
                            Eigen::Matrix4d& H_0_6, Eigen::Matrix4d& H_0_7);

    // @param T                     desired pose in homogeneous format
    // @param qs                    A 16 by 7 array containing angles that would get the arm to T
    // @param redundant_parameter   Value of the redundant parameter (null space)
    // @return                      number of solutions (0 -> 16)
    int ik_with_redundant_param(Eigen::Matrix4d& T, Eigen::Matrix<double, 16, 7, Eigen::RowMajor>& qs, const double redundant_parameter);

    // @param a, alpha, d, theta    DH parameters
    // @param A                     resulting homogeneous transformation matrix
    void dh_matrix(const double a, const double alpha, const double d, const double theta, Eigen::Matrix4d& A);

}

#endif // JACO3_IK_H_INCLUDED
