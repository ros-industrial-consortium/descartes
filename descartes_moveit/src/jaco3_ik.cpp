#include "descartes_moveit/jaco3_ik.h"

//temp imports
//#include <chrono>

namespace jaco3_kinematics {

    void forward_kinematics(const Eigen::VectorXd& q, Eigen::Matrix4d& H_0_1, Eigen::Matrix4d& H_0_2,
                            Eigen::Matrix4d& H_0_3, Eigen::Matrix4d& H_0_4, Eigen::Matrix4d& H_0_5,
                            Eigen::Matrix4d& H_0_6, Eigen::Matrix4d& H_0_7) {
        Eigen::Matrix4d A1;
        Eigen::Matrix4d A2;
        Eigen::Matrix4d A3;
        Eigen::Matrix4d A4;
        Eigen::Matrix4d A5;
        Eigen::Matrix4d A6;
        Eigen::Matrix4d A7;
        dh_matrix(0, PI/2, d1, q(0), A1);
        dh_matrix(0, -PI/2, d2, q(1), A2);
        dh_matrix(0, PI/2, d3, q(2), A3);
        dh_matrix(0, -PI/2, d4, q(3), A4);
        dh_matrix(0, PI/2, d5, q(4), A5);
        dh_matrix(0, -PI/2, d6, q(5), A6);
        dh_matrix(0, 0, d7, q(6), A7);

        H_0_1 = A1;
        H_0_2 = H_0_1 * A2;
        H_0_3 = H_0_2 * A3;
        H_0_4 = H_0_3 * A4;
        H_0_5 = H_0_4 * A5;
        H_0_6 = H_0_5 * A6;
        H_0_7 = H_0_6 * A7;
    }

    int ik_with_redundant_param(Eigen::Matrix4d& T, Eigen::Matrix<double, 16, 7, Eigen::RowMajor>& qs, const double redundant_parameter) {

        // backtrack the wrist position (origin frame 5 and 6) from the end-effector pose (kinematic decoupling)
        Eigen::Vector4d wrist_position(0,0,0,1);
        wrist_position.block<3,1>(0,0) = T.block<3,1>(0,3) + T.block<3,3>(0,0) * Eigen::Vector3d(0,0,-d7);

        // calculate theta1 from redundant parameter. For now just equal
        // two configuration indices for theta1
        const double theta1s[2] = {redundant_parameter, -redundant_parameter};

        int row_index = 0;
        for (int cfg_ind1=0; cfg_ind1<2; cfg_ind1++)
        {
            const double theta1 = theta1s[cfg_ind1];
            // express the desired wrist position in frame 1', frame1 but shifted over to coincide with the origin
            // of frame2
            // construct A1_prime based on theta1
            Eigen::Matrix4d A1;
            dh_matrix(0, PI/2, d1, theta1, A1);
            Eigen::Matrix4d trans_z1 = Eigen::Matrix4d::Identity();
            trans_z1(2,3) = d2;
            Eigen::Matrix4d A1_prime = A1 * trans_z1;
            // wrist position in frame 1'
            Eigen::Vector4d w_prime = A1_prime.inverse() * wrist_position;

            double length_w_prime = w_prime.head(3).norm();

            ////////////////////////// theta4 //////////////////////////
            double theta4_unadjusted_1 = -PI + acos((-pow(length_w_prime,2) + d3_sq + d4_sq + d5_sq) /
                                                    (2 * d3 * norm_d4d5));

            double theta4_1 = acos(norm_d4d5 / d5 * cos(theta4_unadjusted_1));
            double theta4_2 = -theta4_1;
            double theta4_3 = -theta4_1;
            double theta4_4 = theta4_1;

            ////////////////////////// theta2 //////////////////////////
            double theta2_1 = atan2(w_prime(1), w_prime(0)) -
                                asin((d3 + norm_d4d5 * sin(PI/2 + theta4_unadjusted_1)) /
                                        w_prime.head(2).norm());
            double theta2_2 = theta2_1;
            double theta2_3 = 2*atan2(w_prime(1), w_prime(0)) - theta2_1 - PI;
            double theta2_4 = theta2_3;

            ////////////////////////// theta3 //////////////////////////
            Eigen::Matrix4d A2;
            dh_matrix(0, -PI/2, d2, theta2_1, A2);
            Eigen::Matrix4d H_0_2 = A1 * A2;
            Eigen::Vector4d w_prime_prime = H_0_2.inverse() * wrist_position;
            double theta3_1 = PI/2 - atan2(d5*sin(-theta4_1), d4) + atan2(w_prime_prime(1), w_prime_prime(0));
            double theta3_2 = theta3_1 - 2*atan2(d5 * sin(-theta4_2), d4);
            double theta3_3 = -theta3_1;
            double theta3_4 = theta3_3 - 2*atan2(d5*sin(-theta4_4), d4);

            // combine in lists so we can loop
            const double theta2s[4] = {theta2_1, theta2_2, theta2_3, theta2_4};
            const double theta3s[4] = {theta3_1, theta3_2, theta3_3, theta3_4};
            const double theta4s[4] = {theta4_1, theta4_2, theta4_3, theta4_4};

            for (int cfg_ind234=0; cfg_ind234<4; cfg_ind234++)
            {
                ///////////////////////// theta5 //////////////////////////
                dh_matrix(0, -PI/2, d2, theta2s[cfg_ind234], A2);
                Eigen::Matrix4d H_0_2 = A1 * A2;
                Eigen::Matrix4d A3;
                dh_matrix(0, PI/2, d3, theta3s[cfg_ind234], A3);
                Eigen::Matrix4d A4;
                dh_matrix(0, -PI/2, d4, theta4s[cfg_ind234], A4);
                Eigen::Matrix4d H_0_4 = H_0_2 * A3 * A4;
                Eigen::Matrix4d H_4_7 = H_0_4.inverse() * T;
                double theta5_1 = -atan2(H_4_7(1,2), -H_4_7(0,2));
                double theta5_2 = -atan2(H_4_7(1,2), -H_4_7(0,2)) - PI;

                ///////////////////////// theta6 //////////////////////////
                double theta6_1 = atan2(sqrt(pow(H_4_7(0,2), 2) + pow(H_4_7(1,2), 2)), H_4_7(2,2));
                double theta6_2 = -atan2(sqrt(pow(H_4_7(0,2), 2) + pow(H_4_7(1,2), 2)), H_4_7(2,2));

                ///////////////////////// theta7 //////////////////////////
                double theta7_1 = -atan2(H_4_7(2,1), H_4_7(2,0));
                double theta7_2 = -atan2(H_4_7(2,1), H_4_7(2,0)) - PI;

                // assemble and store
                Eigen::VectorXd q1(7);
                Eigen::VectorXd q2(7);
                q1 << theta1, theta2s[cfg_ind234], theta3s[cfg_ind234], theta4s[cfg_ind234], theta5_1, theta6_1, theta7_1;
                q2 << theta1, theta2s[cfg_ind234], theta3s[cfg_ind234], theta4s[cfg_ind234], theta5_2, theta6_2, theta7_2;
                qs.block<1,7>(row_index,0) = q1;
                row_index++;
                qs.block<1,7>(row_index,0) = q2;
                row_index++;
            }
        }

        return 16;

    }

    void dh_matrix(const double a, const double alpha, const double d, const double theta, Eigen::Matrix4d& A) {
        A(0,0) = cos(theta);
        A(0,1) = -sin(theta)*cos(alpha);
        A(0,2) = sin(theta)*sin(alpha);
        A(0,3) = a*cos(theta);
        A(1,0) = sin(theta);
        A(1,1) = cos(theta)*cos(alpha);
        A(1,2) = -cos(theta)*sin(alpha);
        A(1,3) = a*sin(theta);
        A(2,0) = 0;
        A(2,1) = sin(alpha);
        A(2,2) = cos(alpha);
        A(2,3) = d;
        A(3,0) = 0; A(3,1) = 0; A(3,2) = 0; A(3,3) = 1;
    }

}

int main(int argc, char* argv[])
{
    // set initial joint velocities
    Eigen::VectorXd q(7);
    q << 0.34, -0.1, -0.4, -0.674, -0.8, 0.15, 1.8;

    // target pose
    Eigen::Matrix4d H_0_1;
    Eigen::Matrix4d H_0_2;
    Eigen::Matrix4d H_0_3;
    Eigen::Matrix4d H_0_4;
    Eigen::Matrix4d H_0_5;
    Eigen::Matrix4d H_0_6;
    Eigen::Matrix4d H_0_7;
    // forward kinematics to populate target pose using joint values
    jaco3_kinematics::forward_kinematics(q, H_0_1, H_0_2, H_0_3, H_0_4, H_0_5, H_0_6, H_0_7);

    // check that the target pose makes sense
    std::cout << H_0_7 << std::endl;

    // test IK
    Eigen::Matrix<double, 16, 7, Eigen::RowMajor> q_solns;
    int nb_solutions;
    double redundant_param;
    redundant_param = q[0];
    //auto start = std::chrono::high_resolution_clock::now();
    nb_solutions = jaco3_kinematics::ik_with_redundant_param(H_0_7, q_solns, redundant_param);
    //auto stop = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //std::cout << duration.count() << std::endl;

    std::cout << q_solns << std::endl;

    // check that the target pose matches the FK(IK) pose
    for (int i=0; i<nb_solutions; i++)
    {
        Eigen::VectorXd q_soln_i(7);
        q_soln_i = q_solns.row(i);
        jaco3_kinematics::forward_kinematics(q_soln_i, H_0_1, H_0_2, H_0_3, H_0_4, H_0_5, H_0_6, H_0_7);
        std::cout << H_0_7 << std::endl;
    }

    return 0;
}