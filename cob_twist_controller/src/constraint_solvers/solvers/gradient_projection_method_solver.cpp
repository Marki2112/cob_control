/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: March, 2015
 *
 * \brief
 *   This header contains the description of the unconstraint solver
 *   Implements methods from constraint_solver_base
 *
 ****************************************************************/
#include "cob_twist_controller/constraint_solvers/solvers/gradient_projection_method_solver.h"

#include <cmath>

#include "cob_twist_controller/constraints/self_motion_magnitude.h"

/**
 * Solve the inverse differential kinematics equation by using GPM.
 * In addtion to the partial solution q_dot = J^+ * v the homogeneous solution (I - J^+ * J) q_dot_0 is calculated.
 * The q_dot_0 results from the sum of the constraint cost function gradients. The terms of the sum are weighted with a factor k_H separately.
 */
Eigen::MatrixXd GradientProjectionMethodSolver::solve(const Vector6d_t& inCartVelocities,
                                                      const JointStates& joint_states)
{
    //ToDo: wrong usage of wording (particular solution vs. homogeneous solution)?
    //jpi * v -> homogeneous_solution
    //P*q_0 -> particular_solution 
    
    Eigen::MatrixXd damped_jpi = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd jpi = pinv_calc_.calculate(this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jpi.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jpi * this->jacobian_data_;
    Eigen::MatrixXd particular_solution = damped_jpi * inCartVelocities;
    Eigen::MatrixXd homogeneous_solution = Eigen::MatrixXd::Zero(particular_solution.rows(), particular_solution.cols());
    KDL::JntArrayVel predict_jnts_vel(joint_states.current_q_.rows());

    // ROS_INFO_STREAM("===== task output =======");
    for (std::set<ConstraintBase_t>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        // ROS_INFO_STREAM("task id: " << (*it)->getTaskId());
        (*it)->update(joint_states, predict_jnts_vel, this->jacobian_data_);
        Eigen::VectorXd q_dot_0 = (*it)->getPartialValues();
        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
        double activation_gain = (*it)->getActivationGain(); // contribution of the homo. solution to the part. solution
        double constraint_k_H = (*it)->getSelfMotionMagnitude(particular_solution, tmpHomogeneousSolution); // gain of homogenous solution (if active)
        homogeneous_solution += (constraint_k_H * activation_gain * tmpHomogeneousSolution);
    }

    Eigen::MatrixXd qdots_out = particular_solution + this->params_.k_H * homogeneous_solution; // weighting with k_H is done in loop
    
    //std::stringstream ss_part;
    //ss_part << "particular_solution: ";
    //for(unsigned int i=0; i<particular_solution.rows(); i++)
    //{   ss_part << particular_solution(i,0) << " , ";    }
    //ROS_INFO_STREAM(ss_part.str());
    //std::stringstream ss_hom;
    //ss_hom << "homogeneous_solution: ";
    //for(unsigned int i=0; i<homogeneous_solution.rows(); i++)
    //{   ss_hom << homogeneous_solution(i,0) << " , ";    }
    //ROS_INFO_STREAM(ss_hom.str());
    
    return qdots_out;
}
