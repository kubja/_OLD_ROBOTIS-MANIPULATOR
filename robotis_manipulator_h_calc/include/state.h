/*
 * calc_state.h
 *
 *  Created on: Jul 22, 2015
 *      Author: thor
 */

#ifndef CALC_STATE_H_
#define CALC_STATE_H_

#include <ros/ros.h>

#include <Eigen/Dense>

#include "callback.h"
#include "jointstate.h"
#include "process.h"
#include "trajectory.h"

class CalcState
{
public:

	bool get_ini_pose;
    bool solve_fk;
    bool solve_ik;

	int cnt; // counter number
	int all_time_steps; // all time steps of movement time

	double mov_time; // movement time
	double smp_time; // sampling time

	Eigen::MatrixXd calc_tra; // calculated trajectory

	/* ----- for moveit data ----- */

	int points; // planned number of via-points

	ros::Duration time_from_start; // planned movement time

	Eigen::MatrixXd display_planned_path_positions; // planned position trajectory
	Eigen::MatrixXd display_planned_path_velocities; // planned velocity trajectory
	Eigen::MatrixXd display_planned_path_accelerations; // planned acceleration trajectory

    /* ----- for inverse kinematics -----*/

    Eigen::MatrixXd curr_task_p, goal_task_p;
    Eigen::Matrix3d curr_task_R, goal_task_R;

    Eigen::Quaterniond curr_task_QR, goal_task_QR;

    Eigen::MatrixXd task_tra;

	CalcState()
	{
		cnt = 0;
		all_time_steps = 0;

		mov_time = 0.0;
        smp_time = 0.05;

		get_ini_pose = false;
        solve_fk = false;
        solve_ik = false;

		points = 10;

		time_from_start = ros::Duration( 10.0 ); // movement duration

		display_planned_path_positions = Eigen::MatrixXd::Zero( points , MAX_JOINT_ID + 1 ); // positions of planned path
		display_planned_path_velocities = Eigen::MatrixXd::Zero( points , MAX_JOINT_ID + 1 ); // positions of planned path
		display_planned_path_accelerations = Eigen::MatrixXd::Zero( points , MAX_JOINT_ID + 1 ); // positions of planned path

        /* ----- for inverse kinematics ----- */

        curr_task_p =   Eigen::MatrixXd::Zero( 3 , 1 );
        curr_task_R =   Eigen::MatrixXd::Zero( 3 , 3 );

        goal_task_p =   Eigen::MatrixXd::Zero( 3 , 1 );
        goal_task_R =   Eigen::MatrixXd::Zero( 3 , 3 );

        task_tra = Eigen::MatrixXd::Zero( points , 3 );
    }

};


#endif /* CALC_STATE_H_ */
