/*
 * calc_test.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <pthread.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <robotis_controller/ControlTorque.h>
#include <robotis_controller/PublishPosition.h>

#include "../include/state.h"

CalcState ManipulatorH = CalcState();

pthread_mutex_t mutex_flag  = PTHREAD_MUTEX_INITIALIZER;

std::string joint_name [ MAX_JOINT_ID + 1 ] =
    { "None", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

ros::Publisher joint_states_pub;
ros::Publisher joint_pub [ MAX_JOINT_ID + 1 ];

moveit_msgs::DisplayTrajectory moveit_msg;
geometry_msgs::Pose ik_msg;

pthread_t	thread_20ms;
bool 		task_20m_running = false;
bool        gazebo_sim = false;

void* thread_20ms_proc( void* arg )
{
    double temp_position[ MAX_JOINT_ID + 1 ];

	ros::Rate loop_rate( 20 );

    static robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    static robot_model::RobotModelPtr 			kinematic_model		=	robot_model_loader.getModel();

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    static robot_state::RobotStatePtr 			kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    static const robot_state::JointModelGroup* 	joint_model_group 	= 	kinematic_model->getJointModelGroup("arm");
    static const std::vector<std::string> 		&joint_names 		= 	joint_model_group->getJointModelNames();

    planning_scene::PlanningScene   planning_scene(kinematic_model);

    collision_detection::CollisionRequest   collision_request;
    collision_detection::CollisionResult    collision_result;

    std::vector<double> _joint_values, joint_values ;

	while( ros::ok() )
	{
        kinematic_state->copyJointGroupPositions( joint_model_group , _joint_values );
        kinematic_state->copyJointGroupPositions( joint_model_group ,  joint_values );

        for ( int id = 1; id <= MAX_JOINT_ID; id++ )
        {
            _joint_values[ id - 1 ]	=	JointState::m_goal_joint_state[ id ].m_position;
            joint_values[ id - 1 ]	=	JointState::m_goal_joint_state[ id ].m_position;
        }

		/*---------- write goal position ----------*/

        if ( task_20m_running == true )
        {
            // send joint trajectory
        	for ( int id = 1; id <= MAX_JOINT_ID; id++ )
            {
                JointState::m_goal_joint_state[ id ].m_position = ManipulatorH.calc_tra.coeff( ManipulatorH.cnt , id );
                joint_values[ id - 1 ] = JointState::m_goal_joint_state[ id ].m_position;
            }
        }

        /*----- check self-collision -----*/

        robot_state::RobotState& collision_state = planning_scene.getCurrentStateNonConst();
        const robot_model::JointModelGroup* collision_model_group = collision_state.getJointModelGroup("arm");

        collision_request.group_name = "arm";

        collision_state.setJointGroupPositions( collision_model_group , joint_values );
        planning_scene.checkSelfCollision( collision_request , collision_result );

        if ( collision_result.collision == true )
        {
            ROS_INFO_STREAM( "Next state will be "
                             << ( collision_result.collision ? "in" : "not in" )
                             << " self collision" );

            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                temp_position[ id ] = _joint_values[ id - 1 ];

            task_20m_running = false;
            ManipulatorH.cnt = 0;
        }
        else
        {
            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                temp_position[ id ] = joint_values[ id - 1 ];

        }

        collision_result.clear();

		/*---------- publish goal joint state (for real robot) ----------*/

        if ( gazebo_sim == false )
        {
            sensor_msgs::JointState joint_msg;
            joint_msg.header.stamp = ros::Time::now();

            pthread_mutex_lock( &mutex_flag );
            for( int id = 1; id <= MAX_JOINT_ID; id++ )
            {
                joint_msg.name.push_back( joint_name[ id ] );
                joint_msg.position.push_back( temp_position[ id ] );
            }
            pthread_mutex_unlock( &mutex_flag );

            joint_states_pub.publish( joint_msg ); // publish joint state
        }


		/*---------- publish joint command (for gazebo) ----------*/

        if ( gazebo_sim == true )
        {
            std_msgs::Float64 msg;

            pthread_mutex_lock( &mutex_flag );
            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
            {
                msg.data = temp_position[ id ];
                joint_pub[ id ].publish( msg ); // publish joint commands
            }
            pthread_mutex_unlock( &mutex_flag );
        }

		/*---------- initialize count number ----------*/

		loop_rate.sleep();
        ManipulatorH.cnt++;

        if ( ManipulatorH.cnt >= ManipulatorH.all_time_steps && task_20m_running == true )
		{
            ROS_INFO("[end] send trajectory");

            task_20m_running = false;
            ManipulatorH.cnt = 0;

            /*----- Forward Kinematics -----*/

            kinematic_state->setJointGroupPositions( joint_model_group , joint_values );

            const Eigen::Affine3d &curr_end_effector_state	=	kinematic_state->getGlobalLinkTransform("end_effector");

            ROS_INFO("----- Current Joint Values -----");
            for( std::size_t id = 1; id <= MAX_JOINT_ID; id++ )
                ROS_INFO("%s: %f", joint_names[ id ].c_str(), joint_values[ id - 1 ]);

            ROS_INFO("----- Current End Effector's State -----");
            ROS_INFO_STREAM( "Current Translation: \n" << curr_end_effector_state.translation() );
            ROS_INFO_STREAM( "Current Rotation: \n" << curr_end_effector_state.rotation() );
		}
	}

	return 0;
}

int main( int argc , char **argv )
{
	ros::init( argc , argv , "robotis_state_publisher" );
    ros::NodeHandle nh("~");

    ROS_INFO("----------------------------------------------------");

    nh.getParam("sim", gazebo_sim);

    if ( gazebo_sim == true )
        ROS_INFO("Simuation Mode");
    else
        ROS_INFO("Robot Demo Mode");

    ROS_INFO("----------------------------------------------------");

    ros::Publisher ct_pub, pp_pub;

    if ( gazebo_sim == false )
    {
        ct_pub = nh.advertise<robotis_controller::ControlTorque>("/control_torque", 10);
        pp_pub = nh.advertise<robotis_controller::PublishPosition>("/publish_position", 10);
        sleep(3.0);

        //to publish all joint position
        robotis_controller::PublishPosition pp;

        for(int i = 1; i <= MAX_JOINT_ID; i++)
        {
            ROS_INFO("joint_name[%d] = %s", i, joint_name[i].c_str());
            pp.name.push_back(joint_name[i]);
            pp.publish.push_back(true);
        }
        pp_pub.publish(pp);
    }

    ros::Subscriber joint_curr_states_sub, joint_real_states_sub;

    if ( gazebo_sim == true )
        joint_curr_states_sub = nh.subscribe("/robotis_manipulator_h/joint_states", 5, joint_curr_states_callback);
    else
        joint_real_states_sub = nh.subscribe("/robotis_manipulator_h/real_joint_states", 5, joint_real_states_callback);

    while ( ManipulatorH.get_ini_pose == false )
        ros::spinOnce();

    ROS_INFO("Set Initial Pose");

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        if ( gazebo_sim == true )
            JointState::m_goal_joint_state[ id ].m_position = JointState::m_curr_joint_state[ id ].m_position;
        else
            JointState::m_goal_joint_state[ id ].m_position = JointState::m_real_joint_state[ id ].m_position;
    }

    /*---------- publisher ----------*/

    if ( gazebo_sim == true )
    {
        // for gazebo
        joint_pub[1] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint1_position_controller/command", 1);
        joint_pub[2] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint2_position_controller/command", 1);
        joint_pub[3] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint3_position_controller/command", 1);
        joint_pub[4] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint4_position_controller/command", 1);
        joint_pub[5] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint5_position_controller/command", 1);
        joint_pub[6] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint6_position_controller/command", 1);
    }
    else
    {
        // for real robot
        joint_states_pub = nh.advertise<sensor_msgs::JointState>("/robotis_manipulator_h/controller_joint_states", 1);

        // torque on
        robotis_controller::ControlTorque ct;
        ct.name.push_back("joint1");
        ct.enable.push_back(true);
        ct.name.push_back("joint2");
        ct.enable.push_back(true);
        ct.name.push_back("joint3");
        ct.enable.push_back(true);
        ct.name.push_back("joint4");
        ct.enable.push_back(true);
        ct.name.push_back("joint5");
        ct.enable.push_back(true);
        ct.name.push_back("joint6");
        ct.enable.push_back(true);
        ct_pub.publish(ct);
    }

    /* moveit subscribe */
    ros::Subscriber joint_fake_states_sub 	 = nh.subscribe("/move_group/fake_controller_joint_states", 5, joint_fake_states_callback);
    ros::Subscriber display_planned_path_sub = nh.subscribe("/move_group/display_planned_path", 		5, display_planned_path_callback);

    /*---------- 20ms thread ----------*/

    pthread_create(&thread_20ms, NULL, thread_20ms_proc, NULL);

	ros::spin();

    return 0;
}
