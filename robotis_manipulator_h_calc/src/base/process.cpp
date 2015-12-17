/*
 * process.cpp
 *
 *  Created on: Jul 15, 2015
 *      Author: sch
 */

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>

#include <control_msgs/JointControllerState.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <math.h>
#include <vector>
#include <Eigen/Dense>

#include "../../include/state.h"

extern CalcState ManipulatorH;

extern std::string joint_name[ MAX_JOINT_ID + 1 ];

extern moveit_msgs::DisplayTrajectory moveit_msg;

extern bool task_20m_running ;

extern pthread_t tra_inter_20ms;

void* moveit_trajectory_proc(void* arg)
{
	/*---------- get planned trajectory from moveit ----------*/

	std::vector<double> via_time;

	for ( int _tra_index = 0; _tra_index < moveit_msg.trajectory.size(); _tra_index++ )
	{
        ManipulatorH.points = moveit_msg.trajectory[ _tra_index ].joint_trajectory.points.size();

        ManipulatorH.display_planned_path_positions.resize	  ( ManipulatorH.points , MAX_JOINT_ID + 1 );
        ManipulatorH.display_planned_path_velocities.resize	  ( ManipulatorH.points , MAX_JOINT_ID + 1 );
        ManipulatorH.display_planned_path_accelerations.resize( ManipulatorH.points , MAX_JOINT_ID + 1 );

		for ( int _point_index = 0; _point_index < moveit_msg.trajectory[ _tra_index ].joint_trajectory.points.size(); _point_index++ )
		{
            ManipulatorH.time_from_start = moveit_msg.trajectory[ _tra_index ].joint_trajectory.points[_point_index].time_from_start;
            via_time.push_back( ManipulatorH.time_from_start.toSec() );

			for ( int _joint_index = 0; _joint_index < moveit_msg.trajectory[ _tra_index ].joint_trajectory.joint_names.size(); _joint_index++ )
			{
				std::string _joint_name 	= moveit_msg.trajectory[ _tra_index ].joint_trajectory.joint_names[ _joint_index ];

				double _joint_position 		= moveit_msg.trajectory[ _tra_index ].joint_trajectory.points[_point_index].positions	 [ _joint_index ];
				double _joint_velocity 	   	= moveit_msg.trajectory[ _tra_index ].joint_trajectory.points[_point_index].velocities	 [ _joint_index ];
				double _joint_acceleration 	= moveit_msg.trajectory[ _tra_index ].joint_trajectory.points[_point_index].accelerations[ _joint_index ];

                for( int _name_index = 1; _name_index <= MAX_JOINT_ID - 1; _name_index++ )
				{
					if( joint_name[ _name_index ] == _joint_name )
					{
                        ManipulatorH.display_planned_path_positions.coeffRef	( _point_index , _name_index ) = _joint_position;
                        ManipulatorH.display_planned_path_velocities.coeffRef	( _point_index , _name_index ) = _joint_velocity;
                        ManipulatorH.display_planned_path_accelerations.coeffRef( _point_index , _name_index ) = _joint_acceleration;
						break;
					}
				}
			}
		}
	}

    ManipulatorH.all_time_steps = ManipulatorH.points;
    ROS_INFO("all_time_steps = %d", ManipulatorH.all_time_steps );

    ManipulatorH.mov_time = ManipulatorH.time_from_start.toSec();
    ROS_INFO("mov_time = %f", ManipulatorH.mov_time);

}
