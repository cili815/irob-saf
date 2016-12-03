/*
 *  dvrk_play_logged_trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-27
 *  
 */

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "dvrk/trajectory_factory.hpp"




int main(int argc, char **argv)
{
	// Check command line arguments
	if (argc < 5) 
	{
		std::cout 
			<< "Use with params: PSM1/PSM2; init_pos/no_init_pos; filename; speed" 
			<< std::endl;
		return 1;
	}
	
	std::istringstream ss2(argv[4]);
	double speed;
	ss2 >> speed;	
		
	
	
	// Initialize ros node
    ros::init(argc, argv, "irob_dvrk_play_logged_trajectory");
    ros::NodeHandle nh;
    
	// Robot control
	
	
  	try {
    	dvrk::PSM psm(nh, dvrk::ArmTypes::typeForString(argv[1]),
    	 dvrk::PSM::ACTIVE);
    	//psm.home();
    
    	// Load trajectory from file
		dvrk::Trajectory<dvrk::Pose> tr(argv[3]);
    	ROS_INFO_STREAM(
    	"Trajectory of "<< tr.size() << " points with "
    					<< 1.0/tr.dt << " sample rate succesfully loaded from "
    					<< argv[3]);
    	
    	// Init position if necessary
    	if (std::string(argv[2]) == "init_pos")
		{
			ROS_INFO("Going to init position...");
			psm.setRobotState(dvrk::Arm::STATE_POSITION_JOINT);
			int init_joint_idx = 2;
			dvrk::Trajectory<double> to_enable_cartesian = 
   			dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   					psm.getJointStateCurrent(init_joint_idx), 
   					0.07, 1.0, 0.1, tr.dt);
			psm.playTrajectory(init_joint_idx, to_enable_cartesian);
   	 
   			
   		}

		// Go to the start point of loaded trajectory
		ROS_INFO("Going to start point of loaded trajectory...");
		psm.setRobotState(dvrk::Arm::STATE_POSITION_CARTESIAN);
		dvrk::Trajectory<dvrk::Pose> to_start = 
   			dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
    				psm.getPoseCurrent(), 
					tr[0],
					2.0, 0.2, tr.dt); 
					
		tr.dt /= speed;
		psm.playTrajectory(to_start);
   		ros::Duration(0.5).sleep();
   			
	
		// Play loaded trajectory
		ROS_INFO("Playing loaded trajectory...");

   		ROS_INFO_STREAM("Speed:\t"<< speed);
    	psm.playTrajectory(tr);
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    
    }catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
  	
    ros::shutdown();
	return 0;
}




