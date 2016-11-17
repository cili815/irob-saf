/*
 * dvrk_move_test.cpp
 *
 *  Created on: 2016. okt. 27.
 *      Author: tamas
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
#include "dvrk/trajectory_factory.hpp"




int main(int argc, char **argv)
{
	// Check command line arguments
	if (argc < 4) 
	{
		std::cout 
			<< "Use with params: PSM1/PSM2; init_pos/no_init_pos; filename" 
			<< std::endl;
		return 1;
	}
	
	// Initialize ros node
    ros::init(argc, argv, "irob_dvrk_play_logged_trajectory");
    ros::NodeHandle nh;
    
	// Robot control
	dvrk::Trajectory<double>* to_enable_cartesian;
	dvrk::Trajectory<dvrk::Pose>* to_start;
  	try {
    	dvrk::Arm psm(nh, dvrk::ArmTypes::typeForString(argv[1]),
    	 dvrk::Arm::ACTIVE);
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
			to_enable_cartesian = 
   			dvrk::TrajectoryFactory::linearTrajectoryWithAcc(
   				psm.getJointStateCurrent(init_joint_idx), 
   				0.07, 0.02, 1.0, tr.dt);
			psm.playTrajectory(init_joint_idx, *to_enable_cartesian);
   	 
   			
   		}

		// Go to the start point of loaded trajectory
		ROS_INFO("Going to start point of loaded trajectory...");
		psm.setRobotState(dvrk::Arm::STATE_POSITION_CARTESIAN);
		to_start = 
   			dvrk::TrajectoryFactory::linearTrajectoryWithAcc(
    		psm.getPoseCurrent(), 
			tr[0],
			0.01,0.2, tr.dt); 
		
		psm.playTrajectory(*to_start);
   		ros::Duration(0.5).sleep();
   			
	
		// Play loaded trajectory
		ROS_INFO("Playing loaded trajectory...");
    	psm.playTrajectory(tr);
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    
    }catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
  	
  	// Exit
  	delete(to_enable_cartesian);
  	delete(to_start);
  	
    ros::shutdown();
	return 0;
}




