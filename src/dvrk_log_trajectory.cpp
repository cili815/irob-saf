/*
 *  dvrk_log_trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-26
 *  
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "dvrk/trajectory_factory.hpp"



int main(int argc, char **argv)
{
	// Check command line arguments
	if (argc < 4) {
		std::cout 
			<< "Use with params: PSM1/PSM2; rate; filename" 
			<< std::endl;
		return 1;
	}
	
	
    std::istringstream ss1(argv[2]);
	int rate_command;
	ss1 >> rate_command;	
		
	double dt = 1.0/ rate_command;
	
	
	// Initialize node
    ros::init(argc, argv, "irob_dvrk_log_trajectory");
    ros::NodeHandle nh;

    dvrk::PSM psm(nh, dvrk::ArmTypes::typeForString(argv[1]), dvrk::PSM::PASSIVE);
    
    // Record trajectory
	dvrk::Trajectory<dvrk::Pose> tr_to_log(dt);
	ROS_INFO_STREAM("Start recording trajectory...");
	psm.recordTrajectory(tr_to_log);
	std::cout << std::endl << "Record stopped" << std::endl;
	
	try	{
		tr_to_log.writeToFile(argv[3]);
	} catch (const std::exception& e) {
  		std::cerr << e.what() << std::endl;
  	}
   	
   	std::cout << std::endl << "Program finished succesfully, shutting down ..."
   		 << std::endl;
   	
   	ros::shutdown();
	return 0;
}






