#include "ros/ros.h"
#include "dynamixel_control/SpeedCtrl.h"
#include "dynamixel_control/SpeedWheelCtrl.h"
#include "dynamixel_control/PositionCtrl.h"
#include "dynamixel_control/GetIDs.h"
#include "dynamixel_control/GetActuatorsPositions.h"

#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"simple_walk");
	ros::NodeHandle n("simple_walk");

	bool move_backwards = false;

	// pulsation of the walking movement
	double pulsation;
	n.param<double>("pulsation", pulsation, 1.0);
	// Speed for the wheels
	int wheel_speed;
	n.param<int>("wheel_speed", wheel_speed, 50);

	// amplitudes of the oscilators
	int amp0, amp1;
	n.param<int>("amplitude_0", amp0, 140);
	n.param<int>("amplitude_1", amp1, 300);

	ROS_INFO_STREAM("Pulsation is set to " << pulsation);
	ROS_INFO_STREAM("Wheels speed is set to " << wheel_speed);

	// Declare to publish on two topics
	ros::Publisher position_pub = n.advertise<dynamixel_control::PositionCtrl>("target_positions",10);
	ros::Publisher speed_pub = n.advertise<dynamixel_control::SpeedCtrl>("target_speeds",10);
	ros::Publisher wheel_speed_pub = n.advertise<dynamixel_control::SpeedWheelCtrl>("target_wheel_speeds",10);

	// Rate at which the main loop will be executed
	ros::Rate loop_rate(100); // 100 Hz

	dynamixel_control::PositionCtrl pos_msg;
	dynamixel_control::SpeedCtrl speed_msg;
	dynamixel_control::SpeedWheelCtrl wheel_speed_msg;
	// will contain the ids of all connected actuators
	std::vector<unsigned char> ids;

	// Client to get the ids of the connected actuators
	ros::ServiceClient ids_client = n.serviceClient<dynamixel_control::GetIDs>("/dynamixel_control/getids");
	dynamixel_control::GetIDs ids_srv;

	// Get the list of joint mode and wheel mode actoators for this robot
	std::vector<int> pos_ctrl_ids_int;
	std::vector<int> speed_ctrl_ids_int;
	n.getParam("actuator_ids/joint_mode", pos_ctrl_ids_int);
	n.getParam("actuator_ids/wheelmode", speed_ctrl_ids_int);

	// Get the configuration parameters for each actuator
	std::vector<bool> invert_position;
	n.getParam("invert_position", invert_position);
	std::vector<int> associated_oscillator;
	n.getParam("associated_oscillator", associated_oscillator);

	// ids of actuators controlled in position (joint mode)
	std::vector<unsigned char> pos_ctrl_ids(pos_ctrl_ids_int.begin(), pos_ctrl_ids_int.end());
	// ids of actuators controlled in speed (wheel mode)
	std::vector<unsigned char> speed_ctrl_ids(speed_ctrl_ids_int.begin(), speed_ctrl_ids_int.end());

	std::vector<int> pos;

	// Ask which dynamixels are connected
	if (ids_client.call(ids_srv))
	{
		ids = ids_srv.response.ids;
	}
	else
	{
		ROS_ERROR("Failed to call service dynamixel_control/getids");
		return 1;
	}

	bool missing_id = false;
	for (unsigned char current_id : pos_ctrl_ids)
	{
		if (std::find(ids.begin(),ids.end(),current_id) == ids.end())
		{
			ROS_ERROR_STREAM("Dynamixel of id " << (int)current_id << " (joint mode) not found on the robot");
			missing_id = true;
		}
	}
	for (unsigned char current_id : speed_ctrl_ids)
	{
		if (std::find(ids.begin(),ids.end(),current_id) == ids.end())
		{
			ROS_ERROR_STREAM("Dynamixel of id " << (int)current_id << " (wheel mode) not found on the robot");
			missing_id = true;
		}
	}
	// exit the program if one or more actuator was not found
	if (missing_id)
		return 1;

	// Show the lists of actuators we are controling
	std::stringstream list_of_ids;
	for(int i = 0; i < speed_ctrl_ids.size(); i++)
		list_of_ids << (int)speed_ctrl_ids[i] << " ";
	ROS_INFO_STREAM("Speed controlled ids : " << list_of_ids.str());
	for(int i = 0; i < pos_ctrl_ids.size(); i++)
		list_of_ids << (int)pos_ctrl_ids[i] << " ";
	ROS_INFO_STREAM("Position controlled ids : " << list_of_ids.str());

	// Client to get the current angular positions of each actuator
	ros::ServiceClient pos_client = n.serviceClient<dynamixel_control::GetActuatorsPositions>("/dynamixel_control/getpositions");
	dynamixel_control::GetActuatorsPositions pos_srv;

	if (pos_client.call(pos_srv))
	{
		pos = pos_srv.response.positions;
	}
	else
	{
		ROS_ERROR("Failed to call service dynamixel_control/getpositions");
		return -1;
	}
	// FIXME: do we even use the positions we get here ?

	std::vector<int> reference_pose;
	n.getParam("reference_pose", reference_pose);

	pos.resize(pos_ctrl_ids.size());
	for (unsigned char id=0; id<pos_ctrl_ids.size(); id++)
	{
		pos[id] = reference_pose[id];
	}

	// Lower motors speeds for initial positionning
	speed_msg.ids = pos_ctrl_ids;
	speed_msg.speeds.clear();
	for(int i = 0; i < speed_msg.ids.size(); i++)
	{
		speed_msg.speeds.push_back(100);
	}
	speed_pub.publish(speed_msg);

	char c;
	std::cout << "Press any key..." << std::endl;
	std::cin >> c;

	// Move the joints to initial position and stop wheels

	pos_msg.ids = pos_ctrl_ids;
	pos_msg.positions = pos;
	position_pub.publish(pos_msg);

	wheel_speed_msg.ids.clear();
	wheel_speed_msg.ids = speed_ctrl_ids;
	wheel_speed_msg.directions.clear();
	wheel_speed_msg.speeds.clear();
	for(int i = 0; i < wheel_speed_msg.ids.size(); i++)
	{
		wheel_speed_msg.directions.push_back(false);
		wheel_speed_msg.speeds.push_back(0);
	}
	wheel_speed_pub.publish(wheel_speed_msg);

	std::cout << "Press any key to start marching toward the enemy..." << std::endl;
	std::cin >> c;

	double begin = ros::Time::now().toSec();
	bool first = true;

	// Start walking
	while(ros::ok())
	{
		double elapsed = ros::Time::now().toSec() - begin;

		// oscillate around the initial position
		int sin0 = (int)(amp0*sin(pulsation*elapsed));
		int sin1 = -(int)(amp1*cos(pulsation*elapsed));

		// phase shift the end of the limbs if we want to move backwards
		if (move_backwards)
			sin1 = -sin1;

		for (unsigned char id=0; id<pos_ctrl_ids.size(); id++)
		{
			int increment;
			switch (associated_oscillator[id])
			{
				case 0:
					increment = sin0;
					break;
				case 1:
					increment = sin1;
					break;
				default:
					ROS_ERROR_STREAM("Actuator " << (int)id <<
						" asks for unexisting oscillator " << associated_oscillator[id]);
					increment = 0;
			}
			increment = (invert_position[id] ? -increment : increment);
			pos_msg.positions[id] = pos[id] + increment;
		}

		position_pub.publish(pos_msg);

		if(first)
		{
			// Start wheels movement
			ROS_INFO_STREAM("Now starting wheel movements");
			wheel_speed_msg.ids.clear();
			wheel_speed_msg.ids = speed_ctrl_ids;
			wheel_speed_msg.directions.clear();
			wheel_speed_msg.speeds.clear();
			for(int i = 0; i < wheel_speed_msg.ids.size(); i++)
			{
				wheel_speed_msg.directions.push_back(i>=3);
				wheel_speed_msg.speeds.push_back(wheel_speed);
			}
			wheel_speed_pub.publish(wheel_speed_msg);
			ros::Duration(0.5).sleep();

			// Restore motors max speeds
			ROS_INFO_STREAM("Restoring motors max speeds");
			speed_msg.ids.clear();
			speed_msg.ids = pos_ctrl_ids;
			speed_msg.speeds.clear();
			for(int i = 0; i < speed_msg.ids.size(); i++)
			{
				speed_msg.speeds.push_back(0);
			}
			speed_pub.publish(speed_msg);
			ros::Duration(0.5).sleep();

			first = false;
			begin = ros::Time::now().toSec(); // reset time (to not mess with the sines)
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
