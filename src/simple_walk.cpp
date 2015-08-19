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

	// pulsation of the walking movement
	double pulsation;
	n.param<double>("pulsation",pulsation,1.0);
	// Speed for the wheels
	int wheel_speed;
	n.param<int>("wheel_speed",wheel_speed,50);

	// amplitudes of the oscilators
	int amp0 = 140, amp1=300, amp3=40;

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

	// ids of actuators controlled in position (joint mode)
	std::vector<unsigned char> pos_ctrl_ids;
	// ids of actuators controlled in speed (wheel mode)
	std::vector<unsigned char> speed_ctrl_ids;

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

	// FIXME: use configuration file instead
	for(int j = 0; j < 3; j++) // originally j < 5
	{
		for(int i = 0; i < 6; i++)
		{
			if(std::find(ids.begin(),ids.end(),j*10+i+1) == ids.end())
			{
				ROS_ERROR("Dynamixel of id %d is not found",j*10+i+1);
				return 1;
			}
			else
			{
				if(j < 4)
					pos_ctrl_ids.push_back(j*10+i+1);
				else if(j == 4)
				{
					speed_ctrl_ids.push_back(j*10+i+1);
				}
			}
		}
	}

	std::stringstream list_of_ids;
	for(int i = 0; i < speed_ctrl_ids.size(); i++)
		list_of_ids << (int)speed_ctrl_ids[i] << " ";
	ROS_INFO_STREAM("Speed controlled ids : " << list_of_ids.str());

	for(int i = 0; i < pos_ctrl_ids.size(); i++)
		list_of_ids << (int)pos_ctrl_ids[i] << " ";
	ROS_INFO_STREAM("Position controlled ids : " << list_of_ids.str());

	pos[0] = 1498; // id 1
	pos[1] = 2048; // id 2
	pos[2] = 2598; // id 3
	pos[3] = 1498; // id 4
	pos[4] = 2048; // id 5
	pos[5] = 2598; // id 6

	int initial_offset = -512;//500;
	for(int i = 0; i < 6; i++)
	{
		// pos[6+i] = 2048 + (i < 3 ? initial_offset : -initial_offset); // id 11 to 16
		// pos[12+i] = 2048 + (i < 3 ? initial_offset : -initial_offset); // id 21 to 26
		// pos[18+i] = 512; // id 31 to 36
		// For Pexod
		pos[6+i] = 2048 + initial_offset; // id 11 to 16
		pos[12+i] = 2048 - initial_offset; // id 21 to 26
		pos[18+i] = 512; // id 31 to 36
	}
	pos.resize(pos_ctrl_ids.size());

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
		int sin1 = (int)(amp1*cos(pulsation*elapsed));
		int sin3 = (int)(amp3*sin(pulsation*elapsed));

		pos_msg.positions[0] = pos[0] + sin0;
		pos_msg.positions[1] = pos[1] - sin0;
		pos_msg.positions[2] = pos[2] + sin0;
		pos_msg.positions[3] = pos[3] + sin0;
		pos_msg.positions[4] = pos[4] - sin0;
		pos_msg.positions[5] = pos[5] + sin0;

		pos_msg.positions[6] = pos[6] + sin1; // 11
		pos_msg.positions[7] = pos[7] - sin1; // 12
		pos_msg.positions[8] = pos[8] + sin1; // 13
		pos_msg.positions[9] = pos[9] - sin1; // 14
		pos_msg.positions[10] = pos[10] + sin1; // 15
		pos_msg.positions[11] = pos[11] - sin1; // 16

		pos_msg.positions[12] = pos[12] + sin1; // 21
		pos_msg.positions[13] = pos[13] - sin1; // 22
		pos_msg.positions[14] = pos[14] + sin1; // 23
		pos_msg.positions[15] = pos[15] - sin1; // 24
		pos_msg.positions[16] = pos[16] + sin1; // 25
		pos_msg.positions[17] = pos[17] - sin1; // 26

		pos_msg.positions[18] = pos[18] - sin3;
		pos_msg.positions[19] = pos[19] + sin3;
		pos_msg.positions[20] = pos[20] - sin3;
		pos_msg.positions[21] = pos[21] - sin3;
		pos_msg.positions[22] = pos[22] + sin3;
		pos_msg.positions[23] = pos[23] - sin3;

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
