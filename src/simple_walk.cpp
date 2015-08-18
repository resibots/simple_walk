#include "ros/ros.h"
#include "dynamixel_control/SpeedWheelCtrl.h"
#include "dynamixel_control/PositionCtrl.h"
#include "dynamixel_control/GetIDs.h"
#include "dynamixel_control/GetActuatorsPositions.h"

#include <iostream>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"simple_walk");
	ros::NodeHandle n("simple_walk");

	double period;
	n.param<double>("period",period,1.0);
	int wheel_speed;
	n.param<int>("wheel_speed",wheel_speed,50);

	printf("Period is set to %f\r\n",period);
	printf("Wheels speed is set to %d\r\n",wheel_speed);

	ros::Publisher speed_pub = n.advertise<dynamixel_control::SpeedWheelCtrl>("target_speeds",10);
	ros::Publisher position_pub = n.advertise<dynamixel_control::PositionCtrl>("target_positions",10);
	ros::Rate loop_rate(100);

	dynamixel_control::PositionCtrl pos_msg;
	dynamixel_control::SpeedWheelCtrl speed_msg;
	std::vector<unsigned char> ids;

	ros::ServiceClient ids_client = n.serviceClient<dynamixel_control::GetIDs>("/dynamixel_control/getids");
	dynamixel_control::GetIDs ids_srv;

	std::vector<unsigned char> pos_ctrl_ids;
	std::vector<unsigned char> speed_ctrl_ids;
	std::vector<int> pos;

	if (ids_client.call(ids_srv))
	{
		ids = ids_srv.response.ids;
	}
	else
	{
		ROS_ERROR("Failed to call service dynamixel_control/getids");
		return 1;
	}

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

	for(int j = 0; j < 5; j++)
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

	printf("Speed controlled ids : ");
	for(int i = 0; i < speed_ctrl_ids.size(); i++)
		printf("%d ",speed_ctrl_ids[i]);
	printf("\r\n");

	printf("Position controlled ids : ");
	for(int i = 0; i < pos_ctrl_ids.size(); i++)
		printf("%d ",pos_ctrl_ids[i]);
	printf("\r\n");

	pos[0] = 2700; // id 1
	pos[1] = 2048; // id 2
	pos[2] = 1400; // id 3
	pos[3] = 2700; // id 4
	pos[4] = 2048; // id 5
	pos[5] = 1400; // id 6

	int initial_offset = -300;//500;
	for(int i = 0; i < 6; i++)
	{
		pos[6+i] = 2048 + (i < 3 ? initial_offset : -initial_offset); // id 11 to 16
		pos[12+i] = 2048 + (i < 3 ? initial_offset : -initial_offset); // id 21 to 26
		pos[18+i] = 512; // id 31 to 36
	}
	pos.resize(pos_ctrl_ids.size());




	// Lower motors speeds for initial positionning
	speed_msg.ids = pos_ctrl_ids;
	speed_msg.directions.clear();
	speed_msg.speeds.clear();
	for(int i = 0; i < speed_msg.ids.size(); i++)
	{
		speed_msg.directions.push_back(false);
		speed_msg.speeds.push_back(100);
	}
	speed_pub.publish(speed_msg);





	char c;
	std::cout << "Waiting for input.." << std::endl;
	std::cin >> c;

	pos_msg.ids = pos_ctrl_ids;
	pos_msg.positions = pos;
	position_pub.publish(pos_msg);

	speed_msg.ids.clear();
	speed_msg.ids = speed_ctrl_ids;
	speed_msg.directions.clear();
	speed_msg.speeds.clear();
	for(int i = 0; i < speed_msg.ids.size(); i++)
	{
		speed_msg.directions.push_back(false);
		speed_msg.speeds.push_back(0);
	}
	speed_pub.publish(speed_msg);


	int amp0 = 160, amp1=300, amp3=40;


	std::cout << "Waiting for input (ready to walk).." << std::endl;
	std::cin >> c;

	double begin = ros::Time::now().toSec();
	bool first = true;

	// Start walking
	while(ros::ok())
	{
		double elapsed = ros::Time::now().toSec() - begin;

		int sin0 = (int)(amp0*sin(period*elapsed));
		int sin1 = (int)(amp1*cos(period*elapsed));
		int sin3 = (int)(amp3*sin(period*elapsed));

		pos_msg.positions[0] = pos[0] + sin0;
		pos_msg.positions[1] = pos[1] - sin0;
		pos_msg.positions[2] = pos[2] + sin0;
		pos_msg.positions[3] = pos[3] + sin0;
		pos_msg.positions[4] = pos[4] - sin0;
		pos_msg.positions[5] = pos[5] + sin0;

		pos_msg.positions[6] = pos[6] + sin1; // 11
		pos_msg.positions[7] = pos[7] - sin1; // 12
		pos_msg.positions[8] = pos[8] + sin1; // 13
		pos_msg.positions[9] = pos[9] + sin1; // 14
		pos_msg.positions[10] = pos[10] - sin1; // 15
		pos_msg.positions[11] = pos[11] + sin1; // 16

		pos_msg.positions[12] = pos[12] + sin1; // 21
		pos_msg.positions[13] = pos[13] - sin1; // 22
		pos_msg.positions[14] = pos[14] + sin1; // 23
		pos_msg.positions[15] = pos[15] + sin1; // 24
		pos_msg.positions[16] = pos[16] - sin1; // 25
		pos_msg.positions[17] = pos[17] + sin1; // 26

		pos_msg.positions[18] = pos[18] - sin3;
		pos_msg.positions[19] = pos[19] + sin3;
		pos_msg.positions[20] = pos[20] - sin3;
		pos_msg.positions[21] = pos[21] - sin3;
		pos_msg.positions[22] = pos[22] + sin3;
		pos_msg.positions[23] = pos[23] - sin3;

		position_pub.publish(pos_msg);

		if(first)
		{
		  //			printf("Waiting for initial positioning to complete\r\n");
		  //			std::cout << "Waiting for input.." << std::endl;
		  //	std::cin >> c;

			// Start wheels mvt
			printf("Now starting wheel movements\r\n");
			speed_msg.ids.clear();
			speed_msg.ids = speed_ctrl_ids;
			speed_msg.directions.clear();
			speed_msg.speeds.clear();
			for(int i = 0; i < speed_msg.ids.size(); i++)
			{
				speed_msg.directions.push_back(i>=3);
				speed_msg.speeds.push_back(wheel_speed);
			}
			speed_pub.publish(speed_msg);
			ros::Duration(0.5).sleep();

			// Restore motors max speeds
			printf("Restoring motors max speeds\r\n");
			speed_msg.ids.clear();
			speed_msg.ids = pos_ctrl_ids;
			speed_msg.directions.clear();
			speed_msg.speeds.clear();
			for(int i = 0; i < speed_msg.ids.size(); i++)
			{
				speed_msg.directions.push_back(false);
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
