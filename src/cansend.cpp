#include <ros/ros.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "std_msgs/String.h"
#include <sstream>
#include <nav_msgs/Odometry.h>

// #include "ultility_files/Msg_VehicleChassisFbk.h"
// #include "ultility_files/Msg_MotionPlanningFbk.h"
// #include "ultility_files/Msg_MovingCmd.h"
// #include "ultility_files/Msg_OBDFbk.h"

#include "JS6904ChassisOnboardSF_ert_rtw/JS6904ChassisOnboardSF.h"

#include "lib.h"
using namespace std;

JS6904ChassisOnboardSFModelClass _model_Obj;
// ultility_files::Msg_MotionPlanningFbk _motionPlanningFbk;
// ultility_files::Msg_MovingCmd _movingCmd;
// ultility_files::Msg_OBDFbk _OBDFbk;
// ultility_files::Msg_VehicleChassisFbk _vehicleChassisFbk;

int VCU_DrivingMode = 0; 
int ParkingStatusRequest = 0;
int GearRequest = 0;
int TurnSignalRequest = 0;
int AutomaticDoorOpeningRequest = 0; 
int StopLampRequest = 0; 
int LowBeamLampRequest = 0; 
int SteeringWheelTurnRequest = 0;
int SlowingDownRequest = 0;
int RormentRequest = 0;
int SystemState = 0;
int SystemFaultCode = 0;
int Life = 0;

void print_usage(char *prg)
{
	fprintf(stderr, "%s - send CAN-frames via CAN_RAW sockets.\n", prg);
	fprintf(stderr, "\nUsage: %s <device> <can_frame>.\n", prg);
	fprintf(stderr, "\n<can_frame>:\n");
	fprintf(stderr, " <can_id>#{data}          for Classical CAN 2.0 data frames\n");
	fprintf(stderr, " <can_id>#R{len}          for Classical CAN 2.0 data frames\n");
	fprintf(stderr, " <can_id>#{data}_{dlc}    for Classical CAN 2.0 data frames\n");
	fprintf(stderr, " <can_id>#R{len}_{dlc}    for Classical CAN 2.0 data frames\n");
	fprintf(stderr, " <can_id>##<flags>{data}  for CAN FD frames\n\n");
	fprintf(stderr, "<can_id>:\n"
	        " 3 (SFF) or 8 (EFF) hex chars\n");
	fprintf(stderr, "{data}:\n"
	        " 0..8 (0..64 CAN FD) ASCII hex-values (optionally separated by '.')\n");
	fprintf(stderr, "{len}:\n"
		 " an optional 0..8 value as RTR frames can contain a valid dlc field\n");
	fprintf(stderr, "_{dlc}:\n"
		 " an optional 9..F data length code value when payload length is 8\n");
	fprintf(stderr, "<flags>:\n"
	        " a single ASCII Hex value (0 .. F) which defines canfd_frame.flags\n\n");
	fprintf(stderr, "Examples:\n");
	fprintf(stderr, "  5A1#11.2233.44556677.88 / 123#DEADBEEF / 5AA# / 123##1 / 213##311223344 /\n"
		 "  1F334455#1122334455667788_B / 123#R / 00000123#R3 / 333#R8_E\n\n");
}

std::string dec2hexstr(int i, int width)
{
	std::stringstream ioss;
	std::string s_temp;
	ioss << std::hex << i;
	ioss >> s_temp;
	std::string s(width - s_temp.size(), '0');
	s += s_temp;
	return s;
}

void DealModelInput()
{
    // _model_Obj.JS6904ChassisOnboardSF_U._vehicleChassisFbk.curvature           = _vehicleChassisFbk.curvature;
    // _model_Obj.JS6904ChassisOnboardSF_U._vehicleChassisFbk.velocity            = _vehicleChassisFbk.velocity;

    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.time_stamp                  = _motionPlanningFbk.time_stamp;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.node_heart                  = _motionPlanningFbk.node_heart;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.state_num                   = _motionPlanningFbk.state_num;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.lateral_control_mode        = _motionPlanningFbk.lateral_control_mode;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.longitudinal_control_mode   = _motionPlanningFbk.longitudinal_control_mode;;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.preview_pose.x              = _motionPlanningFbk.preview_pose.x;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.preview_pose.y              = _motionPlanningFbk.preview_pose.y;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.preview_pose.z              = _motionPlanningFbk.preview_pose.z;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.preview_pose.roll           = _motionPlanningFbk.preview_pose.roll;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.preview_pose.pitch          = _motionPlanningFbk.preview_pose.pitch;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.preview_pose.yaw            = _motionPlanningFbk.preview_pose.yaw;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.current_pose.x              = _motionPlanningFbk.current_pose.x;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.current_pose.y              = _motionPlanningFbk.current_pose.y;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.current_pose.z              = _motionPlanningFbk.current_pose.z;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.current_pose.roll           = _motionPlanningFbk.current_pose.roll;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.current_pose.pitch          = _motionPlanningFbk.current_pose.pitch;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.current_pose.yaw            = _motionPlanningFbk.current_pose.yaw;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.target_curvature            = _motionPlanningFbk.target_curvature;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.target_vehicle_velocity     = _motionPlanningFbk.target_vehicle_velocity;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.target_accelerated_speed    = _motionPlanningFbk.target_accelerated_speed;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.target_brake_pressure       = _motionPlanningFbk.target_brake_pressure;;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.direct_steering_signal      = _motionPlanningFbk.direct_steering_signal;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.direct_throttle_signal      = _motionPlanningFbk.direct_throttle_signal;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.direct_brake_pedal_signal   = _motionPlanningFbk.direct_brake_pedal_signal;
    // _model_Obj.JS6904ChassisOnboardSF_U._motionPlanningFbk.direct_shift_signal         = _motionPlanningFbk.direct_shift_signal;

    // _model_Obj.JS6904ChassisOnboardSF_U._movingCmd.moving_task_suspend = _movingCmd.moving_task_suspend;
    // _model_Obj.JS6904ChassisOnboardSF_U._movingCmd.moving_task_pause = _movingCmd.moving_task_pause;
    // _model_Obj.JS6904ChassisOnboardSF_U._movingCmd.emergency_brake = _movingCmd.emergency_brake;
    // _model_Obj.JS6904ChassisOnboardSF_U._movingCmd.node_heart = _movingCmd.node_heart;


    // _model_Obj.JS6904ChassisOnboardSF_U._OBDFbk.emergency_brake = _OBDFbk.emergency_brake;
    // _model_Obj.JS6904ChassisOnboardSF_U._OBDFbk.moving_task_pause = _OBDFbk.moving_task_pause;
   
}


void SubRemoteCmdHandler(const nav_msgs::Odometry::ConstPtr& msg)
{
	VCU_DrivingMode = msg->pose.pose.positon.x;
	ParkingStatusRequest = msg->pose.pose.position.y;
	GearRequest = msg->pose.pose.position.z;
	TurnSignalRequest = msg->pose.pose.orientation.x;
	AutomaticDoorOpeningRequest = msg->pose.pose.orientation.y; 
	StopLampRequest = msg->pose.pose.orientation.z; 
	LowBeamLampRequest = msg->pose.pose.orientation.w; 
	SteeringWheelTurnRequest = (msg->twist.twist.linear.x + 870) * 10;
	SlowingDownRequest = msg->twist.twist.linear.y * 2.5;
	RormentRequest = msg->twist.twist.linear.z * 2.5;
	SystemState = msg->twist.twist.angular.x
	SystemFaultCode = msg->twist.twist.angular.y;
	Life = msg->twist.twist.linear.z;
}

int main(int argc, char **argv)
{
	// system("sudo modprobe pcan");
	ros::init(argc, argv, "can_send"); 
	ros::NodeHandle _node;
	
	// ros通讯
	ros::Subscriber _subRemoteCmd = _node.subscribe<nav_msgs::Odometry>("/can_test", 1, SubRemoteCmdHandler);

	ros::Rate loop_rate(20);

	int s; /* can raw socket */ 
	int required_mtu;
	
	// required
	int required1;
	int required2;
	
	int mtu;
	int enable_canfd = 1;
	struct sockaddr_can addr;
	struct canfd_frame frame;
	
	// struct ctrl_send
	struct canfd_frame ctrl_send1;
	struct canfd_frame ctrl_send2;
	
	// define data
	int data1[8] = {0};
	int data2[8] = {0};
	
	struct ifreq ifr;



	_model_Obj.initialize();

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}
	
	// strncpy(ifr.ifr_name, argv[1], IFNAMSIZ - 1);
	// 在程序中给定can0，只在can0上发送
	strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		perror("if_nametoindex");
		return 1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	while (ros::ok())
	{
		ros::spinOnce();
		DealModelInput();
		_model_Obj.step();
		required_mtu = 16;
	
		// 定义发送ID
		std::string send_frame_str1 = "18FF909F#";
		std::string send_frame_str2 = "18EF918F#";
		
		// 定义发送数据的内容，数据来自模型；数据放入字符串中
		std::string send_data_str1;
		std::string send_data_str2;
		
		// data1
		data1[0] = (ParkingStatusRequest * 64) | VCU_DrivingMode;
		data1[1] = (LowBeamLampRequest * 128) | (StopLampRequest * 32) | (AutomaticDoorOpeningRequest * 16) | (TurnSignalRequest * 4) | GearRequest;
		data1[2] = SteeringWheelTurnRequest & 0xff; 
		data1[3] = (SteeringWheelTurnRequest & 0xff00) >> 8;
		data1[4] = SlowingDownRequest;
		data1[5] = RormentRequest;
		
		// data2
		data2[0] = SystemState;
		data2[1] = SystemFaultCode;
		data2[7] = Life << 4;
		
		for(int i = 0;i < 8;i++) {
			string temp_str1 = dec2hexstr(data1[i], 2);
			string temp_str2 = dec2hexstr(data2[i], 2);
			
			for(int j = 0; j < temp_str1.size(); j++) {
				send_data_str1.push_back(temp_str1[j]);
			}
			
			for (int k = 0; k < temp_str2.size(); k++) {
				send_data_str2.push_back(temp_str2[k]);
			}
		}
		
		// msg
		send_frame_str1 += send_data_str1;	
		send_frame_str2 += send_data_str2;
	
		// define char array
		char send_msg1[25];
		char send_msg2[25];
		
		// 发送的整个帧转为字符数组，并转为can frame
		strcpy(send_msg1, send_frame_str1.c_str());
		strcpy(send_msg2, send_frame_str2.c_str());
		
		// ROS_INFO("string size:%ld", send_frame_str.size());
		required1 = parse_canframe(send_msg1, &ctrl_send1);
		required2 = parse_canframe(send_msg2, &ctrl_send2);
			
		// send msg1
		if (!required1){
			fprintf(stderr, "\nWrong CAN-frame format!\n\n");
			print_usage(argv[0]);
			return 1;
		}
	
	
		if (required1 > (int)CAN_MTU) {
			/* check if the frame fits into the CAN netdevice */
			if (ioctl(s, SIOCGIFMTU, &ifr) < 0) {
				perror("SIOCGIFMTU");
				return 1;
			}
			mtu = ifr.ifr_mtu;
	
			if (mtu != CANFD_MTU) {
				printf("CAN interface is not CAN FD capable - sorry.\n");
				return 1;
			}
	
			/* interface is ok - try to switch the socket into CAN FD mode */
			if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
						&enable_canfd, sizeof(enable_canfd))){
				printf("error when enabling CAN FD support\n");
				return 1;
			}
	
			/* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
			frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len));
			
		}
		
		if (write(s, &ctrl_send1, required1) != required1) 
		{
			perror("write");
			return 1;
		}
		
		// send msg2
		if (write(s, &ctrl_send2, required2) != required2) 
			{
				perror("write");
				return 1;
			}
		
		// sleep 
		loop_rate.sleep();
	}
	
	close(s);

	return 0;
}
