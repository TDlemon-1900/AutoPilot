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

void SubMotionPlanningFbkHandler(const nav_msgs::Odometry::ConstPtr& msg)
{
    _model_Obj.JS6904ChassisOnboardSF_U.motion_msg_heart++;
}

void SubRemoteCmdHandler(const nav_msgs::Odometry::ConstPtr& msg)
{
    // _model_Obj.XiaoBaiChassisOnboardSF_U.moving_task_pause = msg->moving_task_pause;
}

void SubMovingCmdHandler(const nav_msgs::Odometry::ConstPtr& msg)
{
    //_movingCmd = *msg;
    // _model_Obj.XiaoBaiChassisOnboardSF_U._movingCmd = _movingCmd;
}

void SubOBDFbkHandler(const nav_msgs::Odometry::ConstPtr& msg)
{
    //_OBDFbk = *msg;
}

void SubVehicleChassisFbk(const nav_msgs::Odometry::ConstPtr& msg)
{
    //_vehicleChassisFbk = *msg;
}

int main(int argc, char **argv)
{
	// system("sudo modprobe pcan");
	ros::init(argc, argv, "can_send");
	ros::NodeHandle _node;
	// ros通讯
	// 接收_motionPlanningFbk
    ros::Subscriber _subMotionPlanningFbk = _node.subscribe<nav_msgs::Odometry>
                                     ("/motion_planning/MotionPlanningFbk", 1, SubMotionPlanningFbkHandler);
    // void SubMotionPlanningFbkHandler(const ultility_files::Msg_MotionPlanningFbk::ConstPtr& msg);
	// 接收RemoteCmd
	ros::Subscriber _subRemoteCmd = _node.subscribe<nav_msgs::Odometry>
                                    ("/remote_interface/RemoteCmd", 1, SubRemoteCmdHandler);
    // void SubRemoteCmdHandler(const ultility_files::Msg_MovingCmd::ConstPtr& msg);
	// 接收_movingCmd报文
    ros::Subscriber _subMovingCmd = _node.subscribe<nav_msgs::Odometry>
                                    ("/human_interface/MovingCmd", 1, SubMovingCmdHandler);
    // void SubMovingCmdHandler(const ultility_files::Msg_MovingCmd::ConstPtr& msg);
	// 接收_OBDFbk报文
	ros::Subscriber _subOBDFbk = _node.subscribe<nav_msgs::Odometry>
                                    ("obd", 1, SubOBDFbkHandler);
	// 接收_vehicleChassisFbk报文
	ros::Subscriber _subVehicleChassisFbk = _node.subscribe<nav_msgs::Odometry>
                                    ("/vehicle_chassis_interface/VehicleChassisFbk", 1, SubVehicleChassisFbk);
    

	ros::Rate loop_rate(20);

	int s; /* can raw socket */ 
	int required_mtu;
	
	// required
	int required1;
	int required2;
	int required3;
	
	int mtu;
	int enable_canfd = 1;
	struct sockaddr_can addr;
	struct canfd_frame frame;
	
	// struct ctrl_send
	struct canfd_frame ctrl_send1;
	struct canfd_frame ctrl_send2;
	struct canfd_frame ctrl_send3;
	
	// define data
	int data1[8] = {10,12,255,2,50,15,12,7};
	int data2[8] = {10,12,255,2,50,15,12,7};
	int data3[8] = {10,12,255,2,50,15,12,7};
	
	struct ifreq ifr;

	int wheel_speed_FL_fault = 1;
	int wheel_speed_FL = 8191;
	int wheel_speed_FR_fault = 1;
	int wheel_speed_FR = 8191;
	int vehicle_speed = 8191;
	int ABS_Vehicle_speed_valid = 1;
	int ABS1_message_counter = 15;
	int ABS1_checksum = 255;
	int APA_Checksum = 255;
	int APA_EpsOnReq = 1;
	int APA_MsgCounter = 15;
	int APA_SteerAngReq = 5000;
	int APA_SteerAngSpdLimt = 5000;
	int EMS_EngineSpeedError = 1;
	int EMS_EngineSpeed = 5000;
	int EMS_EngineStatus = 3;
	int EleStrFailFlag = 3;
	int ValidDriveSteeringTorque = 1;
	int DriveSteeringTorque = 10;
	int SAS1_msg_counter = 15;
	int SAS1_checksum = 15;
	int steering_angle_valid = 1;
	int calibrated_status = 1;
	int steering_angle_spd = 120;
	int steering_angle = 500;
	int EPS_ControlMode = 3;
	int EPS_ExpectedState = 255;
	int EPS_ExpectedCorner = 5000;
	int EPS_ExpectedTorque = 25;
	int SpeedOfCar = 220;
	int CheckSum = 255;
	int EPS_ActualState = 255;
	int EPS_ActualCorner = 5000;
	int EPS_ActualTorque = 25;
	int EPS_AngleSensorStatus = 255;
	int EPS_SensorStatus = 1;
	int EPS_IndicatingLightStatus = 1;
	int VCU_DrivingMode = 3;
	int BrakeSwitch = 1;
	int ParkingStatusFeedback = 1;
	int GearFeedback = 3;
	int DoorStatus = 1;
	int GearOperationStatus = 1;
	int ParkingOperationStatus = 1;
	int VCU_FaultLevel = 3;
	int BrakingControlMode = 1;
	int ManualTakeoverStatus = 1;
	int EPB_Status = 1;
	int ActualMasterCylinderPressure = 20;
	int MaximumSpeed = 255;
	int SteeringWheelTorqueSignal = 30;
	int SteeringWheelAngle = 2000;
	int AverageSpeedOfFrontAxle = 100;
	int LongitudinalAcceleration = 500;
	int MotorFeedbackTorque = 2000;
	int MaximumBrakingDeceleration = 25;
	int PedalPercentage = 100;
	int VCUDrivingMode = 3;
	int ParkingStatusRequest = 1;
	int GearRequest = 3;
	int TurnSignalRequest = 3;
	int AutomaticDoorOpeningRequest = 1;
	int StopLampRequest = 1;
	int LowBeamLampRequest = 1;
	int SteeringWheelTurnRequest = 5000;
	int SlowingDownRequest = 200;
	int RormentRequest = 1000;
	int SystemState = 3;
	int SystemFaultCode = 255;
	int BrakeRequestPercentage = 100;
	int DrivingRequestsPercent = 100;
	int Life = 15;

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
	// parse_canframe函数发送标准帧时返回值为canframe结构体的size，为16
	required_mtu = 16;

	// 定义发送ID
	std::string send_frame_str1 = "300#";
	std::string send_frame_str2 = "211#";
	std::string send_frame_str3 = "201#";
	
	// 定义发送数据的内容，数据来自模型；数据放入字符串中
	std::string send_data_str1;
	std::string send_data_str2;
	std::string send_data_str3;

	// data1
	data1[0] = (SAS1_checksum * 16) | SAS1_msg_counter;
	data1[1] = (calibrated_status * 2) | steering_angle_valid;
	data1[2] = steering_angle_spd * 0.25;
	data1[3] = (((steering_angle + 780) * 10) & 0xff00) >> 8;
	data1[4] = ((steering_angle + 780) * 10) & 0xff;      //steering_angle错，实际赋值500，显示501.5
	data1[7] = EPS_ControlMode;
	
	// data2
	data2[0] = (SAS1_checksum * 16) | SAS1_msg_counter;
	data2[1] = (calibrated_status * 2) | steering_angle_valid;
	data2[2] = steering_angle_spd * 0.25;
	data2[3] = (((steering_angle + 780) * 10) & 0xff00) >> 8;
	data2[4] = ((steering_angle + 780) * 10) & 0xff;      //steering_angle错，实际赋值500，显示501.5
	data2[7] = EPS_ControlMode;
	
	// data3
	data3[0] = (SAS1_checksum * 16) | SAS1_msg_counter;
	data3[1] = (calibrated_status * 2) | steering_angle_valid;
	data3[2] = steering_angle_spd * 0.25;
	data3[3] = (((steering_angle + 780) * 10) & 0xff00) >> 8;
	data3[4] = ((steering_angle + 780) * 10) & 0xff;      //steering_angle错，实际赋值500，显示501.5
	data3[7] = EPS_ControlMode;
	
	for(int i = 0;i < 8;i++) {
		string temp_str1 = dec2hexstr(data1[i], 2);
		string temp_str2 = dec2hexstr(data2[i], 2);
		string temp_str3 = dec2hexstr(data3[i], 2);
		
		for(int j = 0; j < temp_str1.size(); j++) {
			send_data_str1.push_back(temp_str1[j]);
		}
		
		for (int k = 0; k < temp_str2.size(); k++) {
			send_data_str2.push_back(temp_str2[k]);
		}
		
		for (int l = 1; l < temp_str3.size(); l++) {
			send_data_str3.push_back(temp_str3[l]);
		}
	}
	
	// msg
	send_frame_str1 += send_data_str1;	
	send_frame_str2 += send_data_str2;
	send_frame_str3 += send_data_str3;

	// define char array
	char send_msg1[20];
	char send_msg2[20];
	char send_msg3[20];
	
	// 发送的整个帧转为字符数组，并转为can frame
	strcpy(send_msg1, send_frame_str1.c_str());
	strcpy(send_msg2, send_frame_str2.c_str());
	strcpy(send_msg3, send_frame_str3.c_str());
	
	// ROS_INFO("string size:%ld", send_frame_str.size());
	required1 = parse_canframe(send_msg1, &ctrl_send1);
	required2 = parse_canframe(send_msg2, &ctrl_send2);
	required3 = parse_canframe(send_msg3, &ctrl_send3);
		
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
	if (!required2){
		fprintf(stderr, "\nWrong CAN-frame format!\n\n");
		print_usage(argv[0]);
		return 1;
	}
	
	
	if (required2 > (int)CAN_MTU) {
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
	
	if (write(s, &ctrl_send2, required2) != required2) 
		{
			perror("write");
			return 1;
		}
	
	// send msg3
	if (!required3){
		fprintf(stderr, "\nWrong CAN-frame format!\n\n");
		print_usage(argv[0]);
		return 1;
	}
	
	
	if (required3 > (int)CAN_MTU) {
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
	
	if (write(s, &ctrl_send3, required3) != required3) 
		{
			perror("write");
			return 1;
		}

	loop_rate.sleep();
}

	close(s);

	return 0;
}
