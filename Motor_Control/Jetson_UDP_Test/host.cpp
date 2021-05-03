#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <netdb.h>
#include <limits.h>
#include <dirent.h>
#include <iomanip>
#include <iostream>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/serial.h>
#include <linux/input.h>
#include <msgpack.hpp>
#include "host.h"
typedef std::chrono::high_resolution_clock Clock;

using namespace std;

// Globals
int                    Host_fd = HOST_ERROR_FD;        // Serial port file descriptor
                            
char                   Host_devname[PATH_MAX] = ""; 	// Serial port devname used to get fd with open
                            
struct                 termios Host_oldtio;  		// Backup of initial tty configuration

Teensycomm_struct_t    teensy_comm;	// A data struct received from Teensy
Jetson_comm_struct_t   jetson_comm;		// A data struct sent to Teensy

float 	               desire_speed[MOTOR_NUM];		// The deisred speed
	
// Receiving the desired speed data by UDP and hold it in this class
class MotorCommand {
public:
	float 	robot_command[MOTOR_NUM];
	MSGPACK_DEFINE(robot_command);
};


class MsgToPC {
public:
	float joint_pos[MOTOR_NUM]; 		// Rotation angle, unit degree
	float joint_vel[MOTOR_NUM]; 		// Rotation speed, unit rpm
	float current[MOTOR_NUM]; 		// Rotation current, unit A
	float acc[3];				// Acceleration of IMU, unit m/s^2
	float gyr[3];				// Gyroscope, unit deg/s
	float mag[3];
	float eular[3];
	float timestamps;
	MSGPACK_DEFINE(joint_pos, joint_vel, torque, comd, acc, gyr, mag, eular, timestamps);
};

// set a object for sending UDP through msgpack
MsgToPC SendMotorData;


int main( ) {
	for (int i = 0; i < MOTOR_NUM; ++i)
		desire_speed[i] = 0.0;
	
	Teensycomm_struct_t *comm;
	int ret;
	
/********************UDP_Receiving_Initializing********************/
	int sock_recv, length_recv;
	socklen_t fromlen;
	struct sockaddr_in server_recv;
	struct sockaddr_in hold_recv;

	sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
	
	length_recv = sizeof(server_recv);
	bzero(&server_recv,length_recv);
	
	server_recv.sin_family = AF_INET;
	server_recv.sin_addr.s_addr = INADDR_ANY;
	server_recv.sin_port = htons(32001);	// Setting port of this program
	bind(sock_recv,(struct sockaddr *)&server_recv,length_recv);
	fromlen = sizeof(struct sockaddr_in);
	MotorCommand recv;		// For holding the received class
/******************************************************************/
	
/*********************UDP_Sending_Initializing*********************/
	int sock_send, length_send;
	struct sockaddr_in client_send;

	sock_send = socket(AF_INET, SOCK_DGRAM, 0);
	
	length_send = sizeof(client_send);
	bzero(&client_send,length_send);
	
	client_send.sin_family = AF_INET;
	client_send.sin_port = htons(32000); // Port of aim program
	inet_pton(AF_INET, "192.168.137.1", &client_send.sin_addr); // Address
	bind(sock_send,(struct sockaddr *)&client_send,length_send);
	//UdpDataSend msg_send;			// For holding the sent class
/******************************************************************/
	char buf_UDP_recv[128]; 			// for holding UDP data
	int command_recv[MOTOR_NUM];

	while(1) {
		auto start_time = Clock::now();
		int datalength = recvfrom(sock_recv, buf_UDP_recv, 128, 0, (struct sockaddr *)&hold_recv, &fromlen);
		msgpack::object_handle oh = msgpack::unpack(buf_UDP_recv, datalength);
		msgpack::object obj = oh.get();
		obj.convert(recv);
		auto end_time = Clock::now();
		for (int i = 0; i < MOTOR_NUM ; i++) {
			desire_speed[i] = recv.robot_command[i];
			//desire_speed[i] = 0.0;
			//printf("%f\t", desire_speed[i]);
		}
		int time = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
		printf("%f\n", (double)time / 1000.0);

		// After receiving the data from Teensy, save it into class SendMotorData

		/*		
		for (int j = 0; j < MOTOR_NUM; ++j) {
			
			// Speed of motors:
			SendMotorData.joint_vel[j] = comm -> joint_vel[j];
			
			// Angle of motors:
			SendMotorData.joint_pos[j] = comm -> joint_pos[j];
			
			// Torque of motors:
			SendMotorData.torque[j] = comm -> torque[j];
			// Desired speed:
			SendMotorData.comd[j] = comm -> comd[j];
			

			
			SendMotorData.timestamps = comm -> timestamps;
			
		}
		//printf("%f\t", SendMotorData.timestamps);
		
		
		for (int i = 0; i < 3; ++i) {
			SendMotorData.acc[i] = comm -> acc[i];
			
			SendMotorData.gyr[i] = comm -> gyr[i];
			
			SendMotorData.mag[i] = comm -> mag[i];
			
			SendMotorData.eular[i] = comm -> eular[i];
			printf("%f\t", SendMotorData.eular[i]);
		}
		printf("\n");
		
		std::stringstream send_stream;
		msgpack::pack(send_stream, SendMotorData);
		std::string const& data = send_stream.str();
		sendto(sock_send, data.c_str(), data.size(), 0, (struct sockaddr *)&client_send, sizeof(client_send));
		*/
		
	}

	return 0;
}

