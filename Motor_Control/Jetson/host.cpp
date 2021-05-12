/*****************host.cpp******************/
// compile:
// sudo g++ -pthread host.cpp
// autorun: edit etc/rc.local:
//			add: path_to_a.out &
// run:
// ./a.out
/*******************************************/
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
#include <thread>
#include <deque>
#include "host.h"

using namespace std;

constexpr int JETSON_PORT = 32001;
constexpr int PC_PORT = 32000;
constexpr char PC_IP_ADDR[] = "192.168.137.1";
// constexpr char PC_IP_ADDR[] = "192.168.1.109";
constexpr int UDP_BUFFER_SIZE = 128;

// Globals
bool RUNNING = true; // indicating whether the main loop is running

// Receiving the desired speed data by UDP and hold it in this class
class MsgFromPC
{
public:
	float robot_command[MOTOR_NUM];
	MSGPACK_DEFINE(robot_command);
};

//
// Class for sending command to PC if using msgpack
//
class MsgToPC
{
public:
	float joint_pos[MOTOR_NUM]; // Rotation angle, unit degree
	float joint_vel[MOTOR_NUM]; // Rotation speed, unit rad/s
	float joint_cur[MOTOR_NUM]; // Rotation current, unit A
	float acc[3];				// Acceleration of IMU, unit m/s^2
	float gyr[3];				// Gyroscope, unit deg/s
	float mag[3];
	float euler[3];
	float timestamps;
	MSGPACK_DEFINE(joint_pos, joint_vel, joint_cur, acc, gyr, mag, euler, timestamps);
};

// set a object for sending UDP through msgpack
MsgToPC msg_to_pc;

class UdpReceiver
{
public:
	std::deque<MsgFromPC> msg_queue;

	/* udp receiver thread */
	void run()
	{
		/********************UDP_Receiving_Initializing********************/
		socklen_t fromlen;
		struct sockaddr_in server_recv;
		struct sockaddr_in hold_recv;

		int sock_recv = socket(AF_INET, SOCK_DGRAM, 0);

		int sock_length_recv = sizeof(server_recv);
		bzero(&server_recv, sock_length_recv);

		server_recv.sin_family = AF_INET;
		server_recv.sin_addr.s_addr = INADDR_ANY;
		server_recv.sin_port = htons(JETSON_PORT); // Setting port of this program
		bind(sock_recv, (struct sockaddr *)&server_recv, sock_length_recv);
		fromlen = sizeof(struct sockaddr_in);
		MsgFromPC recv;						// For holding the received class
		char buf_UDP_recv[UDP_BUFFER_SIZE]; // for holding UDP data
		/******************************************************************/
		while (RUNNING)
		{
			int datalength = recvfrom(sock_recv, buf_UDP_recv, UDP_BUFFER_SIZE, 0, (struct sockaddr *)&hold_recv, &fromlen);
			msgpack::object_handle oh = msgpack::unpack(buf_UDP_recv, datalength);
			msgpack::object obj = oh.get();
			obj.convert(recv);
			msg_queue.push_front(recv);
			while(msg_queue.size()>1){
				msg_queue.pop_back();
			}
		}
	}
};

int main()
{
	for (int i = 0; i < MOTOR_NUM; ++i)
		desired_pos[i] = 0.0;

	Teensycomm_struct_t *comm;
	int ret;

	// Initialize serial port
	if (Host_init_port(HOST_DEV_SERIALNB))
	{
		fprintf(stderr, "Error initializing serial port.\n");
		exit(-1);
	}

	UdpReceiver udp_receiver = UdpReceiver();
	std::thread udp_recv_thread(&UdpReceiver::run,&udp_receiver); // run udpRecv in a separate thread

	/*********************UDP_Sending_Initializing*********************/
	struct sockaddr_in client_send;

	int sock_send = socket(AF_INET, SOCK_DGRAM, 0);

	int sock_length_send = sizeof(client_send);
	bzero(&client_send, sock_length_send);

	client_send.sin_family = AF_INET;
	client_send.sin_port = htons(PC_PORT);				   // Port of aim program
	inet_pton(AF_INET, PC_IP_ADDR, &client_send.sin_addr); // Address
	bind(sock_send, (struct sockaddr *)&client_send, sock_length_send);
	//UdpDataSend msg_send;			// For holding the sent class
	/******************************************************************/
	float time_former = 0.0;
	while (1)
	{
		if(udp_receiver.msg_queue.size()>0){
			MsgFromPC recv = udp_receiver.msg_queue.front();
			for (int i = 0; i < MOTOR_NUM; i++)
			{
				desired_pos[i] = recv.robot_command[i];
			}
		}
		//printf("%f\t", desired_pos[0]);
		// Serial exchange with teensy
		if ((ret = Host_comm_update(HOST_DEV_SERIALNB, &comm)))
		{
			fprintf(stderr, "Error %d in Host_comm_update.\n", ret);
			break;
		}
		// After receiving the data from Teensy, save it into class msg_to_pc
		
		msg_to_pc.timestamps = comm->timestamps;
		for (int j = 0; j < MOTOR_NUM; ++j)
		{
			// Speed of motors:
			msg_to_pc.joint_vel[j] = comm->joint_vel[j];
			// Angle of motors:
			msg_to_pc.joint_pos[j] = comm->joint_pos[j];
			// Current of motors:
			msg_to_pc.joint_cur[j] = comm->joint_cur[j];
		}
		
		time_former = msg_to_pc.timestamps;
		for (int i = 0; i < 3; ++i)
		{
			msg_to_pc.acc[i] = comm->acc[i];
			msg_to_pc.gyr[i] = comm->gyr[i];
			msg_to_pc.mag[i] = comm->mag[i];
			msg_to_pc.euler[i] = comm->euler[i];
			//printf("%f\t", msg_to_pc.euler[i]);
		}
		//printf("\n");

		std::stringstream send_stream;
		msgpack::pack(send_stream, msg_to_pc);
		std::string const &data = send_stream.str();
		sendto(sock_send, data.c_str(), data.size(), 0, (struct sockaddr *)&client_send, sizeof(client_send));
	}
	RUNNING = false;
	Host_release_port(HOST_DEV_SERIALNB);

	if (udp_recv_thread.joinable())
	{
		udp_recv_thread.joinable();
	}

	return 0;
}
