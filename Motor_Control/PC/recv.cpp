#include <iostream>
#include <iomanip>
#include <WS2tcpip.h>
#include <msgpack.hpp>
#include <vector>
#include <string>
#include <msgpack.hpp>
#include <fstream>
#define MOTOR_NUM 12
// Include the Winsock library (lib) file
#pragma comment (lib, "ws2_32.lib")

using namespace std;

// Using this class to hold data from RPi
class MsgToPC {
public:
	float joint_pos[MOTOR_NUM]; 	// Rotation angle, unit rad
	float joint_vel[MOTOR_NUM]; 	// Rotation speed, unit rad/s
	float joint_cur[MOTOR_NUM]; 	// Rotation current, unit A
	float acc[3];		// Acceleration of IMU, unit m/s^2
	float gyr[3];		// Gyroscope, unit rad/s
	float mag[3];		// Magnetometer, unit 
	float eular[3];
	float timestamps;
	MSGPACK_DEFINE(joint_pos, joint_vel, joint_cur, acc, gyr, mag, eular, timestamps);
};

void main()
{
	// Open a txt file named flexipod_data.txt
	ofstream file;
	//file.open("flexipod_data.txt");

	// UDP initializing
	WSADATA data;

	WORD version = MAKEWORD(2, 2);

	// Start WinSock
	int wsOk = WSAStartup(version, &data);
	if (wsOk != 0)
	{
		// Not ok! Get out quickly
		cout << "Can't start Winsock! " << wsOk;
		return;
	}

	SOCKET in = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in serverHint;
	serverHint.sin_addr.S_un.S_addr = ADDR_ANY; // Us any IP address available on the machine
	serverHint.sin_family = AF_INET; // Address format is IPv4
	serverHint.sin_port = htons(32000); // Convert from little to big endian

	bind(in, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR;

	sockaddr_in client; // Use to hold the client information (port / ip address)
	int clientLength = sizeof(client); // The size of the client information

	char buf[1024];
	unsigned short k;
	int output = 0;

	int step = 0;
	while (true)
	{

		int datalength = recvfrom(in, buf, 1024, 0, (sockaddr*)&client, &clientLength);

		msgpack::object_handle oh = msgpack::unpack(buf, datalength);

		msgpack::object obj = oh.get();
		// you can convert object to UDPdata class directly
		MsgToPC recv;
		obj.convert(recv);

		// Save data into txt file
		/*
		file << recv[0].timestamps << "\t";
		file << recv[0].joint_pos[0] << "\t" << recv[0].joint_pos[1] << "\t" << recv[0].joint_pos[2] << "\t" << recv[0].joint_pos[3] << "\t";
		file << recv[0].rpm[0] << "\t" << recv[0].rpm[1] << "\t" << recv[0].rpm[2] << "\t" << recv[0].rpm[3] << "\t";
		file << recv[0].joint_cur[0] << "\t" << recv[0].joint_cur[1] << "\t" << recv[0].joint_cur[2] << "\t" << recv[0].joint_cur[3] << "\t";
		file << recv[0].command[0] << "\t" << recv[0].command[1] << "\t" << recv[0].command[2] << "\t" << recv[0].command[3] << "\t";
		file << recv[0].acc[0] << "\t" << recv[0].acc[1] << "\t" << recv[0].acc[2] << "\t";
		file << recv[0].gyr[0] << "\t" << recv[0].gyr[1] << "\t" << recv[0].gyr[2] << "\t";
		file << recv[0].mag[0] << "\t" << recv[0].mag[1] << "\t" << recv[0].mag[2] << "\t";
		file << recv[0].eular[0] << "\t" << recv[0].eular[1] << "\t" << recv[0].eular[2] << "\n";
		*/
		printf("%f\t%f\t%f\n", recv.eular[0], recv.eular[1], recv.eular[2]);
		//printf("%f\n", recv[0].timestamps);
	}
	// Close file
	//file.close();
	// Close socket
	closesocket(in);

	// Shutdown winsock
	WSACleanup();
}
