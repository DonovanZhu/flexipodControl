#include <iostream>
#include <iomanip>
#include <WS2tcpip.h>
#include <string>
#include <windows.h>
#include <conio.h>
#include <vector>
#include <string>
#include <msgpack.hpp>
#include <sstream>

// Include the Winsock library (lib) file
#pragma comment (lib, "ws2_32.lib")

#define MOTOR_NMB 12
// Saves us from typing std::cout << etc. etc. etc.
using namespace std;

class MotorCommand {
public:
	float robot_command[MOTOR_NMB];
	MSGPACK_DEFINE(robot_command);
};

MotorCommand Command;

void main()
{
	WSADATA data;

	WORD version = MAKEWORD(2, 2);

	int wsOk = WSAStartup(version, &data);

	sockaddr_in server;
	server.sin_family = AF_INET; // AF_INET = IPv4 addresses
	server.sin_port = htons(32001); // 32001 is the port number of the program on  Jetso Nano
	inet_pton(AF_INET, "10.42.0.1", &server.sin_addr); // 10.42.0.1 is the address of Jetson Nano

	SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);

	float speed_command[MOTOR_NMB] = {0.0, 0.0, 0.0};
	MotorCommand SendCommand;
	
	int j = 0;
	for (int i = 0; i < MOTOR_NMB; i++)
		SendCommand.robot_command[i] = 0.0;
	int kbinput;
	while (true)
	{
		if (_kbhit())
		{
			kbinput = _getch();
			if (kbinput == '1') {
				SendCommand.robot_command[0] += 0.02;
			}
			else if (kbinput == 'q') {
				SendCommand.robot_command[0] -= 0.02;
			}
			else if (kbinput == '2') {
				SendCommand.robot_command[1] += 0.02;
			}
			else if (kbinput == 'w') {
				SendCommand.robot_command[1] -= 0.02;
			}
			else if (kbinput == '3') {
				SendCommand.robot_command[2] += 0.02;
			}
			else if (kbinput == 'e') {
				SendCommand.robot_command[2] -= 0.02;
			}
			else if (kbinput == '4') {
				SendCommand.robot_command[3] += 0.02;
			}
			else if (kbinput == 'r') {
				SendCommand.robot_command[3] -= 0.02;
			}
			else if (kbinput == '5') {
				SendCommand.robot_command[4] += 0.02;
			}
			else if (kbinput == 't') {
				SendCommand.robot_command[4] -= 0.02;
			}
			else if (kbinput == '6') {
				SendCommand.robot_command[5] += 0.02;
			}
			else if (kbinput == 'y') {
				SendCommand.robot_command[5] -= 0.02;
			}
			else if (kbinput == 'a') {
				SendCommand.robot_command[6] += 0.02;
			}
			else if (kbinput == 'z') {
				SendCommand.robot_command[6] -= 0.02;
			}
			else if (kbinput == 's') {
				SendCommand.robot_command[7] += 0.02;
			}
			else if (kbinput == 'x') {
				SendCommand.robot_command[7] -= 0.02;
			}
			else if (kbinput == 'd') {
				SendCommand.robot_command[8] += 0.02;
			}
			else if (kbinput == 'c') {
				SendCommand.robot_command[8] -= 0.02;
			}
			else if (kbinput == 'f') {
				SendCommand.robot_command[9] += 0.02;
			}
			else if (kbinput == 'v') {
				SendCommand.robot_command[9] -= 0.02;
			}
			else if (kbinput == 'g') {
				SendCommand.robot_command[10] += 0.02;
			}
			else if (kbinput == 'b') {
				SendCommand.robot_command[10] -= 0.02;
			}
			else if (kbinput == 'h') {
				SendCommand.robot_command[11] += 0.02;
			}
			else if (kbinput == 'n') {
				SendCommand.robot_command[11] -= 0.02;
			}
			else if (kbinput == '0') {
				for (int i = 0; i < MOTOR_NMB; i++)
					SendCommand.robot_command[i] = 0.0;
			}
		}

		if (j > 8000)
		{
			system("cls");
			j = 0;
		}
		j++;


		std::stringstream send_stream;
		msgpack::pack(send_stream, SendCommand);
		std::string const& data = send_stream.str();

		std::cout << SendCommand.robot_command[0] << " " << SendCommand.robot_command[1] << " " << SendCommand.robot_command[2]
			<< " " << SendCommand.robot_command[3] << " " << SendCommand.robot_command[4] << " " << SendCommand.robot_command[5]
			<< " " << SendCommand.robot_command[6] << " " << SendCommand.robot_command[7] << " " << SendCommand.robot_command[8]
			<< " " << SendCommand.robot_command[9] << " " << SendCommand.robot_command[10] << " " << SendCommand.robot_command[11] << endl;

		sendto(out, data.c_str(), data.size(), 0, (sockaddr*)&server, sizeof(server));

	}
	closesocket(out);

	WSACleanup();
}
