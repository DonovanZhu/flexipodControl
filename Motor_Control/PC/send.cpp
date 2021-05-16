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

// Joint angle command 
class MsgFromPC {
public:
	float robot_command[MOTOR_NMB];
	MSGPACK_DEFINE(robot_command);
};

MsgFromPC Command;

void main()
{
	/*********************** UDP initialization ***********************/
	WSADATA data;

	WORD version = MAKEWORD(2, 2);

	int wsOk = WSAStartup(version, &data);

	sockaddr_in server;
	server.sin_family = AF_INET; // AF_INET = IPv4 addresses
	server.sin_port = htons(32001); // Little to big endian conversion
	inet_pton(AF_INET, "192.168.137.7", &server.sin_addr); // Convert from string to byte array
	//inet_pton(AF_INET, "192.168.1.96", &server.sin_addr); // Convert from string to byte array

	SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);
	/*****************************************************************/
	MsgFromPC SendCommand;
	
	int j = 0;
	for (int i = 0; i < MOTOR_NMB; i++)
		SendCommand.robot_command[i] = 0.0;
	int kbinput;
	while (true)
	{
		/************************ Keyboard Input ************************/
		if (_kbhit())
		{
			kbinput = _getch();
			// Motor 1 counter-clockwise
			if (kbinput == '1') {
				SendCommand.robot_command[0] += 0.02;
			}
			// Motor 1 clockwise
			else if (kbinput == 'q') {
				SendCommand.robot_command[0] -= 0.02;
			}
			// Motor 2 counter-clockwise
			else if (kbinput == '2') {
				SendCommand.robot_command[1] += 0.02;
			}
			// Motor 2 clockwise
			else if (kbinput == 'w') {
				SendCommand.robot_command[1] -= 0.02;
			}
			// Motor 3 counter-clockwise
			else if (kbinput == '3') {
				SendCommand.robot_command[2] += 0.02;
			}
			// Motor 3 clockwise
			else if (kbinput == 'e') {
				SendCommand.robot_command[2] -= 0.02;
			}
			// Motor 4 counter-clockwise
			else if (kbinput == '4') {
				SendCommand.robot_command[3] += 0.02;
			}
			// Motor 4 clockwise
			else if (kbinput == 'r') {
				SendCommand.robot_command[3] -= 0.02;
			}
			// Motor 5 counter-clockwise
			else if (kbinput == '5') {
				SendCommand.robot_command[4] += 0.02;
			}
			// Motor 5 clockwise
			else if (kbinput == 't') {
				SendCommand.robot_command[4] -= 0.02;
			}
			// Motor 6 counter-clockwise
			else if (kbinput == '6') {
				SendCommand.robot_command[5] += 0.02;
			}
			// Motor 6 clockwise
			else if (kbinput == 'y') {
				SendCommand.robot_command[5] -= 0.02;
			}
			// Motor 7 counter-clockwise
			else if (kbinput == 'a') {
				SendCommand.robot_command[6] += 0.02;
			}
			// Motor 7 clockwise
			else if (kbinput == 'z') {
				SendCommand.robot_command[6] -= 0.02;
			}
			// Motor 8 counter-clockwise
			else if (kbinput == 's') {
				SendCommand.robot_command[7] += 0.02;
			}
			// Motor 8 clockwise
			else if (kbinput == 'x') {
				SendCommand.robot_command[7] -= 0.02;
			}
			// Motor 9 counter-clockwise
			else if (kbinput == 'd') {
				SendCommand.robot_command[8] += 0.02;
			}
			// Motor 9 clockwise
			else if (kbinput == 'c') {
				SendCommand.robot_command[8] -= 0.02;
			}
			// Motor 10 counter-clockwise
			else if (kbinput == 'f') {
				SendCommand.robot_command[9] += 0.02;
			}
			// Motor 10 clockwise
			else if (kbinput == 'v') {
				SendCommand.robot_command[9] -= 0.02;
			}
			// Motor 11 counter-clockwise
			else if (kbinput == 'g') {
				SendCommand.robot_command[10] += 0.02;
			}
			// Motor 11 clockwise
			else if (kbinput == 'b') {
				SendCommand.robot_command[10] -= 0.02;
			}
			// Motor 12 counter-clockwise
			else if (kbinput == 'h') {
				SendCommand.robot_command[11] += 0.02;
			}
			// Motor 12 clockwise
			else if (kbinput == 'n') {
				SendCommand.robot_command[11] -= 0.02;
			}
			// All motors set to 0
			else if (kbinput == '0') {
				for (int i = 0; i < MOTOR_NMB; i++)
					SendCommand.robot_command[i] = 0.0;
			}
			/****************************************************************/
		}

		// Clean screen
		if (j > 8000)
		{
			system("cls");
			j = 0;
		}
		j++;

		// Pack data as msgpack
		std::stringstream send_stream;
		msgpack::pack(send_stream, SendCommand);
		std::string const& data = send_stream.str();

		std::cout << SendCommand.robot_command[0] << " " << SendCommand.robot_command[1] << " " << SendCommand.robot_command[2]
			<< " " << SendCommand.robot_command[3] << " " << SendCommand.robot_command[4] << " " << SendCommand.robot_command[5]
			<< " " << SendCommand.robot_command[6] << " " << SendCommand.robot_command[7] << " " << SendCommand.robot_command[8]
			<< " " << SendCommand.robot_command[9] << " " << SendCommand.robot_command[10] << " " << SendCommand.robot_command[11] << endl;

		// Send data
		sendto(out, data.c_str(), data.size(), 0, (sockaddr*)&server, sizeof(server));

	}
	closesocket(out);

	WSACleanup();
}
