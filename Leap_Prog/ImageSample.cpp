
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <WS2tcpip.h>
#include <string>
#include <windows.h>
extern "C"
{
#include "LeapC.h"
#include "ExampleConnection.h"
}

#pragma comment (lib, "ws2_32.lib")
using namespace std;

//	The leap Motion sends data in continue
//	The value of SAMPLING defines the number of sample we will receive by the Leap Motion
//	By default SAMPLING is 10 to receive 1/10 data of the Leap Motion
//	If SAMPLING is 1, you will receive to many data to command the robot
#define SAMPLING 10
int cpt = 0;

//Struc send by socket
struct handData_s {
	bool handDetected;
	float x;
	float y;
	float z;
	float grabbingPercents;
	string toString(void);
};

//	Defines the socket format :
//	if handDetected
//		SOCKET = "Xhand Yhand Zhand grabbingPercents"
//	else SOCKET = 'none' (Note: 'none' is not recognize by Python, that is mean Python no receive socket instead of receive 'none')
string handData_s::toString(void)
{
	string result;
	if (handDetected == TRUE)
	{
		result = to_string(x);
		result += " ";
		result += to_string(y);
		result += " ";
		result += to_string(z);
		result += " ";
		result += to_string(grabbingPercents);
	}
	else result = 'none';
	return result;
}


/** Callback for when a frame of tracking data is available. */
// Send the socket with hand information
extern "C" static void OnFrame(const LEAP_TRACKING_EVENT *frame){
	handData_s Leap;
	// Structure to store the WinSock version. This is filled in
	  // on the call to WSAStartup()
	WSADATA data;

	// To start WinSock, the required version must be passed to
	// WSAStartup(). This server is going to use WinSock version
	// 2 so I create a word that will store 2 and 2 in hex i.e.
	// 0x0202
	WORD version = MAKEWORD(2, 2);
	// Start WinSock

	int wsOk = WSAStartup(version, &data);
	if (wsOk != 0)
	{
		// Not ok! Get out quickly
		cout << "Can't start Winsock! " << wsOk;
		return;
	}

	string s;
	int sendOk;
	////////////////////////////////////////////////////////////
	// CONNECT TO THE SERVER
	////////////////////////////////////////////////////////////
	if (cpt >= SAMPLING)
	{
		cpt = 0;
		// Create a hint structure for the server
		Leap.handDetected = false;
		sockaddr_in server;
		server.sin_family = AF_INET; // AF_INET = IPv4 addresses
		server.sin_port = htons(54001); // Little to big endian conversion
		inet_pton(AF_INET, "127.0.0.1", &server.sin_addr); // Convert from string to byte array

		// Socket creation, note that the socket type is datagram
		SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);
		system("cls");
		printf("Frame %lli with %i hands.\n", (long long int)frame->info.frame_id, frame->nHands);
		for (uint32_t h = 0; h < frame->nHands; h++) {
			LEAP_HAND* hand = &frame->pHands[h];
			printf("    Hand id %i is a %s hand with position (%f, %f, %f).\nHand is grabbing percentage: %f\n",
				hand->id,
				(hand->type == eLeapHandType_Left ? "left" : "right"),
				hand->palm.position.x,
				hand->palm.position.y,
				hand->palm.position.z,
				hand->pinch_strength);
			if (frame->nHands > 0)
			{
				Leap.handDetected = true;
			}
			else Leap.handDetected = false;
			Leap.x = hand->palm.position.x;
			Leap.y = hand->palm.position.y;
			Leap.z = hand->palm.position.z;
			Leap.grabbingPercents = hand->pinch_strength;
			s = Leap.toString();
			sendOk = sendto(out, s.c_str(), s.size() + 1, 0, (sockaddr*)&server, sizeof(server));
			//printf("SOCKET SENDED\n");
			if (sendOk == SOCKET_ERROR)
			{
				cout << "That didn't work! " << WSAGetLastError() << endl;
			}
		}
		// Close the socket
		closesocket(out);

		// Close down Winsock
		WSACleanup();
	}
	else
	{
		cpt++;
	}
	
}



int main(int argc, char** argv) {
	
   //Set callback function pointer
   ConnectionCallbacks.on_frame = &OnFrame;

   LEAP_CONNECTION *connection = OpenConnection();
   LeapSetPolicyFlags(*connection, eLeapPolicyFlag_Images, 0);
   printf("Press Enter to exit program.\n");
   getchar();

   return 0;
}