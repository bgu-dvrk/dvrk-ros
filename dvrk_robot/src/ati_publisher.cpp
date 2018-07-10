
// Reference for client socket: https://msdn.microsoft.com/en-us/library/windows/desktop/bb530741(v=vs.85).aspx
// Linux platform modification: https://stackoverflow.com/questions/28027937/cross-platform-sockets

// Steps:
// 1. Connecet ethernet cable between the two computers
// 2. ping 169.254.105.22 until there a connection is established
// 3. rosrun this code


#include <message_filters/subscriber.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdio.h>

// include - ros
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

// include headers based on OS
#if defined WIN32
#include <winsock.h>  // WinSock subsystem
#elif defined __linux__
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#endif

// redefine some types and constants based on OS
#if defined WIN32
typedef int socklen_t;  // Unix socket length
#elif defined __linux__
typedef int SOCKET;
#define INVALID_SOCKET -1  // WinSock invalid socket
#define SOCKET_ERROR   -1  // basic WinSock error
#define SHUTDOWN_ERROR  0  // close() socket error
#define closesocket(s) close(s);  // Unix uses file descriptors, WinSock doesn't...
#endif

using namespace std;

SOCKET s; // local socket
SOCKET ConnectSocket = INVALID_SOCKET;
#define DEFAULT_PORT "6861"
#define DEFAULT_SERVER_IP "169.254.105.22"
#define DEFAULT_BUFLEN 512

int start_client() {
	
	int iResult;

	struct addrinfo *result = NULL, *ptr = NULL, hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(DEFAULT_SERVER_IP, DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
	    printf("getaddrinfo failed: %d\n", iResult);
	    return 1;
	}

	// Attempt to connect to the first address returned by the call to getaddrinfo
	ptr=result;

	// Create a SOCKET for connecting to server
	ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

	if (ConnectSocket == INVALID_SOCKET) {
	    printf("Error at socket()");
	    freeaddrinfo(result);
	    return 1;
	}

	// Connect to server.
	iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
	    close(ConnectSocket);
	    ConnectSocket = INVALID_SOCKET;
	}

	// free the resources returned by getaddrinfo
	freeaddrinfo(result);
	if (ConnectSocket == INVALID_SOCKET) {
	    printf("Unable to connect to server!\n");
	    return 1;
	}
	
		
	printf("Connected...\n");
	return 0;
}

int close_socket() {
	int iResult;
	// shutdown the connection and close socket
	printf("Closing socket\n");
	iResult = shutdown(ConnectSocket, SHUT_RDWR);
	if (iResult == SHUTDOWN_ERROR) {
		close(ConnectSocket);
		printf("Socket closed\n");
		return 0;
	} else {
		return 1;
	}
}

int start_data_transfer(){

	int recvbuflen = DEFAULT_BUFLEN;
	char *sendbuf = "this is a test";
	char recvbuf[DEFAULT_BUFLEN];
	int iResult;
	int ret;

	ros::NodeHandle n;
	ros::Publisher ati_pub = n.advertise<std_msgs::Float32MultiArray>("ati_ft", 1);
	ros::Rate loop_rate(1000);

	// Send an initial buffer
	iResult = send(ConnectSocket, sendbuf, (int) strlen(sendbuf), 0);
	if (iResult == SOCKET_ERROR) {
	    printf("send failed\n");
	    close(ConnectSocket);
	    return 1;
	}

	printf("Bytes Sent: %ld\n", iResult);

	int count = 0;
	std_msgs::Float32MultiArray msg;
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
	msg.layout.dim[0].size = 6;
	msg.layout.dim[0].stride = 1;
	msg.layout.dim[0].label = "FT";

	// Receive data until the server closes the connection
	do {
		iResult = send(ConnectSocket, sendbuf, (int) strlen(sendbuf), 0);
		if (iResult == SOCKET_ERROR) {
		    printf("send failed\n");
		    return 1;
		}
	    	iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
		if (iResult > 0) {
			stringstream ss(recvbuf);
			vector<double> result;
			msg.data.clear();
			while( ss.good() ) {
			    	string substr;
			    	getline( ss, substr, ',' );
				result.push_back( atof(substr.c_str()) );
			}
			//printf("Bytes received: %d\n", iResult);
			//printf("Massged received: %s\n", recvbuf);
			//cout.precision(20);
			//for (int i = 0; i < 6; i++) {
			//	cout << "Value " << i << ": " << fixed << result.at(i) << endl;
			//}
			msg.data.insert(msg.data.end(), result.begin(), result.end());
			ati_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
			++count;
		} else if (iResult == 0) {
			printf("Connection closed\n");
		} else {
			printf("recv failed\n");
		}
	} while (iResult > 0 && ros::ok());
}


int main(int argc, char** argv)
{
	int ret;
	ros::init(argc, argv, "comm_data_node");
	ret = start_client();
	ret = start_data_transfer();
	ret = close_socket();
	//ros::spin();
	return 0;
}

   
