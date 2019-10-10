#include "KinovaTypes.h"
#include <iostream>
#ifdef __linux__ 
#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <stdio.h>
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#include "CommunicationLayer.h"
#include "CommandLayer.h"
#include <conio.h>
#include <iostream>
#endif


using namespace std;

//A handle to the API.
#ifdef __linux__ 
void * commandLayer_handle;
#elif _WIN32
HINSTANCE commandLayer_handle;
#endif

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetAngularCommand)(AngularPosition &);

int main(int argc, char* argv[])
{
	int programResult = 0;

#ifdef __linux__ 
	//We load the API
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyMoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
	MyInitFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");
	MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
	MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");
#elif _WIN32
	//We load the API.
	commandLayer_handle = LoadLibrary(TEXT("CommandLayerWindows.dll"));

	//We load the functions from the library
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyGetDevices = (int(*)(KinovaDevice[MAX_KINOVA_DEVICE], int&)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	MyGetAngularCommand = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularCommand");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
#endif

	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetAngularCommand == NULL) ||
		(MyMoveHome == NULL) || (MyInitFingers == NULL))

	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
		programResult = 0;
	}
	else
	{
		cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

		int result = (*MyInitAPI)();

		AngularPosition currentCommand;

		cout << "Initialization's result :" << result << endl;

		KinovaDevice list[MAX_KINOVA_DEVICE];

		int devicesCount = MyGetDevices(list, result);

		for (int i = 0; i < devicesCount; i++)
		{
			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

			//Setting the current device as the active device.
			MySetActiveDevice(list[i]);

			cout << "Send the robot to HOME position" << endl;
			MyMoveHome();

			cout << "Initializing the fingers" << endl;
			MyInitFingers();

			TrajectoryPoint pointToSend;
			pointToSend.InitStruct();

			//We specify that this point will be used an angular(joint by joint) velocity vector.
			pointToSend.Position.Type = ANGULAR_VELOCITY;

			pointToSend.Position.Actuators.Actuator1 = 0;
			pointToSend.Position.Actuators.Actuator2 = 0;
			pointToSend.Position.Actuators.Actuator3 = 0;
			pointToSend.Position.Actuators.Actuator4 = 0;
			pointToSend.Position.Actuators.Actuator5 = 0;
			pointToSend.Position.Actuators.Actuator6 = 48; //joint 6 at 48 degrees per second.

			pointToSend.Position.Fingers.Finger1 = 0;
			pointToSend.Position.Fingers.Finger2 = 0;
			pointToSend.Position.Fingers.Finger3 = 0;

			for (int i = 0; i < 300; i++)
			{
				//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
				MySendBasicTrajectory(pointToSend);
#ifdef __linux__ 
		 		usleep(5000);
#elif _WIN32
				Sleep(5);	
#endif
			}

			pointToSend.Position.Actuators.Actuator6 = -20; //joint 6 at -20 degrees per second.

			for (int i = 0; i < 300; i++)
			{
				//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
				MySendBasicTrajectory(pointToSend);
#ifdef __linux__ 
		 		usleep(5000);
#elif _WIN32
				Sleep(5);	
#endif
			}

			cout << "Send the robot to HOME position" << endl;
			MyMoveHome();

			//We specify that this point will be an angular(joint by joint) position.
			pointToSend.Position.Type = ANGULAR_POSITION;

			//We get the actual angular command of the robot.
			MyGetAngularCommand(currentCommand);

			pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1 + 30;
			pointToSend.Position.Actuators.Actuator2 = currentCommand.Actuators.Actuator2;
			pointToSend.Position.Actuators.Actuator3 = currentCommand.Actuators.Actuator3;
			pointToSend.Position.Actuators.Actuator4 = currentCommand.Actuators.Actuator4;
			pointToSend.Position.Actuators.Actuator5 = currentCommand.Actuators.Actuator5;
			pointToSend.Position.Actuators.Actuator6 = currentCommand.Actuators.Actuator6;

			cout << "*********************************" << endl;
			cout << "Sending the first point to the robot." << endl;
			MySendBasicTrajectory(pointToSend);

			pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1 - 60;
			cout << "Sending the second point to the robot." << endl;
			MySendBasicTrajectory(pointToSend);

			pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1;
			cout << "Sending the third point to the robot." << endl;
			MySendBasicTrajectory(pointToSend);

			cout << "*********************************" << endl << endl << endl;
		}

		cout << endl << "WARNING: Your robot is now set to angular control. If you use the joystick, it will be a joint by joint movement." << endl;
		cout << endl << "C L O S I N G   A P I" << endl;
		result = (*MyCloseAPI)();
		programResult = 1;
	}

#ifdef __linux__ 
	dlclose(commandLayer_handle);
#elif _WIN32
	FreeLibrary(commandLayer_handle);
#endif

	return programResult;

}
