
#include <stdio.h>
#include <iostream>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

using namespace std;

#define Create_Comport "COM3"

bool isRecord = true;

int main()
{
	CreateData	robotData;
	RobotConnector	robot;

	ofstream	record;
	record.open("../data/robot.txt");

	if( !robot.Connect(Create_Comport) )
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");

	char c = cvWaitKey(0);
	bool clockwise = true;

	int frontLeft = robotData.cliffSignal[2];
	int frontRight = robotData.cliffSignal[1];


	for (int i = 0; i < 100; i++) {
		if (frontLeft > 720 && frontRight < 650) {
			clockwise = true;
		}
		else {
			clockwise = false;
		}
	}

	while(true)
	{
		char stop = cvWaitKey(30);
		if (stop == 'b') break;
		double vl = 0;
		double vr = 0;

		frontLeft = robotData.cliffSignal[2];
		frontRight = robotData.cliffSignal[1];
		
		if (stop == 't') {
			if (clockwise) {
				while (!(frontLeft > 720 && frontRight < 650)) {
					frontLeft = robotData.cliffSignal[2];
					frontRight = robotData.cliffSignal[1];
					vl = -1;
					vr = 1;

					if (!robot.DriveDirect((int)(vl*Create_MaxVel), (int)(vr*Create_MaxVel)))
						cout << "SetControl Fail" << endl;                   


					if (!robot.ReadData(robotData))
						cout << "ReadData Fail" << endl;


					cout << frontLeft << " " << frontRight << endl;
					cvWaitKey(30);

				}

			}
			else {
				while (!(frontRight > 720 && frontLeft < 650)) {
					frontLeft = robotData.cliffSignal[2];
					frontRight = robotData.cliffSignal[1];

					vl = -1;
					vr = 1;
					if (!robot.DriveDirect((int)(vl*Create_MaxVel), (int)(vr*Create_MaxVel)))
						cout << "SetControl Fail" << endl;


					if (!robot.ReadData(robotData))
						cout << "ReadData Fail" << endl;

					cout << frontLeft << " " << frontRight << endl;

					cvWaitKey(30);
				}
			}		
			clockwise = !clockwise;
		}

		if (frontLeft > 720 && frontRight < 650) {
			vl = 2;
			vr = 2;
		}

		if (frontLeft < 720 && frontRight > 650) {
			vl = 2;
			vr = 2;
		}

		if (frontLeft < 700 && frontRight < 600) {
			if (clockwise) {
				vl = 0.5;
				vr = 1;
			}
			else {
				vl = 1;
				vr = 0.5;
			}
		}

		if (frontLeft > 700 && frontRight > 600) {
			if (clockwise) {
				vl = 1;
				vr = 0.5;
			}
			else {
				vl = 0.5;
				vr = 1;
			}
		}		

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		
		if( !robot.DriveDirect(velL, velR) )
			cout << "SetControl Fail" << endl;

		if( !robot.ReadData(robotData) )
			cout << "ReadData Fail" << endl;

		cout << robotData.cliffSignal[0] << "\t" << robotData.cliffSignal[1] << "\t" << robotData.cliffSignal[2] << "\t" << robotData.cliffSignal[3] << endl;
	}

	robot.Disconnect();

	return 0;
}