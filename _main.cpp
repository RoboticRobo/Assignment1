
#include <stdio.h>
#include <iostream>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

using namespace std;

#define Create_Comport "COM3"

// > cut is white
// < cut is black
#define cut 650

bool isRecord = true;
int frontLeft, frontRight, lefts, rights;

CreateData	robotData;
RobotConnector	robot;

void updateData() {
	cvWaitKey(20);

	if (!robot.ReadData(robotData))
		cout << "ReadData Fail" << endl;

	frontLeft = robotData.cliffSignal[1];
	frontRight = robotData.cliffSignal[2];
	lefts = robotData.cliffSignal[0];
	rights = robotData.cliffSignal[3];

}
void walk(double vl, double vr) {
	if (!robot.DriveDirect((int)(vl*Create_MaxVel), (int)(vr*Create_MaxVel)))
		cout << "SetControl Fail" << endl;
}

int findDirection() {

	updateData();
	
	// black and white
	return (frontLeft < cut && frontRight > cut);
}

void printSensor() {
	updateData();

	for (int i = 0; i < 4; i++) {
		cout << robotData.cliffSignal[i] << "\t";
	}
	cout << endl;

}

int getCut(int  type) {
	if (type == 0)
		return 990;
	else if (type == 1)
		return 644;
	else if (type == 2)
		return 834;
	else
		return 845;
}

bool isWhite(int val, int type) {
	return (val > getCut(type) );
}

bool isBlack(int val, int type) {
	return (val < getCut(type) );
}

int main()
{

	ofstream record;
	record.open("../data/robot.txt");

	if (!robot.Connect(Create_Comport))
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");


	// wait key to start track line
	char c = cvWaitKey(0);	

	// initialize sensor data
	updateData();

	if (c == 'a') {
		while (!(robotData.bumper[0] || robotData.bumper[1])) {
			updateData();
			walk(1, 1);
		}

		for (int i = 0; i < 10000; i++) {
			
			walk(-1, -1);
		}
		
		for (int i = 0; i < 10000; i++) {
			walk(1, -1);
		}
		walk(0, 0);

		while (!(isBlack(frontLeft, 1) && isBlack(frontRight, 2) && isBlack(lefts, 0) && isBlack(rights, 3))) {
			walk(1, 1);
		}
		while (!(isWhite(frontLeft, 1) && isWhite(frontRight, 2) && isWhite(lefts, 0) && isWhite(rights, 3))) {
			walk(1, 1);
		}
		while (!(isBlack(frontLeft, 1) && isBlack(frontRight, 2) && isBlack(lefts, 0) && isBlack(rights, 3))) {
			walk(1, 1);
		}
		while (!(isWhite(frontLeft, 1) && isWhite(frontRight, 2))) {
			walk(1, 1);
		}
		while (isWhite(frontLeft, 1) != isWhite(frontRight, 2)) {
			walk(1, -1);
		}
		
	}
	



	// find direction of robot
	bool clockwise = findDirection();


	// default is counter clockwise
	while (true)
	{
		char btn = cvWaitKey(20);
		if (btn == 'b') break;

		double vl = 0, vr = 0;

		//////////////////////////////////////////////
		// change direction of robot
		//////////////////////////////////////////////
		if (btn == 't') {
			while (true) {
				updateData();
				vl = -1;
				vr = 1;

				walk(vl, vr);

				if (clockwise && isWhite(frontLeft,1) && isBlack(frontRight,2))
					break;

				if (!clockwise && isBlack(frontLeft,1) && isWhite(frontRight,2))
					break;


			}
			clockwise = !clockwise;
		}
		//////////////////////////////////////////////



		//////////////////////////////////////////////
		// control (left, right and up
		//////////////////////////////////////////////
		// BOTH BLACK
		cout << lefts << " " << frontLeft << " " << frontRight << " " << rights << endl;
		if (isBlack(frontLeft, 1) && isBlack(frontRight, 2) && isBlack(lefts, 0) && isBlack(rights,3)) {
			// cout << "all black" b<< endl;
			while (true) {
				updateData();

				if (clockwise) {
					vl = 1;
					vr = 0.2;
				}
				else {
					vl = 0.2;
					vr = 1;
				}

				walk(vl, vr);

				if ((isWhite(frontLeft,1) && isWhite(frontRight,2)) || (isWhite(frontLeft, 1) != isWhite(frontRight, 2))) {
					break;
				}
			}
		}
		else if (isBlack(frontLeft,1) && isBlack(frontRight,2)) {
			vl = 1;
			vr = 0.65;
		}
		// BOTH WHITE
		else if (isWhite(frontLeft,1) && isWhite(frontRight,2)) {
			vl = 0.65;
			vr = 1;
		}
		// WHITE AND BLACK
		else if (isWhite(frontLeft,1) != isWhite(frontRight,2)) {
			vl = 2;
			vr = 2;
		}


		//////////////////////////////////////////////
		// swap velocity of each wheel if direction of robot is clockwise
		//////////////////////////////////////////////
		if (!clockwise) {
			double temp = vl;
			vl = vr;
			vr = temp;
		}

		//////////////////////////////////////////////
		// walk
		//////////////////////////////////////////////
		walk(vl, vr);


		updateData();
		//////////////////////////////////////////////
		// show value of cliff sensor
		//////////////////////////////////////////////
		//printSensor();


	}
		cvWaitKey(0);

	robot.Disconnect();

	return 0;
}
