
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
int rounds[6];
int turns = 0;

CreateData robotData;
RobotConnector robot;

void updateData()
{
	cvWaitKey(20);

	if (!robot.ReadData(robotData))
		cout << "ReadData Fail" << endl;

	frontRight = robotData.cliffSignal[2];
	frontLeft = robotData.cliffSignal[1];
	lefts = robotData.cliffSignal[0];
	rights = robotData.cliffSignal[3];
}
void walk(double vl, double vr)
{
	if (!robot.DriveDirect((int)(vl * Create_MaxVel), (int)(vr * Create_MaxVel)))
		cout << "SetControl Fail" << endl;
}

int findDirection()
{

	updateData();

	// black and white
	return (frontLeft < cut && frontRight > cut);
}

void printSensor()
{
	updateData();

	for (int i = 0; i < 4; i++)
	{
		cout << robotData.cliffSignal[i] << "\t";
	}
	cout << endl;
}

int getCut(int type)
{
	if (type == 0)
		return 990;
	else if (type == 1)
		return 644;
	else if (type == 2)
		return 834;
	else
		return 845;
}

bool isWhite(int val, int type)
{
	return (val > getCut(type));
}

bool isBlack(int val, int type)
{
	return (val < getCut(type));
}

bool findPath()
{
	while (!(robotData.bumper[0] || robotData.bumper[1]))
	{
		walk(1, 1);
		updateData();
	}

	// walk back
	walk(-1, -1);
	cvWaitKey(300);
	updateData();

	// spin 5 sec
	walk(1, -1);
	cvWaitKey(600);
	updateData();

	/* while (!(isBlack(frontLeft, 1) && isBlack(frontRight, 2) && isBlack(lefts, 0) && isBlack(rights, 3)))
	{
	walk(1, 1);
	} */
	while (!(isBlack(frontLeft, 1) && isBlack(frontRight, 2) && isBlack(lefts, 0) && isBlack(rights, 3)))
	{
		cout << "black 1" << endl;
		walk(1, 1);
		updateData();
		if ((robotData.bumper[0] || robotData.bumper[1]))
			return false;
	}
	while (!(isWhite(frontLeft, 1) && isWhite(frontRight, 2) && isWhite(lefts, 0) && isWhite(rights, 3)))
	{
		cout << "white 1" << endl;
		walk(1, 1);
		updateData();
		if ((robotData.bumper[0] || robotData.bumper[1]))
			return false;
	}
	while (!(isBlack(frontLeft, 1) && isBlack(frontRight, 2) && isBlack(lefts, 0) && isBlack(rights, 3)))
	{
		cout << "black 2" << endl;
		walk(1, 1);
		updateData();
		if ((robotData.bumper[0] || robotData.bumper[1]))
			return false;
	}
	while (!(isWhite(frontLeft, 1) && isWhite(frontRight, 2)))
	{
		cout << "white 2" << endl;
		walk(1, 1);
		updateData();
		if ((robotData.bumper[0] || robotData.bumper[1]))
			return false;
	}
	while (isWhite(frontLeft, 1) == isWhite(frontRight, 2))
	{
		cout << "rotate" << endl;
		walk(1, -1);
		updateData();
	}

	return true;
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

	rounds[0] = 0;
	rounds[1] = 0;
	rounds[2] = 0;
	rounds[3] = 0;
	rounds[4] = 0;
	rounds[5] = 0;

	// wait key to start track line
	char c = cvWaitKey(0);

	// initialize sensor data
	updateData();

	// bump and find the path
	if (c == 'a')
	{
		bool onPath = false;
		onPath = findPath();
		while (!onPath)
		{
			onPath = findPath();
		}
	}

	// find direction of robot
	bool clockwise = findDirection();

	// default is counter clockwise
	while (true)
	{
		char btn = cvWaitKey(20);
		if (btn == 'b')
			break;

		double vl = 0, vr = 0;

		//////////////////////////////////////////////
		// change direction of robot
		//////////////////////////////////////////////
		if (btn == 't')
		{
			while (true)
			{
				updateData();
				vl = -1;
				vr = 1;

				walk(vl, vr);

				if (clockwise && isWhite(frontLeft, 1) && isBlack(frontRight, 2))
					break;

				if (!clockwise && isBlack(frontLeft, 1) && isWhite(frontRight, 2))
					break;
			}
			clockwise = !clockwise;
		}
		//////////////////////////////////////////////

		//////////////////////////////////////////////
		// control (left, right and up
		//////////////////////////////////////////////
		// BOTH BLACK
		if (isBlack(frontLeft, 1) && isBlack(frontRight, 2) && isBlack(lefts, 0) && isBlack(rights, 3))
		{
			while (true)
			{
				updateData();

				if (clockwise)
				{
					vl = 1;
					vr = 0.35;
				}
				else
				{
					vl = 0.35;
					vr = 1;
				}

				walk(vl, vr);

				if ((isWhite(frontLeft, 1) && isWhite(frontRight, 2)) || (isWhite(frontLeft, 1) != isWhite(frontRight, 2)))
				{
					cvWaitKey(50);
					turns += 1;
					//turns += 1;
					//	if (turns > 4) turns = 4;
					break;
				}
			}
		}
		else if (isBlack(frontLeft, 1) && isBlack(frontRight, 2))
		{
			vl = 1;
			vr = 0.6;
		}
		// BOTH WHITE
		else if (isWhite(frontLeft, 1) && isWhite(frontRight, 2))
		{
			vl = 0.6;
			vr = 1;
		}
		// WHITE AND BLACK
		else if (isWhite(frontLeft, 1) != isWhite(frontRight, 2))
		{
			vl = 1;
			vr = 1;
		}

		//////////////////////////////////////////////
		// swap velocity of each wheel if direction of robot is clockwise
		//////////////////////////////////////////////
		if (!clockwise)
		{
			double temp = vl;
			vl = vr;
			vr = temp;
		}

		//////////////////////////////////////////////
		// walk
		//////////////////////////////////////////////
		if (turns > 4)
			break;
		walk(vl, vr);
		rounds[turns] += 1;

		if (turns == 4 && rounds[4] > (rounds[2] - rounds[0]))
		{
			cout << rounds[0] << " " << rounds[1] << " " << rounds[2] << " " << rounds[3] << " " << rounds[4] << endl;
			break;
		}

		updateData();
		//////////////////////////////////////////////
		// show value of cliff sensor
		//////////////////////////////////////////////
		//printSensor();
	}

	robot.Disconnect();

	return 0;
}