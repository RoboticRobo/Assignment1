
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
int frontLeft, frontRight;

CreateData	robotData;
RobotConnector	robot;

void updateData() {
    if( !robot.ReadData(robotData) )
        cout << "ReadData Fail" << endl;

    frontLeft = robotData.cliffSignal[1];
    frontRight = robotData.cliffSignal[2];
}

void walk(int vl, int vr) {
    if( !robot.DriveDirect((int)(vl*Create_MaxVel), (int)(vr*Create_MaxVel)) )
        cout << "SetControl Fail" << endl;
}

int findDirection(){
    updateData();

    // black and white
    return (frontLeft < cut && frontRight > cut);
}

void printSensor() {
    updateData();

    for(int i = 0; i < 4; i++) {
        cout << robotData.cliffSignal[i] << "\t";
    }
    cout << endl;
}

bool isWhite(int val) {
    return (val > cut);
}

bool isBlack(int val) {
    return (val < cut);
}

int main()
{

    ofstream record;
	record.open("../data/robot.txt");

	if( !robot.Connect(Create_Comport) )
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");


    // wait key to start track line
	char c = cvWaitKey(0);

    // find direction of robot
    bool clockwise = findDirection();


    // default is counter clockwise
	while(true)
	{
		char btn = cvWaitKey(30);
		if (btn == 'b') break;

		double vl = 0, vr = 0;

        updateData();

        //////////////////////////////////////////////
        // change direction of robot
        //////////////////////////////////////////////
		if (btn == 't') {
            while(true) {
                updateData();
                vl = -1;
                vr = 1;

                walk(vl, vr);
                cvWaitKey(30);

                if ( clockwise && isWhite(frontLeft) && isBlack(frontRight))
                    break;

                if (!clockwise && isBlack(frontLeft) && isWhite(frontRight))
                    break;

            }
			clockwise = !clockwise;
		}
		//////////////////////////////////////////////



        //////////////////////////////////////////////
        // control (left, right and up
        //////////////////////////////////////////////
		// WHITE AND BLACK
		if (isWhite(frontLeft) ^ isWhite(frontRight)) {
			vl = 2;
			vr = 2;
		}
        // BOTH BLACK
		if (isBlack(frontLeft) && isBlack(frontRight)) {
            vl = 1;
            vr = 0.5;
		}
        // BOTH WHITE
		if (isWhite(frontLeft) && isWhite(frontRight)) {
            vl = 0.5;
            vr = 1;
		}


        //////////////////////////////////////////////
        // swap velocity of each wheel if direction of robot is clockwise
        //////////////////////////////////////////////
		if(clockwise) {
            double temp = vl;
            vl = vr;
            vr = temp;
		}


        //////////////////////////////////////////////
        // walk
        //////////////////////////////////////////////
        walk(vl, vr);


        //////////////////////////////////////////////
        // show value of cliff sensor
        //////////////////////////////////////////////
        printSensor();
	}

	robot.Disconnect();

	return 0;
}
