// DifferentialDriveRobot.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <conio.h>
#include <ctime>
#include <random>

#include "LinearAccDeceleration.h"
#include "RotationalMovement.h"
#include "TestOutputVelocityFile.h"
//#include "FollowQRCode.h"
#include "FollowQRCode_2.h"
#include "TrajectoryClosedLoop.h"
#include "TrajectoryClosedLoop2.h"
#include "TrajectoryClosedLoop3.h"
#include "ReadWriteFile.h"
#include "TestKALMANFilter.h"


double RPMToVelocity(double RPM)
{
    double radius = 13.5; // [unit: cm]
    return (2 * PI * radius * RPM) / 60;
}

void TestLinearAccDeceleration()
{
    FILE* file = fopen("D:/LinearAccDeceleration_Setting.txt", "r");
    double distance = GetValueFromFile(file, "TotalDistance[cm]");
    double velocityStart = GetValueFromFile(file, "StartVelocity[cm/sec]");            //[unit:cm / s]
    double maxVelocity = GetValueFromFile(file, "MaxVelocity[cm/sec]");              //[unit:cm / s]
    double veclocityEnd = GetValueFromFile(file, "EndVelocity[cm/sec]");
    double Acceleration = GetValueFromFile(file, "Acceleration[cm/sec^2]");
    double Deceleration = GetValueFromFile(file, "Deceleration[cm/sec^2]");
    printf("---- Setting File -----\n");
    printf("distance %f\n", distance);
    printf("velocityStart %f\n", velocityStart);
    printf("maxVelocity %f\n", maxVelocity);
    printf("veclocityEnd %f\n", veclocityEnd);
    double startTime = 0.1;
    double InitPositionTime = 0.1;
    

    CLinearAccDeceleration* CLinearAccDecelerationObj = new CLinearAccDeceleration();
    CLinearAccDecelerationObj->m_dMaxVelocity = maxVelocity;
    CLinearAccDecelerationObj->MAX_ACCELERATION = Acceleration;
    CLinearAccDecelerationObj->MAX_DECELERATION = Deceleration;
    CLinearAccDecelerationObj->Set(distance, velocityStart, veclocityEnd, startTime, InitPositionTime);

    double v1 = velocityStart;
    double v2 = CLinearAccDecelerationObj->m_dVelocityMax;
    double v3 = veclocityEnd;
    double t1 = CLinearAccDecelerationObj->m_dAccelerationTime;
    double t2 = CLinearAccDecelerationObj->m_dConstantVelocityTime;
    double t3 = CLinearAccDecelerationObj->m_dDecelerationTime;
    printf("\n---- Result File -----\n");
    printf("Start Velocity (cm/sec): %f \n", v1);
    printf("Constant Velocity (cm/sec): %f \n", v2);
    printf("End Velocity (cm/sec): %f \n", v3);
    printf("Acceleration Time (sec): %f \n", t1);
    printf("Constant Velocity Time (sec): %f \n", t2);
    printf("Deceleration Time (sec): %f \n", t3);

    CLinearAccDecelerationObj->WriteFile();
}

void TestRotationalMotion()
{
    CRotationalMotion* CRotationalMotionObj = new CRotationalMotion();
    CRotationalMotionObj->m_bClockwise = true;
    CRotationalMotionObj->m_dStartTime = 0.0;
    CRotationalMotionObj->RotationAboutCM(90, 15);

    printf("\n------ Rotational parameters-------\n");
    printf("Total Time Taken: %f\n", CRotationalMotionObj->m_dTotalTime);
    printf("Left Wheel velocity: %f\n", CRotationalMotionObj->m_dLWVelocity);
    printf("Right Wheel velocity: %f\n", CRotationalMotionObj->m_dRWVelocity);
}

void TestVelocityFile()
{
    CTestOutputVelocity* CTestOutputVelocityObj = new CTestOutputVelocity();
    //CTestOutputVelocityObj->ReadFile();
}

void TestFollowQRCode()
{
    CFollowRobot* CFollowRobotObj = new CFollowRobot();
    CFollowRobotObj->Initialize();
}

void TestFollowTrajectory()
{
    //CFollowTrajectory_ver1* CFollowTrajectoryObj = new CFollowTrajectory_ver1();
    //CFollowTrajectoryObj->Initialize();
    CIMRFollowTrajectory* CIMRFollowTrajectoryObj = new CIMRFollowTrajectory();
    CIMRFollowTrajectoryObj->Initialize();
}


void TestKalmanfilter()
{
    CTestKalmanFilter* CTestKalmanFilterObj = new CTestKalmanFilter();
    CTestKalmanFilterObj->Initialize();
}

int main()
{
    srand((unsigned)time(NULL));
    //TestLinearAccDeceleration();
    //TestRotationalMotion();
    //TestVelocityFile();
    //TestFollowQRCode();
    TestFollowTrajectory();
    //TestKalmanfilter();
    
    
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
