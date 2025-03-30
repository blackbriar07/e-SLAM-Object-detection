#pragma once
#include <math.h>
#include <vector>
#include <conio.h>
#include "Geometry.h"
#include "PIDControl.h"

class CTrajectoryGoal
{
public:
	Pose m_GoalPose;
	double m_NeighbourhoodRadius;
	double m_thresholdOrientation;
	int CurrentGoal; 
	int TotalNumofGoals;
	int PathTrajectoryCase;
	double m_fXpathData[1000] = {0};
	double m_fYpathData[1000] = {0};
	bool PathtextData;
public:
	CTrajectoryGoal(bool ExternalData = false)
	{
		m_NeighbourhoodRadius = 5.0; //cm
		m_thresholdOrientation = 2.0;		
		PathTrajectoryCase = 2;
		TotalNumofGoals = PathTrajectoryCase;
		CurrentGoal = -1;
		PathtextData = ExternalData;
		if (!PathtextData)
			ChangeGoal();
		else
		{
			FILE* file = fopen("PathData/StraightLineData_raw.txt", "r");
			ReadRawTestPathData(file);
			//WriteDataIntoArray();
			ChangeGoalTextData();
		}
	}
	void WriteDataIntoArray()
	{
		FILE* fileOut = fopen("PathData/StraightLineData_Array.txt", "w");
		fprintf(fileOut, "%d,\n", TotalNumofGoals);
		for (int i = 0; i < TotalNumofGoals; i++)
		{
			fprintf(fileOut, "{%.2f,%.2f,%.2f},\n", m_fXpathData[i], m_fYpathData[i], 0.0);
		}
		fclose(fileOut);
		printf("printing done\n");
	}
	bool GoalReached(Pose pIMRPose, double& DistanceError, double& OrientationError)
	{
		double XError = pIMRPose.m_Point.x - m_GoalPose.m_Point.x;
		double YError = pIMRPose.m_Point.y - m_GoalPose.m_Point.y;
		DistanceError = sqrt(XError * XError + YError * YError);
		OrientationError = pIMRPose.m_dOrientation - m_GoalPose.m_dOrientation;
		if (DistanceError < m_NeighbourhoodRadius && OrientationError < m_thresholdOrientation)
			return true;
		else
			return false;

	}
	void ChangeGoalTextData()
	{
		CurrentGoal++;
		m_GoalPose.m_Point.x = m_fXpathData[CurrentGoal]; //400.0;
		m_GoalPose.m_Point.y = m_fYpathData[CurrentGoal]; //25
		m_GoalPose.m_dOrientation = 0.000;//PI;
	}
	void ChangeGoal()
	{
		CurrentGoal++;
		switch (PathTrajectoryCase)
		{			
			case 1:
				if (CurrentGoal == 0)
				{
					m_GoalPose.m_Point.x = 400.00001; //400.0;
					m_GoalPose.m_Point.y = 0.00001; //25
					m_GoalPose.m_dOrientation = PI;//PI;
				}
				break;
			case 2:
				if (CurrentGoal == 0)
				{
					m_GoalPose.m_Point.x = 400.0;
					m_GoalPose.m_Point.y = 0.0; //25
					m_GoalPose.m_dOrientation = PI;
				}
				if (CurrentGoal == 1)
				{
					m_GoalPose.m_Point.x = 400.0;
					m_GoalPose.m_Point.y = 100.0; //25
					m_GoalPose.m_dOrientation = PI;
				}
				break;
			case 3:
				if (CurrentGoal == 0)
				{
					m_GoalPose.m_Point.x = 400.0;
					m_GoalPose.m_Point.y = 0.0; //25
					m_GoalPose.m_dOrientation = PI;
				}
				if (CurrentGoal == 1)
				{
					m_GoalPose.m_Point.x = 400.0;
					m_GoalPose.m_Point.y = 100.0; //25
					m_GoalPose.m_dOrientation = PI;
				}
				if (CurrentGoal == 2)
				{
					m_GoalPose.m_Point.x = 0.0;
					m_GoalPose.m_Point.y = 100.0;
					m_GoalPose.m_dOrientation = PI;
				}
				break;
			case 4:
				if (CurrentGoal == 0)
				{
					m_GoalPose.m_Point.x = 400.0;
					m_GoalPose.m_Point.y = 0.0; //25
					m_GoalPose.m_dOrientation = PI;
				}
				if (CurrentGoal == 1)
				{
					m_GoalPose.m_Point.x = 400.0;
					m_GoalPose.m_Point.y = 100.0; //25
					m_GoalPose.m_dOrientation = PI;
				}
				if (CurrentGoal == 2)
				{
					m_GoalPose.m_Point.x = 0.0;
					m_GoalPose.m_Point.y = 100.0;
					m_GoalPose.m_dOrientation = PI;
				}
				if (CurrentGoal == 3)
				{
					m_GoalPose.m_Point.x = 0.0;
					m_GoalPose.m_Point.y = 0.0;
					m_GoalPose.m_dOrientation = PI;
				}
				break;
		}			
	}
	void ReadRawTestPathData(FILE* file)
	{
		if (file == nullptr) {
			perror("Error opening file");
			return;
		}

		// Read and print each line of the file
		char buffer[256];
		float t, x, y;   // Variables to store extracted values
		int Count = 0;
		while (fgets(buffer, sizeof(buffer), file) != nullptr)
		{
			//printf("%s", buffer);
			if (sscanf(buffer, "T%f X%f Y%f", &t, &x, &y) == 3) {
				// Print the extracted values
				m_fXpathData[Count] = (double)x;
				m_fYpathData[Count] = (double)y;
				//printf("T: %.6f, X: %.6f, Y: %.6f\n", t, x, y);
			}
			else {
				// Handle lines that do not match the expected format
				fprintf(stderr, "Line format incorrect: %s", buffer);
			}
			Count++;
		}
		TotalNumofGoals = Count;
		fclose(file);
		return;
	}
};

class CIMR
{
public:
	Pose m_CurrentPose;
	Pose m_PreviousPose;
	double m_fVelocityLeftWheel;
	double m_fVelocityRightWheel;
	double m_fLinearVelocity;
	double m_fAngularVelocity;
	double m_fDistanceIMRToGoal;
	double m_fDeltaTheta;
	double m_fTotalTime;
	double m_fMaintainDistance;
	double m_dWheelRadius;
	bool m_bGoalReached;
	bool m_bPositionControl;
	bool m_bOrientationControl;
	CGeometry CGeometryobj;

public:
	CIMR()
	{
		m_CurrentPose.m_Point.x = -100.0000001;
		m_CurrentPose.m_Point.y = 160.0000001;
		m_CurrentPose.m_dOrientation = 0.0;
		m_PreviousPose.m_Point.x = 0.0;
		m_PreviousPose.m_Point.y = 0.0;
		m_PreviousPose.m_dOrientation = 0.0;
		m_fVelocityLeftWheel =  21.6;
		m_fVelocityRightWheel = 21.6;
		m_fLinearVelocity = 21.6;
		m_fAngularVelocity = 0.0;
		m_fMaintainDistance = 100;
		m_fDistanceIMRToGoal = 0.0;
		m_dWheelRadius = 13.5;
		m_fDeltaTheta = 0.0;
		m_fTotalTime = 0.0;
		m_bGoalReached = false;
		m_bPositionControl = false;
		m_bOrientationControl = false;
	}
	void UpdatePose()
	{
		m_PreviousPose = m_CurrentPose;
		m_CurrentPose.m_Point.x += ((m_fVelocityLeftWheel + m_fVelocityRightWheel) / 2) * cos(m_PreviousPose.m_dOrientation) * (SamplingTime * 0.001);
		m_CurrentPose.m_Point.y += ((m_fVelocityLeftWheel + m_fVelocityRightWheel) / 2) * sin(m_PreviousPose.m_dOrientation) * (SamplingTime * 0.001);
		m_CurrentPose.m_dOrientation += ((m_fVelocityRightWheel - m_fVelocityLeftWheel) / IMRWIDTH) * (SamplingTime * 0.001);

		printf("(X, Y, theta): (%f,%f,%f)\n", m_CurrentPose.m_Point.x, m_CurrentPose.m_Point.y, m_CurrentPose.m_dOrientation);
		//m_CurrentPose.m_Point.x = Disturbance(m_CurrentPose.m_Point.x);
		//m_CurrentPose.m_Point.y = Disturbance(m_CurrentPose.m_Point.y);
	}
	void CalculateError(CTrajectoryGoal* TrajectoryGoalObject)
	{
		TrajectoryGoalObject->GoalReached(m_CurrentPose, m_fDistanceIMRToGoal, m_fDeltaTheta);
		//printf("m_fDeltaTheta rad %f\n", m_fDeltaTheta);
		//printf("m_fDeltaTheta deg %f\n", m_fDeltaTheta * r2d);
	}
	void LimitVelocities()
	{
		if (m_fVelocityLeftWheel > 0)
		{
			if (m_fVelocityLeftWheel < MIN_VELOCITY)
				m_fVelocityLeftWheel = MIN_VELOCITY;
			if (m_fVelocityLeftWheel > MAX_VELOCITY)
				m_fVelocityLeftWheel = MAX_VELOCITY;
		}
		if (m_fVelocityLeftWheel < 0)
		{
			if (fabs(m_fVelocityLeftWheel) < MIN_VELOCITY)
				m_fVelocityLeftWheel = -MIN_VELOCITY;
			if (fabs(m_fVelocityLeftWheel) > MAX_VELOCITY)
				m_fVelocityLeftWheel = -MAX_VELOCITY;
		}
		if (m_fVelocityRightWheel > 0)
		{
			if (m_fVelocityRightWheel < MIN_VELOCITY)
				m_fVelocityRightWheel = MIN_VELOCITY;
			if (m_fVelocityRightWheel > MAX_VELOCITY)
				m_fVelocityRightWheel = MAX_VELOCITY;
		}
		if (m_fVelocityRightWheel < 0)
		{
			if (fabs(m_fVelocityRightWheel) < MIN_VELOCITY)
				m_fVelocityRightWheel = -MIN_VELOCITY;
			if (fabs(m_fVelocityRightWheel) > MAX_VELOCITY)
				m_fVelocityRightWheel = -MAX_VELOCITY;
		}
	}
	void Control_GetToGoalPosition(CTrajectoryGoal* GoalPose, bool ChangeOrientation = false)
	{
		//if (ChangeOrientation)
		PID_OrientationControl(GoalPose);
		PID_PositionControl(GoalPose);
		
		printf("m_fVelocityLeftWheel %f\n", m_fVelocityLeftWheel);
		printf("m_fVelocityRightWheel %f\n", m_fVelocityRightWheel);
		
		if (fabs(m_fVelocityLeftWheel) > 0.5)
			m_fVelocityLeftWheel = Disturbance(m_fVelocityLeftWheel, -1, 1);
		if (fabs(m_fVelocityRightWheel) > 0.5)
			m_fVelocityRightWheel = Disturbance(m_fVelocityRightWheel, -1, 1);
		
		printf("m_fVelocityLeftWheel after randomness %f\n", m_fVelocityLeftWheel);
		printf("m_fVelocityRightWheel after randomness %f\n", m_fVelocityRightWheel);

		LimitVelocities();

		/*
		if (m_fVelocityLeftWheel > MAX_VELOCITY)
			m_fVelocityLeftWheel = MAX_VELOCITY;
		if (m_fVelocityRightWheel > MAX_VELOCITY)
			m_fVelocityRightWheel = MAX_VELOCITY;
		*/

		printf("m_fVelocityLeftWheel after normalization %f\n", m_fVelocityLeftWheel);
		printf("m_fVelocityRightWheel after normalization %f\n", m_fVelocityRightWheel);

		//printf("m_fVelocityLeftWheel %f\n", m_fVelocityLeftWheel);
		//printf("m_fVelocityLeftWheel %f\n", m_fVelocityLeftWheel);
	}
	void PID_PositionControl(CTrajectoryGoal* GoalPose)
	{
		if (!m_bPositionControl)
			return;
		double x_error = (m_CurrentPose.m_Point.x - GoalPose->m_GoalPose.m_Point.x);
		double y_error = (m_CurrentPose.m_Point.y - GoalPose->m_GoalPose.m_Point.y);
		double distance_error = x_error * x_error + y_error * y_error;//m_fDistanceIMRToGoal * m_fDistanceIMRToGoal;
		printf("GoalPose:(%f,%f,%f) CurrentPose:(%f,%f,%f)\n", GoalPose->m_GoalPose.m_Point.x, GoalPose->m_GoalPose.m_Point.y,
			GoalPose->m_GoalPose.m_dOrientation, m_CurrentPose.m_Point.x, m_CurrentPose.m_Point.y, m_CurrentPose.m_dOrientation);
		double e_theta = CGeometryobj.DetermineOrientationNeeded(m_CurrentPose.m_Point, GoalPose->m_GoalPose.m_Point, m_CurrentPose.m_dOrientation);
		//double theta_m = atan2(GoalPose->m_GoalPose.m_Point.y - m_CurrentPose.m_Point.y, GoalPose->m_GoalPose.m_Point.x - m_CurrentPose.m_Point.x);
		//printf("theta_m %f\n", theta_m);
		//double e_theta = theta_m - m_CurrentPose.m_dOrientation;
		printf("etheta %f\n", e_theta);
		
		double theta_m_dot = ((m_CurrentPose.m_Point.x - GoalPose->m_GoalPose.m_Point.x) * ((m_fVelocityLeftWheel + m_fVelocityRightWheel) / 2) * sin(m_PreviousPose.m_dOrientation)
			- (m_CurrentPose.m_Point.y - GoalPose->m_GoalPose.m_Point.y) * ((m_fVelocityLeftWheel + m_fVelocityRightWheel) / 2) * cos(m_PreviousPose.m_dOrientation)) / (distance_error);
		printf("dtheta %f\n", theta_m_dot);
		
		double e_theta_dot = theta_m_dot - ((m_fVelocityRightWheel - m_fVelocityLeftWheel) / IMRWIDTH);
		printf("detheta %f\n", e_theta_dot);
		
		double kP = 30.5;//95.6;//17.0; //34.6
		double kD = 1.0;//30.6;//12.5; //1.6
		double PI_value = kP * e_theta + kD * e_theta_dot;
		m_fAngularVelocity = PI_value;
		if (m_fDistanceIMRToGoal < 0.2)
		{	
			m_bPositionControl = true;
			m_bOrientationControl = false;
			printf("Position Goal Reached\n");
			if (GoalPose->CurrentGoal == (GoalPose->TotalNumofGoals - 1))
			{
				m_bGoalReached = true;	
				m_fLinearVelocity = 0.0;
				m_fAngularVelocity = 0.0;
				m_fVelocityLeftWheel = m_fLinearVelocity - m_fAngularVelocity;
				m_fVelocityRightWheel = m_fLinearVelocity + m_fAngularVelocity;
				return;
			}
			m_fLinearVelocity = 0.0;
			m_fAngularVelocity = 0.0;
			if (!GoalPose->PathtextData)
				GoalPose->ChangeGoal();
			else
				GoalPose->ChangeGoalTextData();
		}
		//else
		//{
			printf("m_fLinearVelocity %f\n", m_fLinearVelocity);
			printf("m_fAngularVelocity %f\n", m_fAngularVelocity);
			m_fVelocityLeftWheel = m_fLinearVelocity - m_fAngularVelocity;
			m_fVelocityRightWheel = m_fLinearVelocity + m_fAngularVelocity;
			printf("m_fVelocityLeftWheel %f\n", m_fVelocityLeftWheel);
			printf("m_fVelocityRightWheel %f\n", m_fVelocityRightWheel);
		//}
	}
	void PID_OrientationControl(CTrajectoryGoal* GoalPose)
	{
		if (!m_bOrientationControl)
			return;
		//double e_theta = GoalPose->m_GoalPose.m_dOrientation - m_CurrentPose.m_dOrientation;
		double e_theta = CGeometryobj.DetermineOrientationNeeded(m_CurrentPose.m_Point,
			GoalPose->m_GoalPose.m_Point, m_CurrentPose.m_dOrientation);
		printf("e_theta Orientation %f\n", e_theta);
		double e_theta_dot  = -((m_fVelocityRightWheel - m_fVelocityLeftWheel) / IMRWIDTH);
		printf("e_theta_dot %f\n", e_theta_dot);

		double kP = 1.6;
		double kD = 0.2;
		double PI_value = kP * e_theta + kD * e_theta_dot;
		m_fAngularVelocity = PI_value;
		if (fabs(e_theta) < 0.0523599)
		{
			m_bPositionControl = true;
			m_bOrientationControl = false;
			printf("Orientation Goal Reached\n");
			//_getch();
			m_fLinearVelocity = 21.6;
			m_fAngularVelocity = 0.0;
		}
		printf("m_fLinearVelocity %f\n", m_fLinearVelocity);
		printf("m_fAngularVelocity %f\n", m_fAngularVelocity);
		m_fVelocityLeftWheel = - m_fAngularVelocity;
		m_fVelocityRightWheel = m_fAngularVelocity;
	}
};


class CFollowTrajectory
{
public:
	double m_fTime;
	double m_fTotSimulationTime;
public:
	CFollowTrajectory()
	{
		m_fTime = 0.0;
		m_fTotSimulationTime = 400.0;
	}
public:
	void Initialize()
	{
		CIMR* CIMRObject = new CIMR();
		CTrajectoryGoal* CTrajectoryGoalObject = new CTrajectoryGoal(true);
		printf("Pose (%f,%f,%f)\n", CIMRObject->m_CurrentPose.m_Point.x, CIMRObject->m_CurrentPose.m_Point.y, CIMRObject->m_CurrentPose.m_dOrientation);
		printf("Desired (%f,%f,%f)\n", CTrajectoryGoalObject->m_GoalPose.m_Point.x, CTrajectoryGoalObject->m_GoalPose.m_Point.y, CTrajectoryGoalObject->m_GoalPose.m_dOrientation);
		double e_theta = CIMRObject->CGeometryobj.DetermineOrientationNeeded(CIMRObject->m_CurrentPose.m_Point,
			CTrajectoryGoalObject->m_GoalPose.m_Point, CIMRObject->m_CurrentPose.m_dOrientation); 
		printf("e_theta %f\n", e_theta);		
		if (fabs(e_theta) > 1e-4)
		{
			CIMRObject->m_bPositionControl = false;
			CIMRObject->m_bOrientationControl = true;
			CIMRObject->Control_GetToGoalPosition(CTrajectoryGoalObject, true);
			printf("Orientation Control Needed \n");
		}
		else
		{
			CIMRObject->m_bPositionControl = true;
			CIMRObject->m_bOrientationControl = false;
			printf("Position Control Needed \n");
		}
		//_getch();
		/*PIDController* PID_linearSpeed = new PIDController();
		PIDController* PID_AngularSpeed = new PIDController();

		// PID_linearSpeed
		PID_linearSpeed->m_dKP = 1.5; // 0.6
		PID_linearSpeed->m_dKD = 0.01; // 0.08
		PID_linearSpeed->m_dKI = 0.03; // 0.03
		PID_linearSpeed->m_nHzPID = 250;
		InitPIDController(PID_linearSpeed);

		// PID_AngularSpeed
		PID_AngularSpeed->m_dKP = 1.5; //0.7 2.0
		PID_AngularSpeed->m_dKD = 0.01;
		PID_AngularSpeed->m_dKI = 0.01;
		PID_AngularSpeed->m_nHzPID = 250;
		InitPIDController(PID_AngularSpeed);*/

		FILE* auf = fopen("D:/PIDFollowTrajectory_Output.csv", "w");
		fprintf(auf, "Time,Distance,Theta,LW_Vel,RW_Vel,Pose_x,Pose_y,Pose_Ori\n");
		CIMRObject->CalculateError(CTrajectoryGoalObject);
		fprintf(auf, "%f,%f,%f,%f,%f,%f,%f,%f\n", m_fTime, CIMRObject->m_fDistanceIMRToGoal, CIMRObject->m_fDeltaTheta,
			CIMRObject->m_fVelocityLeftWheel, CIMRObject->m_fVelocityRightWheel, CIMRObject->m_CurrentPose.m_Point.x,
			CIMRObject->m_CurrentPose.m_Point.y, CIMRObject->m_CurrentPose.m_dOrientation);
		while (m_fTime < m_fTotSimulationTime)
		{
			CIMRObject->CalculateError(CTrajectoryGoalObject);
			CIMRObject->Control_GetToGoalPosition(CTrajectoryGoalObject);
			CIMRObject->UpdatePose();
			
			//printf("distance error %f\n", CIMRObject->m_fDistanceIMRToGoal);
			//printf("theta error %f\n", CIMRObject->m_fDeltaTheta);
			fprintf(auf, "%f,%f,%f,%f,%f,%f,%f,%f\n", m_fTime, CIMRObject->m_fDistanceIMRToGoal, CIMRObject->m_fDeltaTheta,
				CIMRObject->m_fVelocityLeftWheel, CIMRObject->m_fVelocityRightWheel, CIMRObject->m_CurrentPose.m_Point.x,
				CIMRObject->m_CurrentPose.m_Point.y, CIMRObject->m_CurrentPose.m_dOrientation);
			//_getch(); 
			if (CIMRObject->m_bGoalReached)
				break;
			
			
			m_fTime += SamplingTime * 0.001;
			printf("m_fTime %f\n", m_fTime);
			printf("\n");
			//_getch();
		}
		//fclose(auf);
	}
};