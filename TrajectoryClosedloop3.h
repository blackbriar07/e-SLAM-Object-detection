#pragma once
#include <math.h>
#include <vector>
#include <conio.h>
#include "Geometry.h"
#include "PIDControl.h"


PIDController* PID_linearSpeed_v = new PIDController();
PIDController* PID_AngularSpeed_v = new PIDController();

FILE* auf_3 = fopen("D:/PIDFollowTrajectory_Output.csv", "w");

class CTrajectory
{
public:
	CTrajectory(bool pReverse)
	{
		m_bReverse = pReverse;
		FILE* file = fopen("PathData/LShaped_raw_20240917.txt", "r");
		ReadRawTestPathData(file);
		WriteInITRIAPPFormat();
		WriteDataIntoArray2();
		if (!m_bReverse)
			ChangeGoalTextDataIndex(1);
		else
			ChangeGoalTextDataIndex(TotalNumofGoals-2);
	}
public:
	Pose m_GoalPose;
	int CurrentGoal;
	int TotalNumofGoals;
	bool m_bReverse;
	Pose m_Trajectory[1000];
	CGeometry Goal_CGeometryobj;
public:
	void PlanVelocityOrientation()
	{
		double PreviousOrientation = 0;
		double NewOrientation = 0;
		m_Trajectory[0].m_dOrientation = 0.0;
		m_Trajectory[0].m_dVelocity = 0.0;
		for (int i = 1; i < TotalNumofGoals; i++)
		{
			if (i == TotalNumofGoals - 1)
			{
				m_Trajectory[i].m_dOrientation = 0.0;
				m_Trajectory[i].m_dVelocity = 0.0;
			}
			else
			{
				NewOrientation = atan2((m_Trajectory[i+1].m_Point.y - m_Trajectory[i].m_Point.y), 
					(m_Trajectory[i + 1].m_Point.x - m_Trajectory[i].m_Point.x));
				while (NewOrientation - PreviousOrientation > PI)
					NewOrientation -= PI2;
				while (NewOrientation - PreviousOrientation < -PI)
					NewOrientation += PI2;
				PreviousOrientation = NewOrientation;
				m_Trajectory[i].m_dOrientation = NewOrientation;
				m_Trajectory[i].m_dVelocity = Goal_CGeometryobj.CalculateDistanceBetweenPoints(m_Trajectory[i + 1].m_Point,
					m_Trajectory[i].m_Point) / (SamplingTime / 1000);
			}
		}
	}
	void ChangeGoalTextDataIndex(int Index)
	{
		//printf("Goal Index %d\n", Index);
		CurrentGoal = Index;
		m_GoalPose = m_Trajectory[Index];
	}
	void ReadRawTestPathData(FILE* file1)
	{
		if (file1 == nullptr) {
			perror("Error opening file");
			return;
		}
		// Read and print each line of the file
		char buffer[256];
		float t, x, y;   // Variables to store extracted values
		int Count = 0;
		while (fgets(buffer, sizeof(buffer), file1) != nullptr)
		{
			//printf("%s", buffer);
			if (sscanf(buffer, "T%f X%f Y%f", &t, &x, &y) == 3) {
				// Print the extracted values
				m_Trajectory[Count].m_Point.x = (double)x;
				m_Trajectory[Count].m_Point.y = (double)y;
				//printf("T: %.6f, X: %.6f, Y: %.6f\n", t, x, y);
			}
			else {
				// Handle lines that do not match the expected format
				fprintf(stderr, "Line format incorrect: %s", buffer);
			}
			Count++;
		}
		TotalNumofGoals = Count;
		printf("TotalNumofGoals %d\n\n", TotalNumofGoals);
		fclose(file1);
		PlanVelocityOrientation();
		return;
	}
	void WriteInITRIAPPFormat()
	{
		FILE* fileOut = fopen("PathData/LShaped_20240917.txt", "w");
		fprintf(fileOut, "%d,\n", TotalNumofGoals);
		for (int i = 0; i < TotalNumofGoals; i++)
		{
			//fprintf(fileOut, "{%.2f,%.2f,%.2f,%.2f},\n", m_Trajectory[i].m_Point.x, m_Trajectory[i].m_Point.y, 
			//	m_Trajectory[i].m_dOrientation, m_Trajectory[i].m_dVelocity);
			fprintf(fileOut, "{%.2f,%.2f,%.2f},\n", m_Trajectory[i].m_Point.x, m_Trajectory[i].m_Point.y,
				m_Trajectory[i].m_dOrientation);
		}
		fclose(fileOut);
		printf("printing done\n");
	}
	void WriteDataIntoArray2()
	{
		FILE* fileOut = fopen("PathData/LShaped_20240917.csv", "w");
		fprintf(fileOut, "X,Y,Ori,Velo,\n");
		for (int i = 0; i < TotalNumofGoals; i++)
		{
			fprintf(fileOut, "%.2f,%.2f,%.2f,%.2f,\n", m_Trajectory[i].m_Point.x, m_Trajectory[i].m_Point.y,
				m_Trajectory[i].m_dOrientation, m_Trajectory[i].m_dVelocity);
		}
		fclose(fileOut);
		printf("printing done\n");
	}
};


class IMRDDRobot
{
public:
	IMRDDRobot(Pose StartingPose, int pTotNumberGoals, bool pReverse)
	{
		m_bReverse = pReverse;
		TotalNumberofGoals = pTotNumberGoals;
		m_CurrentPose = StartingPose;
		if (!m_bReverse)
			CurrentGoalIndex = 0;
		else
			CurrentGoalIndex = TotalNumberofGoals -1;
		m_fVelocityLeftWheel = 0;
		m_fVelocityRightWheel = 0;
		m_fLinearVelocity = 0;
		m_fAngularVelocity = 0;
		m_fDistanceIMRToGoal = 0;
		m_fDeltaTheta = 0;
		m_fTotalTime = 0;
		m_dWheelRadius = 0;
	}
public:
	Pose m_CurrentPose;
	Pose m_PreviousPose;
	int CurrentGoalIndex;
	int TotalNumberofGoals;
	double m_Deltab[3] = { 0 };
	double m_DeltaA[3][2] = { 0 };
	double m_fVelocityLeftWheel;
	double m_fVelocityRightWheel;
	double m_fLinearVelocity;
	double m_fAngularVelocity;
	double m_fDistanceIMRToGoal;
	double m_fDeltaTheta;
	double m_fTotalTime;
	double m_dWheelRadius;
	bool m_bReverse;
	CGeometry CGeometryobj;
public:
	void LimitVelocities()
	{
		double CalculatedLeftVelocity = m_fVelocityLeftWheel;
		double CalculatedRightVelocity = m_fVelocityRightWheel;
		double Ratio = CalculatedRightVelocity / CalculatedLeftVelocity;
		if (CalculatedLeftVelocity != 0)
			m_fVelocityLeftWheel = fmax(fmin(fabs(CalculatedLeftVelocity), MAX_VELOCITY), MIN_VELOCITY) * (CalculatedLeftVelocity < 0 ? -1 : 1);
		if (CalculatedRightVelocity != 0)
			m_fVelocityRightWheel = fmax(fmin(fabs(CalculatedRightVelocity), MAX_VELOCITY), MIN_VELOCITY) * (CalculatedRightVelocity < 0 ? -1 : 1);
		
		if (fmax(fabs(CalculatedRightVelocity), fabs(CalculatedLeftVelocity)) > MAX_VELOCITY)
		{
			if (fabs(CalculatedRightVelocity) > fabs(CalculatedLeftVelocity))
			{
				m_fVelocityRightWheel = CalculatedRightVelocity < 0 ? -MAX_VELOCITY : MAX_VELOCITY;
				m_fVelocityLeftWheel = m_fVelocityRightWheel / Ratio;
			}
			if (fabs(CalculatedRightVelocity) < fabs(CalculatedLeftVelocity))
			{
				m_fVelocityLeftWheel = CalculatedLeftVelocity < 0 ? -MAX_VELOCITY : MAX_VELOCITY;  
				m_fVelocityRightWheel = m_fVelocityLeftWheel * Ratio;
			}
		}
		printf("Limiting Velocities\n");
		printf("%f %f\n", m_fVelocityLeftWheel, m_fVelocityRightWheel);
	}
	int GetCurrentGoalPoseIndex(CTrajectory* GoalPose)
	{
		int PositionIndex = 0;
		int CurrentGoalPoseNum = GoalPose->CurrentGoal;
		double MinDistance = 100000000;
		Pose NextGoalPose;
		if (!m_bReverse)
		{
			for (int i = max(0, CurrentGoalIndex - 1); i < min(GoalPose->TotalNumofGoals, CurrentGoalIndex + 5); i++)
			{
				NextGoalPose = GoalPose->m_Trajectory[i];
				double fDistance = CGeometryobj.CalculateDistanceBetweenPoints(m_CurrentPose.m_Point, NextGoalPose.m_Point);
				if (MinDistance == 100000000 || fDistance <= MinDistance)
				{
					MinDistance = fDistance;
					PositionIndex = i;
				}
			}
		}
		else
		{
			for (int i = min(GoalPose->TotalNumofGoals - 1, CurrentGoalIndex + 1); i > max(-1, CurrentGoalIndex - 5); i--)
			{
				NextGoalPose = GoalPose->m_Trajectory[i];
				double fDistance = CGeometryobj.CalculateDistanceBetweenPoints(m_CurrentPose.m_Point, NextGoalPose.m_Point);
				if (MinDistance == 100000000 || fDistance <= MinDistance)
				{
					MinDistance = fDistance;
					PositionIndex = i;
				}
			}
		}
		return PositionIndex;
	}
	void GetGoalPoseIndex(CTrajectory* GoalPose)
	{
		CurrentGoalIndex = GetCurrentGoalPoseIndex(GoalPose);
		//printf("CurrentGoal Index for IMR %d\n", CurrentGoalIndex);
	}
	void CheckPoseAndSetGoal(CTrajectory* GoalPose)
	{
		GetGoalPoseIndex(GoalPose);
		if (!m_bReverse)
			GoalPose->ChangeGoalTextDataIndex(min(CurrentGoalIndex + 3, GoalPose->TotalNumofGoals - 1));
		else
			GoalPose->ChangeGoalTextDataIndex(max(CurrentGoalIndex - 3, 0));
	}
	void ImposeDisturbance(bool StartInitiate)
	{
		double lb = 0;
		double ub = 0;
		if (!StartInitiate)
		{
			lb = -3;
			ub = 3;
		}
		else
		{
			lb = -6;
			ub = 6;
		}
		if (fabs(m_fVelocityLeftWheel) > 0.5)
			m_fVelocityLeftWheel = Disturbance(m_fVelocityLeftWheel, lb, ub);
		if (fabs(m_fVelocityRightWheel) > 0.5)
			m_fVelocityRightWheel = Disturbance(m_fVelocityRightWheel, lb, ub);
	}
public:
	void UpdatePose()
	{
		m_PreviousPose = m_CurrentPose;
		m_fLinearVelocity = (m_fVelocityLeftWheel + m_fVelocityRightWheel) / 2;
		m_fAngularVelocity = (m_fVelocityRightWheel - m_fVelocityLeftWheel) / IMRWIDTH;
		m_CurrentPose.m_Point.x += m_fLinearVelocity * cos(m_PreviousPose.m_dOrientation) * (SamplingTime * 0.001);
		m_CurrentPose.m_Point.y += m_fLinearVelocity * sin(m_PreviousPose.m_dOrientation) * (SamplingTime * 0.001);
		m_CurrentPose.m_dOrientation += m_fAngularVelocity * (SamplingTime * 0.001);
		m_CurrentPose.m_dVelocity = m_fLinearVelocity;
	}
	void LeastSquares(Pose GoalPose)
	{
		
		double a = cos(m_CurrentPose.m_dOrientation) * (SamplingTime * 0.001) / 2;
		double b = sin(m_CurrentPose.m_dOrientation) * (SamplingTime * 0.001) / 2;
		double c = (SamplingTime * 0.001) / IMRWIDTH;
		double d = (m_fVelocityLeftWheel + m_fVelocityRightWheel) / 2;
		m_DeltaA[0][0] = a;
		m_DeltaA[0][1] = a;
		m_DeltaA[1][0] = b;
		m_DeltaA[1][1] = b;
		m_DeltaA[2][0] = -c;
		m_DeltaA[2][1] = c;

		double KP_Position = 0.5;
		double KP_Angle = 0.5 ;
		m_Deltab[0] = PIDFeedback(PID_linearSpeed_v, (GoalPose.m_Point.x - m_CurrentPose.m_Point.x)); //GoalPose.m_Point.x - m_CurrentPose.m_Point.x)* KP_Position;
		m_Deltab[1] = PIDFeedback(PID_linearSpeed_v, (GoalPose.m_Point.y - m_CurrentPose.m_Point.y));//(GoalPose.m_Point.y - m_CurrentPose.m_Point.y)* KP_Position;
		m_Deltab[2] = PIDFeedback(PID_AngularSpeed_v, (GoalPose.m_dOrientation - m_CurrentPose.m_dOrientation)); // (GoalPose.m_dOrientation - m_CurrentPose.m_dOrientation)* KP_Angle;
		// Perform Least squares
		double At[2][3];
		CGeometryobj.transposeMatrix3(m_DeltaA, At, 3, 2);		
		double AtA[2][2];
		CGeometryobj.multiplyMatrices32(At, m_DeltaA, AtA, 2, 2, 3);		
		double Atb[2];
		CGeometryobj.multiplyMatrixVector31(At, m_Deltab, Atb, 2, 3);
		double invAtA[2][2];
		if (CGeometryobj.inverse2x2(AtA, invAtA))
		{
			double x[2];
			CGeometryobj.multiplyMatrixVector21(invAtA, Atb, x, 2, 2);
			printf("Least Square Solution:\n");
			CGeometryobj.printVector(x, 2);
			m_fVelocityLeftWheel = x[0];
			m_fVelocityRightWheel = x[1];
			return;
		}
		m_fVelocityLeftWheel = 0;
		m_fVelocityRightWheel = 0;		
	}
	void CalculatePositionandAngleError(Pose GoalPose, double& DistanceError, double& AngleError)
	{
		// Calculate position error
		double error_x = GoalPose.m_Point.x - m_CurrentPose.m_Point.x;
		double error_y = GoalPose.m_Point.y - m_CurrentPose.m_Point.y;
		double error_theta = GoalPose.m_dOrientation - m_CurrentPose.m_dOrientation;

		// Convert position error to polar coordinates
		DistanceError = sqrt(error_x * error_x + error_y * error_y);
		AngleError = CGeometryobj.DetermineOrientationNeeded(m_CurrentPose.m_Point, GoalPose.m_Point,
			m_CurrentPose.m_dOrientation);
	}
	void RefineWithPIDController(Pose GoalPose)
	{
		double DistanceError = 0.0;
		double AngleError = 0.0;
		CalculatePositionandAngleError(GoalPose, DistanceError, AngleError);
		// PID control for linearand angular velocities
		double v_correction = PIDFeedback(PID_linearSpeed_v, DistanceError);
		double omega_correction = PIDFeedback(PID_AngularSpeed_v, AngleError);	
		m_fVelocityLeftWheel = m_fVelocityLeftWheel - v_correction - (IMRWIDTH / 2) * omega_correction;
		m_fVelocityRightWheel = m_fVelocityRightWheel - v_correction + (IMRWIDTH / 2) * omega_correction;
		printf("Refined PID Solution:\n");
		printf("%f %f\n", m_fVelocityLeftWheel, m_fVelocityRightWheel);
	}
};


class CIMRFollowTrajectory
{
public:
	CIMRFollowTrajectory()
	{
		m_fTime = 0.0;
		bStartInitiate = false;
		m_bReverse = false;
	}
public:
	CGeometry CGeometryobj;
	double m_fTime;
	bool m_bReverse;
	bool bStartInitiate;
public:
	void PrintData(CTrajectory* CTrajectory_obj, IMRDDRobot* IMRDDRobot_obj)
	{
		fprintf(auf_3, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", m_fTime, IMRDDRobot_obj->m_fDistanceIMRToGoal, IMRDDRobot_obj->m_fDeltaTheta,
			IMRDDRobot_obj->m_fVelocityLeftWheel, IMRDDRobot_obj->m_fVelocityRightWheel, IMRDDRobot_obj->m_CurrentPose.m_Point.x,
			IMRDDRobot_obj->m_CurrentPose.m_Point.y, IMRDDRobot_obj->m_CurrentPose.m_dOrientation, IMRDDRobot_obj->m_CurrentPose.m_dVelocity,
			CTrajectory_obj->m_GoalPose.m_Point.x,
			CTrajectory_obj->m_GoalPose.m_Point.y, CTrajectory_obj->m_GoalPose.m_dOrientation, CTrajectory_obj->m_GoalPose.m_dVelocity);
	}
	void Initialize()
	{
		// PID_linearSpeed
		PID_linearSpeed_v->m_dKP = 0.5;//1.5; // 0.6
		PID_linearSpeed_v->m_dKD = 0.0; // 0.08
		PID_linearSpeed_v->m_dKI = 0.000; // 0.03
		PID_linearSpeed_v->m_nHzPID = 250;
		InitPIDController(PID_linearSpeed);

		// PID_AngularSpeed
		PID_AngularSpeed_v->m_dKP = 0.5; //0.7 2.0
		PID_AngularSpeed_v->m_dKD = 0.1;
		PID_AngularSpeed_v->m_dKI = 0.001;
		PID_AngularSpeed_v->m_nHzPID = 250;
		InitPIDController(PID_AngularSpeed);

		fprintf(auf_3, "Time,Distance,Theta,LW_Vel,RW_Vel,Pose_x,Pose_y,Pose_Ori,Pose_Vel,Goal_x,Goal_y,Goal_Orientation,Goal_vel\n");

		CTrajectory* CTrajectory_obj = new CTrajectory(m_bReverse);
		Pose CurrentPose;
		if (!m_bReverse)
			CurrentPose = CTrajectory_obj->m_Trajectory[0];
		else
			CurrentPose = CTrajectory_obj->m_Trajectory[CTrajectory_obj->TotalNumofGoals-1];
		IMRDDRobot* IMRDDRobot_obj = new IMRDDRobot(CurrentPose, CTrajectory_obj->TotalNumofGoals, m_bReverse);
		
		if (!m_bReverse)
			MotionTestForward(CTrajectory_obj, IMRDDRobot_obj);
		else
			MotionTestBackward(CTrajectory_obj, IMRDDRobot_obj);		
		fclose(auf_3);
	}
	void MotionTestForward(CTrajectory* CTrajectory_obj, IMRDDRobot* IMRDDRobot_obj)
	{
		while (CTrajectory_obj->CurrentGoal < CTrajectory_obj->TotalNumofGoals - 1)
		{
			CGeometryobj.PrintPose(CTrajectory_obj->m_GoalPose, "CurrentGoal");
			CGeometryobj.PrintPose(IMRDDRobot_obj->m_CurrentPose, "CurrentPose");
			IMRDDRobot_obj->LeastSquares(CTrajectory_obj->m_GoalPose);
			//IMRDDRobot_obj->RefineWithPIDController(CTrajectory_obj->m_GoalPose);
			IMRDDRobot_obj->LimitVelocities();
			IMRDDRobot_obj->ImposeDisturbance(bStartInitiate);
			IMRDDRobot_obj->UpdatePose();
			IMRDDRobot_obj->CheckPoseAndSetGoal(CTrajectory_obj);
			printf("CurrentPose Index: %d\n", IMRDDRobot_obj->CurrentGoalIndex);
			printf("CurrentGoal Index: %d\n", CTrajectory_obj->CurrentGoal);
			printf("\n");
			PrintData(CTrajectory_obj, IMRDDRobot_obj);
			m_fTime += SamplingTime * 0.001;
			bStartInitiate = true;
			//_getch();
		}		
	}
	void MotionTestBackward(CTrajectory* CTrajectory_obj, IMRDDRobot* IMRDDRobot_obj)
	{
		printf("CurrentPose Index: %d\n", IMRDDRobot_obj->CurrentGoalIndex);
		printf("CurrentGoal Index: %d\n", CTrajectory_obj->CurrentGoal);
		while (CTrajectory_obj->CurrentGoal > 0)
		{
			CGeometryobj.PrintPose(CTrajectory_obj->m_GoalPose, "CurrentGoal");
			CGeometryobj.PrintPose(IMRDDRobot_obj->m_CurrentPose, "CurrentPose");
			IMRDDRobot_obj->LeastSquares(CTrajectory_obj->m_GoalPose);
			//IMRDDRobot_obj->RefineWithPIDController(CTrajectory_obj->m_GoalPose);
			IMRDDRobot_obj->LimitVelocities();
			IMRDDRobot_obj->ImposeDisturbance(bStartInitiate);
			IMRDDRobot_obj->UpdatePose();
			IMRDDRobot_obj->CheckPoseAndSetGoal(CTrajectory_obj);
			printf("CurrentPose Index: %d\n", IMRDDRobot_obj->CurrentGoalIndex);
			printf("CurrentGoal Index: %d\n", CTrajectory_obj->CurrentGoal);
			printf("\n");
			PrintData(CTrajectory_obj, IMRDDRobot_obj);
			m_fTime += SamplingTime * 0.001;
			bStartInitiate = true;
			//_getch();
		}
	}
};