#pragma once
#include <math.h>
#include <vector>
#include <conio.h>
#include "Geometry.h"
#include "PIDControl.h"

bool isBetween(double Point, double LB, double UB)
{
	return (Point >= fmin(LB, UB) && Point <= fmax(LB, UB));
}

class CTrajectoryGoal_ver1
{
public:
	Pose m_GoalPose;
	double m_NeighbourhoodRadius;
	double m_thresholdOrientation;
	int CurrentGoal;
	int TotalNumofGoals;
	int PathTrajectoryCase;
	double m_fXpathData[1000] = { 0 };
	double m_fYpathData[1000] = { 0 };
	bool PathtextData;
	CGeometry Goal_CGeometryobj;
public:
	CTrajectoryGoal_ver1(bool ExternalData = false)
	{
		m_NeighbourhoodRadius = 5.0; //cm
		m_thresholdOrientation = 2.0;
		PathTrajectoryCase = 2;
		TotalNumofGoals = PathTrajectoryCase;
		CurrentGoal = -1;
		PathtextData = ExternalData;
		{
			FILE* file = fopen("PathData/StraightLineData_raw.txt", "r");
			ReadRawTestPathData(file);
			//WriteDataIntoArray();
			//_getch();
			//ChangeGoalTextData();
			ChangeGoalTextDataIndex(2);
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
		printf("Goal (X, Y, theta): (%f,%f,%f)\n", m_GoalPose.m_Point.x, m_GoalPose.m_Point.y, m_GoalPose.m_dOrientation);
		double XError = pIMRPose.m_Point.x - m_GoalPose.m_Point.x;
		double YError = pIMRPose.m_Point.y - m_GoalPose.m_Point.y;
		DistanceError = sqrt(XError * XError + YError * YError);
		OrientationError = pIMRPose.m_dOrientation - m_GoalPose.m_dOrientation;
		if (DistanceError < m_NeighbourhoodRadius && OrientationError < m_thresholdOrientation)
			return true;
		else
			return false;

	}
	void GetPerpendicularLineFromGoalPose(Point2D& ResultPerpendicularLine)
	{
		Point2D NextGoalPose;
		NextGoalPose.x = m_fXpathData[CurrentGoal + 1];
		NextGoalPose.y = m_fYpathData[CurrentGoal + 1];
		NextGoalPose.z = 0;
 		Goal_CGeometryobj.GetPerpendicularLineEquationGivenTwoPoints(m_GoalPose.m_Point, NextGoalPose, ResultPerpendicularLine);
	}
	bool CheckIfIMRCrossedCurrentGoal(Pose CurrentIMRPosition, Pose PreviousIMRPosition)
	{
		Point2D ResultLine;
		Goal_CGeometryobj.PrintPose(CurrentIMRPosition, "CurrentIMRPosition");
		Goal_CGeometryobj.PrintPose(PreviousIMRPosition, "PreviousIMRPosition");
		Goal_CGeometryobj.ReturnEquationofLine(CurrentIMRPosition.m_Point, PreviousIMRPosition.m_Point, ResultLine);
		Point2D ResultPerpendicularLine;
		GetPerpendicularLineFromGoalPose(ResultPerpendicularLine);
		Point2D ResultPoint;
		if (Goal_CGeometryobj.GetIntersectionPointBetweenLines(ResultPerpendicularLine, ResultLine, ResultPoint))
		{
			printf("ResultPoint (%f, %f)\n", ResultPoint.x, ResultPoint.y);
			printf("----------------------------\n");
			if (isBetween(ResultPoint.x, CurrentIMRPosition.m_Point.x, PreviousIMRPosition.m_Point.x) &&
				isBetween(ResultPoint.y, CurrentIMRPosition.m_Point.y, PreviousIMRPosition.m_Point.y))
			{
				printf("Intersction Occured ***********\n");
				return true;
			}
		}
		return false;
	}
	bool CheckIfIMRCrossedCurrentGoal2(Pose CurrentIMRPosition, Pose PreviousIMRPosition, int pCurrentGoalPoseIndex)
	{
		Point2D ResultLine;
		Goal_CGeometryobj.PrintPose(CurrentIMRPosition, "CurrentIMRPosition");
		Goal_CGeometryobj.PrintPose(PreviousIMRPosition, "PreviousIMRPosition");
		Goal_CGeometryobj.ReturnEquationofLine(CurrentIMRPosition.m_Point, PreviousIMRPosition.m_Point, ResultLine);
		Point2D ResultPerpendicularLine;
		/////      
		Point2D pCurrentGoalPose;
		pCurrentGoalPose.x = m_fXpathData[pCurrentGoalPoseIndex];
		pCurrentGoalPose.y = m_fYpathData[pCurrentGoalPoseIndex];
		pCurrentGoalPose.z = 0;
		Point2D NextGoalPose;
		NextGoalPose.x = m_fXpathData[pCurrentGoalPoseIndex + 1];
		NextGoalPose.y = m_fYpathData[pCurrentGoalPoseIndex + 1];
		NextGoalPose.z = 0;
		Goal_CGeometryobj.GetPerpendicularLineEquationGivenTwoPoints(pCurrentGoalPose, NextGoalPose, ResultPerpendicularLine);
		GetPerpendicularLineFromGoalPose(ResultPerpendicularLine);
		/// 
		Point2D ResultPoint;
		if (Goal_CGeometryobj.GetIntersectionPointBetweenLines(ResultPerpendicularLine, ResultLine, ResultPoint))
		{
			printf("ResultPoint (%f, %f)\n", ResultPoint.x, ResultPoint.y);
			printf("----------------------------\n");
			if (isBetween(ResultPoint.x, CurrentIMRPosition.m_Point.x, PreviousIMRPosition.m_Point.x) &&
				isBetween(ResultPoint.y, CurrentIMRPosition.m_Point.y, PreviousIMRPosition.m_Point.y))
			{
				printf("Intersction Occured ***********\n");
				return true;
			}
		}
		return false;
	}
	void ChangeGoalTextData()
	{
		printf("I am here\n");
		_getch();
		CurrentGoal++;
		m_GoalPose.m_Point.x = m_fXpathData[CurrentGoal]; //400.0;
		m_GoalPose.m_Point.y = m_fYpathData[CurrentGoal]; //25
		m_GoalPose.m_dOrientation = 0.000;//PI;
	}
	void ChangeGoalTextDataIndex(int Index)
	{
		printf("Goal Index %d\n", Index);
		CurrentGoal = Index;
		m_GoalPose.m_Point.x = m_fXpathData[Index]; //400.0;
		m_GoalPose.m_Point.y = m_fYpathData[Index]; //25
		m_GoalPose.m_dOrientation = 0.000;//PI;
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
		fclose(file1);
		return;
	}
};

class CIMR_ver1
{
public:
	Pose m_CurrentPose;
	Pose m_PreviousPose;
	int CurrentGoalIndex;
	double m_fVelocityLeftWheel;
	double m_fVelocityRightWheel;
	double m_fHallSensorVelocityLeftWheel;
	double m_fHallSensorVelocityRightWheel;
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
	bool m_bDistanceWithinLimits;
	bool m_bAngleWithinLimits;
	CGeometry CGeometryobj;

public:
	CIMR_ver1()
	{
		m_CurrentPose.m_Point.x = 10.000000;
		m_CurrentPose.m_Point.y = 160.000000;
		m_CurrentPose.m_dOrientation = 0.0;
		CurrentGoalIndex = 0;
		m_PreviousPose.m_Point.x = 0.0;
		m_PreviousPose.m_Point.y = 0.0;
		m_PreviousPose.m_dOrientation = 0.0;
		m_fVelocityLeftWheel = 21.6;
		m_fVelocityRightWheel = 21.6;
		m_fHallSensorVelocityLeftWheel = 0.0;
		m_fHallSensorVelocityRightWheel = 0.0;
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
		m_bDistanceWithinLimits = false;
		m_bAngleWithinLimits = false;
	}
	void UpdatePose()
	{
		m_PreviousPose = m_CurrentPose;
		m_fHallSensorVelocityLeftWheel = m_fVelocityLeftWheel; // Disturbance(m_fVelocityLeftWheel, -2, 2);
		m_fHallSensorVelocityRightWheel = m_fVelocityRightWheel; // Disturbance(m_fVelocityRightWheel, -2, 2);
		m_CurrentPose.m_Point.x += ((m_fHallSensorVelocityLeftWheel + m_fHallSensorVelocityRightWheel) / 2) * cos(m_PreviousPose.m_dOrientation) * (SamplingTime * 0.001);
		m_CurrentPose.m_Point.y += ((m_fHallSensorVelocityLeftWheel + m_fHallSensorVelocityRightWheel) / 2) * sin(m_PreviousPose.m_dOrientation) * (SamplingTime * 0.001);
		m_CurrentPose.m_dOrientation += ((m_fHallSensorVelocityRightWheel - m_fHallSensorVelocityLeftWheel) / IMRWIDTH) * (SamplingTime * 0.001);
		printf("(X, Y, theta): (%f,%f,%f)\n", m_CurrentPose.m_Point.x, m_CurrentPose.m_Point.y, m_CurrentPose.m_dOrientation);
	}
	void CalculateError(CTrajectoryGoal_ver1* TrajectoryGoalObject)
	{
		printf("Current (X, Y, theta): (%f,%f,%f)\n", m_CurrentPose.m_Point.x, m_CurrentPose.m_Point.y, m_CurrentPose.m_dOrientation);
		TrajectoryGoalObject->GoalReached(m_CurrentPose, m_fDistanceIMRToGoal, m_fDeltaTheta);
	}
	int GetGoalPoseIndex(CTrajectoryGoal_ver1* GoalPose)
	{
		if (!GoalPose->CheckIfIMRCrossedCurrentGoal(m_CurrentPose, m_PreviousPose))
			return GoalPose->CurrentGoal;
		int PositionIndex = 0;
		int CurrentGoalPoseNum = GoalPose->CurrentGoal;
		double MinDistance = 100000000;
		Point2D NextGoalPose;
		printf("CurrentGoalPoseNum %d\n", CurrentGoalPoseNum);
		printf("max(0, CurrentGoalPoseNum + 1) %d\n", max(0, CurrentGoalPoseNum + 1));
		printf("min(GoalPose->TotalNumofGoals, CurrentGoalPoseNum + 5) %d\n", min(GoalPose->TotalNumofGoals, CurrentGoalPoseNum + 5));
		for (int i = max(0, CurrentGoalPoseNum - 1); i < min(GoalPose->TotalNumofGoals, CurrentGoalPoseNum + 5); i++)
		{
			NextGoalPose.x = GoalPose->m_fXpathData[i];
			NextGoalPose.y = GoalPose->m_fYpathData[i];
			double fDistance = CGeometryobj.CalculateDistanceBetweenPoints(m_CurrentPose.m_Point, NextGoalPose);
			if (MinDistance == 100000000 || fDistance <= MinDistance)
			{
				MinDistance = fDistance;
				PositionIndex = i;
			}
		}
		printf("PositionIndex %d\n", PositionIndex);
		return PositionIndex;
	}
	int GetGoalPoseIndex2(CTrajectoryGoal_ver1* GoalPose)
	{
		if (!GoalPose->CheckIfIMRCrossedCurrentGoal(m_CurrentPose, m_PreviousPose))
			return GoalPose->CurrentGoal;
		int PositionIndex = 0;
		int CurrentGoalPoseNum = GoalPose->CurrentGoal;
		GoalPose->ChangeGoalTextData();
		while (GoalPose->CheckIfIMRCrossedCurrentGoal(m_CurrentPose, m_PreviousPose))
		{
			GoalPose->ChangeGoalTextData();
		}
		return 0;
	}
	int GetCurrentGoalPoseIndex(CTrajectoryGoal_ver1* GoalPose)
	{
		int PositionIndex = 0;
		int CurrentGoalPoseNum = GoalPose->CurrentGoal;
		double MinDistance = 100000000;
		Point2D NextGoalPose;
		printf("CurrentGoalPoseNum %d\n", CurrentGoalPoseNum);
		printf("max(0, CurrentGoalPoseNum + 1) %d\n", max(0, CurrentGoalPoseNum + 1));
		printf("min(GoalPose->TotalNumofGoals, CurrentGoalPoseNum + 5) %d\n", min(GoalPose->TotalNumofGoals, CurrentGoalPoseNum + 5));
		for (int i = max(0, CurrentGoalIndex - 1); i < min(GoalPose->TotalNumofGoals, CurrentGoalIndex + 5); i++)
		{
			//printf("i %d\n", i);
			NextGoalPose.x = GoalPose->m_fXpathData[i];
			NextGoalPose.y = GoalPose->m_fYpathData[i];
			double fDistance = CGeometryobj.CalculateDistanceBetweenPoints(m_CurrentPose.m_Point, NextGoalPose);
			//printf("fDistance %f\n", fDistance);
			if (MinDistance == 100000000 || fDistance <= MinDistance)
			{
				MinDistance = fDistance;
				PositionIndex = i;
			}
		}
		printf("PositionIndex %d\n", PositionIndex);
		return PositionIndex;
	}
	int GetGoalPoseIndex3(CTrajectoryGoal_ver1* GoalPose)
	{
		CurrentGoalIndex = GetCurrentGoalPoseIndex(GoalPose);
		GoalPose->ChangeGoalTextDataIndex(min(CurrentGoalIndex + 4, GoalPose->TotalNumofGoals - 1));
		return 0;
	}
	bool PopPose(double LeftWheelVelocity, double RightWheelVelocity, CTrajectoryGoal_ver1* GoalPose, bool p_bLastGoal=false)
	{
		printf("----------------------------\n");
		Point2D PreviousGoalPose = GoalPose->m_GoalPose.m_Point;
		Point2D NextGoalPose; 
		if (!p_bLastGoal)
		{
			NextGoalPose.x = GoalPose->m_fXpathData[GoalPose->CurrentGoal + 1];
			NextGoalPose.y = GoalPose->m_fYpathData[GoalPose->CurrentGoal + 1];
		}
		else
		{
			NextGoalPose.x = GoalPose->m_fXpathData[GoalPose->CurrentGoal - 1];
			NextGoalPose.y = GoalPose->m_fYpathData[GoalPose->CurrentGoal - 1];
		}
		Point2D ResultPerpendicularLine;
		printf("PreviousGoalPose (%f, %f)\n", PreviousGoalPose.x, PreviousGoalPose.y);
		printf("NextGoalPose (%f, %f)\n", NextGoalPose.x, NextGoalPose.y);
		if (CGeometryobj.GetPerpendicularLineEquationGivenTwoPoints(PreviousGoalPose, NextGoalPose, ResultPerpendicularLine))
		{
			printf("ResultPerpendicularLine (A%f, B%f, B%f)\n", ResultPerpendicularLine.x, ResultPerpendicularLine.y,
				ResultPerpendicularLine.z);
			printf("IMR_CurrentPose (%f, %f)\n", m_CurrentPose.m_Point.x, m_CurrentPose.m_Point.y);
			printf("IMR_previousPose (%f, %f)\n", m_PreviousPose.m_Point.x, m_PreviousPose.m_Point.y);
			printf("----------------------------\n");
			Point2D ResultLine;
			CGeometryobj.ReturnEquationofLine(m_CurrentPose.m_Point, m_PreviousPose.m_Point, ResultLine);
			Point2D ResultPoint;
			printf("ResultLine (A%f, B%f, B%f)\n", ResultLine.x, ResultLine.y,
				ResultLine.z);
			if (CGeometryobj.GetIntersectionPointBetweenLines(ResultPerpendicularLine, ResultLine, ResultPoint))
			{
				printf("ResultPoint (%f, %f)\n", ResultPoint.x, ResultPoint.y);
				printf("----------------------------\n");
				if ((ResultPoint.x >= m_CurrentPose.m_Point.x && ResultPoint.x <= m_PreviousPose.m_Point.x) ||
					(ResultPoint.x >= m_PreviousPose.m_Point.x && ResultPoint.x <= m_CurrentPose.m_Point.x))
					return true;
				if ((ResultPoint.y >= m_CurrentPose.m_Point.y && ResultPoint.y <= m_PreviousPose.m_Point.y) ||
					(ResultPoint.y >= m_PreviousPose.m_Point.y && ResultPoint.y <= m_CurrentPose.m_Point.y))
					return true;
			}
			return false;
		}
		return false;
	}
	void ImposeDisturbance()
	{
		if (fabs(m_fVelocityLeftWheel) > 0.5)
			m_fVelocityLeftWheel = Disturbance(m_fVelocityLeftWheel, -3, 3);
		if (fabs(m_fVelocityRightWheel) > 0.5)
			m_fVelocityRightWheel = Disturbance(m_fVelocityRightWheel, -3, 3);
	}
	void LimitVelocities()
	{
		if (m_fVelocityLeftWheel != 0)
			m_fVelocityLeftWheel = fmax(fmin(fabs(m_fVelocityLeftWheel), MAX_VELOCITY), MIN_VELOCITY) * (m_fVelocityLeftWheel < 0 ? -1 : 1);
		if (m_fVelocityRightWheel != 0)
			m_fVelocityRightWheel = fmax(fmin(fabs(m_fVelocityRightWheel), MAX_VELOCITY), MIN_VELOCITY) * (m_fVelocityRightWheel < 0 ? -1 : 1);
	}
	void LimitVelocities2(double CalculatedLeftVelocity, double CalculatedRightVelocity)
	{
		double Ratio = CalculatedRightVelocity / CalculatedLeftVelocity;
		if (CalculatedLeftVelocity != 0)
			m_fVelocityLeftWheel = fmax(fmin(fabs(CalculatedLeftVelocity), MAX_VELOCITY), MIN_VELOCITY) * (CalculatedLeftVelocity < 0 ? -1 : 1);
		if (CalculatedRightVelocity != 0)
			m_fVelocityRightWheel = fmax(fmin(fabs(CalculatedRightVelocity), MAX_VELOCITY), MIN_VELOCITY) * (CalculatedRightVelocity < 0 ? -1 : 1);

		if (CalculatedRightVelocity > MAX_VELOCITY &&  fabs(CalculatedRightVelocity) > fabs(CalculatedLeftVelocity))
		{
			m_fVelocityRightWheel = MAX_VELOCITY;
			m_fVelocityLeftWheel = m_fVelocityRightWheel / Ratio;
		}
		if (CalculatedLeftVelocity > MAX_VELOCITY && fabs(CalculatedRightVelocity) < fabs(CalculatedLeftVelocity))
		{
			m_fVelocityLeftWheel = MAX_VELOCITY;
			m_fVelocityRightWheel = m_fVelocityLeftWheel * Ratio;
		}
	}
	void IgnoreDesiredPose(CTrajectoryGoal_ver1* GoalPose)
	{
		if (GoalPose->CurrentGoal < (GoalPose->TotalNumofGoals - 1))
		{
			if (PopPose(m_fVelocityLeftWheel, m_fVelocityRightWheel, GoalPose))
			{
				GoalPose->ChangeGoalTextData();
				//m_fVelocityLeftWheel = -m_fAngularVelocity;
				//m_fVelocityRightWheel = m_fAngularVelocity;
				LimitVelocities();
				ImposeDisturbance();
				printf("m_fVelocityLeftWheel %f\n", m_fVelocityLeftWheel);
				printf("m_fVelocityRightWheel %f\n", m_fVelocityRightWheel);
				printf("Change the desired pose *****************************************\n");
			}
			printf("Changed Goal (X, Y, theta): (%f,%f,%f)\n", GoalPose->m_GoalPose.m_Point.x, GoalPose->m_GoalPose.m_Point.y,
				GoalPose->m_GoalPose.m_dOrientation);
		}
		else if (GoalPose->CurrentGoal == (GoalPose->TotalNumofGoals - 1))
		{
			if (PopPose(m_fVelocityLeftWheel, m_fVelocityRightWheel, GoalPose, true))
			{
				m_bGoalReached = true;
				m_fVelocityLeftWheel = 0;
				m_fVelocityRightWheel = 0;
				printf("Change the desired pose *****************************************\n");
			}
			printf("Changed Goal (X, Y, theta): (%f,%f,%f)\n", GoalPose->m_GoalPose.m_Point.x, GoalPose->m_GoalPose.m_Point.y,
				GoalPose->m_GoalPose.m_dOrientation);
		}
	}
	void IgnoreDesiredPose2(CTrajectoryGoal_ver1* GoalPose)
	{
		//GetGoalPoseIndex2(GoalPose);
		GetGoalPoseIndex3(GoalPose);
		/*int GoalPositionIndex = GetGoalPoseIndex(GoalPose);
		printf("GoalPositionIndex %d\n", GoalPositionIndex);
		printf("GoalPose->CurrentGoal %d\n", GoalPose->CurrentGoal);
		if (GoalPositionIndex > GoalPose->CurrentGoal)
		{
			GoalPose->CurrentGoal = GoalPositionIndex - 1;
			GoalPose->ChangeGoalTextData();
			printf("GoalPose->CurrentGoal after changing %d\n", GoalPose->CurrentGoal);
		}*/
	}
	void SetVelocity(double PID_v, double PID_w)
	{
		printf("PID_v %f\n", PID_v);
		printf("PID_w %f\n", PID_w);
		m_fLinearVelocity = PID_v;
		m_fAngularVelocity = PID_w;
		double CalculatedLeftVelocity = m_fLinearVelocity - m_fAngularVelocity;
		double CalculatedRightVelocity = m_fLinearVelocity + m_fAngularVelocity;
		printf("CalculatedLeftVelocity %f\n", CalculatedLeftVelocity);
		printf("CalculatedRightVelocity %f\n", CalculatedRightVelocity);
		//LimitVelocities();
		LimitVelocities2(CalculatedLeftVelocity, CalculatedRightVelocity);
		printf("m_fVelocityLeftWheel after normalization %f\n", m_fVelocityLeftWheel);
		printf("m_fVelocityRightWheel after normalization %f\n", m_fVelocityRightWheel);
		ImposeDisturbance();
		printf("m_fVelocityLeftWheel after randomness %f\n", m_fVelocityLeftWheel);
		printf("m_fVelocityRightWheel after randomness %f\n", m_fVelocityRightWheel);
	}
	void Control_GetToGoalPosition(CTrajectoryGoal_ver1* GoalPose, PIDController* pLinearVelocityPID, PIDController* pAngularVelocityPID,
		bool ChangeOrientation = false)
	{
		double PID_v = 0.0;
		double PID_w = 0.0;
		if (GoalPose->CurrentGoal >= GoalPose->TotalNumofGoals - 1)
			m_bGoalReached = true;
		PID_linearVelocityControl(pLinearVelocityPID, GoalPose, 1, PID_v);
		if (m_bGoalReached)
			return;
		PID_AngularVelocityControl(pAngularVelocityPID, GoalPose, 0.0174533, PID_w);
		SetVelocity(PID_v, PID_w);		
		//IgnoreDesiredPose(GoalPose);
	}
	void PID_linearVelocityControl(PIDController* PID_linearSpeed, CTrajectoryGoal_ver1* GoalPose, double threshold, double& PID_v)
	{
		double d_error = m_fDistanceIMRToGoal;
		printf("m_fDistanceIMRToGoal %f\n", m_fDistanceIMRToGoal);
		if (d_error < threshold)
		{
			d_error = 0.0;
			m_bDistanceWithinLimits = true;
			printf("Goal within threshold\n");
			if (GoalPose->CurrentGoal == (GoalPose->TotalNumofGoals - 1))
			{
				m_bGoalReached = true;
				m_fLinearVelocity = 0.0;
				m_fAngularVelocity = 0.0;
				m_fVelocityLeftWheel = m_fLinearVelocity - m_fAngularVelocity;
				m_fVelocityRightWheel = m_fLinearVelocity + m_fAngularVelocity;
				return;
			}
			//GoalPose->ChangeGoalTextData();
		}
		else
			m_bDistanceWithinLimits = false;
		PID_v = PIDFeedback(PID_linearSpeed, d_error);
	}
	void PID_AngularVelocityControl(PIDController* PID_AngularSpeed, CTrajectoryGoal_ver1* GoalPose, double threshold, double& PID_w)
	{
		double e_theta = CGeometryobj.DetermineOrientationNeeded(m_CurrentPose.m_Point, GoalPose->m_GoalPose.m_Point,
			m_CurrentPose.m_dOrientation);
		printf("e_theta Orientation %f\n", e_theta);
		double d_error = e_theta;
		//if (fabs(e_theta) < threshold)
		//{
		//	d_error = 0.0;
		//	m_bAngleWithinLimits = true;
		//}
		//else
		//	m_bAngleWithinLimits = false;
		PID_w = PIDFeedback(PID_AngularSpeed, d_error);
	}	
	void PID_OrientationControl(CTrajectoryGoal_ver1* GoalPose)
	{
		if (!m_bOrientationControl)
			return;
		//double e_theta = GoalPose->m_GoalPose.m_dOrientation - m_CurrentPose.m_dOrientation;
		double e_theta = CGeometryobj.DetermineOrientationNeeded(m_CurrentPose.m_Point,
			GoalPose->m_GoalPose.m_Point, m_CurrentPose.m_dOrientation);
		printf("e_theta Orientation %f\n", e_theta);
		double e_theta_dot = -((m_fVelocityRightWheel - m_fVelocityLeftWheel) / IMRWIDTH);
		double kP = 1.6;
		double kD = 0.2;
		double PD_value = kP * e_theta + kD * e_theta_dot;
		m_fAngularVelocity = PD_value;
		m_fLinearVelocity = 0.0;
		if (fabs(e_theta) < 0.0523599)
		{
			m_bPositionControl = true;
			m_bOrientationControl = false;
			printf("Orientation Goal Reached\n");
			m_fLinearVelocity = 0.0;
			m_fAngularVelocity = 0.0;
		}
		printf("m_fLinearVelocity %f\n", m_fLinearVelocity);
		printf("m_fAngularVelocity %f\n", m_fAngularVelocity);
		m_fVelocityLeftWheel = -m_fAngularVelocity;
		m_fVelocityRightWheel = m_fAngularVelocity;
	}
};

PIDController* PID_linearSpeed = new PIDController();
PIDController* PID_AngularSpeed = new PIDController();


class CFollowTrajectory_ver1
{
public:
	double m_fTime;
	double m_fTotSimulationTime;
public:
	CFollowTrajectory_ver1()
	{
		m_fTime = 0.0;
		m_fTotSimulationTime = 400.0;
	}
public:
	void CorrectOrientation(CIMR_ver1* CIMRObject_, CTrajectoryGoal_ver1* CTrajectoryGoalObject_, FILE* mfile)
	{
		printf("Pose (%f,%f,%f)\n", CIMRObject_->m_CurrentPose.m_Point.x, CIMRObject_->m_CurrentPose.m_Point.y, CIMRObject_->m_CurrentPose.m_dOrientation);
		printf("Desired (%f,%f,%f)\n", CTrajectoryGoalObject_->m_GoalPose.m_Point.x, CTrajectoryGoalObject_->m_GoalPose.m_Point.y, CTrajectoryGoalObject_->m_GoalPose.m_dOrientation);
		double e_theta = CIMRObject_->CGeometryobj.DetermineOrientationNeeded(CIMRObject_->m_CurrentPose.m_Point,
			CTrajectoryGoalObject_->m_GoalPose.m_Point, CIMRObject_->m_CurrentPose.m_dOrientation);
		printf("e_theta %f\n", e_theta);
		if (fabs(e_theta) > 1e-4)
		{
			CIMRObject_->m_bPositionControl = false;
			CIMRObject_->m_bOrientationControl = true;
		}

		CIMRObject_->m_bAngleWithinLimits = false;
		while (CIMRObject_->m_bOrientationControl)
		{
			CIMRObject_->CalculateError(CTrajectoryGoalObject_);
			// Have to test this code
			double PID_w = 0;
			CIMRObject_->PID_AngularVelocityControl(PID_AngularSpeed, CTrajectoryGoalObject_, 0.015, PID_w);
			if (CIMRObject_->m_bAngleWithinLimits)
				break;
			CIMRObject_->SetVelocity(0.0, PID_w);
			// Testing Code done
			//CIMRObject_->PID_OrientationControl(CTrajectoryGoalObject_);
			CIMRObject_->LimitVelocities();
			CIMRObject_->ImposeDisturbance();
			CIMRObject_->UpdatePose();
			fprintf(mfile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", m_fTime, CIMRObject_->m_fDistanceIMRToGoal, CIMRObject_->m_fDeltaTheta,
				CIMRObject_->m_fVelocityLeftWheel, CIMRObject_->m_fVelocityRightWheel, CIMRObject_->m_CurrentPose.m_Point.x,
				CIMRObject_->m_CurrentPose.m_Point.y, CIMRObject_->m_CurrentPose.m_dOrientation, CTrajectoryGoalObject_->m_GoalPose.m_Point.x,
				CTrajectoryGoalObject_->m_GoalPose.m_Point.y);
			m_fTime += SamplingTime * 0.001;
			printf("\n");
		}
		CIMRObject_->m_bPositionControl = true;
		CIMRObject_->m_bOrientationControl = false;
		printf("Follow the trajectory initiated\n");
		return;
	}
	void Initialize()
	{ 
		CIMR_ver1* CIMRObject_ = new CIMR_ver1();
		CTrajectoryGoal_ver1* CTrajectoryGoalObject_ = new CTrajectoryGoal_ver1(true);

		// PID_linearSpeed
		PID_linearSpeed->m_dKP = 0.2;//1.5; // 0.6
		PID_linearSpeed->m_dKD = 0.4; // 0.08
		PID_linearSpeed->m_dKI = 0.000; // 0.03
		PID_linearSpeed->m_nHzPID = 250;
		InitPIDController(PID_linearSpeed);
		     
		// PID_AngularSpeed
		PID_AngularSpeed->m_dKP = 15.0; //0.7 2.0
		PID_AngularSpeed->m_dKD = 1.2 ;
		PID_AngularSpeed->m_dKI = 0.00;
		PID_AngularSpeed->m_nHzPID = 250;
		InitPIDController(PID_AngularSpeed);

		FILE* auf = fopen("D:/PIDFollowTrajectory_Output.csv", "w");
		fprintf(auf, "Time,Distance,Theta,LW_Vel,RW_Vel,Pose_x,Pose_y,Pose_Ori,Goal_x,Goal_y\n");
		CorrectOrientation(CIMRObject_, CTrajectoryGoalObject_, auf);
		_getch();
		
		while (m_fTime < m_fTotSimulationTime)
		{
			CIMRObject_->m_bDistanceWithinLimits = false;
			CIMRObject_->m_bAngleWithinLimits = false;
			CIMRObject_->CalculateError(CTrajectoryGoalObject_);
			CIMRObject_->Control_GetToGoalPosition(CTrajectoryGoalObject_, PID_linearSpeed, PID_AngularSpeed);
			CIMRObject_->UpdatePose();
			fprintf(auf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", m_fTime, CIMRObject_->m_fDistanceIMRToGoal, CIMRObject_->m_fDeltaTheta,
				CIMRObject_->m_fVelocityLeftWheel, CIMRObject_->m_fVelocityRightWheel, CIMRObject_->m_CurrentPose.m_Point.x,
				CIMRObject_->m_CurrentPose.m_Point.y, CIMRObject_->m_CurrentPose.m_dOrientation, CTrajectoryGoalObject_->m_GoalPose.m_Point.x,
				CTrajectoryGoalObject_->m_GoalPose.m_Point.y);
			
			if (CIMRObject_->m_bGoalReached)
				break;
			CIMRObject_->IgnoreDesiredPose2(CTrajectoryGoalObject_);

			m_fTime += SamplingTime * 0.001;
			printf("m_fTime %f\n", m_fTime);
			printf("\n");
			//_getch();
		}
		fclose(auf);
	}
};