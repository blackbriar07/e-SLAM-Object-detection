#pragma once
#include <math.h>
#include <vector>
#include <conio.h>
#include "Geometry.h"
#include "PIDControl.h"
#include "ReadWriteFile.h"
#include "KalmanFilter.h"


class CQRCode
{
public:
	Pose m_CurrentPose;
	Pose m_PreviousPose;
public:
	CQRCode() 
	{
		m_CurrentPose.m_Point.x = 500.0;
		m_CurrentPose.m_Point.y = 0.0; //25
		m_CurrentPose.m_dOrientation = 0.0;
		m_PreviousPose.m_Point.x = 100.0;
		m_PreviousPose.m_Point.y = 0.0;
		m_PreviousPose.m_dOrientation = 0.0;
	}
};

class CIMR_Trajectory
{
public:
	Pose m_CurrentPose;
	Pose m_PreviousPose;
	Pose m_VecIMRToQRCode;
	double m_fVelocityLeftWheel;
	double m_fVelocityRightWheel;
	double m_fLinearVelocity;
	double m_fAngularVelocity;
	double m_fDistanceIMRToQRCode;
	double m_fDeltaTheta;
	double m_fOrientationofCameraRelativeToIMR;
	double m_fTotalTime;
	double m_fMaintainDistance;
	double m_fMaintainAngle;
	double FOV_rad;
	double FOV_cm;
	double QRCode_ThresholdAngle;
	double m_fTimeSpentOutsideFOV;
	bool InRange_FOV;
	bool m_bVelocityZero;
	bool m_bDistanceWithinLimits;
	bool m_bAngleWithinLimits;
	bool m_bStartFollowing;
	CGeometry CGeometryobj;
	CKalmanFilter CKalmanFilterobj;

public:
	CIMR_Trajectory()
	{
		m_CurrentPose.m_Point.x = -800.0;
		m_CurrentPose.m_Point.y = 0.0;
		m_CurrentPose.m_dOrientation = 0.0;
		m_PreviousPose.m_Point.x = 0.0;
		m_PreviousPose.m_Point.y = 0.0;
		m_PreviousPose.m_dOrientation = 0.0;
		m_fVelocityLeftWheel = 0.0;
		m_fVelocityRightWheel = 0.0;
		m_fLinearVelocity = 0.0;
		m_fAngularVelocity = 0.0;
		m_fMaintainDistance = 100;
		m_fMaintainAngle = 0.0;
		m_fDistanceIMRToQRCode = 0.0;
		m_fDeltaTheta = 0.0;
		m_fTotalTime = 0.0;
		m_fOrientationofCameraRelativeToIMR = 0.0;
		FOV_rad = 0.261799;
		FOV_cm = 500;
		QRCode_ThresholdAngle = 1.2217;
		m_fTimeSpentOutsideFOV = 0;
		InRange_FOV = false;
		m_bVelocityZero = false;
		m_bDistanceWithinLimits = false;
		m_bAngleWithinLimits = false;	
		m_bStartFollowing = false;

		// Setting up the specification for kalman filter
		CKalmanFilterobj.m_nStates = 5;
		CKalmanFilterobj.m_nInputs = 0;
		CKalmanFilterobj.m_nOutputs = 3;
	}
	void UpdatePose()
	{
		m_PreviousPose = m_CurrentPose;
		m_CurrentPose.m_Point.x += ((m_fVelocityLeftWheel + m_fVelocityRightWheel) / 2) * cos(m_PreviousPose.m_dOrientation) * (SamplingTime * 0.001);
		m_CurrentPose.m_Point.y += ((m_fVelocityLeftWheel + m_fVelocityRightWheel) / 2) * sin(m_PreviousPose.m_dOrientation) * (SamplingTime * 0.001);
		m_CurrentPose.m_dOrientation += ((m_fVelocityRightWheel - m_fVelocityLeftWheel) / IMRWIDTH) * (SamplingTime * 0.001);
		
		//m_CurrentPose.m_Point.x = Disturbance(m_CurrentPose.m_Point.x);
		//m_CurrentPose.m_Point.y = Disturbance(m_CurrentPose.m_Point.y);
	}
	bool InsideFOV(Pose QRCodeWorldFramePose)
	{
		Point2D QRCodeWorldFramePoint = QRCodeWorldFramePose.m_Point;

		Point2D TrianglePoint1;
		Point2D TrianglePoint2;
		Point2D TrianglePoint3;

		TrianglePoint1.x = m_CurrentPose.m_Point.x;
		TrianglePoint1.y = m_CurrentPose.m_Point.y;

		TrianglePoint2.x = m_CurrentPose.m_Point.x + FOV_cm * cos(m_CurrentPose.m_dOrientation + FOV_rad);
		TrianglePoint2.y = m_CurrentPose.m_Point.y + FOV_cm * sin(m_CurrentPose.m_dOrientation + FOV_rad);

		TrianglePoint3.x = m_CurrentPose.m_Point.x + FOV_cm * cos(m_CurrentPose.m_dOrientation - FOV_rad);
		TrianglePoint3.y = m_CurrentPose.m_Point.y + FOV_cm * sin(m_CurrentPose.m_dOrientation - FOV_rad);

		//CGeometryobj.PrintPose(m_CurrentPose, "IMRPose");
		//CGeometryobj.PrintPoint(QRCodeWorldFramePoint, "QRPoint");
		//CGeometryobj.PrintPoint(TrianglePoint1, "TrianglePoint1");
		//CGeometryobj.PrintPoint(TrianglePoint2, "TrianglePoint2");
		//CGeometryobj.PrintPoint(TrianglePoint3, "TrianglePoint3");

		Point2D TrianglePoints[3] = { TrianglePoint1 ,TrianglePoint2 ,TrianglePoint3 };

		m_VecIMRToQRCode.m_dOrientation = QRCodeWorldFramePose.m_dOrientation - m_CurrentPose.m_dOrientation;
		//if (CGeometryobj.PointInsideTriangle(TrianglePoints, QRCodeWorldFramePoint) &&
		//	m_fOrientationofCameraRelativeToIMR < QRCode_ThresholdAngle)
		if (CGeometryobj.PointInsideTriangle(TrianglePoints, QRCodeWorldFramePoint) &&
			fabs(m_VecIMRToQRCode.m_dOrientation) < QRCode_ThresholdAngle)
		{
			m_fTimeSpentOutsideFOV = 0;
			return true;
		}
		m_fTimeSpentOutsideFOV += (SamplingTime * 0.001);
		return false;

	}
	void CalculateCameraVector(Pose QRCodeWorldFramePose)
	{
		m_fDeltaTheta = CGeometryobj.CalculateVectorIMRTOQRCode(QRCodeWorldFramePose, m_CurrentPose, m_VecIMRToQRCode);
		m_fDistanceIMRToQRCode = m_VecIMRToQRCode.m_Point.y;
	}
	void CalculateError(Pose QRCodeWorldFramePose)
	{
		Point2D QRCodeWorldFramePoint = QRCodeWorldFramePose.m_Point;
		InRange_FOV = InsideFOV(QRCodeWorldFramePose);
		CalculateCameraVector(QRCodeWorldFramePose);
		if (!InRange_FOV && m_bStartFollowing)
		{
			QRCodeWorldFramePoint.x = CKalmanFilterobj.m_mX_Priori(0, 0);
			QRCodeWorldFramePoint.y = CKalmanFilterobj.m_mX_Priori(1, 0);
			QRCodeWorldFramePose.m_dOrientation = CKalmanFilterobj.m_mX_Priori(2, 0);
			CalculateCameraVector(QRCodeWorldFramePose);
			m_VecIMRToQRCode.m_dOrientation = QRCodeWorldFramePose.m_dOrientation - m_CurrentPose.m_dOrientation;
		}		
		printf("vector (X, Y, theta): (%f,%f,%f)\n", m_VecIMRToQRCode.m_Point.x, m_VecIMRToQRCode.m_Point.y, m_VecIMRToQRCode.m_dOrientation);
		
	}
	void NormalizeVelocities()
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
	void Control_FollowQRCode(PIDController* PID_linearSpeed, PIDController* PID_AngularSpeed)
	{
		double PID_v;
		double PID_w;
		PID_linearVelocityControl(PID_linearSpeed, 5, PID_v);
		PID_AngularVelocityControl(PID_AngularSpeed, 0.0174533, PID_w);
		printf("PID_v %f\n", PID_v);
		printf("PID_w %f\n", PID_w);
		m_fLinearVelocity = PID_v; 
		m_fAngularVelocity = PID_w;
		printf("m_fLinearVelocity %f\n", m_fLinearVelocity);
		printf("m_fAngularVelocity %f\n", m_fAngularVelocity);
		m_fVelocityLeftWheel = m_fLinearVelocity - m_fAngularVelocity;
		m_fVelocityRightWheel = m_fLinearVelocity + m_fAngularVelocity;

		if (fabs(m_fVelocityLeftWheel) > 1e-1 && fabs(m_fVelocityRightWheel) > 1e-1)
			NormalizeVelocities();
		
		if (fabs(m_fVelocityLeftWheel) > 0.5)
			m_fVelocityLeftWheel = Disturbance(m_fVelocityLeftWheel, -0.5, 0.5);
		if (fabs(m_fVelocityRightWheel) > 0.5)
			m_fVelocityRightWheel = Disturbance(m_fVelocityRightWheel, -0.5, 0.5);

		

		if (m_bDistanceWithinLimits && m_bAngleWithinLimits)
		{
			m_fVelocityLeftWheel = 0.0;
			m_fVelocityRightWheel = 0.0;
		}
		printf("m_fVelocityLeftWheel %f\n", m_fVelocityLeftWheel);
		printf("m_fVelocityRightWheel %f\n", m_fVelocityRightWheel);
	}
	void PID_linearVelocityControl(PIDController* PID_linearSpeed, double threshold, double& PID_v)
	{
		double dError = m_fDistanceIMRToQRCode - m_fMaintainDistance;
		printf("linear dError before %f\n", dError);
		if (-1 * threshold < dError && dError < threshold)
		{
			dError = 0.0;
			m_bDistanceWithinLimits = true;
		}
		else
			m_bDistanceWithinLimits = false;
		printf("linear dError %f\n", dError);
		
		PID_v = PIDFeedback(PID_linearSpeed, dError);
	}
	void PID_AngularVelocityControl(PIDController* PID_AngularSpeed, double threshold, double& PID_w)
	{
		double dError = m_fDeltaTheta;
		if (-1 * threshold < dError && dError < threshold)
		{
			dError = 0.0;
			m_bAngleWithinLimits = true;
		}
		else
			m_bAngleWithinLimits = false;
		printf("angle dError %f\n", dError);
		PID_w = PIDFeedback(PID_AngularSpeed, dError);
	}
	void KalmanFilterCorrection(Pose QRCodeWorldFramePose)
	{
		CKalmanFilterobj.CalculateKF(QRCodeWorldFramePose.m_Point.x, QRCodeWorldFramePose.m_Point.y, 
			QRCodeWorldFramePose.m_dOrientation);
	}
};

class CFollowRobot
{
public:
	double m_fTime;
	double m_fTotSimulationTime;
	vector<Pose> QRCodeTrajectory;
	vector<double> QRCodeTrajectoryTime;
	CGeometry CGeometryobj;
	int SimulationLength;
public:
	CFollowRobot()
	{
		m_fTime = 0.0;
		m_fTotSimulationTime = 400.0;
		SimulationLength = 0;
	}
public:
	int ReadFileForQRCodeTrajectory(const char* filename)
	{
		FILE* file = fopen(filename, "r");

	// Check if the file was opened successfully
		if (file == nullptr) {
			perror("Error opening file");
			return 1;
		}

		// Read and print each line of the file
		char buffer[256];
		vector<vector<string>> All_Data;
		int lineCount = 0;
		while (fgets(buffer, sizeof(buffer), file) != nullptr) {
			if (lineCount == 0)
			{
				lineCount++;
				continue;
			}
			
			string wordBeforeComma;
			string wordAfterComma;
			vector<string> line_Data;
			for (int i = 0; buffer[i] != '\0'; ++i) {
				// Check if the character is a whitespace or a newline
				if (buffer[i] == ',') 
				{
					line_Data.push_back(wordBeforeComma.c_str());			
					wordBeforeComma = "";
				}
				else if (buffer[i] == '\n')
				{
					line_Data.push_back(wordBeforeComma.c_str());
					All_Data.push_back(line_Data);
				}
				else {
					// Append the character to the current word					
					if (isspace(buffer[i]))
						continue;
					wordBeforeComma += buffer[i];
				}
			}
			lineCount++;
		}
		for (int i = 0; i < All_Data.size(); i++)
		{
			int Count = 0;
			Pose Posevalue;
			for (int j = 0; j < All_Data[i].size(); j++)
			{
				if (Count == 0)
					QRCodeTrajectoryTime.push_back(stod(All_Data[i][j]));
				else if (Count == 5)
					Posevalue.m_Point.x = stod(All_Data[i][j]);
				else if (Count == 6)
					Posevalue.m_Point.y = stod(All_Data[i][j]);
				else if (Count == 7)
					Posevalue.m_dOrientation = stod(All_Data[i][j]);
				Count++;
			}
			QRCodeTrajectory.push_back(Posevalue);
		}
		SimulationLength = int(All_Data.size());
		// Close the file
		fclose(file);
		return 0;
	}
	void TestKalmanFilter()
	{
		CKalmanFilter* CKalmanFilterObj = new CKalmanFilter(5,0,3);
		//delete CKalmanFilterObj;
		FILE* auf = fopen("D:/TestKalmnaFilter_Output.csv", "w");
		fprintf(auf, "R_x, R_y, R_Ori, KF_x, KF_y, KF_Ori\n");
		for (int i = 0; i < SimulationLength; i++)
		{
			printf("i %d\n", i);
			if (i == 0)
				continue;
			double delta_t = (QRCodeTrajectoryTime[i] - QRCodeTrajectoryTime[i - 1]);
			double qx = QRCodeTrajectory[i].m_Point.x;
			double qy = QRCodeTrajectory[i].m_Point.y;
			double qori = QRCodeTrajectory[i].m_dOrientation;
			double q_dot = CGeometryobj.CalculateDistanceBetweenPoints(QRCodeTrajectory[i].m_Point, QRCodeTrajectory[i-1].m_Point)/ delta_t;
			double qori_dot = (QRCodeTrajectory[i].m_dOrientation - QRCodeTrajectory[i-1].m_dOrientation) / delta_t;
			if (i == 1)
			{
				//printf("%f, %f, %f, %f, %f", qx, qy, qori, q_dot, qori_dot, delta_t);
				CKalmanFilterObj->Initialize(qx, qy, qori, q_dot, qori_dot, delta_t);
				//continue;
			}
			CKalmanFilterObj->CalculateKF(qx, qy, qori);
			
			//printf("(R_x, R_x, R_x): (%f, %f, %f)\n", qx, qy, qori);
			//printf("(KF_x, KF_x, KF_x): (%f, %f, %f)\n", CKalmanFilterObj->m_mX_Posteriori(0,0), CKalmanFilterObj->m_mX_Posteriori(1, 0), CKalmanFilterObj->m_mX_Posteriori(2, 0));
			fprintf(auf, "%f, %f, %f, %f, %f, %f\n", qx, qy, qori, CKalmanFilterObj->m_mX_Priori(0, 0), CKalmanFilterObj->m_mX_Priori(1, 0), CKalmanFilterObj->m_mX_Priori(2, 0));
			//_getch();

		}
		fclose(auf);
		//CKalmanFilter* CKalmanFilterObj = new CKalmanFilter(0.0);
		delete CKalmanFilterObj;
	}
	void Initialize()
	{
		//const char* filename = "QRCodeTrajectory_Data/Eight_Case.csv";
		//const char* filename = "QRCodeTrajectory_Data/Data_Case2_1.csv";
		const char* filename = "QRCodeTrajectory_Data/Data_Case2_2.csv";
		ReadFileForQRCodeTrajectory(filename);
		//TestKalmanFilter();
		//return;
		//for (int j = 0; j < QRCodeTrajectory.size(); j++)
		//{
		//	printf("Time%f X%f Y%f Ori%f\n", QRCodeTrajectoryTime[j], QRCodeTrajectory[j].m_Point.x, QRCodeTrajectory[j].m_Point.y, QRCodeTrajectory[j].m_dOrientation);
		//}
		printf("Simulation length %d\n\n", SimulationLength);
		//return;
		CIMR_Trajectory* CIMRObject = new CIMR_Trajectory();
		CQRCode* CQRCodeObject = new CQRCode();

		

		int SimualtionCount = 0;

		//Point2D QRCodePoint = QRCodeTrajectory[SimualtionCount].m_Point;
		CIMRObject->CalculateError(QRCodeTrajectory[SimualtionCount]);

		PIDController* PID_linearSpeed = new PIDController();
		PIDController* PID_AngularSpeed = new PIDController();

		// PID_linearSpeed
		PID_linearSpeed->m_dKP = 0.3; // 1.55 0.6
		PID_linearSpeed->m_dKD = 0.03; // 0.08
		PID_linearSpeed->m_dKI = 0.00; // 0.03
		PID_linearSpeed->m_nHzPID = 250;
		InitPIDController(PID_linearSpeed);

		// PID_AngularSpeed
		PID_AngularSpeed->m_dKP = 17.5; //0.7 2.0
		PID_AngularSpeed->m_dKD = 1.01;
		PID_AngularSpeed->m_dKI = 0.01;
		PID_AngularSpeed->m_nHzPID = 250;
		InitPIDController(PID_AngularSpeed);

		FILE* auf = fopen("D:/PIDFollowRobot_Output.csv", "w");
		fprintf(auf, "Time,Distance,dTheta,QRToIMR_Theta, QRToIMR_X,QRToIMR_Y,InRange, LW_Vel,RW_Vel,Pose_x,Pose_y,Pose_Ori,QRPose_x,QRPose_y,QRPose_Ori\n");		

		
		while (CIMRObject->m_fDistanceIMRToQRCode >= CIMRObject->m_fMaintainDistance + 5)
		{
			CIMRObject->Control_FollowQRCode(PID_linearSpeed, PID_AngularSpeed);

			CIMRObject->UpdatePose();
			if (CIMRObject->InRange_FOV)
				printf("In range\n");
			else
				printf("Not in range\n");
			m_fTime += SamplingTime * 0.001;
			printf("m_fTime %f\n", m_fTime);
			printf("\n");

			Pose QRCodePose = QRCodeTrajectory[SimualtionCount];
			Point2D QRCodePoint = QRCodeTrajectory[SimualtionCount].m_Point;

			CIMRObject->CalculateError(QRCodePose);
			printf("distance error %f\n", CIMRObject->m_fDistanceIMRToQRCode);
			printf("theta error %f\n", CIMRObject->m_fDeltaTheta);
			

			fprintf(auf, "%f,%f,%f,%f, %f,%f,%d, %f,%f,%f,%f,%f,%f,%f,%f\n", m_fTime, CIMRObject->m_fDistanceIMRToQRCode, CIMRObject->m_fDeltaTheta,
				CIMRObject->m_VecIMRToQRCode.m_dOrientation,
				CIMRObject->m_VecIMRToQRCode.m_Point.x,
				CIMRObject->m_VecIMRToQRCode.m_Point.y,
				CIMRObject->InRange_FOV,
				CIMRObject->m_fVelocityLeftWheel, CIMRObject->m_fVelocityRightWheel, CIMRObject->m_CurrentPose.m_Point.x,
				CIMRObject->m_CurrentPose.m_Point.y, CIMRObject->m_CurrentPose.m_dOrientation, QRCodePoint.x, QRCodePoint.y,
				QRCodeTrajectory[SimualtionCount].m_dOrientation);
			//_getch();
		}
		CIMRObject->m_bStartFollowing = true;
		//fclose(auf);
		//return;
		SimualtionCount++;

		// Initialising Kalman Filter ///////////////////////////////////////////////
		double delta_t = (QRCodeTrajectoryTime[SimualtionCount] - QRCodeTrajectoryTime[SimualtionCount - 1]);
		double qx = QRCodeTrajectory[SimualtionCount].m_Point.x;
		double qy = QRCodeTrajectory[SimualtionCount].m_Point.y;
		double qori = QRCodeTrajectory[SimualtionCount].m_dOrientation;
		double q_dot = CGeometryobj.CalculateDistanceBetweenPoints(QRCodeTrajectory[SimualtionCount].m_Point, QRCodeTrajectory[SimualtionCount - 1].m_Point) / delta_t;
		double qori_dot = (QRCodeTrajectory[SimualtionCount].m_dOrientation - QRCodeTrajectory[SimualtionCount - 1].m_dOrientation) / delta_t;

		CIMRObject->CKalmanFilterobj.Initialize(qx, qy, qori, q_dot, qori_dot, delta_t);
		/////////////////////////////////////////////////////////////////////////////////////////

		while (m_fTime < m_fTotSimulationTime)
		{
			Pose QRCodePose = QRCodeTrajectory[SimualtionCount];
			Point2D QRCodePoint = QRCodeTrajectory[SimualtionCount].m_Point;
			CIMRObject->KalmanFilterCorrection(QRCodePose);
			CIMRObject->CalculateError(QRCodePose);
			printf("distance error %f\n", CIMRObject->m_fDistanceIMRToQRCode);
			printf("theta error %f\n", CIMRObject->m_fDeltaTheta);

			fprintf(auf, "%f,%f,%f,%f, %f,%f,%d, %f,%f,%f,%f,%f,%f,%f,%f\n", m_fTime, CIMRObject->m_fDistanceIMRToQRCode, CIMRObject->m_fDeltaTheta,
				CIMRObject->m_VecIMRToQRCode.m_dOrientation,
				CIMRObject->m_VecIMRToQRCode.m_Point.x,
				CIMRObject->m_VecIMRToQRCode.m_Point.y,
				CIMRObject->InRange_FOV,
				CIMRObject->m_fVelocityLeftWheel, CIMRObject->m_fVelocityRightWheel, CIMRObject->m_CurrentPose.m_Point.x,
				CIMRObject->m_CurrentPose.m_Point.y, CIMRObject->m_CurrentPose.m_dOrientation, QRCodePoint.x, QRCodePoint.y,
				QRCodeTrajectory[SimualtionCount].m_dOrientation);


			CIMRObject->Control_FollowQRCode(PID_linearSpeed, PID_AngularSpeed);
			
			CIMRObject->UpdatePose();
			m_fTime += SamplingTime * 0.001;
			printf("m_fTime %f\n", m_fTime);
			printf("\n");
			printf("SimulationCount %d\n", SimualtionCount);
			if (SimualtionCount < SimulationLength-1)
				SimualtionCount++;
			//_getch();
		}
		fclose(auf);
	}
};