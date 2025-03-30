#pragma once
#include <math.h>
#include <vector>
#include <conio.h>
#include "Geometry.h"
#include <Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


class CKalmanFilter
{
public:
	CKalmanFilter(){}
	CKalmanFilter(int p_nStates, int p_nInputs, int p_nOutputs)
	{
		m_nStates = p_nStates;
		m_nInputs = p_nInputs;
		m_nOutputs = p_nOutputs;
	}
public:
	int m_nStates;
	int m_nInputs;
	int m_nOutputs;
	int m_nRawDataSimulate;
	double m_fSamplingTime;
public:
	MatrixXd m_mX;
	MatrixXd m_mP_Priori;
	MatrixXd m_mP_Posteriori;
	MatrixXd m_mX_Priori;
	VectorXd m_mX_Posteriori;
public:
	MatrixXd m_mStateTransition; // kalman gain
	MatrixXd m_mU;
	MatrixXd m_mA; // Jacobian matrix
	MatrixXd m_mQ; // process noise covariance
	MatrixXd m_mH; // Transformation matrix
	MatrixXd m_mR;
	MatrixXd m_mY;
	MatrixXd m_mZ; // measurement matrix
	MatrixXd m_mS;
	MatrixXd m_mK; // kalman gain
public:
	void Initialize(double p_x, double p_y, double p_theta, double p_LinearVelocity, double p_AngularVelocity,
		double Sampling_Time)
	{
		//printf("%f, %f, %f, %f, %f", p_x, p_y, p_theta, p_LinearVelocity, p_AngularVelocity);
		Add_Input(p_x, p_y, p_theta, p_LinearVelocity, p_AngularVelocity);
		Create(Sampling_Time);
	}
	void Create(double fSamplingTime)
	{
		m_fSamplingTime = fSamplingTime;

		m_mStateTransition.resize(m_nStates, m_nStates);
		m_mStateTransition.setZero();
		m_mX.resize(m_nStates, 1);
		m_mX.setZero();
		m_mX_Priori.resize(m_nStates, 1);
		m_mX_Priori.setZero();
		
		m_mU.resize(m_nStates, 1);
		m_mU.setZero();

		m_mP_Priori.resize(m_nStates, m_nStates);
		m_mP_Priori.setZero();
		
		m_mA.resize(m_nStates, m_nStates);
		m_mA.setZero();
		m_mQ.resize(m_nStates, m_nStates);
		m_mQ.setZero();
		for (int i = 0; i < m_nStates; i++)
		{
			for (int j = 0; j < m_nStates; j++)
			{
				if (i == 0 && j == 0)
					m_mQ(i, j) = 0.01;
				if (i == 1 && j == 1)
					m_mQ(i, j) = 0.01;
				if (i == 2 && j == 2)
					m_mQ(i, j) = 0.05;
				if (i == 3 && j == 3)
					m_mQ(i, j) = 0.01;
				if (i == 4 && j == 4)
					m_mQ(i, j) = 0.01;
			}
		}
		// Rx,Tx = (0.01,2.5)  Ry,Ty = (0.01,2.5)  Rtheta,Ttheta = (0.01,2.5)
		m_mY.resize(m_nOutputs, 1);
		m_mY.setZero();
		m_mZ.resize(m_nOutputs, 1);
		m_mZ.setZero();
		m_mS.resize(m_nOutputs, m_nOutputs);
		m_mS.setZero();
		m_mR.resize(m_nOutputs, m_nOutputs);
		m_mR.setZero();
		for (int i = 0; i < m_nOutputs; i++)
		{
			for (int j = 0; j < m_nOutputs; j++)
			{
				//if (i == j)
					//m_mR(i, j) = 2.5;
				if (i == 0 && j == 0)
					m_mR(i, j) = 2.5;
				if (i == 1 && j == 1)
					m_mR(i, j) = 2.5;
				if (i == 2 && j == 2)
					m_mR(i, j) = 0.01;
			}
		}

		m_mK.resize(m_nStates, m_nOutputs);
		m_mK.setZero();
		m_mH.resize(m_nOutputs, m_nStates);
		m_mH(0, 0) = 1.0;
		m_mH(0, 1) = 0.0;
		m_mH(0, 2) = 0.0;
		m_mH(0, 3) = 0.0;
		m_mH(0, 4) = 0.0;

		m_mH(1, 0) = 0.0;
		m_mH(1, 1) = 1.0;
		m_mH(1, 2) = 0.0;
		m_mH(1, 3) = 0.0;
		m_mH(1, 4) = 0.0;

		m_mH(2, 0) = 0.0;
		m_mH(2, 1) = 0.0;
		m_mH(2, 2) = 1.0;
		m_mH(2, 3) = 0.0;
		m_mH(2, 4) = 0.0;

	}
	void Add_Input(double p_x, double p_y, double p_theta, double p_LinearVelocity, double p_AngularVelocity)
	{	
		m_mX_Posteriori.resize(m_nStates);
		m_mX_Posteriori(0) = p_x;
		m_mX_Posteriori(1) = p_y;
		m_mX_Posteriori(2) = p_theta;
		m_mX_Posteriori(3) = p_LinearVelocity;
		m_mX_Posteriori(4) = p_AngularVelocity;
		m_mP_Posteriori.resize(m_nStates, m_nStates);
		m_mP_Posteriori.setZero();
	}
	void CalculateKF(double pMeasuredX, double pMeasuredY, double pMeasuredTheta)
	{
		PredictionStep();
		m_mX(0, 0) = pMeasuredX;
		m_mX(1, 0) = pMeasuredY;
		m_mX(2, 0) = pMeasuredTheta;
		MeasurementStep();
		CorrectionStep();
	}
	void Calculate_JacobianA()
	{
		m_mA(0, 0) = 1.0;
		m_mA(0, 1) = 0.0;
		m_mA(0, 2) = -1* m_fSamplingTime*(m_mX_Priori(3,0)*sin(m_mX_Priori(2, 0))* m_mX_Priori(4, 0) + 
			0.5* m_mX_Priori(3, 0)* m_mX_Priori(4, 0) * m_mX_Priori(4, 0) * cos(m_mX_Priori(2, 0)) * m_fSamplingTime);
		m_mA(0, 3) = m_fSamplingTime * (cos(m_mX_Priori(2, 0)) - 0.5 * m_mX_Priori(4, 0)*sin(m_mX_Priori(2, 0)) * m_fSamplingTime);
		m_mA(0, 4) = -0.5 * m_mX_Priori(3, 0) * sin(m_mX_Priori(2, 0)) * m_fSamplingTime * m_fSamplingTime;
  
		m_mA(1, 0) = 0.0;
		m_mA(1, 1) = 1.0;
		m_mA(1, 2) = m_fSamplingTime * (m_mX_Priori(3, 0) * cos(m_mX_Priori(2, 0)) * m_mX_Priori(4, 0) -
			0.5 * m_mX_Priori(3, 0) * m_mX_Priori(4, 0) * m_mX_Priori(4, 0) * sin(m_mX_Priori(2, 0)) * m_fSamplingTime);
		m_mA(1, 3) = m_fSamplingTime * (sin(m_mX_Priori(2, 0)) + 0.5 * m_mX_Priori(4, 0) * cos(m_mX_Priori(2, 0)) * m_fSamplingTime);
		m_mA(1, 4) = 0.5 * m_mX_Priori(3, 0) * cos(m_mX_Priori(2, 0)) * m_fSamplingTime * m_fSamplingTime;

		m_mA(2, 0) = 0.0;
		m_mA(2, 1) = 0.0;
		m_mA(2, 2) = 1.0;
		m_mA(2, 3) = 0.0;
		m_mA(2, 4) = m_fSamplingTime;

		m_mA(3, 0) = 0.0;
		m_mA(3, 1) = 0.0;
		m_mA(3, 2) = 0.0;
		m_mA(3, 3) = 1.0;
		m_mA(3, 4) = 0.0;

		m_mA(4, 0) = 0.0;
		m_mA(4, 1) = 0.0;
		m_mA(4, 2) = 0.0;
		m_mA(4, 3) = 0.0;
		m_mA(4, 4) = 1.0;
	}
	void Calculate_XPriori()
	{
		m_mStateTransition(0, 0) = 1.0;
		m_mStateTransition(0, 1) = 0.0;
		m_mStateTransition(0, 2) = 0.0;
		m_mStateTransition(0, 3) = cos(m_mX_Posteriori(2, 0)) * m_fSamplingTime;
		m_mStateTransition(0, 4) = -0.5 * m_mX_Posteriori(3, 0) * sin(m_mX_Posteriori(2, 0)) * m_fSamplingTime * m_fSamplingTime;

		m_mStateTransition(1, 0) = 0.0;
		m_mStateTransition(1, 1) = 1.0;
		m_mStateTransition(1, 2) = 0.0;
		m_mStateTransition(1, 3) = sin(m_mX_Posteriori(2, 0)) * m_fSamplingTime;
		m_mStateTransition(1, 4) = 0.5 * m_mX_Posteriori(3, 0) * cos(m_mX_Posteriori(2, 0)) * m_fSamplingTime * m_fSamplingTime;

		m_mStateTransition(2, 0) = 0.0;
		m_mStateTransition(2, 1) = 0.0;
		m_mStateTransition(2, 2) = 1.0;
		m_mStateTransition(2, 3) = 0.0;
		m_mStateTransition(2, 4) = m_fSamplingTime;

		m_mStateTransition(3, 0) = 0.0;
		m_mStateTransition(3, 1) = 0.0;
		m_mStateTransition(3, 2) = 0.0;
		m_mStateTransition(3, 3) = 1.0; // +Disturbance(0.0, -1, 1);
		m_mStateTransition(3, 4) = 0.0;

		m_mStateTransition(4, 0) = 0.0;
		m_mStateTransition(4, 1) = 0.0;
		m_mStateTransition(4, 2) = 0.0;
		m_mStateTransition(4, 3) = 0.0;
		m_mStateTransition(4, 4) = 1.0; // +Disturbance(0.0, -1, 1);
	}
	void PredictionStep()
	{
		Calculate_JacobianA();
		Calculate_XPriori();
		m_mU(3, 0) = Disturbance(0.0, -1, 1);
		m_mU(4 , 0) = Disturbance(0.0, -1, 1);
		//cout << "m_mX_Posteriori" << m_mX_Posteriori << endl;
		m_mX_Priori = m_mStateTransition * m_mX_Posteriori + m_mU;
		m_mP_Priori = m_mA * m_mP_Posteriori * m_mA.transpose() + m_mQ;
	}
	void MeasurementStep()
	{
		m_mZ = m_mH * m_mX;
		m_mY = m_mZ - m_mH * m_mX_Priori;
		m_mS = m_mH * m_mP_Priori * m_mH.transpose() + m_mR;
	}
	void CorrectionStep()
	{
		m_mK = m_mP_Priori * m_mH.transpose() * m_mS.inverse();
		m_mX_Posteriori = m_mX_Priori + m_mK * m_mY;
		m_mP_Posteriori = m_mP_Priori - m_mK * m_mH * m_mP_Priori;
	}
};



class CKalmanFilterIMRQR
{
public:
	CKalmanFilterIMRQR() {}
	CKalmanFilterIMRQR(int p_nStates, int p_nInputs, int p_nOutputs)
	{
		m_nStates = p_nStates;
		m_nInputs = p_nInputs;
		m_nOutputs = p_nOutputs;
	}
public:
	int m_nStates;
	int m_nInputs;
	int m_nOutputs;
	int m_nRawDataSimulate;
	double m_fSamplingTime;
public:
	CGeometry CGeometryObj;
	MatrixXd m_mX;
	MatrixXd m_mP_Priori;
	MatrixXd m_mP_Posteriori;
	MatrixXd m_mX_Priori;
	VectorXd m_mX_Posteriori;
	MatrixXd m_mX_TempPriori;
public:
	MatrixXd m_mStateTransition; // kalman gain
	MatrixXd m_mU;
	MatrixXd m_mA; // Jacobian matrix
	MatrixXd m_mQ; // process noise covariance
	MatrixXd m_mH; // Transformation matrix
	MatrixXd m_mR;
	MatrixXd m_mY;
	MatrixXd m_mZ; // measurement matrix
	MatrixXd m_mS;
	MatrixXd m_mK; // kalman gain
public:
	void Initialize(double p_x, double p_y, double p_theta, double p_LinearVelocity, double p_AngularVelocity,
		double Sampling_Time)
	{
		//printf("%f, %f, %f, %f, %f", p_x, p_y, p_theta, p_LinearVelocity, p_AngularVelocity);
		Add_Input(p_x, p_y, p_theta, p_LinearVelocity, p_AngularVelocity);
		Create(Sampling_Time);
	}
	void Create(double fSamplingTime)
	{
		m_fSamplingTime = fSamplingTime;

		m_mStateTransition.resize(m_nStates, m_nStates);
		m_mStateTransition.setZero();
		m_mX.resize(m_nStates, 1);
		m_mX.setZero();
		m_mX_Priori.resize(m_nStates, 1);
		m_mX_Priori.setZero();
		m_mX_TempPriori.resize(m_nStates, 1);
		m_mX_TempPriori.setZero();

		m_mU.resize(m_nStates, 1);
		m_mU.setZero();

		m_mP_Priori.resize(m_nStates, m_nStates);
		m_mP_Priori.setZero();

		m_mA.resize(m_nStates, m_nStates);
		m_mA.setZero();
		m_mQ.resize(m_nStates, m_nStates);
		m_mQ.setZero();
		for (int i = 0; i < m_nStates; i++)
		{
			for (int j = 0; j < m_nStates; j++)
			{
				if (i == j)
					m_mQ(i, j) = 1.0;
			}
		}

		m_mY.resize(m_nOutputs, 1);
		m_mY.setZero();
		m_mZ.resize(m_nOutputs, 1);
		m_mZ.setZero();
		m_mS.resize(m_nOutputs, m_nOutputs);
		m_mS.setZero();
		m_mR.resize(m_nOutputs, m_nOutputs);
		m_mR.setZero();
		for (int i = 0; i < m_nOutputs; i++)
		{
			for (int j = 0; j < m_nOutputs; j++)
			{
				if (i == j)
					m_mR(i, j) = 1.0;
			}
		}

		m_mK.resize(m_nStates, m_nOutputs);
		m_mK.setZero();
		m_mH.resize(m_nOutputs, m_nStates);
		m_mH(0, 0) = 1.0;
		m_mH(0, 1) = 0.0;
		m_mH(0, 2) = 0.0;
		m_mH(0, 3) = 0.0;
		m_mH(0, 4) = 0.0;

		m_mH(1, 0) = 0.0;
		m_mH(1, 1) = 1.0;
		m_mH(1, 2) = 0.0;
		m_mH(1, 3) = 0.0;
		m_mH(1, 4) = 0.0;

		m_mH(2, 0) = 0.0;
		m_mH(2, 1) = 0.0;
		m_mH(2, 2) = 1.0;
		m_mH(2, 3) = 0.0;
		m_mH(2, 4) = 0.0;

	}
	void Add_Input(double p_x, double p_y, double p_theta, double p_LinearVelocity, double p_AngularVelocity)
	{
		m_mX_Posteriori.resize(m_nStates);
		m_mX_Posteriori(0) = p_x;
		m_mX_Posteriori(1) = p_y;
		m_mX_Posteriori(2) = p_theta;
		m_mX_Posteriori(3) = p_LinearVelocity;
		m_mX_Posteriori(4) = p_AngularVelocity;
		m_mP_Posteriori.resize(m_nStates, m_nStates);
		m_mP_Posteriori.setZero();
	}
	void CalculateKF(double IMRx, double IMRy, double IMROri, double IMRlinearVel, double IMRAngularVel, 
		double pMeasuredX, double pMeasuredY, double pMeasuredTheta)
	{
		PredictionStep(IMRx, IMRy, IMROri, IMRlinearVel, IMRAngularVel);
		m_mX(0, 0) = pMeasuredX;
		m_mX(1, 0) = pMeasuredY;
		m_mX(2, 0) = pMeasuredTheta;
		MeasurementStep();
		CorrectionStep();
	}
	void Calculate_JacobianA()
	{
		m_mA(0, 0) = 1.0;
		m_mA(0, 1) = 0.0;
		m_mA(0, 2) = -1 * m_fSamplingTime * (m_mX_Priori(3, 0) * sin(m_mX_Priori(2, 0)) * m_mX_Priori(4, 0) +
			0.5 * m_mX_Priori(3, 0) * m_mX_Priori(4, 0) * m_mX_Priori(4, 0) * cos(m_mX_Priori(2, 0)) * m_fSamplingTime);
		m_mA(0, 3) = m_fSamplingTime * (cos(m_mX_Priori(2, 0)) - 0.5 * m_mX_Priori(4, 0) * sin(m_mX_Priori(2, 0)) * m_fSamplingTime);
		m_mA(0, 4) = -0.5 * m_mX_Priori(3, 0) * sin(m_mX_Priori(2, 0)) * m_fSamplingTime * m_fSamplingTime;

		m_mA(1, 0) = 0.0;
		m_mA(1, 1) = 1.0;
		m_mA(1, 2) = m_fSamplingTime * (m_mX_Priori(3, 0) * cos(m_mX_Priori(2, 0)) * m_mX_Priori(4, 0) -
			0.5 * m_mX_Priori(3, 0) * m_mX_Priori(4, 0) * m_mX_Priori(4, 0) * sin(m_mX_Priori(2, 0)) * m_fSamplingTime);
		m_mA(1, 3) = m_fSamplingTime * (sin(m_mX_Priori(2, 0)) + 0.5 * m_mX_Priori(4, 0) * cos(m_mX_Priori(2, 0)) * m_fSamplingTime);
		m_mA(1, 4) = 0.5 * m_mX_Priori(3, 0) * cos(m_mX_Priori(2, 0)) * m_fSamplingTime * m_fSamplingTime;

		m_mA(2, 0) = 0.0;
		m_mA(2, 1) = 0.0;
		m_mA(2, 2) = 1.0;
		m_mA(2, 3) = 0.0;
		m_mA(2, 4) = m_fSamplingTime;

		m_mA(3, 0) = 0.0;
		m_mA(3, 1) = 0.0;
		m_mA(3, 2) = 0.0;
		m_mA(3, 3) = 1.0;
		m_mA(3, 4) = 0.0;

		m_mA(4, 0) = 0.0;
		m_mA(4, 1) = 0.0;
		m_mA(4, 2) = 0.0;
		m_mA(4, 3) = 0.0;
		m_mA(4, 4) = 1.0;
	}
	void Calculate_XPriori()
	{
		m_mStateTransition(0, 0) = 1.0;
		m_mStateTransition(0, 1) = 0.0;
		m_mStateTransition(0, 2) = 0.0;
		m_mStateTransition(0, 3) = cos(m_mX_Posteriori(2, 0)) * m_fSamplingTime;
		m_mStateTransition(0, 4) = -0.5 * m_mX_Posteriori(3, 0) * sin(m_mX_Posteriori(2, 0)) * m_fSamplingTime * m_fSamplingTime;

		m_mStateTransition(1, 0) = 0.0;
		m_mStateTransition(1, 1) = 1.0;
		m_mStateTransition(1, 2) = 0.0;
		m_mStateTransition(1, 3) = sin(m_mX_Posteriori(2, 0)) * m_fSamplingTime;
		m_mStateTransition(1, 4) = 0.5 * m_mX_Posteriori(3, 0) * cos(m_mX_Posteriori(2, 0)) * m_fSamplingTime * m_fSamplingTime;

		m_mStateTransition(2, 0) = 0.0;
		m_mStateTransition(2, 1) = 0.0;
		m_mStateTransition(2, 2) = 1.0;
		m_mStateTransition(2, 3) = 0.0;
		m_mStateTransition(2, 4) = m_fSamplingTime;

		m_mStateTransition(3, 0) = 0.0;
		m_mStateTransition(3, 1) = 0.0;
		m_mStateTransition(3, 2) = 0.0;
		m_mStateTransition(3, 3) = 1.0; // +Disturbance(0.0, -1, 1);
		m_mStateTransition(3, 4) = 0.0;

		m_mStateTransition(4, 0) = 0.0;
		m_mStateTransition(4, 1) = 0.0;
		m_mStateTransition(4, 2) = 0.0;
		m_mStateTransition(4, 3) = 0.0;
		m_mStateTransition(4, 4) = 1.0; // +Disturbance(0.0, -1, 1);
	}
	void PredictionStep(double IMRx, double IMRy, double IMROri, double IMRlinearVel, double IMRAngularVel)
	{
		Calculate_JacobianA();
		Calculate_XPriori();
		m_mU(3, 0) = Disturbance(0.0, -0.5, 0.5); //-IMRlinearVel;
		m_mU(4, 0) = Disturbance(0.0, -0.5, 0.5); //-IMRAngularVel;
		m_mX_TempPriori = m_mStateTransition * m_mX_Posteriori;
		Pose QRCodePose;
		Pose IMRPose;
		Pose m_VecIMRToQRCode;
		QRCodePose.m_Point.x = m_mX_TempPriori(0, 0);
		QRCodePose.m_Point.y = m_mX_TempPriori(1, 0);
		QRCodePose.m_dOrientation = m_mX_TempPriori(2, 0);
		IMRPose.m_Point.x = IMRx;
		IMRPose.m_Point.y = IMRy;
		IMRPose.m_dOrientation = IMROri;
		CGeometryObj.CalculateVectorIMRTOQRCode(QRCodePose, IMRPose, m_VecIMRToQRCode);
		m_mX_Priori(0, 0) = m_VecIMRToQRCode.m_Point.x;
		m_mX_Priori(1, 0) = m_VecIMRToQRCode.m_Point.y;
		m_mX_Priori(2, 0) = m_mX_TempPriori(2, 0) -IMROri;
		m_mX_Priori(3, 0) = m_mU(3, 0);
		m_mX_Priori(4, 0) = m_mU(4, 0);
		m_mX_Priori = m_mStateTransition * m_mX_Posteriori + m_mU;
		m_mP_Priori = m_mA * m_mP_Posteriori * m_mA.transpose() + m_mQ;
	}
	void MeasurementStep()
	{
		m_mZ = m_mH * m_mX;
		m_mY = m_mZ - m_mH * m_mX_Priori;
		m_mS = m_mH * m_mP_Priori * m_mH.transpose() + m_mR;
	}
	void CorrectionStep()
	{
		m_mK = m_mP_Priori * m_mH.transpose() * m_mS.inverse();
		m_mX_Posteriori = m_mX_Priori + m_mK * m_mY;
		m_mP_Posteriori = m_mP_Priori - m_mK * m_mH * m_mP_Priori;
	}
};