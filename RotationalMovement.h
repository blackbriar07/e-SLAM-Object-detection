#pragma once
#include <cmath>
#include "Geometry.h"



class CRotationalMotion
{
public:
	double m_dAngularVelocity;
	double m_dICCRadius;
	double m_dDegreeRequired;
	double m_dLWVelocity;
	double m_dRWVelocity;
	double m_dInitialOrientation;
	double m_dStartTime;
	double m_dTotalTime;
	bool m_bClockwise;
public:
	CRotationalMotion()
	{
		m_dAngularVelocity = 0.0;
		m_dICCRadius = 0.0;
		m_dDegreeRequired = 0.0;
		m_dLWVelocity = 0.0;
		m_dRWVelocity = 0.0;
		m_dInitialOrientation = 0.0;
		m_dStartTime = 0.0;
		m_dTotalTime = 0.0;
		m_bClockwise = false;
	}
	// Change Orientation
	void RotationAboutCM(double dDegree, double dWheelSpeed) 
	{
		m_dDegreeRequired = dDegree;
		if (m_bClockwise)
		{
			m_dLWVelocity = dWheelSpeed;
			m_dRWVelocity = -1 * dWheelSpeed;
		}
		else
		{
			m_dLWVelocity = -1 * dWheelSpeed;
			m_dRWVelocity = dWheelSpeed;			
		}
		m_dTotalTime = m_dStartTime + (IMRWIDTH / fabs(m_dRWVelocity - m_dLWVelocity)) * (dDegree * d2r);
	}
	void RotationAboutICC(double dDegree, double dICCRadius, double dOmega) // Omega: Angular Velocity
	{
		m_dDegreeRequired = dDegree;
		m_dICCRadius = dICCRadius;
		if (m_bClockwise)
		{
			m_dLWVelocity = dOmega * (m_dICCRadius + IMRWIDTH /2);
			m_dRWVelocity = dOmega * (m_dICCRadius - IMRWIDTH / 2);
		}
		else
		{
			m_dLWVelocity = dOmega * (m_dICCRadius - IMRWIDTH / 2);
			m_dRWVelocity = dOmega * (m_dICCRadius + IMRWIDTH / 2);
		}
		m_dTotalTime = m_dStartTime + (IMRWIDTH / fabs(m_dRWVelocity - m_dLWVelocity)) * (dDegree * d2r);
	}

};