#pragma once
#include "Geometry.h"


#define MAX_TURN_VELOCITY				60					// [unit: cm/s]
#define MIN_TURN_VELOCITY				10					// [unit: cm/s]
//#define MAX_ACCELERATION				1					// [unit: cm/s^2]
//#define MAX_DECELERATION				1					// [unit: cm/s^2]
#define MAX_STEERING_ANGLE				(90 * d2r)			// [unit: radian]
#define MIN_STEERING_ANGLE				(60 * d2r)			// [unit: radian]
#define SAMPLINGTIME					500					// [unit: ms]
#define PI                              3.141592653589793238462643

class CLinearAccDeceleration
{
public:
	CLinearAccDeceleration()
	{
		m_bIsFeasible = false;
		m_bIsAcc = false;
		m_dTotalTime = 0;
		m_nClockCount = 0;
	}
public:
	double m_dInitPositionTime = 0;
	double m_dStartTime = 0;
	double m_dDistance = 0;
	double m_dVelocityStart = 0;
	double m_dVelocityEnd = 0;
	bool   m_bIsFeasible = 0;
	bool   m_bIsAcc = 0;
	double m_du = 0;
	double m_dTotalTime = 0;
	double m_dMaxVelocity = 0;
	double m_dMaxTurnVelocity = 0;

	double m_dVelocityMax = 0;
	double m_dAccelerationTime = 0;
	double m_dDecelerationTime = 0;
	double MAX_ACCELERATION = 1.0; // [unit: cm/s^2]
	double MAX_DECELERATION = 1.0; // [unit: cm/s^2]
	double m_dConstantVelocityTime = 0;
private:
	int m_nClockCount = 0;
	double m_dWheelRadius = 13.5; // [unit: cm]
public:
	void Set(double dDistance, double dVelocityStart, double dVelocityEnd,
		double dStartTime, double dInitPositionTime)
	{
		m_dDistance = dDistance;
		m_dVelocityStart = dVelocityStart;
		m_dVelocityEnd = dVelocityEnd;
		m_dInitPositionTime = dInitPositionTime;
		m_dStartTime = dStartTime;
		if (m_dStartTime > 0)
			m_nClockCount = 1;
		m_bIsFeasible = IsFeasible();
		MinimumTimeControl();
	}
	bool IsFeasible()
	{
		if (m_dVelocityStart > m_dMaxVelocity)
			return false;
		if (m_dVelocityEnd > m_dMaxVelocity)
			return false;
		m_bIsAcc = m_dVelocityEnd > m_dVelocityStart;
		double dAverageSpeed = (m_dVelocityEnd + m_dVelocityStart) / 2;
		double dDifferenceSpeed = m_dVelocityEnd - m_dVelocityStart;
		double dTime = m_dDistance / dAverageSpeed;
		if (m_bIsAcc)
			return dDifferenceSpeed / dTime < MAX_ACCELERATION;
		return -dDifferenceSpeed / dTime < MAX_DECELERATION;
	}
	bool PopU(double& u, double& dTime)
	{
		double dTimeOld = dTime;
		double dTimeNew = m_nClockCount * SAMPLINGTIME / 1000.;
		double dDeltaTime = dTimeNew - dTimeOld;
		double dLocalTimeOld = dTimeOld - m_dInitPositionTime;
		double dLocalTimeNew = dTimeNew - m_dInitPositionTime;
		double dDifferenceSpeed = m_dVelocityEnd - m_dVelocityStart;
		double t2 = m_dAccelerationTime + m_dConstantVelocityTime;
		if (dLocalTimeNew > m_dTotalTime)
			return false;
		if (dLocalTimeNew < m_dAccelerationTime)
		{
			double dVelocityOld = m_dVelocityStart + MAX_ACCELERATION * dLocalTimeOld;
			double dVelocityNew = m_dVelocityStart + MAX_ACCELERATION * dLocalTimeNew;
			double dAverageVelocity = (dVelocityOld + dVelocityNew) / 2;
			m_du += dAverageVelocity * dDeltaTime / m_dDistance;
		}
		else if (dLocalTimeNew < t2)
		{
			if (dLocalTimeOld < m_dAccelerationTime)
			{
				double dVelocityOld = m_dVelocityStart + MAX_ACCELERATION * dLocalTimeOld;
				double dVelocityNew = m_dVelocityStart + MAX_ACCELERATION * m_dAccelerationTime;
				double dAverageVelocity = (dVelocityOld + dVelocityNew) / 2;
				m_du += dAverageVelocity * (m_dAccelerationTime - dLocalTimeOld) / m_dDistance;
				dDeltaTime = dLocalTimeNew - m_dAccelerationTime;
			}
			m_du += m_dVelocityMax * dDeltaTime / m_dDistance;
		}
		else
		{
			if (dLocalTimeOld < m_dAccelerationTime)
			{
				double dVelocityOld = m_dVelocityStart + MAX_ACCELERATION * dLocalTimeOld;
				double dVelocityNew = m_dVelocityStart + MAX_ACCELERATION * m_dAccelerationTime;
				double dAverageVelocity = (dVelocityOld + dVelocityNew) / 2;
				m_du += dAverageVelocity * (m_dAccelerationTime - dLocalTimeOld) / m_dDistance;
				dDeltaTime = dLocalTimeNew - m_dAccelerationTime;
				dLocalTimeOld = m_dAccelerationTime;
				if (dDeltaTime > m_dConstantVelocityTime)
				{
					m_du += m_dVelocityMax * m_dConstantVelocityTime / m_dDistance;
					dDeltaTime -= m_dConstantVelocityTime;
					dLocalTimeOld = t2;
				}
			}
			else if (dLocalTimeOld < t2)
			{
				m_du += m_dVelocityMax * (t2 - dLocalTimeOld) / m_dDistance;
				dDeltaTime = dLocalTimeNew - t2;
				dLocalTimeOld = t2;
			}
			double dVelocityOld = m_dVelocityMax - MAX_DECELERATION * (dLocalTimeOld - t2);
			double dVelocityNew = m_dVelocityMax - MAX_DECELERATION * (dLocalTimeNew - t2);
			double dAverageVelocity = (dVelocityOld + dVelocityNew) / 2;
			m_du += dAverageVelocity * dDeltaTime / m_dDistance;
		}
		u = m_du;
		dTime = dTimeNew;
		m_nClockCount++;
		return true;
	}
	void MinimumTimeControl()
	{
		m_du = 0;
		double dVelocityMax = DetermineMaximumVelocity();
		if (dVelocityMax > m_dMaxVelocity)
			m_dVelocityMax = m_dMaxVelocity;
		else
			m_dVelocityMax = dVelocityMax;
		m_dAccelerationTime = (m_dVelocityMax - m_dVelocityStart) / MAX_ACCELERATION;
		m_dDecelerationTime = (m_dVelocityMax - m_dVelocityEnd) / MAX_DECELERATION;
		m_dConstantVelocityTime = (m_dDistance - (m_dAccelerationTime * (m_dVelocityMax + m_dVelocityStart) / 2) -
			m_dDecelerationTime * (m_dVelocityMax + m_dVelocityEnd) / 2) / m_dVelocityMax;
		m_dTotalTime = m_dAccelerationTime + m_dConstantVelocityTime + m_dDecelerationTime;
		int a = 1;
	}
	double DetermineMaximumVelocity()
	{
		double dVelocityMax = m_dMaxVelocity;		   
		for (int i = 0; i < 100; i++)
		{
			double dAccelerationTime = (dVelocityMax - m_dVelocityStart) / MAX_ACCELERATION;
			double dDecelerationTime = (dVelocityMax - m_dVelocityEnd) / MAX_DECELERATION;
			double dVelocityMaxNew = (m_dDistance - (dAccelerationTime * m_dVelocityStart / 2) -
				(dDecelerationTime * m_dVelocityEnd / 2)) /
				(dAccelerationTime + dDecelerationTime) * 2;
			if (fabs(dVelocityMaxNew - dVelocityMax) < 1e-5)
				break;
			dVelocityMax = (dVelocityMaxNew + dVelocityMax) / 2;
		}
		return dVelocityMax;
	}
	
	void WriteFile()
	{
		if (!m_bIsFeasible)
		{
			printf("NOT FEASIBLE\n");
		}
		FILE* auf = fopen("D:/LinearAccDeceleration_Output.txt", "w");
		if (!m_bIsFeasible)
			fprintf(auf, "********NOT FEASIBLE*********\n");
		fprintf(auf, "Total Distance[cm],%f\n", m_dDistance);
		fprintf(auf, "Start Velocity[cm/sec],%f\n", m_dVelocityStart);
		fprintf(auf, "Max Velocity[cm/sec],%f\n", m_dVelocityMax);
		fprintf(auf, "End Velocity[cm/sec],%f\n", m_dVelocityEnd);		
		fprintf(auf, "Acceleration[cm/sec^2],%f\n", double(MAX_ACCELERATION));
		fprintf(auf, "Deceleration[cm/sec^2],%f\n", double(MAX_DECELERATION));
		fprintf(auf, "-----------------------\n");
		fprintf(auf, "Start RPM,%f\n", VelocityToRPM(m_dVelocityStart, m_dWheelRadius));
		fprintf(auf, "Max RPM,%f\n", VelocityToRPM(m_dVelocityMax, m_dWheelRadius));
		fprintf(auf, "End RPM,%f\n", VelocityToRPM(m_dVelocityEnd, m_dWheelRadius));
		fprintf(auf, "IncreaseInRPM[rpm/sec^2],%f\n", VelocityToRPM(MAX_ACCELERATION, m_dWheelRadius));
		fprintf(auf, "DecreaseInRPM[rpm/sec^2],%f\n", VelocityToRPM(MAX_DECELERATION, m_dWheelRadius));
		fprintf(auf, "-----------------------\n");
		fprintf(auf, "Acceleration Time[sec],%f\n", m_dAccelerationTime);
		fprintf(auf, "Constant Velocity Time[sec],%f\n", m_dConstantVelocityTime);
		fprintf(auf, "Deceleration Time[sec],%f\n", m_dDecelerationTime);
		fprintf(auf, "Total Time[sec],%f\n", m_dAccelerationTime + m_dConstantVelocityTime + m_dDecelerationTime);
		fclose(auf);
	}
	
};