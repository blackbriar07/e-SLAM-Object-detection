#pragma once
#include <math.h>
#include <vector>

typedef struct
{
	double m_dKP;
	double m_dKI;
	double m_dKD;
	int m_nHzPID;
	double m_dErrorIntegration;
	double m_dError;
} PIDController;

void InitPIDController(PIDController* pPID)
{
	pPID->m_dErrorIntegration = 0;
	pPID->m_dError = 0;
}

void SetPIDParameter(PIDController* pPID, PIDController pid)
{
	*pPID = pid;
	InitPIDController(pPID);
}

double PIDFeedback(PIDController* pid, double dError)
{
	double dOutput = pid->m_dKP * dError + pid->m_dKD * (dError - pid->m_dError) + pid->m_dKI * pid->m_dErrorIntegration;
	pid->m_dErrorIntegration += dError;
	pid->m_dError = dError;
	return dOutput;
}
