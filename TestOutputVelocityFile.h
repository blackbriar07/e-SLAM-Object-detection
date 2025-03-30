#pragma once
#include "Geometry.h"
#include <string>

using namespace std;


class CTestOutputVelocity
{
public:
	vector<double> m_vLWheelVelocities;
	vector<double> m_vRWheelVelocities;
	vector<Pose> m_PIMRPose;
	vector<double> m_Time;
	double m_dStartTime;
	double m_dSamplingTime; // [unit: ms]
	double m_dWheelRadius;
	Pose m_StartPose;
public:
	CTestOutputVelocity()
	{
		m_StartPose.m_Point.x = 0;
		m_StartPose.m_Point.y = 0;
		m_StartPose.m_dOrientation = 0;
		m_dSamplingTime = 333;
		m_dWheelRadius = 13.5; // [unit: cm]
		ReadFile();
		//_getch();
		CalculateMotion();
	}
	void ReadFile()
	{
		FILE* file = fopen("D:/DifferentialDriveRobot/TestData/0403/on_ground.csv", "r");
		if (file == nullptr) {
			perror("Error opening file");
			return;
		}
		char buffer[256];
		vector<string> words;
		while (fgets(buffer, sizeof(buffer), file) != nullptr) {
			printf("%s", buffer);
			string Numbers;
			int NumOfWord = 0;
			for (int i = 0; buffer[i] != '\0'; ++i) 
			{
				
				if (buffer[i] == ',')
				{					
					if (!Numbers.empty())
						words.push_back(Numbers);
					Numbers = "";
					NumOfWord += 1;
					continue;
				}
				else if (buffer[i] == '\n')
				{
					if (!Numbers.empty())
						words.push_back(Numbers);
					Numbers = "";
					if (words.size() > 3)
					{
						m_vRWheelVelocities.push_back(RPMToVelocity(stod(words.back()),m_dWheelRadius));
						m_vLWheelVelocities.push_back(RPMToVelocity(stod(words[words.size() - 2]), m_dWheelRadius));
						//printf("Vl %s\n", words.back().c_str());
						//printf("VR %s\n", words[words.size() - 2].c_str());
					}
					words.clear();
					words.shrink_to_fit();
					break;
				}
				Numbers += buffer[i];
				//printf("Numbers %s\n", Numbers.c_str());
			}
		}
		fclose(file);
		return;
	}
	void CalculateMotion()
	{
		double x = m_StartPose.m_Point.x;
		double y = m_StartPose.m_Point.y;
		double VL, VR;
		double orientation = m_StartPose.m_dOrientation;
		int DataLength = int(m_vLWheelVelocities.size()); 
		double CurrentTime = m_dStartTime;
		m_PIMRPose.push_back(m_StartPose);
		m_Time.push_back(CurrentTime);
		for (int i = 0; i < DataLength; i++)
		{
			VL = m_vLWheelVelocities[i];
			VR = m_vRWheelVelocities[i];
			//printf("VL %f\n", VL);
			//printf("VR %f\n", VR);
			x += ((VL + VR) / 2) * cos(orientation) * (m_dSamplingTime * 0.001);
			y += ((VL + VR) / 2) * sin(orientation) * (m_dSamplingTime * 0.001);			
			orientation += ((VR - VL)/ IMRWIDTH) * (m_dSamplingTime * 0.001);
			//printf("orientation %f\n", orientation);
			//_getch();
			CurrentTime += (m_dSamplingTime * 0.001);
			Pose CurrentPose;
			CurrentPose.m_Point.x = x;
			CurrentPose.m_Point.y = y;
			CurrentPose.m_dOrientation = orientation;
			m_PIMRPose.push_back(CurrentPose);
			m_Time.push_back(CurrentTime);
		}

		FILE* auf = fopen("D:/Pose_Output.csv", "w");
		fprintf(auf, "Time[sec],Vel_L[cm/sec],Vel_R[cm/sec],XPos[cm/sec],YPos[cm/sec],Ori[Deg],\n");
		for (int i = 0; i < m_Time.size(); i++)
		{
			fprintf(auf, "%f,%f,%f,%f,%f,%f\n", m_Time[i], m_vLWheelVelocities[i], m_vRWheelVelocities[i],
				m_PIMRPose[i].m_Point.x, m_PIMRPose[i].m_Point.y, m_PIMRPose[i].m_dOrientation * r2d);
		}
		fclose(auf);
	}
};