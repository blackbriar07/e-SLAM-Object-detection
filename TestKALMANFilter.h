#pragma once
#include "Geometry.h"
#include "KalmanFilter.h"


class CTestKalmanFilter
{
public:
	CTestKalmanFilter()
	{
		m_dSimulationLength = 0;
		CKalmanFilterIMRQRobj.m_nStates = 5;
		CKalmanFilterIMRQRobj.m_nInputs = 0;
		CKalmanFilterIMRQRobj.m_nOutputs = 3;

		CKalmanFilterObj.m_nStates = 5;
		CKalmanFilterObj.m_nInputs = 0;
		CKalmanFilterObj.m_nOutputs = 3;
	}
private:
	vector<Pose> m_vQRCodeRealTrajectory;
	vector<Pose> m_vIMRRealTrajectory;
	vector<Pose> m_vIMRToQRCodeVector;
	vector<double> m_vIMRRealLinearVelocity;
	vector<double> m_vIMRRealAngularVelocity;
	vector<double> m_vTime;
	vector<int> m_dInRange;
	int m_dSimulationLength;
	CKalmanFilterIMRQR CKalmanFilterIMRQRobj;
	CKalmanFilter CKalmanFilterObj;
private:
	int ReadFileForQRCodeTrajectoryRealData(const char* filename)
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
			Pose PosevalueIMRToQRVector;
			for (int j = 0; j < All_Data[i].size(); j++)
			{
				if (Count == 1)
					m_vTime.push_back(stod(All_Data[i][j]));				
				else if (Count == 8)
					PosevalueIMRToQRVector.m_Point.x = stod(All_Data[i][j]);
				else if (Count == 9)
					PosevalueIMRToQRVector.m_Point.y = stod(All_Data[i][j]);
				else if (Count == 13)
					PosevalueIMRToQRVector.m_dOrientation = stod(All_Data[i][j]);
				Count++;
			}
			m_vIMRToQRCodeVector.push_back(PosevalueIMRToQRVector);
		}
		m_dSimulationLength = int(All_Data.size());
		// Close the file
		fclose(file);
		for (int i = 0; i < m_dSimulationLength; i++)
		{
			printf("IMRQRx%f IMRQRy%f IMRQROri%f \n",
				m_vIMRToQRCodeVector[i].m_Point.x, m_vIMRToQRCodeVector[i].m_Point.y, m_vIMRToQRCodeVector[i].m_dOrientation);
		}
		return 0;
	}
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
			Pose PosevalueRealQR;
			Pose PosevalueRealIMR;
			Pose PosevalueIMRToQRVector;
			double LW;
			int Inrange;
			double RW;
			for (int j = 0; j < All_Data[i].size(); j++)
			{
				if (Count == 0)
					m_vTime.push_back(stod(All_Data[i][j]));
				else if (Count == 3)
					PosevalueIMRToQRVector.m_dOrientation = stod(All_Data[i][j]);
				else if (Count == 4)
					PosevalueIMRToQRVector.m_Point.x = stod(All_Data[i][j]);
				else if (Count == 5)
					PosevalueIMRToQRVector.m_Point.y = stod(All_Data[i][j]);
				else if (Count == 6)
					Inrange = stoi(All_Data[i][j]);
				else if (Count == 9)
					PosevalueRealIMR.m_Point.x = stod(All_Data[i][j]);
				else if (Count == 10)
					PosevalueRealIMR.m_Point.y = stod(All_Data[i][j]);
				else if (Count == 11)
					PosevalueRealIMR.m_dOrientation = stod(All_Data[i][j]);
				else if (Count == 12)
					PosevalueRealQR.m_Point.x = stod(All_Data[i][j]);
				else if (Count == 13)
					PosevalueRealQR.m_Point.y = stod(All_Data[i][j]);
				else if (Count == 14)
					PosevalueRealQR.m_dOrientation = stod(All_Data[i][j]);
				else if (Count == 7)
					LW = stod(All_Data[i][j]);
				else if (Count == 8)
					RW = stod(All_Data[i][j]);								
				Count++;
			}
			m_vIMRRealLinearVelocity.push_back((RW + LW) / 2);
			m_vIMRRealAngularVelocity.push_back((RW - LW) / IMRWIDTH);
			m_vQRCodeRealTrajectory.push_back(PosevalueRealQR);
			m_vIMRRealTrajectory.push_back(PosevalueRealIMR);
			m_vIMRToQRCodeVector.push_back(PosevalueIMRToQRVector);
			m_dInRange.push_back(Inrange);
		}
		m_dSimulationLength = int(All_Data.size());
		// Close the file
		fclose(file);
		return 0;
	}
public:
	void Initialize()
	{
		//const char* filename = "testKF/TestEight.csv";
		const char* filename = "testKF/TestEight.csv";
		//const char* filename = "testKF/TestStraight.csv";
		//const char* filename = "testKF/CornerAndPoseValue_1.csv"; // real data

		ReadFileForQRCodeTrajectory(filename);
		//ReadFileForQRCodeTrajectoryRealData(filename);
		/*for (int i = 0; i < m_dSimulationLength; i++)
		{
			printf("Time%f Range%d IMRQRx%f IMRQRy%f IMRQROri%f IMRx%f IMRy%f IMRori%f LV%f AV%f QRx%f QRy%f QROri%f\n", m_vTime[i],
				m_dInRange[i],
				m_vIMRToQRCodeVector[i].m_Point.x, m_vIMRToQRCodeVector[i].m_Point.y, m_vIMRToQRCodeVector[i].m_dOrientation,
				m_vIMRRealTrajectory[i].m_Point.x, m_vIMRRealTrajectory[i].m_Point.y, m_vIMRRealTrajectory[i].m_dOrientation,
				m_vIMRRealLinearVelocity[i], m_vIMRRealAngularVelocity[i],
				m_vQRCodeRealTrajectory[i].m_Point.x, m_vQRCodeRealTrajectory[i].m_Point.y, m_vQRCodeRealTrajectory[i].m_dOrientation);
		}*/
		//TestBeginKalmanFilterIMRQR();
		TestBeginKalmanFilter();
	}
	void TestBeginKalmanFilterIMRQR()
	{
		double IMRx = 0.0;
		double IMRy = 0.0;
		double IMRori = 0.0;
		double IMR_LV = 0.0;
		double IMR_AV = 0.0;
		CKalmanFilterIMRQRobj.Initialize(m_vIMRToQRCodeVector[0].m_Point.x, m_vIMRToQRCodeVector[0].m_Point.y,
			m_vIMRToQRCodeVector[0].m_dOrientation, 0.0, 0.0, 0.333);
		FILE* auf = fopen("D:/TestKalmnaFilter_Output.csv", "w");
		fprintf(auf, "Time, R_x, R_y, R_Ori, KF_x, KF_y, KF_Ori, KF_q, KF_alpha\n");
		for (int i = 1; i < m_dSimulationLength; i++)
		{
			IMRx += m_vIMRRealLinearVelocity[i] * cos(IMRori) * (SamplingTime * 0.001);
			IMRy += m_vIMRRealLinearVelocity[i] * sin(IMRori) * (SamplingTime * 0.001);
			IMRori += m_vIMRRealAngularVelocity[i] * (SamplingTime * 0.001);

			CKalmanFilterIMRQRobj.CalculateKF(IMRx, IMRy, IMRori, m_vIMRRealLinearVelocity[i], m_vIMRRealAngularVelocity[i],
				m_vIMRToQRCodeVector[i].m_Point.x, m_vIMRToQRCodeVector[i].m_Point.y, m_vIMRToQRCodeVector[i].m_dOrientation);
			//_getch();
			fprintf(auf, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", m_vTime[i], m_vIMRToQRCodeVector[i].m_Point.x, m_vIMRToQRCodeVector[i].m_Point.y, m_vIMRToQRCodeVector[i].m_dOrientation,
				CKalmanFilterIMRQRobj.m_mX_Posteriori(0, 0), CKalmanFilterIMRQRobj.m_mX_Posteriori(1, 0), CKalmanFilterIMRQRobj.m_mX_Posteriori(2, 0),
				CKalmanFilterIMRQRobj.m_mX_Posteriori(3, 0), CKalmanFilterIMRQRobj.m_mX_Posteriori(4, 0));
		}
		fclose(auf);
	}
	void TestBeginKalmanFilter()
	{
		double IMRx = 0.0;
		double IMRy = 0.0;
		double IMRori = 0.0;
		double IMR_LV = 0.0;
		double IMR_AV = 0.0;
		CKalmanFilterObj.Initialize(m_vIMRToQRCodeVector[0].m_Point.x, m_vIMRToQRCodeVector[0].m_Point.y,
			m_vIMRToQRCodeVector[0].m_dOrientation, 0.0, 0.0, 0.333);
		FILE* auf = fopen("D:/TestKalmnaFilter_Output2.csv", "w");
		fprintf(auf, "R_x, R_y, R_Ori, KF_x, KF_y, KF_Ori, KF_q, KF_alpha\n");
		bool InitializeKalmanFIlter = false;
		for (int i = 1; i < m_dSimulationLength; i++)
		{
			/*if (i == 1 && !InitializeKalmanFIlter)
			{
				CKalmanFilterObj.Initialize(m_vIMRToQRCodeVector[i].m_Point.x, m_vIMRToQRCodeVector[i].m_Point.y,
					m_vIMRToQRCodeVector[i].m_dOrientation, 0.0, 0.0, 0.333);
					InitializeKalmanFIlter = true;
			}*/
			if (m_dInRange[i] == 1 && !InitializeKalmanFIlter)
			{
				CKalmanFilterObj.Initialize(m_vIMRToQRCodeVector[i].m_Point.x, m_vIMRToQRCodeVector[i].m_Point.y,
					m_vIMRToQRCodeVector[i].m_dOrientation, 0.0, 0.0, 0.333);
				InitializeKalmanFIlter = true;
			}

			CKalmanFilterObj.CalculateKF(m_vIMRToQRCodeVector[i].m_Point.x, m_vIMRToQRCodeVector[i].m_Point.y,
					m_vIMRToQRCodeVector[i].m_dOrientation);
			fprintf(auf, "%f, %f, %f, %f, %f, %f, %f, %f\n",m_vIMRToQRCodeVector[i].m_Point.x, m_vIMRToQRCodeVector[i].m_Point.y, m_vIMRToQRCodeVector[i].m_dOrientation,
				CKalmanFilterObj.m_mX_Posteriori(0, 0), CKalmanFilterObj.m_mX_Posteriori(1, 0), CKalmanFilterObj.m_mX_Posteriori(2, 0),
				CKalmanFilterObj.m_mX_Posteriori(3, 0), CKalmanFilterObj.m_mX_Posteriori(4, 0));
		}
		fclose(auf);
	}
};