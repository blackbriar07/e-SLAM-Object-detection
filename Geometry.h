#pragma once
#include <math.h>
#include <vector>
#include <ctime>
#include <random>

using namespace std;

#define d2r								0.0174533
#define r2d								57.2958
#define PI                              3.141592653589793238462643
#define PI2                             2*3.141592653589793238462643
#define IMRWIDTH								46 // [unit: cm]

#define MAX_VELOCITY					30					// [unit: cm/s]
#define MIN_VELOCITY					7				// [unit: cm/s]

double SamplingTime = 333; // [unit: ms]


struct Point2D {   
	double x;           
	double y;
	double z;
};

struct Pose {
	Point2D m_Point;
	double m_dOrientation;
	double m_dVelocity;
};

double VelocityToRPM(double dVelocity, double dWheelRadius)
{
	return (dVelocity * 60) / (2 * PI * dWheelRadius);
}

double Disturbance(double number, double lb, double ub)
{
	//srand(std::time(0));
	//double noise = double(rand() % (ub - lb + 1)) + lb;
	//int noise = lb + (rand() % (ub-lb+1));
	double noise = lb + static_cast<double>(std::rand()) / RAND_MAX * (ub - lb);
	//printf("noise %f\n", noise);
	return number - noise;
}

double RPMToVelocity(double RPM, double dWheelRadius)
{
	return (RPM * 2 * PI * dWheelRadius) / 60;
}

class CGeometry
{
public:
	CGeometry() {};
public:
	Point2D ProducePointAtGivenAngle(Point2D A, double dAngle, double dDistance)
	{
		Point2D B;
		B.x = A.x + dDistance * cos(dAngle);
		B.y = A.y + dDistance * sin(dAngle);
		B.z = 0.0;
		//printf("(Bx, By):(%f,%f)\n", B.x, B.y);
		return B;
	}

	double FindAngleBetweenLineGivenPoints(Point2D A, Point2D B, Point2D C, Point2D D) // Line AB , Line CD
	{
		double slope_line1 = (B.y - A.y) / (B.x - A.x);
		double slope_line2 = (D.y - C.y) / (D.x - C.x);
		double angle_line1 = atan(slope_line1);
		double angle_line2 = atan(slope_line2);
		double angle_between_lines = 0;
		angle_between_lines = (angle_line2 - angle_line1);
		angle_between_lines = angle_between_lines * r2d;
		return angle_between_lines ;
	}
	double FindAngleBetweenVectorsGivenPoints(Point2D A, Point2D B, Point2D C, Point2D D) // Line AB , Line CD
	{
		Point2D Vec1;
		Point2D Vec2;
		Vec1.x = B.x - A.x;
		Vec1.y = B.y - A.y;
		Vec2.x = D.x - C.x;
		Vec2.y = D.y - C.y;
		double Mod_Vec1 = sqrt(Vec1.x * Vec1.x + Vec1.y * Vec1.y);
		double Mod_Vec2 = sqrt(Vec2.x * Vec2.x + Vec2.y * Vec2.y);
		double Dot_Product = Vec1.x * Vec2.x + Vec1.y * Vec2.y;
		// Calculate the angle between the lines
		double angle_between_lines = acos(Dot_Product/(Mod_Vec1* Mod_Vec2));
		return angle_between_lines;
	}
	double FindAngleBetweenVectorsGivenPoints2(Point2D A, Point2D B, Point2D C, Point2D D, bool& RightWheelMore) // Line AB , Line CD
	{
		Point2D Vec1;
		Point2D Vec2;
		Vec1.x = B.x - A.x;
		Vec1.y = B.y - A.y;
		Vec2.x = D.x - C.x;
		Vec2.y = D.y - C.y;
		double Mod_Vec1 = sqrt(Vec1.x * Vec1.x + Vec1.y * Vec1.y);
		double Mod_Vec2 = sqrt(Vec2.x * Vec2.x + Vec2.y * Vec2.y);
		double Dot_Product = Vec1.x * Vec2.x + Vec1.y * Vec2.y;
		// Calculate the angle between the lines
		double angle_between_lines = acos(Dot_Product / (Mod_Vec1 * Mod_Vec2));
		//angle_between_lines = angle_between_lines *r2d;
		double determinant = Vec1.x * Vec2.y - Vec1.y * Vec2.x;
		if (determinant > 0)
		{
			if (angle_between_lines < PI)
			{
				RightWheelMore = true;
				return angle_between_lines;
			}
			RightWheelMore = false;
			return 2*PI - angle_between_lines;
		}			
		else if (determinant < 0)
		{
			if (angle_between_lines < PI)
			{
				RightWheelMore = false;
				return angle_between_lines;
			}
			RightWheelMore = true;
			return 2 * PI - angle_between_lines;
		}
		if (angle_between_lines == 0)
		{
			RightWheelMore = true;
			return 0;
		}
		if (fabs(angle_between_lines - PI) <= 1e-3)
		{
			RightWheelMore = true;
			return angle_between_lines;
		}	
		return angle_between_lines;
	}
	double DetermineOrientationNeeded(Point2D InitialPoint, Point2D FinalPoint, double dCurrentOrientation = 0)
	{
		Point2D B = ProducePointAtGivenAngle(InitialPoint, dCurrentOrientation, 10);
		bool RightWheelMore = false;
		double DegreeAngle = FindAngleBetweenVectorsGivenPoints2(InitialPoint, B, InitialPoint, FinalPoint, RightWheelMore);
		if (!RightWheelMore)
		{
			DegreeAngle = -1 * DegreeAngle;
		}
		return DegreeAngle;
	}
	double DetermineOrientationNeededGivenVector(Point2D A, Point2D B)
	{
		double DotProductValue = A.x * B.x + A.y * B.y;
		double Mod_A = sqrt(A.x * A.x + A.y * A.y);
		double Mod_B = sqrt(B.x * B.x + B.y * B.y);
		double angle = acos(DotProductValue / (Mod_A * Mod_B));
		return angle;
	}
	bool PointInsideTriangle(const Point2D Trianglepoints[3], Point2D Point)
	{
		double T_x1 = Trianglepoints[0].x;
		double T_y1 = Trianglepoints[0].y;
		double T_x2 = Trianglepoints[1].x;
		double T_y2 = Trianglepoints[1].y;
		double T_x3 = Trianglepoints[2].x;
		double T_y3 = Trianglepoints[2].y;



		double denominator = (T_y2 - T_y3) * (T_x1 - T_x3) + (T_x3 - T_x2) * (T_y1 - T_y3);
		double alpha = ((T_y2 - T_y3) * (Point.x - T_x3) + (T_x3 - T_x2) * (Point.y - T_y3)) / denominator;
		double beta = ((T_y3 - T_y1) * (Point.x - T_x3) + (T_x1 - T_x3) * (Point.y - T_y3)) / denominator;
		double gamma = 1.0 - alpha - beta;

		if (0 <= alpha && alpha <= 1 && 0 <= beta && beta <= 1 and 0 <= gamma && gamma <= 1)
			return true;
		return false;
	}
	double CalculateDistanceBetweenPoints(Point2D pointA, Point2D pointB)
	{
		double Xdist = (pointA.x - pointB.x);
		double Ydist = (pointA.y - pointB.y);
		double distance = Xdist * Xdist + Ydist * Ydist;
		return sqrt(distance);
	}
	bool GetIntersectionPointBetweenLines(Point2D Line12, Point2D Line23, Point2D& ResultPoint) // Line Equation : Ax + By + C = 0
	{
		double A1 = Line12.x;
		double B1 = Line12.y;
		double C1 = Line12.z;
		double A2 = Line23.x;
		double B2 = Line23.y;
		double C2 = Line23.z;
		// Calculate the determinant
		double determinant = A1 * B2 - A2 * B1;
		// Check if the lines are parallel(determinant is zero)
		if (determinant == 0)
			return false;
		// Calculate the intersection point coordinates
		ResultPoint.x = (C2 * B1 - C1 * B2) / determinant;
		ResultPoint.y = (A2 * C1 - A1 * C2) / determinant;
		return true;
	}
	void ReturnEquationofLine(Point2D Point1, Point2D Point2, Point2D& ResultLineEquation)
	{
		double x1 = Point1.x;
		double y1 = Point1.y;
		double x2 = Point2.x;
		double y2 = Point2.y;
		if (fabs(x1 - x2) < 1e-4)
		{
			// The line is vertical(parallel to y - axis)
			//printf("here A\n");
			ResultLineEquation.x = 1;
			ResultLineEquation.y = 0;
			ResultLineEquation.z = -1*x1;
		}
		else
		{
			// Calculate the slope(m) of the line
			double slope = (y2 - y1) / (x2 - x1);
			// Use the slope - intercept form(y = mx + b) to find the y - intercept(b)
			double b = y1 - slope * x1;
			// Rearrange the equation to the standard form Ax + By + C = 0
			ResultLineEquation.x = -1*slope;
			ResultLineEquation.y = 1;
			ResultLineEquation.z = -b;
		}
		return;
	}
	
	void GetPerpendicularLineEquationfromPointonLine(Point2D Point, Point2D LineSegmentA, Point2D& ResultLineEquation)
	{
		double A = LineSegmentA.x;
		double B = LineSegmentA.y;
		double C = LineSegmentA.z;
		double x = Point.x;
		double y = Point.y;
		if (fabs(A) < 1e-2)
		{
			// The original line is parallel to the x - axis(horizontal)
			// The perpendicular line will be parallel to the y - axis(vertical)
			ResultLineEquation.x = 1;
			ResultLineEquation.y = 0;
			ResultLineEquation.z = -1 * x;
		}
		else if (fabs(B) < 1e-2)
		{
			// The original line is parallel to the y - axis(vertical)
			// The perpendicular line will be parallel to the x - axis(horizontal)
			ResultLineEquation.x = 0;
			ResultLineEquation.y = 1;
			ResultLineEquation.z = -1 * y;
		}
		else
		{
			// Calculate the slope of the given line
			double slope_original = -1* (A / B);
			// Calculate the slope of the perpendicular line
			double slope_perpendicular = B / A;
			
			// Use the point - slope form to find the y - intercept of the perpendicular line
			double y_intercept_perpendicular = y - slope_perpendicular * x;
				// Set coefficients for the perpendicular line
			ResultLineEquation.x = -1 * slope_perpendicular;
			ResultLineEquation.y = 1;
			ResultLineEquation.z = -1 * y_intercept_perpendicular;
		}
	}
	bool GetPerpendicularLineEquationGivenTwoPoints(Point2D FirstPoint, Point2D SecondPoint, Point2D& ResultLine)
	{
		double x1 = FirstPoint.x;
		double y1 = FirstPoint.y;
		double x2 = SecondPoint.x;
		double y2 = SecondPoint.y;
		if (y2 == y1 && x1 != x2)
		{
			ResultLine.x = 1; // A
			ResultLine.y = 0; // B
			ResultLine.z = -x1; // C
			return true;
		}
		else
		{
			double m = (x1 - x2) / (y2 - y1);
			// y - intercept of the perpendicular line
			double b = y1 - m * x1;
			ResultLine.x = -m; // A
			ResultLine.y = 1; // B
			ResultLine.z = -b; // C
			return true;
		}
		return false;
	}
	double CalculateVectorIMRTOQRCode(Pose QRCodeWorldFramePose, Pose IMRWorldFramePose, Pose& IMRVectorToQR)
	{
		double XVec = QRCodeWorldFramePose.m_Point.x - IMRWorldFramePose.m_Point.x;
		double YVec = QRCodeWorldFramePose.m_Point.y - IMRWorldFramePose.m_Point.y;
		double distance = sqrt(XVec * XVec + YVec * YVec);
		Point2D B = ProducePointAtGivenAngle(IMRWorldFramePose.m_Point, IMRWorldFramePose.m_dOrientation, distance + 500);
		Point2D LineEquationIMRToPointB;
		ReturnEquationofLine(IMRWorldFramePose.m_Point, B, LineEquationIMRToPointB);
		Point2D perpendicularLineEquation;
		GetPerpendicularLineEquationfromPointonLine(QRCodeWorldFramePose.m_Point, LineEquationIMRToPointB, perpendicularLineEquation);
		Point2D IntersectionPoint;
		GetIntersectionPointBetweenLines(perpendicularLineEquation, LineEquationIMRToPointB, IntersectionPoint);
		double AngleNeeded = 0;
		double angle = DetermineOrientationNeeded(IMRWorldFramePose.m_Point, QRCodeWorldFramePose.m_Point, IMRWorldFramePose.m_dOrientation);
		IMRVectorToQR.m_Point.y = CalculateDistanceBetweenPoints(IMRWorldFramePose.m_Point, IntersectionPoint);
		if (angle <= 0)
			IMRVectorToQR.m_Point.x = -1 * CalculateDistanceBetweenPoints(QRCodeWorldFramePose.m_Point, IntersectionPoint);
		else
			IMRVectorToQR.m_Point.x = CalculateDistanceBetweenPoints(QRCodeWorldFramePose.m_Point, IntersectionPoint);
		AngleNeeded = atan(IMRVectorToQR.m_Point.x / IMRVectorToQR.m_Point.y);
		return AngleNeeded;
	}


	// Least Square Solution
	void transposeMatrix4(const double A[][2], double result[][4], int n, int m) {
		for (int i = 0; i < n; ++i) {
			for (int j = 0; j < m; ++j) {
				result[j][i] = A[i][j];
			}
		}
	}
	void transposeMatrix3(const double A[][2], double result[][3], int n, int m) {
		for (int i = 0; i < n; ++i) {
			for (int j = 0; j < m; ++j) {
				result[j][i] = A[i][j];
			}
		}
	}
	void multiplyMatrices42(const double A[][4], const double B[][2], double result[][2], int n, int m, int k) {
		for (int i = 0; i < n; ++i) {
			for (int j = 0; j < m; ++j) {
				result[i][j] = 0.0;
				for (int l = 0; l < k; ++l) {
					result[i][j] += A[i][l] * B[l][j];
				}
			}
		}
	}
	void multiplyMatrices32(const double A[][3], const double B[][2], double result[][2], int n, int m, int k) 
	{
		for (int i = 0; i < n; ++i) {
			for (int j = 0; j < m; ++j) {
				result[i][j] = 0.0;
				for (int l = 0; l < k; ++l) {
					result[i][j] += A[i][l] * B[l][j];
				}
			}
		}
	}
	void multiplyMatrixVector41(const double A[][4], const double x[], double result[], int n, int m) {
		for (int i = 0; i < n; ++i) {
			result[i] = 0.0;
			for (int j = 0; j < m; ++j) {
				result[i] += A[i][j] * x[j];
			}
		}
	}
	void multiplyMatrixVector31(const double A[][3], const double x[], double result[], int n, int m) 
	{
		for (int i = 0; i < n; ++i) {
			result[i] = 0.0;
			for (int j = 0; j < m; ++j) {
				result[i] += A[i][j] * x[j];
			}
		}
	}
	void multiplyMatrixVector21(const double A[][2], const double x[], double result[], int n, int m) {
		for (int i = 0; i < n; ++i) {
			result[i] = 0.0;
			for (int j = 0; j < m; ++j) {
				result[i] += A[i][j] * x[j];
			}
		}
	}
	bool inverse2x2(const double A[][2], double invA[][2]) {
		double a = A[0][0];
		double b = A[0][1];
		double c = A[1][0];
		double d = A[1][1];

		double det = a * d - b * c;
		if (det == 0) {
			return false;
		}

		double invDet = 1.0 / det;
		invA[0][0] = d * invDet;
		invA[0][1] = -b * invDet;
		invA[1][0] = -c * invDet;
		invA[1][1] = a * invDet;
		return true;
	}
	void printVector(const double vec[], int size) {
		for (int i = 0; i < size; ++i) {
			cout << vec[i] << " ";
		}
		cout << endl;	}
	void PrintPoint(Point2D point, const char* c)
	{
		printf("%s: (%f,%f)\n", c, point.x, point.y);
	}
	void PrintPose(Pose pose, const char* c)
	{
		printf("%s: (%f,%f) ori: %f\n", c, pose.m_Point.x, pose.m_Point.y,pose.m_dOrientation);
	}

};
