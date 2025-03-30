#include "ITRI_APP.h"

#define sqr(x) ((x) * (x))
#define WHEEL_LEFT					0
#define WHEEL_RIGHT					1
#define MODE_MANUAL					0
#define MODE_AUTO					1
#define MODE_NUMBER_INDEX_ONE		1
#define MODE_NUMBER_INDEX_TWO		2
#define MODE_NUMBER_INDEX_THREE		3
#define CIRCLE_DIAMETER				84.82
#define MATH_PI 					3.14159265
#define ADVALUE_THROTTLE_RPM_SCALE	100
#define ADVALUE_THROTTLE_THRESHOLD	2600
#define ADVALUE_STEERING_THRESHOLD	100
#define RPM_RANGE					30
#define AXEL_LENGTH					46.0 // [cm]
#define MAXIMUM_RPM 				40
#define MINIMUM_RPM 				6
#define WHEEL_DIAMETER				27.0
#define SECOND_TO_MINUTE			60
#define RECORD_SIZE					400
#define SAMPLING_FREQUENCY          5
#define SEARCH_                     1
#define TEST_FLUCTUATING_RPM        1   
 
typedef struct
{
	float x;
	float y;
	float theta;
} MyPointPlane;

int bMotion = 0;
int dCurrentOscillation = 0;
MyPointPlane m_ptOutput = {0, 0, 0};
MyPointPlane m_ptDesired = {0, 0, 0};
// Goals
int TotalNumberofGoals = 34;
int CurrentGoalPoseIndex = -1;
int CurrentIMRPoseIndex = 0;

CITRICar ITRICar;
int car;
double dSpeedCmd;
float dRPM_Left = 0.0;
float dRPM_Right = 0.0;
/////////////////////////////////////////////////////////// Rohit
float fLeftWheelVelocity = 0;
float fRightWheelVelocity = 0;
float PreviouLVal = 0;
float PreviouRVal = 0;
float SamplingTime = 0;
bool TakeControl = false;
bool bPositionControl = false;
bool bOrientationControl = false;
bool GoalReached = false;
float dTime = 0.0;
float BlackoutTime = 0.0;
PIDController VCLeftWheel = {30, 0.5, 3.0, 50};
PIDController VCRightWheel = {30, 0.5, 3.0, 50};

int PrintCounter = 0;
#define MAX_POINTS 	250


/////////////////////////////////////////////////////////// Rohit

void NormalizeRPM()
{
	float CalculatedRPM_Left = dRPM_Left;
	float CalculatedRPM_Right = dRPM_Right;
	float Ratio = CalculatedRPM_Right / CalculatedRPM_Left;	
	if (CalculatedRPM_Left != 0)
		dRPM_Left = CalculatedRPM_Left / _fabs(CalculatedRPM_Left) * _fmax(MINIMUM_RPM, _fmin(_fabs(CalculatedRPM_Left), MAXIMUM_RPM));
	if (CalculatedRPM_Right != 0)
		dRPM_Right = CalculatedRPM_Right / _fabs(CalculatedRPM_Right) * _fmax(MINIMUM_RPM, _fmin(_fabs(CalculatedRPM_Right), MAXIMUM_RPM));
	if (_fmax(_fabs(CalculatedRPM_Right), _fabs(CalculatedRPM_Left)) > MAXIMUM_RPM)
	{
		if (_fabs(CalculatedRPM_Right) > _fabs(CalculatedRPM_Left))
		{
			dRPM_Right = CalculatedRPM_Right < 0 ? -MAXIMUM_RPM : MAXIMUM_RPM;
			dRPM_Left = dRPM_Right / Ratio;
		}
		if (_fabs(CalculatedRPM_Right) < _fabs(CalculatedRPM_Left))
		{
			dRPM_Left = CalculatedRPM_Left < 0 ? -MAXIMUM_RPM : MAXIMUM_RPM;
			dRPM_Right = dRPM_Left * Ratio;
		}
	}	
}

float VelocityToRPM(float pVelocity)
{
	return (pVelocity) * SECOND_TO_MINUTE / (WHEEL_DIAMETER * MATH_PI);
}

float RPMToVelocity(float pRPM)
{
	return (pRPM) * (WHEEL_DIAMETER * MATH_PI) / SECOND_TO_MINUTE;
}

void MBITPositionControl()
{
	NormalizeRPM();
}

void InitMotionParameters(bool Reverse)
{
}

void UpdateIMRPose()
{
	SamplingTime = (float)1/SAMPLING_FREQUENCY;
	fLeftWheelVelocity =  (((double)ITRICar.m_wheel[WHEEL_LEFT].m_drive.m_nStepCount[CURRENT_STATE] - 
	PreviouLVal) * SAMPLING_FREQUENCY * MATH_PI * (WHEEL_DIAMETER / 2) / 45);				
	fRightWheelVelocity = -1 * ((double)ITRICar.m_wheel[WHEEL_RIGHT].m_drive.m_nStepCount[CURRENT_STATE] - 
	PreviouRVal) * SAMPLING_FREQUENCY * MATH_PI * (WHEEL_DIAMETER / 2) / 45;			
	m_ptOutput.x += (fLeftWheelVelocity + fRightWheelVelocity) / 2 * _cos(m_ptOutput.theta) * SamplingTime;				
	m_ptOutput.y += (fLeftWheelVelocity + fRightWheelVelocity) / 2 * _sin(m_ptOutput.theta) * SamplingTime;				
	m_ptOutput.theta += (fRightWheelVelocity - fLeftWheelVelocity) / AXEL_LENGTH * SamplingTime ;	
	if (m_ptOutput.theta >= 2 * MATH_PI)
		m_ptOutput.theta = _fabs(m_ptOutput.theta - 2 * MATH_PI);
	else if (m_ptOutput.theta <= -2 * MATH_PI)
		m_ptOutput.theta = _fabs(m_ptOutput.theta + 2 * MATH_PI);
	PreviouLVal = ITRICar.m_wheel[WHEEL_LEFT].m_drive.m_nStepCount[CURRENT_STATE];
	PreviouRVal = ITRICar.m_wheel[WHEEL_RIGHT].m_drive.m_nStepCount[CURRENT_STATE];
	dTime += SamplingTime;
}


////////////////////////////////////////////////
void ResetWheel(HubMotorWheel* pWheel)
{
	pWheel->m_dTotalMileage = 0;
	pWheel->m_dMeterPerStep = 1. / (3 * POLE_NUMBER) * CIRCLE_DIAMETER;
	pWheel->m_motor.nPoleNumber = POLE_NUMBER;
    pWheel->m_drive.m_dRPMLowpass = 0;
    pWheel->m_drive.m_dRPMInstantaneous = 0;
}

void WheelControl(HubMotorWheel* pWheel, double dSpeedLinear, double dSpeedRotate)
{
	if (car == 0)
		dSpeedCmd = dRPM_Left;
	else
		dSpeedCmd = -dRPM_Right;
	dSpeedCmd = _fmin(MAXIMUM_RPM, _fmax(-MAXIMUM_RPM, dSpeedCmd));
	GetLowpassDirectionalSpeed(&pWheel->m_drive);
	MotorSetDirectionAndDuty(&pWheel->m_drive, 
		PIDFeedback(&pWheel->m_drive.m_ctrl, dSpeedCmd, pWheel->m_drive.m_dRPMLowpass));
}

/////////////////////////////
void InitPIDControllers()
{
	for (int i = 0; i < HUB_MOTORS; i++)
		InitPIDController(&ITRICar.m_wheel[i].m_drive.m_ctrl);
}

void ResetMotorOutput()
{
	for (int i = 0; i < 2; i++)
		ResetOutput(&ITRICar.m_wheel[i].m_drive);
}

void SetRotation()
{
	if (SEARCH_)
	{
			BlackoutTime += 0.1;
			if  (BlackoutTime > 0.75)
				return;
			dRPM_Left = dRPM_Left;
			dRPM_Right = dRPM_Right;
	}
}

void MotorSpeedControl(uint64_t nFastLoopCount)
{
	for (int i = 1; i >= 0; i--)
	{
		car = i;
		WheelControl(&ITRICar.m_wheel[i],
			ITRICar.m_dSpeedThrottle * (1 - _fabs(ITRICar.m_dRotateRatio)), ITRICar.m_dSpeedThrottle * ITRICar.m_dRotateRatio);
	}
}

void ReadDIOControl()
{
	ITRICar.m_nWorkMode[CURRENT_STATE] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
	ITRICar.m_nWorkMode[CURRENT_STATE] = MODE_AUTO;
	if (ITRICar.m_nWorkMode[PREVIOUS_STATE] != ITRICar.m_nWorkMode[CURRENT_STATE])
		InitPIDControllers();
}

void ControlPinInit(GPortPinArray ppa)
{
	ITRICar.m_ppaControl = ppa;
	NuEnableInputPPA(ppa); 
	ReadDIOControl();
}

void ReadHandleThreeMode(void)
{
	for (int i = 0; i < ITRICar.m_ppaControl.m_nPins; i++)
		ITRICar.nReadModeNumberPin[i] = NuGetBit(ITRICar.m_ppaControl.m_gpp[i]);
}

////////////////////////////////////////
int AD_VALUE_SIZE	= ADC_CHANNELS * 10;

void CarPWMDisablePinInit(GPortPin gpp)
{
	PortPin pp = PP(gpp);
	ITRICar.m_pwmDisable = gpp;
	NuEnableOutputPP(gpp);
	GPIO_SetBits(pp.m_typedef, pp.m_GPIO);
}


void ITRICarInit(bool bUseSteering, DWORD nPWMMode, int nPWMHz)
{
	CreateSineTable();
	mgrPWM.m_nHzSignal = nPWMHz;
	mgrPWM.m_nModePWM = nPWMMode;
	mgrPWM.m_nCallFunction = PWM_CALL_HALF_BLDC;
	ITRICar.m_bUseSteering = bUseSteering;	
	ITRICar.m_dSpeedThrottle = 0;
	ITRICar.m_dRotateRatio = 0;
	ITRICar.m_nWorkMode[CURRENT_STATE] = ITRICar.m_nWorkMode[PREVIOUS_STATE] = MODE_MANUAL;
	for (int i = 0; i < ADC_CHANNELS; i++)
	{
		ITRICar.m_dLPFValue[i] = 0;
		ResetWheel(&ITRICar.m_wheel[i]);
	}
	ITRICar.dADValueToSpeedCmd = (double) RPM_RANGE / ADVALUE_THROTTLE_RPM_SCALE;
	ITRICar.m_wheel[WHEEL_LEFT].m_drive.m_bInitStart = false;
	ITRICar.m_wheel[WHEEL_RIGHT].m_drive.m_bInitStart = false;
	CarPWMDisablePinInit(B2);
	CarPWMDisablePinInit(C0);
	GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_0, Bit_RESET);
	BLDCMotorInit(&ITRICar.m_wheel[WHEEL_LEFT].m_drive, 1,
		true, nPWMMode,
		(GPortPinArray) {3, E8, E10, E12},  
		B3, 
		(PWMModule) {ST_TIMER_1, nPWMHz, (GPortPinArray) {7, A8, E11, A10, B12, A7, B14, B15}}, none,
		VCLeftWheel);		
	BLDCMotorInit(&ITRICar.m_wheel[WHEEL_RIGHT].m_drive, 2,
		true, nPWMMode,			
		(GPortPinArray) {3, E7, E9, E15},
		B4, 
		(PWMModule) {ST_TIMER_8, nPWMHz, (GPortPinArray) {7, C6, C7, C8, A6, A5, B0, B1}}, none,
		VCRightWheel);
	ITRICar.m_nPIDFastLoopSkip = FASTLOOP_HZ / ITRICar.m_wheel[WHEEL_LEFT].m_drive.m_ctrl.m_nHzPID;
	InitMotionParameters(bMotion);	
}

void ReadAIOControl()
{
			
}

bool MotorBrake(bool bStop)
{
	if (bStop)
		ResetMotorOutput();
	return bStop;
}

///////////////////////////////////////////////
void ITRICarMain(void)
{
	ITRICarInit(false /*use steering*/, MODE_BISYNC, 20000 /*Hz*/); 
	ResetMotorOutput();
	JsonWriteIntoJsonPair("ITRICar", "ON"); 
}

#define RECORD_ARRAY_SIZE 80000
uint16_t RecordArrayCount = 1;
int PrintfCount = 0;

void ITRICarReportWrite(bool bReadWrite)
{
	char szText[MAX_JSON_VALUE];		
	NuTimeReport(szText);
	fprintf(USART_FILE,"{\"SetBeepValues\"=\"%s,%f,%f,%d\"}",
		szText,
		VelocityToRPM(fLeftWheelVelocity),
		VelocityToRPM(fRightWheelVelocity),
		bMotion);
}

void ITRICarReport(void)
{
	if (!m_bSystemReportInited)
	{
		fprintf(USART_FILE, "{\"OpenCSVFile\" = \"D:/test123.csv\"}"); 
		fprintf(USART_FILE, "{\"SetKeys\" = \"Time,WheelLeft$103,WheelRight$104,Motion,,GDE100,101,102,103,104\"}"); 
		fprintf(USART_FILE, "{\"GDETransfer\" = \"200,3,6,100,7,101,8,102\"}");
		m_bSystemReportInited = true;
	}
	ITRICarReportWrite(true);
	if (RecordArrayCount == (uint16_t) RECORD_ARRAY_SIZE || GoalReached)
		ITRICarReportWrite(true);
	if (PrintfCount == (uint16_t) RECORD_ARRAY_SIZE)
		ITRICarReportWrite(false);	
}

void MotorCalculateMileage(HubMotorWheel* pWheel)
{
	pWheel->m_dTotalMileage = pWheel->m_dMeterPerStep * pWheel->m_drive.m_nStepCount[CURRENT_STATE];
}

   void ITRICarNormalLoop(void)
{ 
	ReadDIOControl();
	if (!MotorBrake(ITRICar.m_nWorkMode[CURRENT_STATE] == MODE_MANUAL))
		ReadAIOControl();
	for (int i = 0; i < 2; i++)
		MotorCalculateMileage(&ITRICar.m_wheel[i]);
	ITRICar.m_nWorkMode[PREVIOUS_STATE] = ITRICar.m_nWorkMode[CURRENT_STATE];
}

void MotionBuiltInTest()
{
	PrintCounter++;		
	UpdateIMRPose();
}

void MotionBuiltInTestNextPosition()
{
	if (RecordArrayCount > (uint16_t) RECORD_ARRAY_SIZE)
	{
		dRPM_Left = 0;
		dRPM_Right = 0;
	}
	RecordArrayCount++;
}

void ITRICarFastLoop(uint64_t nFastLoopCount)
{
	for (int i = 1; i >= 0; i--)
	{
		GetHallSensorState(&ITRICar.m_wheel[i].m_drive);
		if (ITRICar.m_wheel[i].m_drive.m_nStepChangeCountDown >= 0)
		{
			if (ITRICar.m_wheel[i].m_drive.m_nStepChangeCountDown == 0)
				DoChangeStepControl(&ITRICar.m_wheel[i].m_drive);
			ITRICar.m_wheel[i].m_drive.m_nStepChangeCountDown--;
		}
	}
	if (ITRICar.m_nWorkMode[CURRENT_STATE] == MODE_AUTO &&
		nFastLoopCount % ITRICar.m_nPIDFastLoopSkip == 0)
		MotorSpeedControl(nFastLoopCount); 
	if (nFastLoopCount % 100 == 0 &&
		RecordArrayCount < (uint16_t) RECORD_ARRAY_SIZE)
		MotionBuiltInTestNextPosition();
	if (nFastLoopCount % 2000 == 0 &&
		RecordArrayCount < (uint16_t) RECORD_ARRAY_SIZE) 
		MotionBuiltInTest();
}


void TestFluctuatingRPM()
{
	if (!TEST_FLUCTUATING_RPM)
		return;
	if (dTime > 10)
	{
		dRPM_Left = 12.0;
		dRPM_Right = 0.0;
	}
	if (dTime > 20)
	{
		dRPM_Left = 0.0;
		dRPM_Right = 12.0;
	}
}

void JITRIApp(DWORD param[])
{
	char szText[MAX_JSON_VALUE];
	JsonPartition(param, szText);
	dRPM_Left = (float)Bound(40000, -40000, Getint32(szText, ",")) / 1000;
	dRPM_Right = (float)Bound(40000, -40000, Getint32(szText, ",")) / 1000;
	bMotion = (int)Bound(100, -100, Getint32(szText, ","));
	Getint32(szText, "]");
	if (bMotion == 0)
	{
		if (TakeControl)
			SetRotation();
		if (BlackoutTime > 0.75)
		{
			dRPM_Left = 0.0;
			dRPM_Right = 0.0;
		}
	}
	else if (bMotion == 1)
	{ 
		BlackoutTime = 0.0;
		TakeControl = true;
	}
}

