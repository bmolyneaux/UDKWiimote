// udkwiimote.cpp : Defines the exported functions for the DLL application.

#include "stdafx.h"
#include <stdio.h>
#include <time.h>

//#include "quaternion.h"	// performing quaternion to Euler conversion in UDK
#include "vector3.h"

#include <process.h>	// for _beginthreadex()

// bool values don't map properly through UDK's DLLBind
// so we return an unsigned int-32 instead
#define INT_VAL_OF_BOOL(x) x?1:0;

// Calibration
#define NOISE_FILTER 1.5
#define CALIB_TIME 5.0
#define YAW_ZERO_ZONE 0.1	// Tolerance level when finding yaw calibration dot in sphere (%)
#define YAW_ZERO_ANGLE 1.0	// Tolerance angle when calibrating yaw

// Constants for computing Madgwick
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta

#define MAX(x,y) (x) > (y) ? (x) : (y)
#define MIN(x,y) (x) < (y) ? (x) : (y)

extern "C"
{
	// Corresponds to UDK's vector type
	struct FVector
	{
		float x,y,z;
	};

	struct Quat
	{
		float X,Y,Z,W;
	};

	// Wiimote instance
	wiimote remote;

	// Wiimote's orientation in a quaternion
	Quat mQuatMadgwick;// = quaternion(1,0,0,0);

	// Rotation rates
	double mYawRate;
	double mPitchRate;
	double mRollRate;

	// Wiimote's stationary rotation rates (non-zero after calibration)
	/*float mCalibrationPitchSpeed = 0;
	float mCalibrationYawSpeed = 0;
	float mCalibrationRollSpeed = 0;*/

	// Has initialization completed?
	bool mInitialized = false;

	// Has calibration completed?
	bool mCalibrated = false;

	HANDLE connectionThreadHandle;
	HANDLE calibrationThreadHandle;
	HANDLE listenThreadHandle;

	// Previous timestamp
	// Stackoverflow post said unit was 100 nanoseconds
	// Seems to actually be a unit of 1 microsecond
	unsigned long long mTime = 0;
	
	// Calibration
	double mCalibrationTimeout;	
	vector3f mMaxNoise;
	vector3f mMinNoise;
	vector3f mBias;
	vector3f mNoiseLevel;
	vector3f mNoiseThreshold;
	vector3f mPrevAngleRates;
	vector3f mAngleRates;
	std::vector<vector3f> mNoise;
	bool mOriInitialized = false;

	// New calibration
	// Use of mNoise vector seems to accumulate error
	// TODO: find out why, fix, and use mNoise instead
	int numCalibrationReadings;
	double pitchSum = 0;
	double yawSum = 0;
	double rollSum = 0;

void IMUupdateMadgwick(double w_x, double w_y, double w_z, double a_x, double a_y, double a_z, double dt)
{
	// Grab the quaternion values
	float SEq_1 = mQuatMadgwick.W;
	float SEq_2 = mQuatMadgwick.X;
	float SEq_3 = mQuatMadgwick.Y;
	float SEq_4 = mQuatMadgwick.Z;

	// Local system variables
	float norm;																// vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;	// quaternion derrivative from gyroscopes elements
	float f_1, f_2, f_3;													// objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;				// objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;				// estimated direction of the gyroscope error

	// Axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;

	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;

	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	J_11or24 = twoSEq_3;
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1;
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

	// Normalise the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;

	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

	// Compute then integrate the estimated quaternion
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;

	// Normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;

	// Store the new orientation
	mQuatMadgwick.W = SEq_1;
	mQuatMadgwick.X = SEq_2;
	mQuatMadgwick.Y = SEq_3;
	mQuatMadgwick.Z = SEq_4;
}

#pragma region Calibration
void InitCalibration(const wiimote_state new_state)
{
	mCalibrationTimeout = CALIB_TIME;
	mMaxNoise = mMinNoise = mBias = vector3f(new_state.MotionPlus.Speed.Yaw, 
		new_state.MotionPlus.Speed.Pitch, new_state.MotionPlus.Speed.Roll);
	mNoiseLevel = mNoiseThreshold = mPrevAngleRates = mAngleRates = vector3f(0,0,0);
	mNoise.clear();
	remote.SetLEDs(0x0f);
}

bool IsCalibrating()
{
	return mCalibrationTimeout > 0;
}

// calculate the bias and std of angulr speeds
// set mBias and mNoiseLevel
void calculateCalibration(void)
{
	int n = mNoise.size();
	vector3f sum = vector3f(0,0,0);
	for (int i=0; i<n; i++) {
		sum += mNoise.at(i);
	}
	
	mBias.x = pitchSum/(double)numCalibrationReadings;
	mBias.y = yawSum/(double)numCalibrationReadings;
	mBias.z = rollSum/(double)numCalibrationReadings;
	//mBias = sum/n;

	sum = vector3f(0,0,0);
	for (int i=0; i<n; i++){
		vector3f diff = mNoise.at(i) - mBias;
		sum += diff % diff;
	}
	sum = sum/n;
	mNoiseLevel = vector3f(sqrt(sum.x), sqrt(sum.y), sqrt(sum.z));
	mNoiseThreshold = mNoiseLevel*3;
}

// calculate the avg and std of the bias in yaw, pitch, and roll
void UpdateCalibration(double dt, const wiimote_state new_state)
{
	vector3f rates(new_state.MotionPlus.Speed.Yaw, new_state.MotionPlus.Speed.Pitch, new_state.MotionPlus.Speed.Roll);	
	mMaxNoise.x = MAX(mMaxNoise.x, rates.x);
	mMaxNoise.y = MAX(mMaxNoise.y, rates.y);
	mMaxNoise.z = MAX(mMaxNoise.z, rates.z);
	mMinNoise.x = MIN(mMinNoise.x, rates.x);
	mMinNoise.y = MIN(mMinNoise.y, rates.y);
	mMinNoise.z = MIN(mMinNoise.z, rates.z);	

	// If the wiimote is moving we need to recalibrate
	if (((vector3f)(rates - ((mMaxNoise + mMinNoise) * 0.5))).length() > NOISE_FILTER) {
		InitCalibration(new_state);
		return;
	}

	// Store the "reading" in mNoise
	mNoise.push_back(rates);

	// Flash leds while calibrating 
	int light = (int)((mCalibrationTimeout - (int)mCalibrationTimeout)*4);
	remote.SetLEDs(0x08>>light);

	mCalibrationTimeout -= dt;

	if (mCalibrationTimeout <= 0){
		remote.SetLEDs(0x01); // reset the LED
		calculateCalibration();		
	}

	pitchSum += new_state.MotionPlus.Speed.Pitch;
	yawSum += new_state.MotionPlus.Speed.Yaw;
	rollSum += new_state.MotionPlus.Speed.Roll;

	numCalibrationReadings++;
}
#pragma endregion Calibration

void HandleMotionPlusUpdate(wiimote	&remote, const wiimote_state &new_state) {
	FILETIME ft;
	GetSystemTimeAsFileTime(&ft);
	unsigned long long newTime = ft.dwHighDateTime; // current time in 1 microsecond (see mTime)
	newTime <<=32;
	newTime |= ft.dwLowDateTime;
	newTime /=10;
	newTime -= 11644473600000000ULL;
	double dt = (newTime - mTime) * 0.000001; // delta time in seconds

	if (!mOriInitialized)	
	{		
		InitCalibration(new_state);
		mTime = newTime;
		mOriInitialized = true;
		return;    
	}	
	else if(IsCalibrating())
	{
		UpdateCalibration(dt, new_state);
		mTime = newTime;
		return;
	}

	// Switch from RH Z-up Wiimote to LH Z-up UDK coordinate system
	mPitchRate = -(new_state.MotionPlus.Speed.Pitch - mBias.x)*3.14159265359/180.0;	// flip pitch
	mYawRate = (new_state.MotionPlus.Speed.Yaw - mBias.y)*3.14159265359/180.0;
	mRollRate = -(new_state.MotionPlus.Speed.Roll - mBias.z)*3.14159265359/180.0;		// flip roll

	double wx = mRollRate;	// angular speed about x-axis
	double wy = mPitchRate;	// angular speed about y-axis
	double wz = mYawRate;	// angular speed about z-axis
	double ax = -new_state.Acceleration.Y;	// switch and flip X and Y
	double ay = -new_state.Acceleration.X;	// switch and flip X and Y
	double az = new_state.Acceleration.Z;

	if (mTime != 0) {
		IMUupdateMadgwick(wx, wy, wz, ax, ay, az, dt);
	}

	mTime = newTime;
}

void on_state_change (wiimote			  &remote,
					  state_change_flags  changed,
					  const wiimote_state &new_state)
{
	// we use this callback to set report types etc. to respond to key events
	//  (like the wiimote connecting or extensions (dis)connecting).
	
	// NOTE: don't access the public state from the 'remote' object here, as it will
	//		  be out-of-date (it's only updated via RefreshState() calls, and these
	//		  are reserved for the main application so it can be sure the values
	//		  stay consistent between calls).  Instead query 'new_state' only.

	// the wiimote just connected
	if(changed & CONNECTED)
	{
		// ask the wiimote to report everything (using the 'non-continous updates'
		//  default mode - updates will be frequent anyway due to the acceleration/IR
		//  values changing):

		// note1: you don't need to set a report type for Balance Boards - the
		//		   library does it automatically.
		
		// note2: for wiimotes, the report mode that includes the extension data
		//		   unfortunately only reports the 'BASIC' IR info (ie. no dot sizes),
		//		   so let's choose the best mode based on the extension status:
		if(new_state.ExtensionType != wiimote::BALANCE_BOARD)
		{
			if(new_state.bExtension)
				remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR_EXT); // no IR dots
			else
				remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR);		//    IR dots
		}
	}

	// a MotionPlus was detected
	if(changed & MOTIONPLUS_DETECTED)
	{
		// enable it if there isn't a normal extension plugged into it
		// (MotionPlus devices don't report like normal extensions until
		//  enabled - and then, other extensions attached to it will no longer be
		//  reported (so disable the M+ when you want to access them again).
		if(remote.ExtensionType == wiimote_state::NONE) 
		{
			bool res = remote.EnableMotionPlus();
			_ASSERT(res);
		}
	}
	// an extension is connected to the MotionPlus
	else if(changed & MOTIONPLUS_EXTENSION_CONNECTED)
	{
		// We can't read it if the MotionPlus is currently enabled, so disable it:
		if(remote.MotionPlusEnabled())
			remote.DisableMotionPlus();
	}
	// an extension disconnected from the MotionPlus
	else if(changed & MOTIONPLUS_EXTENSION_DISCONNECTED)
	{
		// enable the MotionPlus data again:
		if(remote.MotionPlusConnected())
			remote.EnableMotionPlus();
	}
	// another extension was just connected:
	else if(changed & EXTENSION_CONNECTED)
	{
		// switch to a report mode that includes the extension data (we will
		//  loose the IR dot sizes)
		// note: there is no need to set report types for a Balance Board.
		if(!remote.IsBalanceBoard())
			remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR_EXT);
	}
	// extension was just disconnected:
	else if(changed & EXTENSION_DISCONNECTED)
	{
		// use a non-extension report mode (this gives us back the IR dot sizes)
		remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR);
	}
	else if(changed & MOTIONPLUS_SPEED_CHANGED)
	{
		HandleMotionPlusUpdate(remote, new_state);
	}
}

	//returns bool value
	__declspec(dllexport) bool wiiMoteInitialize()
	{
		remote.ChangedCallback		= on_state_change;
		remote.CallbackTriggerFlags = (state_change_flags)(CONNECTED |
			EXTENSION_CHANGED |
			MOTIONPLUS_CHANGED);
		
		mPrevAngleRates.x = mPrevAngleRates.y = mPrevAngleRates.z = 0;
		mAngleRates.x = mAngleRates.y = mAngleRates.z = 0;

		mQuatMadgwick.W = 1;
		mQuatMadgwick.X = mQuatMadgwick.Y = mQuatMadgwick.Z = 0;

		mInitialized = true;

		return INT_VAL_OF_BOOL(true);
	}

	unsigned __stdcall asynchronousConnect(void* param)
	{
		bool status = remote.Connect(wiimote::FIRST_AVAILABLE);
		printf( "%s\n", (status) ? "Connection established." : "Connection failed." );
		return 0;
	}

	//returns bool value
	__declspec(dllexport) unsigned int wiiMoteConnect()
	{
		if (!mInitialized) {
			printf( "Wiimote not initialized.\nFirst call wiiMoteInitialize()!" );
			return INT_VAL_OF_BOOL(false);
		}

		connectionThreadHandle = (HANDLE)_beginthreadex(NULL, 0, asynchronousConnect, NULL, 0, NULL);
		SetThreadPriority(connectionThreadHandle, WORKER_THREAD_PRIORITY);

		/*bool status = remote.Connect(wiimote::FIRST_AVAILABLE);
		printf( "Connecting: %s\n", (status) ? "true" : "false" );*/

		return INT_VAL_OF_BOOL(true);
	}

	//returns bool value
	__declspec(dllexport) unsigned int wiiMoteIsConnected()
	{
		return INT_VAL_OF_BOOL(remote.IsConnected());
	}

	//returns bool value
	// TODO: reimplement
	__declspec(dllexport) unsigned int wiiMoteCalibrate()
	{
		return INT_VAL_OF_BOOL(true);
	}

	//returns bool value
	__declspec(dllexport) unsigned int wiiMoteIsCalibrated()
	{
		return INT_VAL_OF_BOOL(IsCalibrating());
	}

	int round(double d)
	{
		return (int)floor(d + 0.5);
	}

	// TODO: remove?
	/*__declspec(dllexport) FVector* wiiMoteMotionPlusRotation()
	{
		static FVector eulerAngles;
		vector3f angles = mQuatMadgwick.euler_angles(true);
		eulerAngles.x = round(angles.x * 182.04 * 180.0 / 3.14159265359);
		eulerAngles.y = round(angles.z * 182.04 * 180.0 / 3.14159265359);
		eulerAngles.z = round(angles.y * 182.04 * 180.0 / 3.14159265359);

		return &eulerAngles;
	}*/

	__declspec(dllexport) Quat* wiiMoteMotionPlusQuaternion()
	{
		static Quat q;

		q.W = mQuatMadgwick.W;
		q.X = mQuatMadgwick.X;
		q.Y = mQuatMadgwick.Y;
		q.Z = mQuatMadgwick.Z;

		return &q;
	}

	__declspec(dllexport) int wiiMoteBatteryPercent()
	{
		int status = remote.BatteryPercent;
		return status;
	}
	
	#pragma region LEDs
	__declspec(dllexport) void wiiMoteSetLEDs(unsigned char binary_data)
	{
		remote.SetLEDs(binary_data);
	}

	__declspec(dllexport) bool wiiMoteLEDLit(int index)
	{
		if (index >= 0 && index <= 3)
			return remote.LED.Lit(index);

		return false;
	}
	#pragma endregion LEDs
	
	#pragma region Rumble
	__declspec(dllexport) void wiiMoteRumble(int val)
	{
		remote.SetRumble(val != 0);
	}

	__declspec(dllexport) void wiiMoteRumbleAsync(int milli)
	{
		if (milli < 0 )		//Insure positive int value
			milli *= -1;

		remote.RumbleForAsync(milli);
	}

	__declspec(dllexport) bool wiiMoteIsRumble()
	{
		return remote.bRumble;
	}
	#pragma endregion Rumble

	#pragma region Accelerometer
	//Raw wiimote analog accelerometer data
	__declspec(dllexport) float wiiMoteAccelX()
	{
		remote.RefreshState();
		return remote.Acceleration.X;
	}

	__declspec(dllexport) float wiiMoteAccelY()
	{
		remote.RefreshState();
		return remote.Acceleration.Y;
	}

	__declspec(dllexport) float wiiMoteAccelZ()
	{
		remote.RefreshState();
		return remote.Acceleration.Z;
	}
	#pragma endregion Accelerometer

	#pragma region Buttons
	//Wiimote button functions
	//['A', 'B', 'Plus', 'Home', 'Minus', 'One', 'Two', 'Up', 'Down', 'Left', 'Right'] -> Array used to generate functions with python
	__declspec(dllexport) bool wiiMoteA()
	{
		remote.RefreshState();
		bool status = remote.Button.A();
		return status;
	}

	__declspec(dllexport) bool wiiMoteB()
	{
		remote.RefreshState();
		bool status = remote.Button.B();
		return status;
	}
	
	__declspec(dllexport) bool wiiMotePlus()
	{
		remote.RefreshState();
		bool status = remote.Button.Plus();
		return status;
	}
	
	__declspec(dllexport) bool wiiMoteHome()
	{
		remote.RefreshState();
		bool status = remote.Button.Home();
		return status;
	}
	
	__declspec(dllexport) bool wiiMoteMinus()
	{
		remote.RefreshState();
		bool status = remote.Button.Minus();
		return status;
	}
	
	__declspec(dllexport) bool wiiMoteOne()
	{
		remote.RefreshState();
		bool status = remote.Button.One();
		return status;
	}
	
	__declspec(dllexport) bool wiiMoteTwo()
	{
		remote.RefreshState();
		bool status = remote.Button.Two();
		return status;
	}
	
	__declspec(dllexport) bool wiiMoteUp()
	{
		remote.RefreshState();
		bool status = remote.Button.Up();
		return status;
	}
	
	__declspec(dllexport) bool wiiMoteDown()
	{
		remote.RefreshState();
		bool status = remote.Button.Down();
		return status;
	}
	
	__declspec(dllexport) bool wiiMoteLeft()
	{
		remote.RefreshState();
		bool status = remote.Button.Left();
		return status;
	}
	
	__declspec(dllexport) bool wiiMoteRight()
	{
		remote.RefreshState();
		bool status = remote.Button.Right();
		return status;
	}
	#pragma endregion Buttons
}