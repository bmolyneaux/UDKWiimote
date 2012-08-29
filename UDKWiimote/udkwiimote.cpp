// udkwiimote.cpp : Defines the exported functions for the DLL application.

#include "stdafx.h"
#include <stdio.h>

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
}

extern "C"
{
	struct FVector
	{
		float x,y,z;
	};

	wiimote remote;

	FVector mPrevAngleRates;
	FVector mAngleRates;

	float mCalibrationPitch = 0;
	float mCalibrationYaw = 0;
	float mCalibrationRoll = 0;

	__declspec(dllexport) bool wiiMoteConnect()
	{
		remote.ChangedCallback		= on_state_change;
		remote.CallbackTriggerFlags = (state_change_flags)(CONNECTED |
			EXTENSION_CHANGED |
			MOTIONPLUS_CHANGED);

		bool status = remote.Connect(wiimote::FIRST_AVAILABLE);
		printf( "Connecting: %s\n", (status) ? "true" : "false" );

		if (status)
		{
			mPrevAngleRates.x = mPrevAngleRates.y = mPrevAngleRates.z = 0;
			mAngleRates.x = mAngleRates.y = mAngleRates.z = 0;
		}

		return status;
	}

	__declspec(dllexport) bool wiiMoteIsConnected()
	{
		return remote.IsConnected();
	}

	__declspec(dllexport) int wiiMoteBatteryPercent()
	{
		int status = remote.BatteryPercent;
		return status;
	}

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

	__declspec(dllexport) void wiiMoteRefresh()
	{
		remote.RefreshState();
	}

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

//Normalized acceleration (1g), used to calculate orientation
	__declspec(dllexport) float wiiMoteAccelOrientX()
	{
		remote.RefreshState();
		return remote.Acceleration.Orientation.X;
	}

	__declspec(dllexport) float wiiMoteAccelOrientY()
	{
		remote.RefreshState();
		return remote.Acceleration.Orientation.Y;
	}

	__declspec(dllexport) float wiiMoteAccelOrientZ()
	{
		remote.RefreshState();
		return remote.Acceleration.Orientation.Z;
	}

//Orientation based on normalized acceleration data
	__declspec(dllexport) float wiiMotePitch()
	{
		remote.RefreshState();
		return remote.Acceleration.Orientation.Pitch;
	}

	__declspec(dllexport) float wiiMoteRoll()
	{
		remote.RefreshState();
		return remote.Acceleration.Orientation.Roll;
	}

// Note: Yaw cannot be calculated based on accelerometer data

//Orientation is only calculated when the wiimote is relatively still
//returns if the orientation is current
	__declspec(dllexport) bool wiiMoteAngleIsCurrent()
	{
		remote.RefreshState();
		return remote.Acceleration.Orientation.UpdateAge < 15;
	}
	#pragma endregion Accelerometer

	#pragma region Motion+
//Motion+ Functions
//Calibrate Motion+
	__declspec(dllexport) void wiiMoteMotionPlusCalibrate()
	{
		printf("Calibrating.\n" );
		
		int i;
		float pitchSum = 0;
		float yawSum = 0;
		float rollSum = 0;

		Sleep(2000);

		for (i = 0; i < 10; i++) {
			Sleep(10);

			remote.RefreshState();

			pitchSum += remote.MotionPlus.Speed.Pitch;
			yawSum += remote.MotionPlus.Speed.Yaw;
			rollSum += remote.MotionPlus.Speed.Roll;
		}

		mCalibrationPitch = pitchSum / 10;
		mCalibrationYaw = yawSum / 10;
		mCalibrationRoll = rollSum / 10;
	}
	
//Raw Motion+ data
	__declspec(dllexport) float wiiMoteMotionPlusPitch()
	{
		printf("Pitch:%f\n",remote.MotionPlus.Raw.Pitch );
		return remote.MotionPlus.Raw.Pitch;
	}

	__declspec(dllexport) float wiiMoteMotionPlusYaw()
	{
		printf("Yaw:  %f\n",remote.MotionPlus.Raw.Yaw );
		return remote.MotionPlus.Raw.Yaw;
	}

	__declspec(dllexport) float wiiMoteMotionPlusRoll()
	{
		printf("Roll: %f\n", remote.MotionPlus.Raw.Roll );
		return remote.MotionPlus.Raw.Roll;
	}
	
//Awesome Motion+ data
	__declspec(dllexport) FVector* wiiMoteMotionPlusRotationDelta( float dt )
	{
		static FVector deltaAngles;
		FVector newAngleRates;
		newAngleRates.x = remote.MotionPlus.Speed.Pitch - mCalibrationPitch;
		newAngleRates.y = remote.MotionPlus.Speed.Yaw - mCalibrationYaw;
		newAngleRates.z = remote.MotionPlus.Speed.Roll - mCalibrationRoll;

		deltaAngles.x = dt*(mPrevAngleRates.x + newAngleRates.x + 4*mAngleRates.x) / 6;
		deltaAngles.y = dt*(mPrevAngleRates.y + newAngleRates.y + 4*mAngleRates.y) / 6;
		deltaAngles.z = dt*(mPrevAngleRates.z + newAngleRates.z + 4*mAngleRates.z) / 6;

		mPrevAngleRates = mAngleRates;
		mAngleRates = newAngleRates;

		return &deltaAngles;
	}


//Motion+ Velocity data (maybe?)
	__declspec(dllexport) float wiiMoteMotionPlusPitchSpeed()
	{
		return remote.MotionPlus.Speed.Pitch;
	}

	__declspec(dllexport) float wiiMoteMotionPlusYawSpeed()
	{
		return remote.MotionPlus.Speed.Yaw;
	}

	__declspec(dllexport) float wiiMoteMotionPlusRollSpeed()
	{
		return remote.MotionPlus.Speed.Roll;
	}
	#pragma endregion Motion+

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