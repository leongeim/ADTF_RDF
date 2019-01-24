/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: geimle $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/

#include "stdafx.h"
#include "cStateMachine.h"
#include "aadc_myclassification_structs.h"
#include "aadc_roadSign_enums.h"
#include "aadc_enums.h"
using namespace Unia;
using namespace roadsignIDs;

#define MP_PROP_CAMERA_OFFSET_LAT "Camera Offset::Lateral"
#define MP_PROP_CAMERA_OFFSET_LON "Camera Offset::Longitudinal"
#define SPEEDUP_FACTOR "SpeedupFactor"
#define OVERTAKING_TRESHOLD "Overtaking_Treshold"

// road sign distance and pose estimation
#define MP_LIMIT_ALPHA    70.0 // [degrees]
#define MP_LIMIT_YAW      15.0 // [degrees]
#define MP_LIMIT_YAW_INIT  8.0 // [degrees]
#define MP_LIMIT_DISTANCE  0.8 // [m]

/// Create filter shell
ADTF_FILTER_PLUGIN("RDF State Machine", OID_ADTF_FILTER_DEF, cStateMachine)

cStateMachine::cStateMachine(const tChar* __info) :cFilter(__info)
{
	SetPropertyStr("Configuration", "");
	SetPropertyBool("Configuration" NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr("Configuration" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
	SetPropertyStr("Configuration" NSSUBPROP_DESCRIPTION, "Configuration file for the roadsign coordinates");

	SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LAT, 0.05);
	SetPropertyBool(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(MP_PROP_CAMERA_OFFSET_LAT NSSUBPROP_DESCRIPTION, "Camera offset in lateral direction");

	SetPropertyFloat(MP_PROP_CAMERA_OFFSET_LON, 0.0);
	SetPropertyBool(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(MP_PROP_CAMERA_OFFSET_LON NSSUBPROP_DESCRIPTION, "Camera offset in longitudinal direction");

	SetPropertyFloat(SPEEDUP_FACTOR, 1.0);
	SetPropertyBool(SPEEDUP_FACTOR NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyStr(SPEEDUP_FACTOR NSSUBPROP_DESCRIPTION, "Speedupfactor");
}

cStateMachine::~cStateMachine()
{
}


tResult cStateMachine::PropertyChanged(const tChar* strName)
{
	RETURN_IF_FAILED(cFilter::PropertyChanged(strName));

	//associate the properties to the member
	if (cString::IsEqual(strName, SPEEDUP_FACTOR))
	{
		m_maxSpeedUpFactor = GetPropertyFloat(SPEEDUP_FACTOR);
		if (m_actualSpeedUpFactor != 1.0)
		{
			m_actualSpeedUpFactor = m_maxSpeedUpFactor;
			m_actualSpeed_changed = true;
			m_runState_changed = true;
		}
	}

	RETURN_NOERROR;
}

tResult cStateMachine::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

		// input jury struct
		tChar const * strDesc1 = pDescManager->GetMediaDescription("tJuryStruct");
		RETURN_IF_POINTER_NULL(strDesc1);
		cObjectPtr<IMediaType> pType1 = new cMediaType(0, 0, 0, "tJuryStruct", strDesc1, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", pType1, this));
		RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));
		RETURN_IF_FAILED(pType1->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescJuryStruct));

		// output driver struct
		tChar const * strDescDriverStruct = pDescManager->GetMediaDescription("tDriverStruct");
		RETURN_IF_POINTER_NULL(strDescDriverStruct);
		cObjectPtr<IMediaType> pTypeDriverStruct = new cMediaType(0, 0, 0, "tDriverStruct", strDescDriverStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(m_DriverStructOutputPin.Create("Driver_Struct", pTypeDriverStruct, this));
		RETURN_IF_FAILED(RegisterPin(&m_DriverStructOutputPin));
		RETURN_IF_FAILED(pTypeDriverStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDriverStruct));

		//input Emergency stop
		tChar const * strDescEmergencyStop = pDescManager->GetMediaDescription("tJuryEmergencyStop");
		RETURN_IF_POINTER_NULL(strDescEmergencyStop);
		cObjectPtr<IMediaType> pTypeEmergencyStatus = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescEmergencyStop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(m_EmergencyStopInputPin.Create("EmergencyStop", pTypeEmergencyStatus, this));
		RETURN_IF_FAILED(RegisterPin(&m_EmergencyStopInputPin));
		RETURN_IF_FAILED(pTypeEmergencyStatus->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStop));

		//input EmergencyBreak status
		RETURN_IF_FAILED(m_EmergencyBreakStatusInputPin.Create("EmergencyBreakStatus", pTypeEmergencyStatus, this));
		RETURN_IF_FAILED(RegisterPin(&m_EmergencyBreakStatusInputPin));
		RETURN_IF_FAILED(pTypeEmergencyStatus->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStop));

		//output EmergencyBreak set
		tChar const * strDescEmergencyBreakSet = pDescManager->GetMediaDescription("tEmergencyBreakSet");
		RETURN_IF_POINTER_NULL(strDescEmergencyBreakSet);
		cObjectPtr<IMediaType> pTypeEmergencyBreakSet = new cMediaType(0, 0, 0, "tEmergencyBreakSet", strDescEmergencyBreakSet, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(m_oOutputEmergencyBreakSet.Create("EmergencyBreakSet", pTypeEmergencyBreakSet, this));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputEmergencyBreakSet));
		RETURN_IF_FAILED(pTypeEmergencyBreakSet->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyBreakSet));


		//create and register pins for speed in
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
		RETURN_IF_FAILED(m_oInputSpeedController.Create("Speed_in", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

		//create and register pins for speed out
		RETURN_IF_FAILED(m_oOutputSpeedController.Create("Speed_out", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oOutputSpeedController));

		//Wheel Tick Input
		tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
		RETURN_IF_POINTER_NULL(strDescWheelData);
		cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelLeftData));
		RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelRightData));
		RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeftStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));
		RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRightStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));

		// input ultrasonic
		tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
		RETURN_IF_POINTER_NULL(strDescUsStruct);
		cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));
		RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));
	}
	else if (eStage == StageNormal)
	{
		m_szIdEmergencyStopSet = tFalse;
		m_szIdEmergencyBreakSet = tFalse;
		m_szIdInputSpeedSet = tFalse;
		m_szIdOutputSpeedSet = tFalse;
		m_szIdsUsStructSet = tFalse;

		// default / initial values

		//states
		m_primaryState = primaryState_run;
		m_runState = runState_stop;
		m_primaryState_changed = false;
		m_runState_changed = false;

		// speed control
		m_actualSpeedState = 0;
		m_actualSpeedLaneDetection = NO_LD_SPEED;
		m_actualSpeed_changed = false;
		m_actualSpeedUpFactor = 1.0;
		m_maxSpeedUpFactor = 1.0;

		// emergency break
		m_emergencyBreakStatus = false;
		m_emergencyBreakStatus_changed = false;
		m_Emergency_Stop_Jury = false;
		m_EmergencyBreakSince = INT32_MIN;

		m_bIDsJuryStructSet = false;

		// tick stamps
		m_actualWheelTicks = 0;

		m_maxSpeedUpFactor = GetPropertyFloat(SPEEDUP_FACTOR);
	}
	else if (eStage == StageGraphReady)
	{
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.

		// simulate ready signal from jury
		//ChangeRunState(runState_ready);

		// simulate start signal from jury
		//ChangeRunState(runState_running);

		m_actualSpeedUpFactor = 1.0;
    m_bIDsJuryStructSet = false;
		TransmitEmergencyBreakSet();    // set initial E.B. distances
	}
	RETURN_NOERROR;
}

tResult cStateMachine::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	if (eStage == StageGraphReady)
	{
	}
	else if (eStage == StageNormal)
	{
	}
	else if (eStage == StageFirst)
	{
	}

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cStateMachine::Stop(__exception)
{
	// call the base class implementatistrDescSignalValueon
	return cFilter::Stop(__exception_ptr);
}

tResult cStateMachine::OnPinEvent(IPin* pSource,
	tInt nEventCode,
	tInt nParam1,
	tInt nParam2,
	IMediaSample* pMediaSample)
{
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (pSource == &m_JuryStructInputPin && m_pDescJuryStruct != NULL)
		{
			ProcessJuryInput(pMediaSample);
		}
		if (pSource == &m_EmergencyStopInputPin && m_pDescriptionEmergencyStop != NULL)
		{
			ProcessEmergencyStop(pMediaSample);
		}
		else if (pSource == &m_EmergencyBreakStatusInputPin && m_pDescriptionEmergencyStop != NULL)
		{
			ProcessEmergencyBreakStatus(pMediaSample);
		}
		else if (pSource == &m_oInputSpeedController && m_pDescriptionSignalValue != NULL)
		{
			ProcessSpeedController(pMediaSample);
		}
		else if (pSource == &m_oInputWheelLeft && m_pDescriptionWheelLeftData != NULL)
		{
			ProcessWheelSampleLeft(pMediaSample);
		}
		else if (pSource == &m_oInputWheelRight && m_pDescriptionWheelRightData != NULL)
		{
			ProcessWheelSampleRight(pMediaSample);
		}
		else if (pSource == &m_oInputUsStruct && m_pDescriptionSignalValue != NULL)
		{
			ProcessUsValues(pMediaSample);
		}
	}
	RETURN_NOERROR;
}

tResult cStateMachine::ProcessJuryInput(IMediaSample* pMediaSample)
{
	tInt8 i8ActionID = -2;
	tInt16 i16entry = -1;

	{
			// focus for sample read lock
			__adtf_sample_read_lock_mediadescription(m_pDescJuryStruct,pMediaSample,pCoder);
			// get the IDs for the items in the media sample
			if(!m_bIDsJuryStructSet)
			{
					pCoder->GetID("i8ActionID", m_szIDJuryStructI8ActionID);
					pCoder->GetID("i16ManeuverEntry", m_szIDJuryStructI16ManeuverEntry);
					m_bIDsJuryStructSet = tTrue;
			}

			pCoder->Get(m_szIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
			pCoder->Get(m_szIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);
	}

	if (i8ActionID == action_GETREADY && m_runState != runState_ready)
		{
			//LOG_INFO(cString::Format("State Machine: Received Request Ready with maneuver ID %d",i16entry));
			ChangeRunState(runState_ready);
			SendState(stateCar_READY);
		}
		else if (i8ActionID == action_START && m_runState != runState_running)
		{
			//LOG_INFO(cString::Format("State Machine: Received Run with maneuver ID %d",i16entry));
			ChangeRunState(runState_running);
			SendState(stateCar_RUNNING);

		}
		else if (i8ActionID == action_STOP && m_runState != runState_stop)
		{
			//LOG_INFO(cString::Format("State Machine: Received Stop with maneuver ID %d",i16entry));
			ChangeRunState(runState_stop);
			ChangePrimaryState(primaryState_run);
			SendState(stateCar_STARTUP);
		}

	RETURN_NOERROR;

}

tResult cStateMachine::SendState(stateCar stateID, tInt16 i16ManeuverEntry)
{
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionDriverStruct->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();

	tInt8 value = tInt8(stateID);

	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
	{
		// focus for sample write lock
		__adtf_sample_write_lock_mediadescription(m_pDescriptionDriverStruct, pMediaSample, pCoder);
		// get the IDs for the items in the media sample
		if (!m_bIDsDriverStructSet)
		{
			pCoder->GetID("i8StateID", m_szIDDriverStructI8StateID);
			pCoder->GetID("i16ManeuverEntry", m_szIDDriverStructI16ManeuverEntry);
			m_bIDsDriverStructSet = tTrue;
		}
		pCoder->Set(m_szIDDriverStructI8StateID, (tVoid*)&value);
		pCoder->Set(m_szIDDriverStructI16ManeuverEntry, (tVoid*)&i16ManeuverEntry);
	}

	pMediaSample->SetTime(_clock->GetStreamTime());
	m_DriverStructOutputPin.Transmit(pMediaSample);

	RETURN_NOERROR;
}


tResult cStateMachine::ProcessEmergencyStop(IMediaSample* pMediaSample)
{
	static tBufferID szIdEmergencyStopValue = 0;
	static bool szIdEmergencyStopSet = false;
	{
		// focus for sample read lock
		__adtf_sample_read_lock_mediadescription(m_pDescriptionEmergencyStop, pMediaSample, pCoder);

		if (!szIdEmergencyStopSet)
		{
			pCoder->GetID("bEmergencyStop", szIdEmergencyStopValue);
			szIdEmergencyStopSet = tTrue;
		}

		pCoder->Get(szIdEmergencyStopValue, (tVoid*)&m_Emergency_Stop_Jury);

	}
	if (m_Emergency_Stop_Jury)
		ChangeRunState(runState_stop);

	RETURN_NOERROR;
}

tResult cStateMachine::ProcessEmergencyBreakStatus(IMediaSample* pMediaSample)
{
	bool ebStatus = false;
	{
		// focus for sample read lock
		__adtf_sample_read_lock_mediadescription(m_pDescriptionEmergencyStop, pMediaSample, pCoder);
		// get the IDs for the items in the media sample
		if (!m_szIdEmergencyStopSet)
		{
			pCoder->GetID("bEmergencyStop", m_szIdEmergencyStopValue);
			m_szIdEmergencyStopSet = tTrue;
		}

		pCoder->Get(m_szIdEmergencyStopValue, (tVoid*)&ebStatus);
	}

	if (ebStatus != m_emergencyBreakStatus)
	{
		m_emergencyBreakStatus_changed = true;
		m_emergencyBreakStatus = ebStatus;
	}

	RETURN_NOERROR;
}

tResult cStateMachine::ProcessSpeedController(IMediaSample* pMediaSample)
{
	tFloat32 speed;
	tUInt32 timestamp;

	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

		//Set all Ids
		if (!m_szIdInputSpeedSet)
		{
			pCoderInput->GetID("f32Value", m_szIdInputspeedControllerValue);
			pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdInputspeedControllerTimeStamp);
			m_szIdInputSpeedSet = tTrue;
		}

		pCoderInput->Get(m_szIdInputspeedControllerValue, (tVoid*)&speed);
		pCoderInput->Get(m_szIdInputspeedControllerTimeStamp, (tVoid*)&timestamp);
	}

	m_actualSpeedLaneDetection = speed;
	m_actualSpeed_changed = true;

	RETURN_NOERROR;
}

tResult cStateMachine::ProcessUsValues(IMediaSample* pMediaSample)
{
	//use mutex
	__synchronized_obj(m_critSecMinimumUsValue);

	//read lock
	__adtf_sample_read_lock_mediadescription(m_pDescriptionUsStruct, pMediaSample, pCoderInput);

	//Set all Ids
	if (!m_szIdsUsStructSet)
	{
		tBufferID idValue, idTimestamp;
		m_szIdUsStructValues.clear();
		m_szIdUsStructTimeStamps.clear();

		pCoderInput->GetID("tFrontLeft.f32Value", idValue);
		pCoderInput->GetID("tFrontLeft.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		pCoderInput->GetID("tFrontCenterLeft.f32Value", idValue);
		pCoderInput->GetID("tFrontCenterLeft.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		pCoderInput->GetID("tFrontCenter.f32Value", idValue);
		pCoderInput->GetID("tFrontCenter.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		pCoderInput->GetID("tFrontCenterRight.f32Value", idValue);
		pCoderInput->GetID("tFrontCenterRight.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		pCoderInput->GetID("tFrontRight.f32Value", idValue);
		pCoderInput->GetID("tFrontRight.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		pCoderInput->GetID("tSideLeft.f32Value", idValue);
		pCoderInput->GetID("tSideLeft.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		pCoderInput->GetID("tSideRight.f32Value", idValue);
		pCoderInput->GetID("tSideRight.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		pCoderInput->GetID("tRearLeft.f32Value", idValue);
		pCoderInput->GetID("tRearLeft.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		pCoderInput->GetID("tRearCenter.f32Value", idValue);
		pCoderInput->GetID("tRearCenter.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		pCoderInput->GetID("tRearRight.f32Value", idValue);
		pCoderInput->GetID("tRearRight.ui32ArduinoTimestamp", idTimestamp);
		m_szIdUsStructValues.push_back(idValue);
		m_szIdUsStructTimeStamps.push_back(idTimestamp);

		m_szIdsUsStructSet = tTrue;
	}

	//iterate through all values
	tFloat32 buf_UsSignal;
	for (int i = 0; i < (int)m_szIdUsStructValues.size(); ++i)
	{
		pCoderInput->Get(m_szIdUsStructValues[i], (tVoid*)&buf_UsSignal);
		//save values
		m_actualDistances[i] = buf_UsSignal;
	}

	RETURN_NOERROR;
}

tFloat32 cStateMachine::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
	return mod(alpha - center + static_cast<tFloat32>(M_PI), 2.0*static_cast<tFloat32>(M_PI)) + center - static_cast<tFloat32>(M_PI);
}

tFloat32 cStateMachine::mod(tFloat32 x, tFloat32 y)
{
	tFloat32 r;
	tFloat32 b_x;
	if (y == floor(y))
		return x - floor(x / y) * y;
	else
	{
		r = x / y;
		if (r < 0.0f)
			b_x = ceil(r - 0.5f);
		else
			b_x = floor(r + 0.5f);

		if (fabs(r - b_x) <= 2.2204460492503131E-16f * fabs(r))
			return 0.0f;
		else
			return (r - floor(r)) * y;
	}
}

/*! workaround for heading update due to pose estimation
	issues with current Aruco version
*/
#define MP_ARUCO_WORKAROUND

tResult cStateMachine::TransmitEmergencyBreakSet()
{
	// default: no sensor is set
	tInt16 newDistances[10] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	switch (m_runState)
	{
		case runState_stop:
		case runState_ready:
		case runState_running:
		{
			// set only the front sensors
			if (m_actualSpeedUpFactor > 1.0)
			{
				newDistances[1] = 15;
				newDistances[2] = 40;
				newDistances[3] = 15;
			}
			else
			{
				newDistances[1] = 10;
				newDistances[2] = 30;
				newDistances[3] = 10;
			}
			break;
		}

		case runState_parking:
		{
			// set all sensors
			newDistances[1] = 5;
			newDistances[2] = 10;
			newDistances[3] = 5;
			newDistances[7] = 5;
			newDistances[8] = 5;
			newDistances[9] = 5;
			break;
		}
	}

	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionEmergencyBreakSet->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();

	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
	{
		// focus for sample write lock
		__adtf_sample_write_lock_mediadescription(m_pDescriptionEmergencyBreakSet, pMediaSample, pCoder);
		// get the IDs for the items in the media sample
		if (!m_szIdEmergencyBreakSet)
		{
			pCoder->GetID("i16EmergencyBreakFrontLeft", m_szIdEmergencyBreakSetFrontLeftValue);
			pCoder->GetID("i16EmergencyBreakFrontCenterLeft", m_szIdEmergencyBreakSetFrontHalfLeftValue);
			pCoder->GetID("i16EmergencyBreakFrontMiddle", m_szIdEmergencyBreakSetFrontMiddleValue);
			pCoder->GetID("i16EmergencyBreakFrontCenterRight", m_szIdEmergencyBreakSetFrontHalfRightValue);
			pCoder->GetID("i16EmergencyBreakFrontRight", m_szIdEmergencyBreakSetFrontRightValue);
			pCoder->GetID("i16EmergencyBreakSideLeft", m_szIdEmergencyBreakSetSideLeftValue);
			pCoder->GetID("i16EmergencyBreakSideRight", m_szIdEmergencyBreakSetSideRightValue);
			pCoder->GetID("i16EmergencyBreakBackLeft", m_szIdEmergencyBreakSetBackLeftValue);
			pCoder->GetID("i16EmergencyBreakBackMiddle", m_szIdEmergencyBreakSetBackMiddleValue);
			pCoder->GetID("i16EmergencyBreakBackRight", m_szIdEmergencyBreakSetBackRightValue);
			m_szIdEmergencyBreakSet = tTrue;
		}

		pCoder->Set(m_szIdEmergencyBreakSetFrontLeftValue, (tVoid*)&newDistances[0]);
		pCoder->Set(m_szIdEmergencyBreakSetFrontHalfLeftValue, (tVoid*)&newDistances[1]);
		pCoder->Set(m_szIdEmergencyBreakSetFrontMiddleValue, (tVoid*)&newDistances[2]);
		pCoder->Set(m_szIdEmergencyBreakSetFrontHalfRightValue, (tVoid*)&newDistances[3]);
		pCoder->Set(m_szIdEmergencyBreakSetFrontRightValue, (tVoid*)&newDistances[4]);
		pCoder->Set(m_szIdEmergencyBreakSetSideLeftValue, (tVoid*)&newDistances[5]);
		pCoder->Set(m_szIdEmergencyBreakSetSideRightValue, (tVoid*)&newDistances[6]);
		pCoder->Set(m_szIdEmergencyBreakSetBackLeftValue, (tVoid*)&newDistances[7]);
		pCoder->Set(m_szIdEmergencyBreakSetBackMiddleValue, (tVoid*)&newDistances[8]);
		pCoder->Set(m_szIdEmergencyBreakSetBackRightValue, (tVoid*)&newDistances[9]);
	}

	pMediaSample->SetTime(_clock->GetStreamTime());
	m_oOutputEmergencyBreakSet.Transmit(pMediaSample);

	RETURN_NOERROR;
}

tResult cStateMachine::ProcessWheelSampleLeft(IMediaSample* pMediaSample)
{
	static bool hasID = false;
	static tBufferID szIDWheelDataUi32WheelTach;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionWheelLeftData, pMediaSample, pCoderInput);

		if (!hasID)
		{
			pCoderInput->GetID("ui32WheelTach", szIDWheelDataUi32WheelTach);
			hasID = tTrue;
		}

		pCoderInput->Get(szIDWheelDataUi32WheelTach, (tVoid*)&wheelCountLeft);
	}

	m_actualWheelTicks = ((wheelCountLeft + wheelCountRight) / 2);

	RETURN_NOERROR;
}

tResult cStateMachine::ProcessWheelSampleRight(IMediaSample* pMediaSample)
{
	static bool hasID = false;
	static tBufferID szIDWheelDataUi32WheelTach;
	{
		__adtf_sample_read_lock_mediadescription(m_pDescriptionWheelRightData, pMediaSample, pCoderInput);

		if (!hasID)
		{
			pCoderInput->GetID("ui32WheelTach", szIDWheelDataUi32WheelTach);
			hasID = tTrue;
		}

		pCoderInput->Get(szIDWheelDataUi32WheelTach, (tVoid*)&wheelCountRight);

	}

	m_actualWheelTicks = ((wheelCountLeft + wheelCountRight) / 2);

	ComputeNextStep();

	RETURN_NOERROR;
}

tResult cStateMachine::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
{
	//use mutex
	__synchronized_obj(m_critSecTransmitControl);

	//init mediasample
	cObjectPtr<IMediaSample> pMediaSample;
	//allocate memory to mediasample
	AllocMediaSample((tVoid**)&pMediaSample);

	//create interaction with ddl
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSignalValue->GetMediaSampleSerializer(&pSerializer);

	//allocate buffer to write in mediasample
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());
	{
		//write lock
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

		//Set all Ids
		if (!m_szIdOutputSpeedSet)
		{
			pCoderInput->GetID("f32Value", m_szIdOutputspeedControllerValue);
			pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdOutputspeedControllerTimeStamp);
			m_szIdOutputSpeedSet = tTrue;
		}

		pCoderInput->Set(m_szIdOutputspeedControllerValue, (tVoid*)&speed);
		pCoderInput->Set(m_szIdOutputspeedControllerTimeStamp, (tVoid*)&timestamp);
	}

	pMediaSample->SetTime(_clock->GetStreamTime());
	m_oOutputSpeedController.Transmit(pMediaSample);

	RETURN_NOERROR;
}

tResult cStateMachine::updateSpeed()
{
	if (m_actualSpeed_changed)
	{
		m_actualSpeed_changed = false;

		tFloat32 newSpeed = DEFAULT_SPEED;

		if (abs(m_actualSpeedState) < abs(newSpeed))
		{
			LOG_WARNING(cString::Format("actualSpeedState:%f" , m_actualSpeedState));
			newSpeed = m_actualSpeedState;
		}

		if (newSpeed == DEFAULT_SPEED)
		{
			LOG_WARNING(cString::Format("m_actualSpeedUpFactor:%f" , m_actualSpeedUpFactor));
			newSpeed *= m_actualSpeedUpFactor;
		}

		if (m_actualSpeedLaneDetection != NO_LD_SPEED && m_actualSpeedState != 0)
		{
			LOG_WARNING(cString::Format("lanedetection:%f" , m_actualSpeedLaneDetection));
			newSpeed = m_actualSpeedLaneDetection;
		}

		TransmitSpeed(newSpeed, 0);
	}

	RETURN_NOERROR;
}

tResult cStateMachine::ChangePrimaryState(primaryStates newPrimaryState)
{
	if (m_primaryState != newPrimaryState)
	{
		m_primaryState = newPrimaryState;
		m_primaryState_changed = true;

		switch (newPrimaryState)
		{
		case primaryState_emergencyBreak:
		{
			if (m_runState != runState_ready && m_runState != runState_stop)
				m_EmergencyBreakSince = cSystem::GetTime();

			break;
		}

		case primaryState_run:
		{
			m_EmergencyBreakSince = INT32_MIN;

			break;
		}
		}
	}

	RETURN_NOERROR;
}

tResult cStateMachine::ChangeRunState(runStates newRunState)
{
	if (m_runState != newRunState)
	{
		m_runState = newRunState;
		m_runState_changed = true;

		switch (newRunState)
		{
		case runState_stop:
		{

			m_actualSpeedState = 0;
			m_actualSpeed_changed = true;
			m_EmergencyBreakSince = INT32_MIN;
			break;
		}

		case runState_ready:
		{

			m_actualSpeedState = 0;
			m_actualSpeed_changed = true;
			m_EmergencyBreakSince = INT32_MIN;
			break;
		}

		case runState_running:
		{

			LOG_WARNING("DEFAULT_SPEED SET");
			m_actualSpeedState = DEFAULT_SPEED;
			m_actualSpeed_changed = true;
			m_EmergencyBreakSince = INT32_MIN;
			break;
		}

		case runState_parking:
		{
			break;
		}
		}
	}

	RETURN_NOERROR;
}

tResult cStateMachine::ComputeNextStep()
{
	if (m_emergencyBreakStatus_changed)
	{
		m_emergencyBreakStatus_changed = false;
		if (m_emergencyBreakStatus)
			ChangePrimaryState(primaryState_emergencyBreak);
		else
			ChangePrimaryState(primaryState_run);
	}

	if (m_runState_changed)
	{
		m_runState_changed = false;
		TransmitEmergencyBreakSet();
	}

	if (m_primaryState_changed)
	{
		m_primaryState_changed = false;
		if (m_primaryState == primaryState_emergencyBreak)
		{
			//TODO:m_ActualPosition wieder eipflegen
			//tFloat32 angle = m_ActualPosition.f32Heading / -1.5 * 270;
			//while (angle < 0)
			//	angle += 360;

			//while (angle >= 360)
			//	angle -= 360;

			//float radAngle = DEG2RAD * angle;
		}
	}

	updateSpeed();


	RETURN_NOERROR;
}
