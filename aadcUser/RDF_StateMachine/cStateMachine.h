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
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#define OID_ADTF_FILTER_DEF "adtf.stateMachine"

#include "aadc_juryEnums.h"
#include "ManeuverList.h"
#include "cStates_Enum.h"
#include "stdafx.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD2DEG static_cast<tFloat32>(180.0/M_PI)
#define DEG2RAD static_cast<tFloat32>(M_PI/180.0)
#define DEFAULT_SPEED (-10)
#define NO_LD_SPEED 999



//!  Template filter for AADC Teams
/*!
* This is a example filter for the AADC
*/
class cStateMachine : public adtf::cFilter
{
	/*! set the filter ID and the version */
	ADTF_FILTER(OID_ADTF_FILTER_DEF, "RDF State Machine", adtf::OBJCAT_DataFilter)

private:
	// input pin for the run command
	cInputPin m_JuryStructInputPin;  //typ tJuryStruct
	tBufferID m_szIDJuryStructI8ActionID;
	tBufferID m_szIDJuryStructI16ManeuverEntry;
	tBool     m_bIDsJuryStructSet;

	// output pin for state from driver
	cOutputPin m_DriverStructOutputPin;  //typ tDriverStruct
	tBufferID  m_szIDDriverStructI8StateID;
	tBufferID  m_szIDDriverStructI16ManeuverEntry;
	tBool      m_bIDsDriverStructSet;

	//  input pin for the emergency break status
	cInputPin m_EmergencyBreakStatusInputPin; //typ tJuryEmergencyStop
	tBufferID m_szIdEmergencyStopValue;
	tBool     m_szIdEmergencyStopSet;

	//  input pin for the emergency break status
	cInputPin m_EmergencyStopInputPin; //typ tJuryEmergencyStop

	cOutputPin  m_oOutputEmergencyBreakSet;  //typ tEmergencyBreakSet
	tBufferID   m_szIdEmergencyBreakSetFrontLeftValue;
	tBufferID   m_szIdEmergencyBreakSetFrontHalfLeftValue;
	tBufferID   m_szIdEmergencyBreakSetFrontMiddleValue;
	tBufferID   m_szIdEmergencyBreakSetFrontHalfRightValue;
	tBufferID   m_szIdEmergencyBreakSetFrontRightValue;
	tBufferID   m_szIdEmergencyBreakSetSideLeftValue;
	tBufferID   m_szIdEmergencyBreakSetSideRightValue;
	tBufferID   m_szIdEmergencyBreakSetBackLeftValue;
	tBufferID   m_szIdEmergencyBreakSetBackMiddleValue;
	tBufferID   m_szIdEmergencyBreakSetBackRightValue;
	tBool       m_szIdEmergencyBreakSet;


	cInputPin    m_oInputWheelLeft;
	cInputPin    m_oInputWheelRight;

	// input pin for speed
	cInputPin   m_oInputSpeedController;   //typ tSignalValue
	tBufferID   m_szIdInputspeedControllerValue;
	tBufferID   m_szIdInputspeedControllerTimeStamp;
	tBool       m_szIdInputSpeedSet;

	// output pin for speed
	cOutputPin  m_oOutputSpeedController;  //typ tSignalValue
	tBufferID   m_szIdOutputspeedControllerValue;
	tBufferID   m_szIdOutputspeedControllerTimeStamp;
	tBool       m_szIdOutputSpeedSet;

	// input pin for ultrasonic struct
	cInputPin              m_oInputUsStruct; //tUltrasonicStruct
	std::vector<tBufferID> m_szIdUsStructValues;
	std::vector<tBufferID> m_szIdUsStructTimeStamps;
	tBool                  m_szIdsUsStructSet;

	tFloat32 m_actualSpeedState;
	tFloat32 m_actualSpeedLaneDetection;
	bool     m_actualSpeed_changed;
	tFloat32 m_actualSpeedUpFactor;
	tFloat32 m_maxSpeedUpFactor;

	tInt64   m_lastTimeStopp;
	tInt64   m_lastTimeCarDetected;
	tInt64   m_lastTimeFollowCarDetected;
	tInt64   m_EmergencyBreakSince;

	bool m_emergencyBreakStatus;
	bool m_emergencyBreakStatus_changed;

	bool m_Emergency_Stop_Jury;

	tFloat32 m_actualDistances[10];

	//mediatype descriptions
	cObjectPtr<IMediaTypeDescription> m_pDescJuryStruct;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionDriverStruct;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionEmergencyStop;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionEmergencyBreakSet;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelLeftData;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionWheelRightData;
	cObjectPtr<IMediaTypeDescription> m_pDescriptionUsStruct;


	primaryStates m_primaryState;     // the primary state of this state machince
	runStates     m_runState;         // the run state of this state machince
	bool          m_runState_changed;
	bool          m_primaryState_changed;

	int wheelCountLeft;
	int wheelCountRight;
	int m_actualWheelTicks;

	cCriticalSection m_critSecTransmitControl;
	cCriticalSection m_critSecMinimumUsValue;

public:
	/*! default constructor for template class
		   \param __info   [in] This is the name of the filter instance.
	*/
	cStateMachine(const tChar* __info);

	/*! default destructor */
	virtual ~cStateMachine();

protected:
	/*! Implements the default cFilter state machine call. It will be
	*	    called automatically by changing the filters state and needs
	*	    to be overwritten by the special filter.
	*    Please see page_filter_life_cycle for further information on when the state of a filter changes.
	*
	*    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
	*        If not using the cException smart pointer, the interface has to
	*        be released by calling Unref().
	*    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
	*    \return Standard Result Code.
	*/
	tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

	/*!
	*   Implements the default cFilter state machine call. It will be
	*   called automatically by changing the filters state and needs
	*   to be overwritten by the special filter.
	*   Please see page_filter_life_cycle for further information on when the state of a filter changes.
	*
	*   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
	*                                   If not using the cException smart pointer, the interface has to
	*                                   be released by calling Unref().
	*   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
	*   \return Returns a standard result code.
	*
	*/
	tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

	tResult Stop(ucom::IException** __exception_ptr = NULL);

	/*! This Function will be called by all pins the filter is registered to.
	*   \param [in] pSource Pointer to the sending pin's IPin interface.
	*   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
	*   \param [in] nParam1 Optional integer parameter.
	*   \param [in] nParam2 Optional integer parameter.
	*   \param [in] pMediaSample Address of an IMediaSample interface pointers.
	*   \return   Returns a standard result code.
	*   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
	*   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
	*/
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

	/*!
	 * \brief sends a DriverStruct to the jury module
	 * \param stateID  the id of the actual state  (see <stateCar>)
	 * \param i16ManeuverEntry  the actual maneuver entry
	 * \return  Returns a standard result code.
	 */
	tResult SendState(stateCar stateID, tInt16 i16ManeuverEntry = 0);

	tResult PropertyChanged(const tChar* strName);

	#pragma region TransmitOutput
		tResult TransmitEmergencyBreakSet();
		tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);
	#pragma endregion

	#pragma region ChangeStates
		tResult ChangePrimaryState(primaryStates newPrimaryState);
		tResult ChangeRunState(runStates newRunState);
	#pragma endregion

	#pragma region Processing
		tResult updateSpeed();
		tResult ComputeNextStep();
	#pragma endregion

	#pragma region ProcessInputs
		tResult ProcessJuryInput(IMediaSample* pMediaSample);
		tResult ProcessEmergencyBreakStatus(IMediaSample* pMediaSample);
		tResult ProcessEmergencyStop(IMediaSample* pMediaSample);
		tResult ProcessSpeedController(IMediaSample* pMediaSample);
		tResult ProcessUsValues(IMediaSample* pMediaSample);
		tResult ProcessWheelSampleLeft(IMediaSample* pMediaSample);
		tResult ProcessWheelSampleRight(IMediaSample* pMediaSample);
	#pragma endregion

	#pragma region Calculation
	tFloat32 normalizeAngle(tFloat32 alpha, tFloat32 center);
	tFloat32 mod(tFloat32 x, tFloat32 y);
	#pragma endregion
};

//*************************************************************************************************
#endif
