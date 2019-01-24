#ifndef _C_LIGHT_CONTROL_H_
#define _C_LIGHT_CONTROL_H_

#define MIN_BREAK_DURATION 1000000

class cLightControl : public adtf::cFilter
{
	//Set the Filter ID and the Version
	ADTF_FILTER("adtf.aadc.LightControl", "Light Control", adtf::OBJCAT_DataFilter);

	protected:
		//MediaType description
		cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
  	cObjectPtr<IMediaTypeDescription> m_pDescriptionBoolSignalValue;
		cObjectPtr<IMediaTypeDescription> m_pDescriptionEmergencyStop;
		cObjectPtr<IMediaTypeDescription> m_pDescriptionBool;

		//Output Pins Light
		cOutputPin m_oOutputHeadLight;    // The output pin for head light
		cOutputPin m_oOutputReverseLight; // The output pin for reverse light
		cOutputPin m_oOutputBrakeLight;   // The output pin for brake light
		cOutputPin m_oOutputHazzardLight; // The output pin for hazzard light
	  cOutputPin m_oOutputTurnRight;    // The output pin for turn right controller
	  cOutputPin m_oOutputTurnLeft;     // The output pin for turn left controller

		cInputPin m_oInputTurnRight;    // The input pin for turn right controller
		cInputPin m_oInputTurnLeft; // The input pin for turn left controller

		//Input Pins Speed
    cInputPin   m_oInputSpeedController;   //typ tSignalValue
    tBufferID   m_szIdInputspeedControllerValue;
    tBufferID   m_szIdInputspeedControllerTimeStamp;
		tBool m_szIdInputSpeedSet;
		tFloat32 newSpeed;

		//Input EmergencyBreak
		cInputPin   m_oInputEmergencyBreakStatus;
		tBool       m_szIdEmergencyBreakStatus;
		tBool 			m_EmergencyBreakStatus_changed;
		tBool 		  m_szIdEmergencyStopSet;
		tBufferID   m_szIdEmergencyStopValue;

		cCriticalSection m_critSecTransmitBool;
		cCriticalSection m_critSecTransmitControl;

		tInt64 m_lastTriggeredTime;
		tFloat32 m_actualSpeedUpFactor;

		//Input Pins Light
		int m_HeadLightOn;
		bool m_HeadLightOn_changed;

		int m_BrakeLightOn;
		bool m_BrakeLightOn_changed;

		int m_HazzardLightOn;
    bool m_HazzardLightOn_changed;

    int m_ReverseLightOn;
    bool m_ReverseLightOn_changed;

    int m_TurnLeftSignalOn;
    bool m_TurnLeftSignalOn_changed;

    int m_TurnRightSignalOn;
		bool m_TurnRightSignalOn_changed;

		tFloat32 m_oldSpeed;
		bool m_actualSpeed_changed;
		tFloat32 m_actualSpeedCarDetection;

	public:
		cLightControl(const tChar* __info);
		virtual ~cLightControl();

	protected:
		tResult Init(tInitStage eStage, ucom::IException** s__exception_ptr);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
		tResult Stop(ucom::IException** __exception_ptr = NULL);
		tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

		//tResult Speed
		tResult updateSpeed();

		//tResult Light
		tResult SetLight(int* lightCounter, bool* changedFlag, bool status);
    tResult SetHazzardLight(bool status);
    tResult SetBrakeLight(bool status);
    tResult SetHeadLight(bool status);
    tResult SetReverseLight(bool status);
    tResult SetTurnLeftSignal(bool status);
    tResult SetTurnRightSignal(bool status);
		tResult ProcessSpeedController(IMediaSample* pMediaSample);
		tResult TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp);
		tResult ResetAllLights();
		tResult StartLight();
		tResult ProcessEmergencyBreakStatus(IMediaSample* pMediaSample);

};
#endif
