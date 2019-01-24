#include "stdafx.h"
#include "cLightControl.h"

//Set the Filter ID and the Version
ADTF_FILTER_PLUGIN("Light Control","adtf.aadc.LightControl", cLightControl);

cLightControl::cLightControl(const tChar* __info):cFilter(__info)
{
}

cLightControl::~cLightControl()
{
}

tResult cLightControl::Shutdown(tInitStage eStage, __exception)
{
    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {cOutputPin m_oOutputHeadLight;    // The output pin for head light
		cOutputPin m_oOutputReverseLight; // The output pin for reverse light
		cOutputPin m_oOutputBrakeLight;   // The output pin for brake light
		cOutputPin m_oOutputHazzardLight; // The output pin for hazzard light
	  cOutputPin m_oOutputTurnRight;    // The output pin for turn right controller
	  cOutputPin m_oOutputTurnLeft;
    }
    else if (eStage == StageFirst)
    {
    }
    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cLightControl::OnPinEvent(IPin* pSource,
                                  tInt nEventCode,
                                  tInt nParam1,
                                  tInt nParam2,
                                  IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);


    if (nEventCode == IPinEventSink::PE_MediaSampleReceived  )
    {
        if (pSource == &m_oInputSpeedController)
        {
            ProcessSpeedController(pMediaSample);
        }
        else if(pSource == &m_oInputEmergencyBreakStatus)
        {
            ProcessEmergencyBreakStatus(pMediaSample);
        }
    }

    RETURN_NOERROR;
}

tResult cLightControl::Stop(__exception)
{
    // call the base class implementatistrDescSignalValueon
    return cFilter::Stop(__exception_ptr);
}

tResult cLightControl::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, 			IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

		//Input Speed
		//create and register pins for speed in
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
    RETURN_IF_FAILED(m_oInputSpeedController.Create("SpeedController", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

    //input Emergency stop
    tChar const * strDescEmergencyStop = pDescManager->GetMediaDescription("tJuryEmergencyStop");
    RETURN_IF_POINTER_NULL(strDescEmergencyStop);
    cObjectPtr<IMediaType> pTypeEmergencyStatusStruct = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescEmergencyStop, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(m_oInputEmergencyBreakStatus.Create("EmergencyBreakStatus", pTypeEmergencyStatusStruct, this));
    RETURN_IF_FAILED(RegisterPin(&m_oInputEmergencyBreakStatus));
    RETURN_IF_FAILED(pTypeEmergencyStatusStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionEmergencyStop));


    // triggering the lights
    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionBool));
    RETURN_IF_FAILED(m_oOutputHeadLight.Create("headLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputHeadLight));
    RETURN_IF_FAILED(m_oOutputBrakeLight.Create("brakeLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputBrakeLight));
    RETURN_IF_FAILED(m_oOutputReverseLight.Create("reverseLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputReverseLight));
    RETURN_IF_FAILED(m_oOutputHazzardLight.Create("hazzardLightEnabled", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputHazzardLight));
    RETURN_IF_FAILED(m_oOutputTurnLeft.Create("turnSignalLeftEnabled_Out", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnLeft));
    RETURN_IF_FAILED(m_oOutputTurnRight.Create("turnSignalRightEnabled_Out", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutputTurnRight));
    RETURN_IF_FAILED(m_oInputTurnLeft.Create("turnSignalLeftEnabled_In", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputTurnLeft));
    RETURN_IF_FAILED(m_oInputTurnRight.Create("turnSignalRightEnabled_In", pTypeBoolSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInputTurnRight));

	}
	else if(eStage == StageNormal)
	{
		// speed control
		m_oldSpeed = 0;
		m_actualSpeed_changed = false;
    newSpeed = 0;
		// light control
		m_HeadLightOn = 0;
		m_HeadLightOn_changed = false;
		m_BrakeLightOn = 0;
		m_BrakeLightOn_changed = false;
		m_HazzardLightOn = 0;
		m_HazzardLightOn_changed = false;
		m_ReverseLightOn = 0;
		m_ReverseLightOn_changed = false;
		m_TurnLeftSignalOn = 0;
		m_TurnLeftSignalOn_changed = false;
		m_TurnRightSignalOn = 0;
		m_TurnRightSignalOn_changed = false;

    m_lastTriggeredTime = 0;

    m_szIdEmergencyBreakStatus = false;
    m_EmergencyBreakStatus_changed = false;
    m_szIdEmergencyStopSet = false;

    ResetAllLights();
    StartLight();
	}
	else if (eStage == StageGraphReady)
	{
    m_actualSpeedUpFactor = 1;
		ResetAllLights();

    StartLight();
	}
	RETURN_NOERROR;
}

tResult cLightControl::ProcessSpeedController(IMediaSample* pMediaSample)
{
    tFloat32 speed;
    tUInt32 timestamp;

    {
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

        //Set all Ids
        if(!m_szIdInputSpeedSet)
        {
            pCoderInput->GetID("f32Value", m_szIdInputspeedControllerValue);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdInputspeedControllerTimeStamp);
            m_szIdInputSpeedSet = tTrue;
        }

        pCoderInput->Get(m_szIdInputspeedControllerValue, (tVoid*)&speed);
        pCoderInput->Get(m_szIdInputspeedControllerTimeStamp, (tVoid*)&timestamp);
    }

    newSpeed = speed;

    if(newSpeed != m_oldSpeed)
    {
      m_actualSpeed_changed = true;
    }

    updateSpeed();
    RETURN_NOERROR;
}

tResult cLightControl::SetLight(int* lightCounter, bool* changedFlag, bool status)
{
    if(status)
    {
        if(*lightCounter == 0)
            *changedFlag = true;
        *lightCounter = 1;
    }
    else
    {
        *lightCounter = 0;
        *lightCounter = max(*lightCounter, 0);
        if(*lightCounter == 0)
            *changedFlag = true;
    }

    RETURN_NOERROR;
}

tResult cLightControl::SetHazzardLight(bool status)
{
    SetLight(&m_HazzardLightOn, &m_HazzardLightOn_changed, status);

    RETURN_NOERROR;
}

tResult cLightControl::SetBrakeLight(bool status)
{
    SetLight(&m_BrakeLightOn, &m_BrakeLightOn_changed, status);

    RETURN_NOERROR;
}
//Licht vorne an oder aus
tResult cLightControl::SetHeadLight(bool status)
{
    SetLight(&m_HeadLightOn, &m_HeadLightOn_changed, status);

    RETURN_NOERROR;
}

tResult cLightControl::SetReverseLight(bool status)
{
    SetLight(&m_ReverseLightOn, &m_ReverseLightOn_changed, status);

    RETURN_NOERROR;
}

tResult cLightControl::SetTurnLeftSignal(bool status)
{
    SetLight(&m_TurnLeftSignalOn, &m_TurnLeftSignalOn_changed, status);

    RETURN_NOERROR;
}

tResult cLightControl::SetTurnRightSignal(bool status)
{
    SetLight(&m_TurnRightSignalOn, &m_TurnRightSignalOn_changed, status);

    RETURN_NOERROR;
}


tResult cLightControl::TransmitBoolValue(cOutputPin* oPin, bool value, tUInt32 timestamp)
{
    //use mutex
    __synchronized_obj(m_critSecTransmitBool);

    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionBool->GetMediaSampleSerializer(&pSerializer);
    pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

    static bool hasID = false;
    static tBufferID szIDBoolValueOutput;
    static tBufferID szIDArduinoTimestampOutput;

    {
        __adtf_sample_write_lock_mediadescription(m_pDescriptionBool, pMediaSample, pCoderOutput);

        if(!hasID)
        {
            pCoderOutput->GetID("bValue", szIDBoolValueOutput);
            pCoderOutput->GetID("ui32ArduinoTimestamp", szIDArduinoTimestampOutput);
            hasID = tTrue;
        }

        pCoderOutput->Set(szIDBoolValueOutput, (tVoid*)&value);
        pCoderOutput->Set(szIDArduinoTimestampOutput, (tVoid*)&timestamp);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());

    oPin->Transmit(pMediaSample);

    RETURN_NOERROR;
}


tResult cLightControl::StartLight()
{
    //Frontlichter
    if (m_HeadLightOn_changed)
    {
        m_HeadLightOn_changed = false;
        if (m_HeadLightOn > 0)
        {
           TransmitBoolValue(&m_oOutputHeadLight, true, 0);
        }
        else
        {
           TransmitBoolValue(&m_oOutputHeadLight, false, 0);
        }
    }
    //Bremslicht
    if(m_BrakeLightOn_changed)
    {
        m_BrakeLightOn_changed = false;
        if(m_BrakeLightOn > 0)
        {
            TransmitBoolValue(&m_oOutputBrakeLight, true, 0);
        }
        else
            TransmitBoolValue(&m_oOutputBrakeLight, false, 0);
        {
        }
    }
    //Rücklicht
    if (m_ReverseLightOn_changed)
    {
        m_ReverseLightOn_changed = false;
        if (m_ReverseLightOn > 0)
        {
            TransmitBoolValue(&m_oOutputReverseLight, true, 0);
        }
        else
        {
            TransmitBoolValue(&m_oOutputReverseLight, false, 0);
        }
    }

    ////Linktsblinker
    if(m_TurnLeftSignalOn_changed)
    {
        m_TurnLeftSignalOn_changed = false;
        if(m_TurnLeftSignalOn > 0)
        {
            if(m_HazzardLightOn == 0)
            {
                if(m_TurnRightSignalOn > 0)
                {
                    m_TurnRightSignalOn = 0;
                }
             TransmitBoolValue(&m_oOutputTurnLeft, true, 0);
            }
        }
     }
     else if(m_HazzardLightOn == 0)
     {
       TransmitBoolValue(&m_oOutputTurnLeft, false, 0);
     }
    ////Rechtsblinker
    if(m_TurnRightSignalOn_changed)
    {
        m_TurnRightSignalOn_changed = false;
        if(m_TurnRightSignalOn > 0)
        {
            if(m_HazzardLightOn == 0)
            {
              if(m_TurnLeftSignalOn > 0)
              {
                  m_TurnLeftSignalOn = 0;
              }
            TransmitBoolValue(&m_oOutputTurnRight, true, 0);
            }
        }
    }
    else
    {
        TransmitBoolValue(&m_oOutputTurnRight, false, 0);
    }

    ////Warnblinker
    if(m_EmergencyBreakStatus_changed)
    {
        m_EmergencyBreakStatus_changed = false;
        if(m_szIdEmergencyBreakStatus)
        {
            TransmitBoolValue(&m_oOutputHazzardLight, true, 0);
        }
        else
        {
            TransmitBoolValue(&m_oOutputHazzardLight, false, 0);
            if(m_TurnLeftSignalOn > 0)
                m_TurnLeftSignalOn_changed = true;
            if(m_TurnRightSignalOn > 0)
                m_TurnRightSignalOn_changed = true;
        }
    }
    RETURN_NOERROR;
}

tResult cLightControl::updateSpeed()
{
    SetHeadLight(true);

    tInt64 actualTime = cSystem::GetTime();
    tBool breakLight = false;
    if(((actualTime - m_lastTriggeredTime) < MIN_BREAK_DURATION))
    {
        breakLight = true;
    }
    else
    {
        breakLight = false;
    }
    SetBrakeLight(breakLight);

    if(m_actualSpeed_changed)
    {
        m_actualSpeed_changed = false;
        //aktuelle Geschwindigkeit
        //newSpeed
        //alte Geschwindigkeit
        //m_oldSpeed

        //wir fahren vorwärts
        if(newSpeed < 0)
        {
            SetHazzardLight(false);
            SetReverseLight(false);

            if(m_oldSpeed > newSpeed)
            {
                SetBrakeLight(false);
                m_lastTriggeredTime = 0;
            }
            if(m_oldSpeed < newSpeed)
            {
              m_lastTriggeredTime = actualTime;
              SetBrakeLight(true);
            }
        }
        else if(newSpeed > 0) //Wir fahren Rückwärts
        {
            SetHazzardLight(false);
            SetReverseLight(true);

            if(m_oldSpeed < newSpeed)
            {
              SetBrakeLight(false);
              m_lastTriggeredTime = 0;
            }
            if(m_oldSpeed > newSpeed)
            {
              m_lastTriggeredTime = actualTime;
              SetBrakeLight(true);
            }
        }
        else //wir stehen!
        {
          if(m_szIdEmergencyBreakStatus)
          {
            SetBrakeLight(true);
          }
          else
          {
            SetReverseLight(false);
            SetBrakeLight(false);
          }
        }

        m_oldSpeed = newSpeed;
    }

    if(newSpeed == 0 && m_oldSpeed == 0)
    {
        m_lastTriggeredTime = actualTime;
    }



    StartLight();

    RETURN_NOERROR;
}

tResult cLightControl::ResetAllLights()
{
    m_HazzardLightOn    = 0;
    m_BrakeLightOn      = 0;
    m_HeadLightOn       = 0;
    m_ReverseLightOn    = 0;
    m_TurnLeftSignalOn  = 0;
    m_TurnRightSignalOn = 0;

    m_HazzardLightOn_changed    = true;
    m_BrakeLightOn_changed      = true;
    m_HeadLightOn_changed       = true;
    m_ReverseLightOn_changed    = true;
    m_TurnLeftSignalOn_changed  = true;
    m_TurnRightSignalOn_changed = true;

    RETURN_NOERROR;
}

tResult cLightControl::ProcessEmergencyBreakStatus(IMediaSample* pMediaSample)
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

  if (ebStatus != m_szIdEmergencyBreakStatus)
  {
    m_EmergencyBreakStatus_changed = true;
    m_szIdEmergencyBreakStatus = ebStatus;
    StartLight();
  }

  RETURN_NOERROR;
}
