#include "ParkControl.h"

//Set the FilterID
ADTF_FILTER_PLUGIN("ParkControl", "adtf.aadc.ParkControl", cParking);
//Konstruktor
cParking::cParking(const tChar *__info):cFilter(__info)
{
    //DO nothing
}
cParking::~cParking()
{
    //DO nothing
}

tResult cParking::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if(eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));
        //Input Speed
        //create and register pins for Speed Input
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));
        RETURN_IF_FAILED(m_oInputSpeedController.Create("SpeedController", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));

        RETURN_IF_FAILED(m_SteeringANG.Create("SteeringAngle", m_SteeringANG, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_SteeringANG));
    }
    else if(eStage == StageNormal)
    {

    }
    else if(eStage == StageGraphReady)
    {
        SteeringAngle();
    }
    RETURN_NOERROR;
}

tResult cParking::Shutdown(tInitStage eStage, __exception)
{
    if(eStage == StageGraphReady)
    {
        //Nothing
    }
    else if(eStage == StageNormal)
    {
        //Nothing
    }
    else if(eStage == StageFirst)
    {
        //Nothing
    }

    //Call the Baseclass implentation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cParking::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
      if(pSource == &m_oInputSpeedController)
      {

      }
        RETURN_IF_POINTER_NULL(pMediaSample);
    }
    RETURN_NOERROR;
}

tResult cParking::ProcessSpeedController(IMediaSample* pMediaSample)
{
    //read-out the incoming Media Sample

    tUInt32 timestamp;

    //sobald der block verlassen wird, wird das lock aufgehoben
    {
        //read lock
        __adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

        //Set all Ids
        if(!m_szIdInputSpeedSet)
        {
            pCoderInput->GetID("f32Value", m_szIdInputSpeedControllerValue);
            pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdInputSpeedControllerTimeStamp);
            m_szIdInputSpeedSet = tTrue;
        }

        pCoderInput->Get(m_szIdInputSpeedControllerValue, (tVoid*)&speed);
        pCoderInput->Get(m_szIdInputSpeedControllerTimeStamp, (tVoid*)&timestamp);
    }

    //if(m_breakActive)
    //{
    //    TransmitSpeed(0, timestamp);
    //}
    //else // !m_breakActive
    //{
    //    TransmitSpeed(speed, timestamp);
//  //      LOG_INFO(cString::Format("transmit speed %f", speed));
    //}

    RETURN_NOERROR;
}

tResult cParking::TransmitSpeed(tFloat32 speed, tUInt32 timestamp)
{
    LOG_INFO(cString::Format("transmit speed %f", speed));
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
        if(!m_szIdOutputSpeedSet)
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
//PROBLEM
tResult TransmitSteeringAngle(tFloat32 Steering)
{
  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

  cObjectPtr<IMediaSerializer> pSerializer;
  m_SteeringANG->GetMediaSampleSerializer(&pSerializer);
  tInt nSize = pSerializer->GetDeserializedSize();

  RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
  {
      // focus for sample write lock
      __adtf_sample_write_lock_mediadescription(pMediaSample,pCoder);
      pCoder->GetID("SteeringAngle: ", m_SteeringANG);
  }

  pMediaSample->SetTime(_clock->GetStreamTime());
  m_SteeringANG.Transmit(pMediaSample);
}

tResult cParking::SteeringAngle()
{
    //Test
    m_SteeringANG = 50.0;

    RETURN_NOERROR;
}
