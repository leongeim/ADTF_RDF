#ifndef PARKCONTROL_H
#define PARKCONTROL_H

#include "stdafx.h"

class cParking : public adtf::cFilter
{
    //Set the FilterID
    ADTF_FILTER("adtf.aadc.ParkControl", "ParkControl", adtf::OBJCAT_DataFilter);
protected:
    //MediaType description
    cObjectPtr<IMediaTypeDescription> m_pDescriptionSignalValue;
    //Input Pins Speed
    cInputPin   m_oInputSpeedController;    //Typ tSignalValue
    tBufferID   m_szIdInputSpeedControllerValue;
    tBufferID   m_szIdInputSpeedControllerTimeStamp;
    tBufferID   m_szIdInputSpeedSet;

    cOutputPin  m_oOutputSpeedController;  //typ tSignalValue
    tBufferID   m_szIdOutputspeedControllerValue;
    tBufferID   m_szIdOutputspeedControllerTimeStamp;
    tBool       m_szIdOutputSpeedSet;

    //Steering
    tFloat32    m_SteeringANG;
    tFloat32    m_SteeringAngle;
    tUInt32     speed;


    /*! The critical section transmit control */
    cCriticalSection m_critSecTransmitControl;

public:
    cParking(const tChar *__info);
    virtual ~cParking();

protected:
    //Standard Method
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    tResult ProcessSpeedController(IMediaSample* pMediaSample);
    tResult TransmitSpeed(tFloat32 speed, tUInt32 timestamp);
    tResult TransmitSteeringAngle(tFloat32 Steering);
    tResult SteeringAngle();
};

#endif // PARKCONTROL_H
