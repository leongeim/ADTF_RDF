#ifndef ROADSIGN_H_
#define ROADSIGN_H_

#define OID_ADTF_FILTER_DEF "adtf.RoadSign"

#include "stdafx.h"

#define DETECTION_THRESHOLD 0.5

typedef struct _roadSign
{
    /*! road sign */
    tInt16 u16Id;

    /*! location */
    tFloat32 f32X;
    tFloat32 f32Y;

    /*! sign search radius */
    tFloat32 f32Radius;

    /*! direction (heading) of the road sign */
    tFloat32 f32Direction;

    tInt u16Cnt;

    tTimeStamp u32ticks;/*! measurement ticks*/

} roadSign;

class RoadSign : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_FILTER_DEF, "RoadSign", adtf::OBJCAT_DataFilter)

    private:
        cInputPin m_oInputRoadSignExt;   // Input pin for the road sign Ext data
        tBufferID m_szIDRoadSignExtI16Identifier;
        tBufferID m_szIDRoadSignExtF32Imagesize;
        tBufferID m_szIDRoadSignExtAf32TVec;
        tBufferID m_szIDRoadSignExtAf32RVec;
        tBool m_bIDsRoadSignExtSet;

        cInputPin m_oInputClassification; // input pin for the classification result

        //Position Output to backend
        cOutputPin m_OutputPostion;
        tBufferID m_szF32X,m_szF32Y,m_szF32Radius,m_szF32Speed,m_szF32Heading;
        tBool m_PosOutputSet;

        /*! currently processed road-sign */
        tInt16 m_i16ID;
        tFloat32 m_f32MarkerSize;
        Mat m_Tvec; /*! translation vector */
        Mat m_Rvec; /*! rotation vector */

        tInt m_ui32Cnt;

        // Position input
        cInputPin  m_oInputPinPosition;        // Input pin for the road sign position
        tBool m_PosInputSet;

        vector<roadSign> m_roadSigns; // storage for the roadsign data

        tFloat32 m_actualSpeedTrafficSignDetection;

        tInt32   m_lastTicksStopSignDetected;
        tInt32   m_lastTicksGiveWaySignDetected;
        tInt32   m_lastTicksHaveWaySignDetected;
        tInt32   m_lastTicksPedestrianSignDetected;
        tInt32 m_lastTicksRelevantSignDetected;

        cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSignExt;

        cObjectPtr<IMediaTypeDescription> m_pDescriptionTrafficSign;
};
