
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
* $Author:: spiesra $  $Date:: 2017-05-22 18:08:00#$ $Rev:: 63774   $
**********************************************************************/
#include "stdafx.h"
#include "cLaneDetection.h"
using namespace roadsignIDs;
using namespace Unia;

// define the ADTF property names to avoid errors
ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC,
	OID_ADTF_FILTER_DEF,
	cLaneDetection)

	cLaneDetection::cLaneDetection(const tChar* __info) : cFilter(__info)
{
	SetPropertyInt("Algorithm::Image Binarization Threshold", 220);
	SetPropertyStr("Algorithm::Image Binarization Threshold" NSSUBPROP_DESCRIPTION, "Threshold for image binarization");
	SetPropertyBool("Algorithm::Image Binarization Threshold" NSSUBPROP_ISCHANGEABLE, tTrue);
	SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MIN, 1);
	SetPropertyInt("Algorithm::Image Binarization Threshold" NSSUBPROP_MAX, 255);

	SetPropertyBool("Algorithm::TestBool", false);
	SetPropertyStr("Algorithm::TestBool" NSSUBPROP_DESCRIPTION, "Bool to turn on/off things");
	SetPropertyBool("Algorithm::TestBool" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::LowerRightY", 5);
	SetPropertyStr("ROI::LowerRightY" NSSUBPROP_DESCRIPTION, "Polygon Lower Right y offset");
	SetPropertyBool("ROI::LowerRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::CurvatureOffsetRight", 1);
	SetPropertyStr("StreetDetection::CurvatureOffsetRight" NSSUBPROP_DESCRIPTION, "Offset for curves");
	SetPropertyBool("StreetDetection::CurvatureOffsetRight" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::CurvatureOffsetLeft", 1);
	SetPropertyStr("StreetDetection::CurvatureOffsetLeft" NSSUBPROP_DESCRIPTION, "Offset for curves");
	SetPropertyBool("StreetDetection::CurvatureOffsetLeft" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::CurvatureOffsetMid", 1);
	SetPropertyStr("StreetDetection::CurvatureOffsetMid" NSSUBPROP_DESCRIPTION, "Offset for curves");
	SetPropertyBool("StreetDetection::CurvatureOffsetMid" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("HoughLines::rho", 2);
	SetPropertyStr("HoughLines::rho" NSSUBPROP_DESCRIPTION, "Hough Lines");
	SetPropertyBool("HoughLines::rho" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("HoughLines::theta", CV_PI / 180);
	SetPropertyStr("HoughLines::theta" NSSUBPROP_DESCRIPTION, "Hough Lines");
	SetPropertyBool("HoughLines::theta" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("HoughLines::threshold", 50);
	SetPropertyStr("HoughLines::threshold" NSSUBPROP_DESCRIPTION, "Hough Lines");
	SetPropertyBool("HoughLines::threshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("HoughLines::minLineLenght", 30);
	SetPropertyStr("HoughLines::minLineLenght" NSSUBPROP_DESCRIPTION, "Hough Lines");
	SetPropertyBool("HoughLines::minLineLenght" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("HoughLines::maxLineGap", 30);
	SetPropertyStr("HoughLines::maxLineGap" NSSUBPROP_DESCRIPTION, "Hough Lines");
	SetPropertyBool("HoughLines::maxLineGap" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::LeftLowerLeftX", 100);
	SetPropertyStr("StreetDetection::LeftLowerLeftX" NSSUBPROP_DESCRIPTION, "Lower Left Street Border");
	SetPropertyBool("StreetDetection::LeftLowerLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::LeftLowerLeftY", 100);
	SetPropertyStr("StreetDetection::LeftLowerLeftY" NSSUBPROP_DESCRIPTION, "Lower Left Street Border");
	SetPropertyBool("StreetDetection::LeftLowerLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::LeftLowerRightX", 100);
	SetPropertyStr("StreetDetection::LeftLowerRightX" NSSUBPROP_DESCRIPTION, "Lower Left Street Border");
	SetPropertyBool("StreetDetection::LeftLowerRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::LeftLowerRightY", 100);
	SetPropertyStr("StreetDetection::LeftLowerRightY" NSSUBPROP_DESCRIPTION, "Lower Left Street Border");
	SetPropertyBool("StreetDetection::LeftLowerRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::LeftUpperLeftX", 100);
	SetPropertyStr("StreetDetection::LeftUpperLeftX" NSSUBPROP_DESCRIPTION, "Upper Left Street Border");
	SetPropertyBool("StreetDetection::LeftUpperLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::LeftUpperLeftY", 100);
	SetPropertyStr("StreetDetection::LeftUpperLeftY" NSSUBPROP_DESCRIPTION, "Upper Left Street Border");
	SetPropertyBool("StreetDetection::LeftUpperLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::LeftUpperRightX", 100);
	SetPropertyStr("StreetDetection::LeftUpperRightX" NSSUBPROP_DESCRIPTION, "Upper Left Street Border");
	SetPropertyBool("StreetDetection::LeftUpperRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::LeftUpperRightY", 100);
	SetPropertyStr("StreetDetection::LeftUpperRightY" NSSUBPROP_DESCRIPTION, "Upper Left Street Border");
	SetPropertyBool("StreetDetection::LeftUpperRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::MidLowerLeftX", 100);
	SetPropertyStr("StreetDetection::MidLowerLeftX" NSSUBPROP_DESCRIPTION, "Lower Middle Street Border");
	SetPropertyBool("StreetDetection::MidLowerLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::MidLowerLeftY", 100);
	SetPropertyStr("StreetDetection::MidLowerLeftY" NSSUBPROP_DESCRIPTION, "Lower Middle Street Border");
	SetPropertyBool("StreetDetection::MidLowerLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::MidLowerRightX", 100);
	SetPropertyStr("StreetDetection::MidLowerRightX" NSSUBPROP_DESCRIPTION, "Lower Middle Street Border");
	SetPropertyBool("StreetDetection::MidLowerRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::MidLowerRightY", 100);
	SetPropertyStr("StreetDetection::MidLowerRightY" NSSUBPROP_DESCRIPTION, "Lower Middle Street Border");
	SetPropertyBool("StreetDetection::MidLowerRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::MidUpperLeftX", 100);
	SetPropertyStr("StreetDetection::MidUpperLeftX" NSSUBPROP_DESCRIPTION, "Upper Middle Street Border");
	SetPropertyBool("StreetDetection::MidUpperLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::MidUpperLeftY", 100);
	SetPropertyStr("StreetDetection::MidUpperLeftY" NSSUBPROP_DESCRIPTION, "Upper Middle Street Border");
	SetPropertyBool("StreetDetection::MidUpperLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::MidUpperRightX", 100);
	SetPropertyStr("StreetDetection::MidUpperRightX" NSSUBPROP_DESCRIPTION, "Upper Middle Street Border");
	SetPropertyBool("StreetDetection::MidUpperRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::MidUpperRightY", 100);
	SetPropertyStr("StreetDetection::MidUpperRightY" NSSUBPROP_DESCRIPTION, "Upper Middle Street Border");
	SetPropertyBool("StreetDetection::MidUpperRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::RightLowerLeftX", 100);
	SetPropertyStr("StreetDetection::RightLowerLeftX" NSSUBPROP_DESCRIPTION, "Lower Right Street Border");
	SetPropertyBool("StreetDetection::RightLowerLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::RightLowerLeftY", 100);
	SetPropertyStr("StreetDetection::RightLowerLeftY" NSSUBPROP_DESCRIPTION, "Lower Right Street Border");
	SetPropertyBool("StreetDetection::RightLowerLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::RightLowerRightX", 100);
	SetPropertyStr("StreetDetection::RightLowerRightX" NSSUBPROP_DESCRIPTION, "Lower Right Street Border");
	SetPropertyBool("StreetDetection::RightLowerRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::RightLowerRightY", 100);
	SetPropertyStr("StreetDetection::RightLowerRightY" NSSUBPROP_DESCRIPTION, "Lower Right Street Border");
	SetPropertyBool("StreetDetection::RightLowerRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::RightUpperLeftX", 100);
	SetPropertyStr("StreetDetection::RightUpperLeftX" NSSUBPROP_DESCRIPTION, "Upper Right Street Border");
	SetPropertyBool("StreetDetection::RightUpperLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::RightUpperLeftY", 100);
	SetPropertyStr("StreetDetection::RightUpperLeftY" NSSUBPROP_DESCRIPTION, "Upper Right Street Border");
	SetPropertyBool("StreetDetection::RightUpperLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::RightUpperRightX", 100);
	SetPropertyStr("StreetDetection::RightUpperRightX" NSSUBPROP_DESCRIPTION, "Upper Right Street Border");
	SetPropertyBool("StreetDetection::RightUpperRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("StreetDetection::RightUpperRightY", 100);
	SetPropertyStr("StreetDetection::RightUpperRightY" NSSUBPROP_DESCRIPTION, "Upper Right Street Border");
	SetPropertyBool("StreetDetection::RightUpperRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("StreetDetection::minAngle", 10);
	SetPropertyStr("StreetDetection::minAngle" NSSUBPROP_DESCRIPTION, "Minimum angle of Line");
	SetPropertyBool("StreetDetection::minAngle" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("StreetDetection::maxAngle", 10);
	SetPropertyStr("StreetDetection::maxAngle" NSSUBPROP_DESCRIPTION, "Maximum angle of line");
	SetPropertyBool("StreetDetection::maxAngle" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("LaneHolding::FaultyLaneThreshold", 10);
	SetPropertyStr("LaneHolding::FaultyLaneThreshold" NSSUBPROP_DESCRIPTION, "Counter for faulty lanes");
	SetPropertyBool("LaneHolding::FaultyLaneThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("LaneHolding::emergencyThreshold", 10);
	SetPropertyStr("LaneHolding::emergencyThreshold" NSSUBPROP_DESCRIPTION, "Counter for ticks until emergency search");
	SetPropertyBool("LaneHolding::emergencyThreshold" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("WarpPerspective::warpLeft", 300);
	SetPropertyStr("WarpPerspective::warpLeft" NSSUBPROP_DESCRIPTION, "X value for input[0]");
	SetPropertyBool("WarpPerspective::warpLeft" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("WarpPerspective::warpRight", 980);
	SetPropertyStr("WarpPerspective::warpRight" NSSUBPROP_DESCRIPTION, "X value for input[1]");
	SetPropertyBool("WarpPerspective::warpRight" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("LaneHolding::FovCurveRight", 50);
	SetPropertyStr("LaneHolding::FovCurveRight" NSSUBPROP_DESCRIPTION, "Field of View distance to car in right curves");
	SetPropertyBool("LaneHolding::FovCurveRight" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("LaneHolding::FovCurveLeft", 50);
	SetPropertyStr("LaneHolding::FovCurveLeft" NSSUBPROP_DESCRIPTION, "Field of View distance to car in left curves");
	SetPropertyBool("LaneHolding::FovCurveLeft" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PID::Kp", 0);
	SetPropertyStr("PID::Kp" NSSUBPROP_DESCRIPTION, "P-Wert des PID-Reglers");
	SetPropertyBool("PID::Kp" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PID::Ki", 0);
	SetPropertyStr("PID::Ki" NSSUBPROP_DESCRIPTION, "I-Wert des PID-Reglers");
	SetPropertyBool("PID::Ki" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PID::Kd", 0);
	SetPropertyStr("PID::Kd" NSSUBPROP_DESCRIPTION, "D-Wert des PID-Reglers");
	SetPropertyBool("PID::Kd" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PID::K1", 0);
	SetPropertyStr("PID::K1" NSSUBPROP_DESCRIPTION, "K1-Wert des Reglers");
	SetPropertyBool("PID::K1" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PID::K2", 0);
	SetPropertyStr("PID::K2" NSSUBPROP_DESCRIPTION, "K2-Wert des Reglers");
	SetPropertyBool("PID::K2" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PID::K3", 0);
	SetPropertyStr("PID::K3" NSSUBPROP_DESCRIPTION, "K3-Wert des Reglers");
	SetPropertyBool("PID::K3" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("PID::K4", 0);
	SetPropertyStr("PID::K4" NSSUBPROP_DESCRIPTION, "K4-Wert des Reglers");
	SetPropertyBool("PID::K4" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("StopLineDetection::stoplineDistance", 10);
	SetPropertyStr("StopLineDetection::stoplineDistance" NSSUBPROP_DESCRIPTION, "Distance zwischen den zwei kanten der Haltelinie");
	SetPropertyBool("StopLineDetection::stoplineDistance" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("StopLineDetection::distanceOffset", 1);
	SetPropertyStr("StopLineDetection::distanceOffset" NSSUBPROP_DESCRIPTION, "Varianz den der Abstand der Haltelinienkanten haben darf");
	SetPropertyBool("StopLineDetection::distanceOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("StopLineDetection::lineLength", 25);
	SetPropertyStr("StopLineDetection::lineLength" NSSUBPROP_DESCRIPTION, "L�nge der Stoplinie");
	SetPropertyBool("StopLineDetection::lineLength" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("StopLineDetection::lengthOffset", 5);
	SetPropertyStr("StopLineDetection::lengthOffset" NSSUBPROP_DESCRIPTION, "Offset der LinienL�nge");
	SetPropertyBool("StopLineDetection::lengthOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("CrossDetection::fourCrossLowerLeftX", 5);
	SetPropertyStr("CrossDetection::fourCrossLowerLeftX" NSSUBPROP_DESCRIPTION, "ROI Koordinaten f�r vierer Kreuzung");
	SetPropertyBool("CrossDetection::fourCrossLowerLeftX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("CrossDetection::fourCrossLowerLeftY", 5);
	SetPropertyStr("CrossDetection::fourCrossLowerLeftY" NSSUBPROP_DESCRIPTION, "ROI Koordinaten f�r vierer Kreuzung");
	SetPropertyBool("CrossDetection::fourCrossLowerLeftY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("CrossDetection::fourCrossUpperRightX", 5);
	SetPropertyStr("CrossDetection::fourCrossUpperRightX" NSSUBPROP_DESCRIPTION, "ROI Koordinaten f�r vierer Kreuzung");
	SetPropertyBool("CrossDetection::fourCrossUpperRightX" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("CrossDetection::fourCrossUpperRightY", 5);
	SetPropertyStr("CrossDetection::fourCrossUpperRightY" NSSUBPROP_DESCRIPTION, "ROI Koordinaten f�r vierer Kreuzung");
	SetPropertyBool("CrossDetection::fourCrossUpperRightY" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Maneuver::VerticalTicks", 20);
	SetPropertyStr("Maneuver::VerticalTicks" NSSUBPROP_DESCRIPTION, "Ticks bis parallel");
	SetPropertyBool("Maneuver::VerticalTicks" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Maneuver::HorizontalTicks", 130);
	SetPropertyStr("Maneuver::HorizontalTicks" NSSUBPROP_DESCRIPTION, "Ticks bis parallel");
	SetPropertyBool("Maneuver::HorizontalTicks" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("Maneuver::PixelToTicks", 1);
	SetPropertyStr("Maneuver::PixelToTicks" NSSUBPROP_DESCRIPTION, "Unrechnungsfaktor Pixel zu Ticks");
	SetPropertyBool("Maneuver::PixelToTicks" NSSUBPROP_ISCHANGEABLE, tTrue);

	m_szIdOutputSteeringAngleSet = tFalse;
	m_szIdOutputManeuverFinishedSet = tFalse;
	m_szIdInputSpeedSet = tFalse;
	m_bIDManeuverSendSet = tFalse;
	m_szIdOutputSpeedSet = tFalse;
}

tResult cLaneDetection::PropertyChanged(const tChar* strName)
{
	RETURN_IF_FAILED(cFilter::PropertyChanged(strName));
	//associate the properties to the member
	if (cString::IsEqual(strName, "Algorithm::Image Binarization Threshold"))
		m_filterProperties.thresholdImageBinarization = GetPropertyInt("Algorithm::Image Binarization Threshold");
	else if (cString::IsEqual(strName, "Algorithm::TestBool"))
		m_filterProperties.testBool = GetPropertyBool("Algorithm::TestBool");

	else if (cString::IsEqual(strName, "HoughLines::rho"))
		m_filterProperties.HLrho = GetPropertyFloat("HoughLines::rho");
	else if (cString::IsEqual(strName, "HoughLines::theta"))
		m_filterProperties.HLtheta = GetPropertyFloat("HoughLines::theta");
	else if (cString::IsEqual(strName, "HoughLines::threshold"))
		m_filterProperties.HLthreshold = GetPropertyInt("HoughLines::threshold");
	else if (cString::IsEqual(strName, "HoughLines::minLineLenght"))
		m_filterProperties.HLminLineLenght = GetPropertyFloat("HoughLines::minLineLenght");
	else if (cString::IsEqual(strName, "HoughLines::maxLineGap"))
		m_filterProperties.HLmaxLineGap = GetPropertyFloat("HoughLines::maxLineGap");

	else if (cString::IsEqual(strName, "StreetDetection::LeftLowerLeftX"))
		m_filterProperties.LeftLowerLeftX = GetPropertyFloat("StreetDetection::LeftLowerLeftX");
	else if (cString::IsEqual(strName, "StreetDetection::LeftLowerLeftY"))
		m_filterProperties.LeftLowerLeftY = GetPropertyFloat("StreetDetection::LeftLowerLeftY");
	else if (cString::IsEqual(strName, "StreetDetection::LeftLowerRightX"))
		m_filterProperties.LeftLowerRightX = GetPropertyFloat("StreetDetection::LeftLowerRightX");
	else if (cString::IsEqual(strName, "StreetDetection::LeftLowerRightY"))
		m_filterProperties.LeftLowerRightY = GetPropertyFloat("StreetDetection::LeftLowerRightY");
	else if (cString::IsEqual(strName, "StreetDetection::LeftUpperLeftX"))
		m_filterProperties.LeftUpperLeftX = GetPropertyFloat("StreetDetection::LeftUpperLeftX");
	else if (cString::IsEqual(strName, "StreetDetection::LeftUpperLeftY"))
		m_filterProperties.LeftUpperLeftY = GetPropertyFloat("StreetDetection::LeftUpperLeftY");
	else if (cString::IsEqual(strName, "StreetDetection::LeftUpperRightX"))
		m_filterProperties.LeftUpperRightX = GetPropertyFloat("StreetDetection::LeftUpperRightX");
	else if (cString::IsEqual(strName, "StreetDetection::LeftUpperRightY"))
		m_filterProperties.LeftUpperRightY = GetPropertyFloat("StreetDetection::LeftUpperRightY");
	else if (cString::IsEqual(strName, "StreetDetection::MidLowerLeftX"))
		m_filterProperties.MidLowerLeftX = GetPropertyFloat("StreetDetection::MidLowerLeftX");
	else if (cString::IsEqual(strName, "StreetDetection::MidLowerLeftY"))
		m_filterProperties.MidLowerLeftY = GetPropertyFloat("StreetDetection::MidLowerLeftY");
	else if (cString::IsEqual(strName, "StreetDetection::MidLowerRightX"))
		m_filterProperties.MidLowerRightX = GetPropertyFloat("StreetDetection::MidLowerRightX");
	else if (cString::IsEqual(strName, "StreetDetection::MidLowerRightY"))
		m_filterProperties.MidLowerRightY = GetPropertyFloat("StreetDetection::MidLowerRightY");
	else if (cString::IsEqual(strName, "StreetDetection::MidUpperLeftX"))
		m_filterProperties.MidUpperLeftX = GetPropertyFloat("StreetDetection::MidUpperLeftX");
	else if (cString::IsEqual(strName, "StreetDetection::MidUpperLeftY"))
		m_filterProperties.MidUpperLeftY = GetPropertyFloat("StreetDetection::MidUpperLeftY");
	else if (cString::IsEqual(strName, "StreetDetection::MidUpperRightX"))
		m_filterProperties.MidUpperRightX = GetPropertyFloat("StreetDetection::MidUpperRightX");
	else if (cString::IsEqual(strName, "StreetDetection::MidUpperRightY"))
		m_filterProperties.MidUpperRightY = GetPropertyFloat("StreetDetection::MidUpperRightY");
	else if (cString::IsEqual(strName, "StreetDetection::RightLowerLeftX"))
		m_filterProperties.RightLowerLeftX = GetPropertyFloat("StreetDetection::RightLowerLeftX");
	else if (cString::IsEqual(strName, "StreetDetection::RightLowerLeftY"))
		m_filterProperties.RightLowerLeftY = GetPropertyFloat("StreetDetection::RightLowerLeftY");
	else if (cString::IsEqual(strName, "StreetDetection::RightLowerRightX"))
		m_filterProperties.RightLowerRightX = GetPropertyFloat("StreetDetection::RightLowerRightX");
	else if (cString::IsEqual(strName, "StreetDetection::RightLowerRightY"))
		m_filterProperties.RightLowerRightY = GetPropertyFloat("StreetDetection::RightLowerRightY");
	else if (cString::IsEqual(strName, "StreetDetection::RightUpperLeftX"))
		m_filterProperties.RightUpperLeftX = GetPropertyFloat("StreetDetection::RightUpperLeftX");
	else if (cString::IsEqual(strName, "StreetDetection::RightUpperLeftY"))
		m_filterProperties.RightUpperLeftY = GetPropertyFloat("StreetDetection::RightUpperLeftY");
	else if (cString::IsEqual(strName, "StreetDetection::RightUpperRightX"))
		m_filterProperties.RightUpperRightX = GetPropertyFloat("StreetDetection::RightUpperRightX");
	else if (cString::IsEqual(strName, "StreetDetection::RightUpperRightY"))
		m_filterProperties.RightUpperRightY = GetPropertyFloat("StreetDetection::RightUpperRightY");
	else if (cString::IsEqual(strName, "StreetDetection::minAngle"))
		m_filterProperties.minAngle = GetPropertyFloat("StreetDetection::minAngle");
	else if (cString::IsEqual(strName, "StreetDetection::maxAngle"))
		m_filterProperties.maxAngle = GetPropertyFloat("StreetDetection::maxAngle");
	else if (cString::IsEqual(strName, "StreetDetection::CurvatureOffsetRight"))
		m_filterProperties.curvatureOffsetRight = GetPropertyInt("StreetDetection::CurvatureOffsetRight");
	else if (cString::IsEqual(strName, "StreetDetection::CurvatureOffsetLeft"))
		m_filterProperties.curvatureOffsetLeft = GetPropertyInt("StreetDetection::CurvatureOffsetLeft");
	else if (cString::IsEqual(strName, "StreetDetection::CurvatureOffsetMid"))
		m_filterProperties.curvatureOffsetMid = GetPropertyInt("StreetDetection::CurvatureOffsetMid");

	else if (cString::IsEqual(strName, "LaneHolding::FaultyLaneThreshold"))
		m_filterProperties.faultyCounterThreshold = GetPropertyInt("LaneHolding::FaultyLaneThreshold");
	else if (cString::IsEqual(strName, "LaneHolding::emergencyThreshold"))
		m_filterProperties.emergencyThreshold = GetPropertyInt("LaneHolding::emergencyThreshold");
	else if (cString::IsEqual(strName, "LaneHolding::FovCurveRight"))
		m_filterProperties.FovCurveRight = GetPropertyFloat("LaneHolding::FovCurveRight");
	else if (cString::IsEqual(strName, "LaneHolding::FovCurveLeft"))
		m_filterProperties.FovCurveLeft = GetPropertyFloat("LaneHolding::FovCurveLeft");

	else if (cString::IsEqual(strName, "WarpPerspective::warpLeft"))
		m_filterProperties.warpLeft = GetPropertyInt("WarpPerspective::warpLeft");
	else if (cString::IsEqual(strName, "WarpPerspective::warpRight"))
		m_filterProperties.warpRight = GetPropertyInt("WarpPerspective::warpRight");

	else if (cString::IsEqual(strName, "PID::Kp"))
		m_filterProperties.Kp = GetPropertyFloat("PID::Kp");
	else if (cString::IsEqual(strName, "PID::Ki"))
		m_filterProperties.Ki = GetPropertyFloat("PID::Ki");
	else if (cString::IsEqual(strName, "PID::Kd"))
		m_filterProperties.Kd = GetPropertyFloat("PID::Kd");
	else if (cString::IsEqual(strName, "PID::K1"))
		m_filterProperties.K1 = GetPropertyFloat("PID::K1");
	else if (cString::IsEqual(strName, "PID::K2"))
		m_filterProperties.K2 = GetPropertyFloat("PID::K2");
	else if (cString::IsEqual(strName, "PID::K3"))
		m_filterProperties.K3 = GetPropertyFloat("PID::K3");
	else if (cString::IsEqual(strName, "PID::K4"))
		m_filterProperties.K4 = GetPropertyFloat("PID::K4");

	else if (cString::IsEqual(strName, "StopLineDetection::stoplineDistance"))
		m_filterProperties.stopLineDistance = GetPropertyFloat("StopLineDetection::stoplineDistance");
	else if (cString::IsEqual(strName, "StopLineDetection::distanceOffset"))
		m_filterProperties.distanceOffset = GetPropertyFloat("StopLineDetection::distanceOffset");
	else if (cString::IsEqual(strName, "StopLineDetection::lineLength"))
		m_filterProperties.stopLineLength = GetPropertyFloat("StopLineDetection::lineLength");
	else if (cString::IsEqual(strName, "StopLineDetection::lengthOffset"))
		m_filterProperties.lengthOffset = GetPropertyFloat("StopLineDetection::lengthOffset");

	else if (cString::IsEqual(strName, "CrossDetection::fourCrossLowerLeftX"))
		m_filterProperties.fourCrossLowerLeftX = GetPropertyInt("CrossDetection::fourCrossLowerLeftX");
	else if (cString::IsEqual(strName, "CrossDetection::fourCrossLowerLeftY"))
		m_filterProperties.fourCrossLowerLeftY = GetPropertyInt("CrossDetection::fourCrossLowerLeftY");
	else if (cString::IsEqual(strName, "CrossDetection::fourCrossUpperRightX"))
		m_filterProperties.fourCrossUpperRightX = GetPropertyInt("CrossDetection::fourCrossUpperRightX");
	else if (cString::IsEqual(strName, "CrossDetection::fourCrossUpperRightY"))
		m_filterProperties.fourCrossUpperRightY = GetPropertyInt("CrossDetection::fourCrossUpperRightY");

	else if (cString::IsEqual(strName, "Maneuver::VerticalTicks"))
		m_filterProperties.verticalTicks = GetPropertyInt("Maneuver::VerticalTicks");
	else if (cString::IsEqual(strName, "Maneuver::HorizontalTicks"))
		m_filterProperties.horizontalTicks = GetPropertyInt("Maneuver::HorizontalTicks");
	else if (cString::IsEqual(strName, "Maneuver::PixelToTicks"))
		m_filterProperties.pixelToTicks = GetPropertyInt("Maneuver::PixelToTicks");

	RETURN_NOERROR;
}

cLaneDetection::~cLaneDetection()
{
}

tResult cLaneDetection::Start(__exception)
{

	return cFilter::Start(__exception_ptr);
}

tResult cLaneDetection::Stop(__exception)
{
	//destroyWindow("Debug");
	return cFilter::Stop(__exception_ptr);
}
tResult cLaneDetection::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

		//Create Wheeldata datatype
		tChar const * strDescWheelData = pDescManager->GetMediaDescription("tWheelData");
		RETURN_IF_POINTER_NULL(strDescWheelData);
		cObjectPtr<IMediaType> pTypeWheelData = new cMediaType(0, 0, 0, "tWheelData", strDescWheelData, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		//init mediatype for US struct pins
		tChar const * strDescUsStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
		RETURN_IF_POINTER_NULL(strDescUsStruct);
		cObjectPtr<IMediaType> pTypeUsStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUsStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

		// Video Output
		RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output_Debug", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

		RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelLeftData));
		RETURN_IF_FAILED(pTypeWheelData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionWheelRightData));
		//RETURN_IF_FAILED(pTypeManeuverData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionManeuverData));

		// Video Output GCL
		m_oGCLOutputPin.Create("GCL", new adtf::cMediaType(MEDIA_TYPE_COMMAND, MEDIA_SUBTYPE_COMMAND_GCL), static_cast<IPinEventSink*>(this));
		RegisterPin(&m_oGCLOutputPin);

		//Create Wheeldata Pins
		RETURN_IF_FAILED(m_oInputWheelLeft.Create("WheelLeftStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputWheelLeft));

		RETURN_IF_FAILED(m_oInputWheelRight.Create("WheelRightStruct", pTypeWheelData, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputWheelRight));

		//Create UltraSonic
		RETURN_IF_FAILED(m_oInputUsStruct.Create("UsStruct", pTypeUsStruct, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputUsStruct));

		//init mediatype for signal values
		tChar const * strDescSteeringAngle = pDescManager->GetMediaDescription("tSignalValue");

		RETURN_IF_POINTER_NULL(strDescSteeringAngle);
		cObjectPtr<IMediaType> pTypeSteeringangle = new cMediaType(0, 0, 0, "tSignalValue", strDescSteeringAngle, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		//get mediatype description interfaces
		RETURN_IF_FAILED(pTypeSteeringangle->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSteeringAngle));
		RETURN_IF_FAILED(pTypeUsStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUsStruct));

		// steering angle outputint
		m_oSteeringAngleOutputPin.Create("Steeringangle", pTypeSteeringangle, static_cast<IPinEventSink*>(this));
		RegisterPin(&m_oSteeringAngleOutputPin);

		//Speed Input
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
		RETURN_IF_POINTER_NULL(strDescSignalValue);
		cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionSignalValue));

		RETURN_IF_FAILED(m_oInputSpeedController.Create("SpeedcontrollerIn", pTypeSignalValue, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oInputSpeedController));
	}
	else if (eStage == StageNormal)
	{
		//Counter of not detected lines
		faultyLineCounterRight = -1;
		faultyLineCounterMiddle = -1;
		faultyLineCounterLeft = -1;
		oldAngleRight = 0;
		oldAngleMiddle = 0;
		oldAngleLeft = 0;
		goThroughCounter = 0;
		rightXPos = 685;
		middleXPos = 572;//567
		leftXPos = 441;
		oldXPosRight = rightXPos;
		oldXPosMiddle = middleXPos;
		oldXPosLeft = leftXPos;
		walkThroughCounter_steeringAngle = 0;
		wheelCountLeft = 0;
		wheelCountRight = 0;
		wheelTickSave = 1000000;
		doManeuver = true;
		maneuverNotSet = true;
		deviationSum = 0;
		xPosDeviationSum = 0;
		oldxPosDeviation = 0;
		oldTime = 0;
		oldAngle = 0;
		firstGoThrough = true;
		lambda.create(2, 4, CV_32FC1);

		EMERGENCY_SEARCH = false;
		EME_SEARCH_NO_STREET_COUNTER = -1;
		last_right_line = Vec4i(0, 0, 0, 0);
		last_middle_line = Vec4i(0, 0, 0, 0);
		last_left_line = Vec4i(0, 0, 0, 0);

		parkingSpaceBeginTicks = 0;
		parkingEndCounter = 0;
		m_szIdsUsStructSet = tFalse;
		isItEmptyCounter = 0;
		lastPedestianCrossingDetected = 0;
		m_stManTicks = INT32_MAX;
		lastSpeedSend = NO_LD_SPEED;
		lastSteeringangle = 0;

		nextManeuverRight = false;
		nextManeuverLeft = false;
		nextManeuverStraight = false;
		nextManeuverParking = false;
		nextManeuverParkingSpaceSearch = false;
		nextManeuverParkingOutLeft = false;
		nextManeuverParkingOutRight = false;

		lastTurnLeftOnSend = false;
		lastTurnRightOnSend = false;

		isOnLeftLane = false;
		overtakingTicks = 100000000000;

		parkingTestRunning = false;
	}
	else if (eStage == StageGraphReady)
	{
		// get the image format of the input video pin
		cObjectPtr<IMediaType> pType;
		RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));

		cObjectPtr<IMediaTypeVideo> pTypeVideo;
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

		// set the image format of the input videinto pin
		if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
		{
			//LOG_ERROR("Invalid Input Format for this filter");
		}
	}

	RETURN_NOERROR;
}



tResult cLaneDetection::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	if (eStage == StageGraphReady)
	{
	}

	return cFilter::Shutdown(eStage, __exception_ptr);
}
tResult cLaneDetection::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if (pSource == &m_oVideoInputPin)
		{
			if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
			{
				RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
			}

			ProcessVideo(pMediaSample);
		}
		else if (pSource == &m_oInputWheelLeft)
		{
			RETURN_IF_FAILED(ProcessWheelSampleLeft(pMediaSample));
		}
		else if (pSource == &m_oInputWheelRight)
		{
			RETURN_IF_FAILED(ProcessWheelSampleRight(pMediaSample));
		}
		else if (pSource == &m_oInputSpeedController)
		{
			RETURN_IF_FAILED(GetSpeed(pMediaSample));
		}
		else if (pSource == &m_oInputUsStruct)
		{
			//if mediasample is from type ultrasonic detect the minimum ultrasonic value
			ProcessMinimumValueUs(pMediaSample);
		}
	}
	else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
	{
		if (pSource == &m_oVideoInputPin)
		{
			//the input format was changed, so the imageformat has to changed in this filter also
			RETURN_IF_FAILED(UpdateInputImageFormat(m_oVideoInputPin.GetFormat()));
		}
	}

	RETURN_NOERROR;
}

tResult cLaneDetection::ProcessWheelSampleLeft(IMediaSample* pMediaSample)
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
	RETURN_NOERROR;
}



tResult cLaneDetection::ProcessWheelSampleRight(IMediaSample* pMediaSample)
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
	RETURN_NOERROR;
}

tResult cLaneDetection::ProcessMinimumValueUs(IMediaSample* pMediaSample)
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

tResult cLaneDetection::ProcessVideo(IMediaSample* pSample)
{
	RETURN_IF_POINTER_NULL(pSample);
	const tVoid* l_pSrcBuffer;
	cv::Mat outputImage;
	vector<Vec4i> lines;
	vector<Point> rightCrossPoints;
	vector<Point> leftCrossPoints;
	Vec4i nearestLineToCarRight = Vec4i(0, 0, 0, 0);
	Vec4i nearestLineToCarMiddle = Vec4i(0, 0, 0, 0);
	Vec4i nearestLineToCarLeft = Vec4i(0, 0, 0, 0);
	Vec4i gapVectorVertical = Vec4i(0, 0, 0, 0);
	Vec4i gapVectorHorizontal = Vec4i(0, 0, 0, 0);
	//receiving data from input sample, and saving to TheInputImage
	if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
	{

		//convert to mat, be sure to select the right pixelformat
		if (tInt32(m_inputImage.total() * m_inputImage.elemSize()) == m_sInputFormat.nSize)
		{
			m_inputImage.data = (uchar*)(l_pSrcBuffer);
			detectLines(outputImage, lines);
		}
	}
	//detect all Lines in the image
	pSample->Unlock(l_pSrcBuffer);


	//compute left, middle and right lane from the "lines"-vector of detectLines()
	detectLane(outputImage, lines, nearestLineToCarRight, nearestLineToCarMiddle, nearestLineToCarLeft);
	// detect crossPoints


	tFloat32 sAngle = 0;

	//Wir haben keine Stra�e
	if (EME_SEARCH_NO_STREET_COUNTER > 5) {

		// Wenn wir zu weit links sind
		if (last_right_line[0] > m_filterProperties.RightLowerRightX || computeAngleFromVec4i(last_right_line) >= 10.0f) {
			// fahren wir nach rechts
			sAngle = 100.0f;
		}
		// Wenn wir zu weit rechts sind
		else {
			// fahren wir nach links
			sAngle = -100.0f;
		}
	}
	else {

		// Wir haben eine Stra�e alles ist super
		sAngle = computeSteeringAngle(nearestLineToCarLeft, nearestLineToCarMiddle, nearestLineToCarRight, outputImage);
	}
	transmitSteeringAngle(sAngle);

	if (!outputImage.empty() && m_oVideoOutputPin.IsConnected() /*&& DEBUG*/)
	{
		UpdateOutputImageFormat(outputImage);

		//create a cImage from CV Matrix (not necessary, just for demonstration9
		cImage newImage;
		newImage.Create(m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBitsPerPixel, m_sOutputFormat.nBytesPerLine, outputImage.data);

		//create the new media sample
		cObjectPtr<IMediaSample> pMediaSample;
		RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
		//updating media sample
		RETURN_IF_FAILED(pMediaSample->Update(_clock->GetStreamTime(), newImage.GetBitmap(), newImage.GetSize(), IMediaSample::MSF_None));
		//transmitting
		RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));

		outputImage.release();
	}

	//if (m_oGCLOutputPin.IsConnected()) transmitGCL(detectionLines, detectedLinePoints, detectedLines);


	RETURN_NOERROR;
}

tResult cLaneDetection::detectLines(Mat &outputImage, vector<Vec4i> &detectedLines) {

	//    Mat outputImage_hsv;

	//    Mat lower_yellow(3, 1, m_inputImage.type());
	//    lower_yellow.at<uchar>(0,0) = 20;
	//    lower_yellow.at<uchar>(1,0) = 100;
	//    lower_yellow.at<uchar>(2,0) = 100;

	//    Mat upper_yellow(3, 1, m_inputImage.type());
	//    upper_yellow.at<uchar>(0,0) = 30;
	//    upper_yellow.at<uchar>(1,0) = 255;
	//    upper_yellow.at<uchar>(2,0) = 255;

	//    Mat mask_yellow, mask_white, mask_yw, mask_roi;
	//    mask_yellow.create(m_inputImage.size(), m_inputImage.type());
	//    mask_white.create(m_inputImage.size(), m_inputImage.type());
	//    mask_yw.create(m_inputImage.size(), m_inputImage.type());
	cvtColor(m_inputImage, outputImage, CV_RGB2GRAY);

	static Mat transformation_x, transformation_y;
	if (firstGoThrough) {
		Point2f inputQuad[4];
		Point2f outputQuad[4];
		lambda = Mat::zeros(outputImage.rows, outputImage.cols, outputImage.type());
		inputQuad[0] = Point2f(m_filterProperties.warpLeft, 500);
		inputQuad[1] = Point2f(m_filterProperties.warpRight, 500);
		inputQuad[2] = Point2f(outputImage.cols, 590);
		inputQuad[3] = Point2f(0, 590);              //
		outputQuad[0] = Point2f(0, 0);
		outputQuad[1] = Point2f(outputImage.cols, 0);
		outputQuad[2] = Point2f(outputImage.cols - 280, outputImage.rows);
		outputQuad[3] = Point2f(280, outputImage.rows);
		lambda = getPerspectiveTransform(inputQuad, outputQuad);
		perspective_to_maps(lambda, outputImage.size(), transformation_x, transformation_y);
		Mat inverseLambda;
		invert(lambda, inverseLambda);
		Mat map_x, map_y, srcTM;
		srcTM = inverseLambda.clone();
		map_x.create(outputImage.size(), CV_32FC1);
		map_y.create(outputImage.size(), CV_32FC1);
		double M11, M12, M13, M21, M22, M23, M31, M32, M33;
		M11 = srcTM.at<double>(0, 0);
		M12 = srcTM.at<double>(0, 1);
		M13 = srcTM.at<double>(0, 2);
		M21 = srcTM.at<double>(1, 0);
		M22 = srcTM.at<double>(1, 1);
		M23 = srcTM.at<double>(1, 2);
		M31 = srcTM.at<double>(2, 0);
		M32 = srcTM.at<double>(2, 1);
		M33 = srcTM.at<double>(2, 2);
		for (int y = 0; y < outputImage.rows; y++) {
			double fy = (double)y;
			for (int x = 0; x < outputImage.cols; x++) {
				double fx = (double)x;
				double w = ((M31 * fx) + (M32 * fy) + M33);
				w = w != 0.0f ? 1.f / w : 0.0f;
				float new_x = (float)((M11 * fx) + (M12 * fy) + M13) * w;
				float new_y = (float)((M21 * fx) + (M22 * fy) + M23) * w;
				map_x.at<float>(y, x) = new_x;
				map_y.at<float>(y, x) = new_y;
			}
		}
		transformation_x.create(outputImage.size(), CV_16SC2);
		transformation_y.create(outputImage.size(), CV_16UC1);
		cv::convertMaps(map_x, map_y, transformation_x, transformation_y, false);
		firstGoThrough = false;
	}

	//inputImage = Mat(m_sInputFormat.nHeight, m_sInputFormat.nWidth, CV_8UC3, (tVoid*)l_pSrcBuffer, m_sInputFormat.nBytesPerLine);

	//warpPerspective(outputImage,outputImage,lambda,outputImage.size());
	remap(outputImage, outputImage, transformation_x, transformation_y, CV_INTER_LINEAR);
	//resize(outputImage, outputImage, Size2d(outputImage.cols/2, outputImage.rows/2));

	//cvtColor(outputImage, outputImage_hsv, CV_RGB2HSV);
	//inRange(outputImage_hsv, lower_yellow, upper_yellow, mask_yellow); //TODO yellow reparieren
	//inRange(outputImage, 230, 255, mask_white);
	//bitwise_or(mask_yellow, mask_white, mask_yw);
	//bitwise_and(outputImage, mask_white, outputImage);

	if (EMERGENCY_SEARCH) {
		//GaussianBlur(outputImage, outputImage, Size(3, 3), 0, 0);
		Canny(outputImage, outputImage, m_filterProperties.thresholdImageBinarization, m_filterProperties.thresholdImageBinarization * 2, 3);
		HoughLinesP(outputImage, detectedLines, m_filterProperties.HLrho, m_filterProperties.HLtheta, m_filterProperties.HLthreshold, m_filterProperties.HLminLineLenght, m_filterProperties.HLmaxLineGap);
	}
	else {
		Rect houghLinesRect(Point(m_filterProperties.fourCrossUpperRightX, m_filterProperties.fourCrossUpperRightY), (Point(m_filterProperties.fourCrossLowerLeftX, m_filterProperties.fourCrossLowerLeftY)));
		Mat out = outputImage(houghLinesRect);

		//GaussianBlur(out, out, Size(3, 3), 0, 0);

		//        if(m_filterProperties.testBool)
		//            threshold(out,out,0,255,CV_THRESH_BINARY | CV_THRESH_OTSU);

		Canny(out, out, m_filterProperties.thresholdImageBinarization, m_filterProperties.thresholdImageBinarization * 2, 3);
		HoughLinesP(out, detectedLines, m_filterProperties.HLrho, m_filterProperties.HLtheta, m_filterProperties.HLthreshold, m_filterProperties.HLminLineLenght, m_filterProperties.HLmaxLineGap);

		for (unsigned int i = 0; i < detectedLines.size(); i++) {
			Vec4i tmp = detectedLines[i];
			tmp[0] += m_filterProperties.fourCrossLowerLeftX;
			tmp[1] += m_filterProperties.fourCrossUpperRightY;
			tmp[2] += m_filterProperties.fourCrossLowerLeftX;
			tmp[3] += m_filterProperties.fourCrossUpperRightY;

			detectedLines[i] = tmp;
		}
	}

	setStartEndPoint(detectedLines);


	RETURN_NOERROR;
}

tResult cLaneDetection::perspective_to_maps(const cv::Mat &perspective_mat, const cv::Size &img_size,
	cv::Mat &map1, cv::Mat &map2)
{
	// invert the matrix because the transformation maps must be
	// bird's view -> original
	cv::Mat inv_perspective(perspective_mat.inv());
	inv_perspective.convertTo(inv_perspective, CV_32FC1);

	// create XY 2D array
	// (((0, 0), (1, 0), (2, 0), ...),
	//  ((0, 1), (1, 1), (2, 1), ...),
	// ...)
	cv::Mat xy(img_size, CV_32FC2);
	float *pxy = (float*)xy.data;
	for (int y = 0; y < img_size.height; y++)
		for (int x = 0; x < img_size.width; x++)
		{
			*pxy++ = x;
			*pxy++ = y;
		}

	// perspective transformation of the points
	cv::Mat xy_transformed;
	cv::perspectiveTransform(xy, xy_transformed, inv_perspective);

	// split x/y to extra maps
	assert(xy_transformed.channels() == 2);
	cv::Mat maps[2]; // map_x, map_y
	cv::split(xy_transformed, maps);

	// remap() with integer maps is faster
	cv::convertMaps(maps[0], maps[1], map1, map2, CV_16SC2);
	RETURN_NOERROR;
}



tResult cLaneDetection::detectLane(Mat & outputImage, vector<Vec4i> &lines, Vec4i &nearestLineToCarRight, Vec4i &nearestLineToCarMiddle, Vec4i &nearestLineToCarLeft) {
	vector<Vec4i> leftLine;
	vector<Vec4i> middleLine;
	vector<Vec4i> rightLine;

	int oldAngleOffset = 25;
	int oldXPosOffset = 25;
	int angleOffsetEMER = 5;
	int leftToMid = 117;
	int MidToRight = 113;
	int distanceOffset = 10;
	int distanceOffsetY = 50;

	if (EMERGENCY_SEARCH)
	{
		for (unsigned int i = 0; i < lines.size(); i++) {
			int xVal = lines[i][0];
			int yVal = lines[i][1];
			float angleLeft = computeAngleFromVec4i(lines[i]);
			for (unsigned int j = 0; j < lines.size(); j++) {
				float angleMiddle = computeAngleFromVec4i(lines[j]);
				if (angleLeft < angleMiddle + angleOffsetEMER && angleLeft > angleMiddle - angleOffsetEMER &&
					(lines[j][0] - lines[i][0]) > leftToMid - distanceOffset && (lines[j][0] - lines[i][0]) < leftToMid + distanceOffset &&
					(lines[j][1] < lines[i][1] + distanceOffsetY && lines[j][1] > lines[i][1] - distanceOffsetY)) {
					for (unsigned int k = 0; k < lines.size(); k++) {
						float angleRight = computeAngleFromVec4i(lines[k]);
						if (angleMiddle < angleRight + angleOffsetEMER && angleMiddle > angleRight - angleOffsetEMER &&
							(lines[k][0] - lines[j][0]) > MidToRight - distanceOffset && (lines[k][0] - lines[j][0]) < MidToRight + distanceOffset &&
							(lines[k][1] < lines[j][1] + distanceOffsetY && lines[k][1] > lines[j][1] - distanceOffsetY)) {
							nearestLineToCarLeft = lines[i];
							nearestLineToCarMiddle = lines[j];
							nearestLineToCarRight = lines[k];
							last_right_line = lines[k];
							last_middle_line = lines[j];
							last_left_line = lines[i];
							LOG_INFO(cString::Format("EME ANGLES: %f %f %f", angleLeft, angleMiddle, angleLeft));
							break;
						}
					}
				}
			}
			if ((xVal > m_filterProperties.RightUpperLeftX) && (xVal < m_filterProperties.RightUpperRightX) &&
				(yVal < m_filterProperties.RightLowerLeftY) && (yVal > m_filterProperties.RightUpperRightY)) {
				rightLine.push_back(lines[i]);
			}
			else if ((xVal > m_filterProperties.MidUpperLeftX) && (xVal < m_filterProperties.MidUpperRightX) &&
				(yVal < m_filterProperties.MidLowerLeftY) && (yVal > m_filterProperties.MidUpperRightY)) {
				middleLine.push_back(lines[i]);
			}
			else if ((xVal > m_filterProperties.LeftUpperLeftX) && (xVal < m_filterProperties.LeftUpperRightX) &&
				(yVal < m_filterProperties.LeftLowerLeftY) && (yVal > m_filterProperties.LeftUpperRightY)) {
				leftLine.push_back(lines[i]);
			}
		}
		// Keine Stra�e gefunden
		if (nearestLineToCarRight == Vec4i(0, 0, 0, 0)) {
			EME_SEARCH_NO_STREET_COUNTER++;
		}
		else {
			EME_SEARCH_NO_STREET_COUNTER = -1;
		}
		if (!rightLine.empty() && !middleLine.empty() && !leftLine.empty() && (((wheelCountLeft + wheelCountRight) / 2) > (lastPedestianCrossingDetected + 200))) {
			EMERGENCY_SEARCH = false;
			EME_SEARCH_NO_STREET_COUNTER = -1;
			last_right_line = Vec4i(0, 0, 0, 0);
			last_middle_line = Vec4i(0, 0, 0, 0);
			last_left_line = Vec4i(0, 0, 0, 0);
		}
	}
	else {
		static int run = 0;
		for (unsigned int i = 0; i < lines.size(); i++)
		{
			int xVal = lines[i][0];
			int yVal = lines[i][1];
			float angle = computeAngleFromVec4i(lines[i]);
			if (angle < m_filterProperties.maxAngle && angle > m_filterProperties.minAngle) {
				//Offset if poly ROI
				float offsetRight = 0;
				if (m_filterProperties.RightLowerLeftX - m_filterProperties.RightUpperLeftX != 0) {
					offsetRight = float(m_filterProperties.RightUpperLeftX - m_filterProperties.RightLowerLeftX) / (m_filterProperties.RightLowerLeftY - m_filterProperties.RightUpperLeftY) * (yVal - m_filterProperties.RightUpperLeftY);
				}
				float offsetMid = 0;
				if (m_filterProperties.MidLowerLeftX - m_filterProperties.MidUpperLeftX != 0) {
					offsetMid = float(m_filterProperties.MidUpperLeftX - m_filterProperties.MidLowerLeftX) / (m_filterProperties.MidLowerLeftY - m_filterProperties.MidUpperLeftY) * (yVal - m_filterProperties.MidUpperLeftY);
				}
				float offsetLeft = 0;
				if (m_filterProperties.LeftLowerLeftX - m_filterProperties.LeftUpperLeftX != 0) {
					offsetLeft = float(m_filterProperties.LeftUpperLeftX - m_filterProperties.LeftLowerLeftX) / (m_filterProperties.LeftLowerLeftY - m_filterProperties.LeftUpperLeftY) * (yVal - m_filterProperties.LeftUpperLeftY);
				}

				LOG_INFO(cString::Format("run %d; xval %d; y val %d",run, xVal, yVal));
				//!!RECHTE LINIE
				//Betrachte die Linie wenn sie im ROI ist
				if ((xVal > m_filterProperties.RightUpperLeftX - offsetRight) && (xVal < m_filterProperties.RightUpperRightX - offsetRight) &&
					(yVal < m_filterProperties.RightLowerLeftY) && (yVal > m_filterProperties.RightUpperRightY)) {
					LOG_INFO("RIGHT LINE");
					//Nehme die Linie wh�rend der ersten 5 frames oder wenn seit N frames keine Linie mehr gefunden wurde
					if (goThroughCounter < 5 || faultyLineCounterRight > m_filterProperties.faultyCounterThreshold) {
						rightLine.push_back(lines[i]);
					}
					else {
						if (faultyLineCounterRight > 0) {
							if (faultyLineCounterMiddle == -1) {
								if ((oldAngleMiddle - oldAngleOffset < angle) && (angle < oldAngleMiddle + oldAngleOffset)) {
									rightLine.push_back(lines[i]);
								}
							}
							else if (faultyLineCounterLeft == -1) {
								if ((oldAngleLeft - oldAngleOffset < angle) && (angle < oldAngleLeft + oldAngleOffset)) {
									rightLine.push_back(lines[i]);
								}
							}
						}
						else {
							//nehme Linie wenn der Winkel und die Xposition nicht zu sehr von der alten Linie abweichen
							if ((oldAngleRight - oldAngleOffset < angle) && (angle < oldAngleRight + oldAngleOffset) &&
								(((oldXPosRight - oldXPosOffset < xVal) && (xVal < oldXPosRight + oldXPosOffset)) || oldXPosRight == 0)) {
								rightLine.push_back(lines[i]);
							}
						}
					}
				}
				//!!MITTLERE LINIE
				else if ((xVal > m_filterProperties.MidUpperLeftX - offsetMid) && (xVal < m_filterProperties.MidUpperRightX - offsetMid) &&
					(yVal < m_filterProperties.MidLowerLeftY) && (yVal > m_filterProperties.MidUpperRightY)) {
						LOG_INFO("MIDDLE LINE");
					if (goThroughCounter < 5 || faultyLineCounterMiddle > m_filterProperties.faultyCounterThreshold) {
						middleLine.push_back(lines[i]);
					}
					else {
						if (faultyLineCounterMiddle > 0) {
							if (faultyLineCounterRight == -1) {
								if ((oldAngleRight - oldAngleOffset < angle) && (angle < oldAngleRight + oldAngleOffset)) {
									middleLine.push_back(lines[i]);
								}
							}
							else if (faultyLineCounterLeft == -1) {
								if ((oldAngleLeft - oldAngleOffset < angle) && (angle < oldAngleLeft + oldAngleOffset)) {
									middleLine.push_back(lines[i]);
								}
							}
						}
						else {
							if ((oldAngleMiddle - oldAngleOffset < angle) && (angle < oldAngleMiddle + oldAngleOffset) &&
								(((oldXPosMiddle - oldXPosOffset < xVal) && (xVal < oldXPosMiddle + oldXPosOffset)) || oldXPosMiddle == 0)) {
								middleLine.push_back(lines[i]);
							}
						}
					}
				}
				//!!LINKE LINIE
				else if ((xVal > m_filterProperties.LeftUpperLeftX - offsetLeft) && (xVal < m_filterProperties.LeftUpperRightX - offsetLeft) &&
					(yVal < m_filterProperties.LeftLowerLeftY) && (yVal > m_filterProperties.LeftUpperRightY)) {
						LOG_INFO("LEFT LINE");
					if (goThroughCounter < 5 || faultyLineCounterLeft > m_filterProperties.faultyCounterThreshold) {
						leftLine.push_back(lines[i]);
					}
					else {
						if (faultyLineCounterLeft > 0) {
							if (faultyLineCounterRight == -1) {
								if ((oldAngleRight - oldAngleOffset < angle) && (angle < oldAngleRight + oldAngleOffset)) {
									leftLine.push_back(lines[i]);
								}
							}
							else if (faultyLineCounterMiddle == -1) {
								if ((oldAngleMiddle - oldAngleOffset < angle) && (angle < oldAngleMiddle + oldAngleOffset)) {
									leftLine.push_back(lines[i]);
								}
							}
						}
						else {
							if ((oldAngleLeft - oldAngleOffset < angle) && (angle < oldAngleLeft + oldAngleOffset) &&
								(((oldXPosLeft - oldXPosOffset < xVal) && (xVal < oldXPosLeft + oldXPosOffset)) || oldXPosLeft == 0)) {
								leftLine.push_back(lines[i]);
							}
						}
					}
				}
			}
		}

		run++;



		if (DEBUG) {


			for (unsigned int i = 0; i < leftLine.size(); i++) {
				cv::line(outputImage, Point(leftLine[i][0], leftLine[i][1]), Point(leftLine[i][2], leftLine[i][3]), Scalar(100, 150, 100), 3, CV_AA);
			}
			for (unsigned int i = 0; i < middleLine.size(); i++) {
				cv::line(outputImage, Point(middleLine[i][0], middleLine[i][1]), Point(middleLine[i][2], middleLine[i][3]), Scalar(100, 150, 100), 3, CV_AA);
			}
			for (unsigned int i = 0; i < rightLine.size(); i++) {
				cv::line(outputImage, Point(rightLine[i][0], rightLine[i][1]), Point(rightLine[i][2], rightLine[i][3]), Scalar(100, 150, 100), 3, CV_AA);
			}

			// Draw Left ROI
			cv::line(outputImage, Point(m_filterProperties.LeftUpperLeftX, m_filterProperties.LeftUpperLeftY), Point(m_filterProperties.LeftUpperRightX, m_filterProperties.LeftUpperRightY), 100, 2);
			cv::line(outputImage, Point(m_filterProperties.LeftUpperRightX, m_filterProperties.LeftUpperRightY), Point(m_filterProperties.LeftLowerRightX, m_filterProperties.LeftLowerRightY), 100, 2);
			cv::line(outputImage, Point(m_filterProperties.LeftLowerRightX, m_filterProperties.LeftLowerRightY), Point(m_filterProperties.LeftLowerLeftX, m_filterProperties.LeftLowerLeftY), 100, 2);
			cv::line(outputImage, Point(m_filterProperties.LeftLowerLeftX, m_filterProperties.LeftLowerLeftY), Point(m_filterProperties.LeftUpperLeftX, m_filterProperties.LeftUpperLeftY), 100, 2);

			// Draw Middle ROI
			cv::line(outputImage, Point(m_filterProperties.MidUpperLeftX, m_filterProperties.MidUpperLeftY), Point(m_filterProperties.MidUpperRightX, m_filterProperties.MidUpperRightY), 100, 2);
			cv::line(outputImage, Point(m_filterProperties.MidUpperRightX, m_filterProperties.MidUpperRightY), Point(m_filterProperties.MidLowerRightX, m_filterProperties.MidLowerRightY), 100, 2);
			cv::line(outputImage, Point(m_filterProperties.MidLowerRightX, m_filterProperties.MidLowerRightY), Point(m_filterProperties.MidLowerLeftX, m_filterProperties.MidLowerLeftY), 100, 2);
			cv::line(outputImage, Point(m_filterProperties.MidLowerLeftX, m_filterProperties.MidLowerLeftY), Point(m_filterProperties.MidUpperLeftX, m_filterProperties.MidUpperLeftY), 100, 2);

			//Draw Right ROI
			cv::line(outputImage, Point(m_filterProperties.RightUpperLeftX, m_filterProperties.RightUpperLeftY), Point(m_filterProperties.RightUpperRightX, m_filterProperties.RightUpperRightY), 100, 2);
			cv::line(outputImage, Point(m_filterProperties.RightUpperRightX, m_filterProperties.RightUpperRightY), Point(m_filterProperties.RightLowerRightX, m_filterProperties.RightLowerRightY), 100, 2);
			cv::line(outputImage, Point(m_filterProperties.RightLowerRightX, m_filterProperties.RightLowerRightY), Point(m_filterProperties.RightLowerLeftX, m_filterProperties.RightLowerLeftY), 100, 2);
			cv::line(outputImage, Point(m_filterProperties.RightLowerLeftX, m_filterProperties.RightLowerLeftY), Point(m_filterProperties.RightUpperLeftX, m_filterProperties.RightUpperLeftY), 100, 2);

		}

		unsigned int maxLinesInROI = 100;
		if (!leftLine.empty() && leftLine.size() < maxLinesInROI) {
			leftLineBuffer = leftLine;
			faultyLineCounterLeft = -1;
		}
		else if (faultyLineCounterLeft == -1) {
			faultyLineCounterLeft = (wheelCountLeft + wheelCountRight) / 2;
		}

		if (!middleLine.empty() && middleLine.size() < maxLinesInROI) {
			middleLineBuffer = middleLine;
			faultyLineCounterMiddle = -1;
		}
		else if (faultyLineCounterMiddle == -1) {
			faultyLineCounterMiddle = (wheelCountLeft + wheelCountRight) / 2;
		}

		if (!rightLine.empty() && rightLine.size() < maxLinesInROI) {
			rightLineBuffer = rightLine;
			faultyLineCounterRight = -1;
		}
		else if (faultyLineCounterRight == -1) {
			faultyLineCounterRight = (wheelCountLeft + wheelCountRight) / 2;
		}



		// Wir haben eine stra�e
		if (faultyLineCounterLeft == -1 && faultyLineCounterMiddle == -1 && faultyLineCounterRight == -1) {
			last_left_line = nearestLineToCarLeft;
			last_middle_line = nearestLineToCarMiddle;
			last_right_line = nearestLineToCarRight;
			//LOG_INFO(cString::Format("LAST_LANE %f %f %f", computeAngleFromVec4i(last_left_line), computeAngleFromVec4i(last_middle_line), computeAngleFromVec4i(last_right_line)));
		}

		if ((wheelCountLeft + wheelCountRight) / 2 - faultyLineCounterLeft > m_filterProperties.emergencyThreshold && faultyLineCounterLeft != -1 &&
			(wheelCountLeft + wheelCountRight) / 2 - faultyLineCounterMiddle > m_filterProperties.emergencyThreshold && faultyLineCounterMiddle != -1 &&
			(wheelCountLeft + wheelCountRight) / 2 - faultyLineCounterRight > m_filterProperties.emergencyThreshold && faultyLineCounterRight != -1) {
			//LOG_INFO("Obacht!");
			EMERGENCY_SEARCH = true;
		}

		adaptROI(nearestLineToCarRight, nearestLineToCarMiddle, nearestLineToCarLeft);
	}
	if (DEBUG) {
		cv::line(outputImage, Point(nearestLineToCarRight[0], nearestLineToCarRight[1]), Point(nearestLineToCarRight[2], nearestLineToCarRight[3]), 255, 3, CV_AA);
		cv::line(outputImage, Point(nearestLineToCarLeft[0], nearestLineToCarLeft[1]), Point(nearestLineToCarLeft[2], nearestLineToCarLeft[3]), 255, 3, CV_AA);
		cv::line(outputImage, Point(nearestLineToCarMiddle[0], nearestLineToCarMiddle[1]), Point(nearestLineToCarMiddle[2], nearestLineToCarMiddle[3]), 255, 3, CV_AA);
	}

	//////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	/// Beim f�nften Durchlauf werden die Werte der aktuellen nearestLineToCar gespeichert
	Vec4i Test;
	Test.zeros();
	if (goThroughCounter >= 5) {
		oldAngleRight = computeAngleFromVec4i(nearestLineToCarRight);
		oldAngleMiddle = computeAngleFromVec4i(nearestLineToCarMiddle);
		oldAngleLeft = computeAngleFromVec4i(nearestLineToCarLeft);

		oldXPosRight = nearestLineToCarRight[0];
		oldXPosMiddle = nearestLineToCarMiddle[0];
		oldXPosLeft = nearestLineToCarLeft[0];
	}

	goThroughCounter++;

	RETURN_NOERROR;
}

tResult cLaneDetection::setStartEndPoint(vector<Vec4i> &lines) {
	for (unsigned int i = 0; i < lines.size(); i++) {
		//wenn yStart kleiner als yEnd dann tausche start und endpunkt
		if (lines[i][1] < lines[i][3]) {
			swap(lines[i][0], lines[i][2]);
			swap(lines[i][1], lines[i][3]);
		}
	}
	RETURN_NOERROR;
}

tFloat32 cLaneDetection::computeAngleFromVec4i(const Vec4i vector) {
	//endpointX - startpoinX
	int x = vector[2] - vector[0];

	//endpointY - startpoinY
	int y = vector[3] - vector[1];

	if (x == 0 && y == 0) {
		return 0;
	}
	return 180 * asin(x / (sqrt(x*x + y * y))) / 3.14159;
}

tFloat32 cLaneDetection::computeRadianFromVec4i(const Vec4i vector) {
	//endpointX - startpoinX
	int x = vector[2] - vector[0];
	//endpointY - startpoinY
	int y = vector[3] - vector[1];

	return asin(x / (sqrt(x*x + y * y)));
}

tResult cLaneDetection::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		//update member variable
		m_sInputFormat = (*pFormat);
		//LOG_INFO(adtf_util::cString::Format("Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
		//create the input matrix
		RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
	}
	RETURN_NOERROR;
}

tFloat32 cLaneDetection::computeSteeringAngle(Vec4i leftLane, Vec4i middleLane, Vec4i rightLane, Mat &outputImage) {
	int bufferSize = 3;

	int rightX = 0;
	int middleX = 0;
	int leftX = 0;
	int laneXPosSumRight = 0;
	int laneXPosSumMiddle = 0;
	int laneXPosSumLeft = 0;
	tFloat32 laneAngleSumRight = 0;
	tFloat32 laneAngleSumMiddle = 0;
	tFloat32 laneAngleSumLeft = 0;

	tFloat32 laneAngleMiddle = computeRadianFromVec4i(middleLane);
	tFloat32 laneAngleRight = computeRadianFromVec4i(rightLane);
	tFloat32 laneAngleLeft = computeRadianFromVec4i(leftLane);

	tFloat32 laneAngleDif = 0;

	//neu
	float xPosDeviation = 0;
	float angleDeviation = 0;

	//Durchschnitt der letzten *bufferSize* X Koordinaten und Winkel der Fahrbahnlinien bilden um Schwankungen zu vermindern
	if (walkThroughCounter_steeringAngle <= bufferSize) {
		laneBufferRight.insert(laneBufferRight.begin(), rightLane[0]);
		laneBufferMiddle.insert(laneBufferMiddle.begin(), middleLane[0]);
		laneBufferLeft.insert(laneBufferLeft.begin(), leftLane[0]);

		angleBufferRight.insert(angleBufferRight.begin(), laneAngleRight);
		angleBufferMiddle.insert(angleBufferMiddle.begin(), laneAngleMiddle);
		angleBufferLeft.insert(angleBufferLeft.begin(), laneAngleLeft);

		walkThroughCounter_steeringAngle++;
	}
	else {
		laneBufferRight.insert(laneBufferRight.begin(), rightLane[0]);
		laneBufferMiddle.insert(laneBufferMiddle.begin(), middleLane[0]);
		laneBufferLeft.insert(laneBufferLeft.begin(), leftLane[0]);
		laneBufferRight.pop_back();
		laneBufferMiddle.pop_back();
		laneBufferLeft.pop_back();

		angleBufferRight.insert(angleBufferRight.begin(), laneAngleRight);
		angleBufferMiddle.insert(angleBufferMiddle.begin(), laneAngleMiddle);
		angleBufferLeft.insert(angleBufferLeft.begin(), laneAngleLeft);
		angleBufferRight.pop_back();
		angleBufferMiddle.pop_back();
		angleBufferLeft.pop_back();

		for (int i = 0; i < bufferSize; i++) {
			laneXPosSumRight += laneBufferRight[i];
			laneXPosSumMiddle += laneBufferMiddle[i];
			laneXPosSumLeft += laneBufferLeft[i];

			laneAngleSumRight += angleBufferRight[i];
			laneAngleSumMiddle += angleBufferMiddle[i];
			laneAngleSumLeft += angleBufferLeft[i];
		}
		rightLane[0] = laneXPosSumRight / bufferSize;
		middleLane[0] = laneXPosSumMiddle / bufferSize;
		leftLane[0] = laneXPosSumLeft / bufferSize;

		laneAngleRight = laneAngleSumRight / bufferSize;
		laneAngleMiddle = laneAngleSumMiddle / bufferSize;
		laneAngleLeft = laneAngleSumLeft / bufferSize;
	}

	if (((computeAngleFromVec4i(rightLane) + computeAngleFromVec4i(middleLane) + computeAngleFromVec4i(leftLane)) / 3 > -20.0)
		&& ((computeAngleFromVec4i(rightLane) + computeAngleFromVec4i(middleLane) + computeAngleFromVec4i(leftLane)) / 3 < 20.0)) {
		//LOG_INFO("GERADE");
		if (faultyLineCounterMiddle == -1) {
			angleDeviation = computeAngleFromVec4i(middleLane);
			laneAngleDif = -laneAngleMiddle;
			middleX = middleXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
			xPosDeviation = middleLane[0] - middleX;
		}
		else if (faultyLineCounterRight == -1) {
			angleDeviation = computeAngleFromVec4i(rightLane);

			laneAngleDif = -laneAngleRight;
			rightX = rightXPos - sin(laneAngleDif) * m_filterProperties.FovCurveLeft;
			xPosDeviation = rightLane[0] - rightX;
		}
		else {
			angleDeviation = computeAngleFromVec4i(leftLane);

			laneAngleDif = -laneAngleLeft;
			leftX = leftXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
			xPosDeviation = leftLane[0] - leftX;
		}

	}
	else if ((computeAngleFromVec4i(rightLane) + computeAngleFromVec4i(middleLane) + computeAngleFromVec4i(leftLane)) / 3 <= -20.0) {
		//LOG_INFO("LINKS");

		if (faultyLineCounterRight == -1) {
			angleDeviation = computeAngleFromVec4i(rightLane);

			laneAngleDif = -laneAngleRight;
			rightX = rightXPos - sin(laneAngleDif) * m_filterProperties.FovCurveLeft;
			xPosDeviation = rightLane[0] - rightX;
		}
		else if (faultyLineCounterMiddle == -1) {
			angleDeviation = computeAngleFromVec4i(middleLane);

			laneAngleDif = -laneAngleMiddle;
			middleX = middleXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
			xPosDeviation = middleLane[0] - middleX;
		}
		else {
			angleDeviation = computeAngleFromVec4i(leftLane);

			laneAngleDif = -laneAngleLeft;
			leftX = leftXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
			xPosDeviation = leftLane[0] - leftX;
		}
	}
	else {
		//LOG_INFO("RECHTS");
		if (faultyLineCounterLeft == -1) {
			angleDeviation = computeAngleFromVec4i(leftLane);

			laneAngleDif = -laneAngleLeft;
			leftX = leftXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
			xPosDeviation = leftLane[0] - leftX;
		}
		else if (faultyLineCounterMiddle == -1) {
			angleDeviation = computeAngleFromVec4i(middleLane);

			laneAngleDif = -laneAngleMiddle;
			middleX = middleXPos - sin(laneAngleDif) * m_filterProperties.FovCurveRight;
			xPosDeviation = middleLane[0] - middleX;
		}
		else {
			angleDeviation = computeAngleFromVec4i(rightLane);

			laneAngleDif = -laneAngleRight;
			rightX = rightXPos - sin(laneAngleDif) * m_filterProperties.FovCurveLeft;
			xPosDeviation = rightLane[0] - rightX;
		}
	}

	//    tUInt64 sampleTimeInt = cSystem::GetTime() - oldTime;
	//    float samleTimeSec = sampleTimeInt/1000000.0;
	//    if(samleTimeSec == 0)
	//        samleTimeSec = 1;
	int ticksSinceLastFrame = (wheelCountLeft + wheelCountRight) / 2 - oldTicks;

	if (carSpeed == 0) {
		xPosDeviationSum = 0;
	}

	xPosDeviationSum += xPosDeviation * ticksSinceLastFrame;
	float xPosDerivation = (xPosDeviation - oldxPosDeviation) / ticksSinceLastFrame;

	float steeringAngle = m_filterProperties.K1*angleDeviation
		+ m_filterProperties.K2*xPosDeviation
		+ m_filterProperties.K3*xPosDerivation
		+ m_filterProperties.K4*xPosDeviationSum;

	oldxPosDeviation = xPosDeviation;
	oldTicks = ticksSinceLastFrame;
	//    oldTime = cSystem::GetTime();

	if (steeringAngle != steeringAngle || steeringAngle > 10000 || steeringAngle < -10000) {
		steeringAngle = 0;
		//LOG_INFO("Steeringangle out of range!!!");
	}
	lastSteeringangle = steeringAngle;

	return steeringAngle;
}

tResult cLaneDetection::GetSpeed(IMediaSample* pMediaSample)
{
	tFloat32 speed;
	//sobald der block verlassen wird, wird das lock aufgehoben
	{
		//read lock
		__adtf_sample_read_lock_mediadescription(m_pDescriptionSignalValue, pMediaSample, pCoderInput);

		//Set all Ids
		if (!m_szIdInputSpeedSet)
		{
			pCoderInput->GetID("f32Value", m_szIdInputspeedControllerValue);
			//pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdInputspeedControllerTimeStamp);
			m_szIdInputSpeedSet = tTrue;
		}

		pCoderInput->Get(m_szIdInputspeedControllerValue, (tVoid*)&speed);
		//pCoderInput->Get(m_szIdInputspeedControllerTimeStamp, (tVoid*)&timestamp);
	}

	carSpeed = speed;
	RETURN_NOERROR;
}

tResult cLaneDetection::adaptROI(Vec4i &lineRight, Vec4i &lineMiddle, Vec4i &lineLeft) {

	tFloat32 laneAngleMiddle = computeRadianFromVec4i(lineMiddle);
	tFloat32 laneAngleRight = computeRadianFromVec4i(lineRight);
	tFloat32 laneAngleLeft = computeRadianFromVec4i(lineLeft);

	//TODO Default Werte aus File holen

	if ((wheelCountLeft + wheelCountRight) / 2 - faultyLineCounterRight > m_filterProperties.faultyCounterThreshold && faultyLineCounterRight != -1 || lineRight == Vec4i(0, 0, 0, 0)) { // TODO Offsets �ndern
		if (faultyLineCounterMiddle == -1) {
			m_filterProperties.RightUpperLeftX = 700 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
			m_filterProperties.RightUpperRightX = 780 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
			m_filterProperties.RightLowerLeftX = 700 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
			m_filterProperties.RightLowerRightX = 780 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
		}
		else if (faultyLineCounterLeft == -1) {
			m_filterProperties.RightUpperLeftX = 700 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
			m_filterProperties.RightUpperRightX = 780 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
			m_filterProperties.RightLowerLeftX = 700 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
			m_filterProperties.RightLowerRightX = 780 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
		}
		else {
			m_filterProperties.RightUpperLeftX = 700;
			m_filterProperties.RightUpperRightX = 780;
			m_filterProperties.RightLowerLeftX = 700;
			m_filterProperties.RightLowerRightX = 780;
			//m_filterProperties.RightUpperLeftX = 630;
			//m_filterProperties.RightUpperRightX = 710;
			//m_filterProperties.RightLowerLeftX = 630;
			//m_filterProperties.RightLowerRightX = 710;
		}
	}
	else {
		m_filterProperties.RightUpperLeftX = 700 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
		m_filterProperties.RightUpperRightX = 780 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
		m_filterProperties.RightLowerLeftX = 700 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
		m_filterProperties.RightLowerRightX = 780 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
	}

	if ((wheelCountLeft + wheelCountRight) / 2 - faultyLineCounterLeft > m_filterProperties.faultyCounterThreshold && faultyLineCounterLeft != -1 || lineLeft == Vec4i(0, 0, 0, 0)) {
		if (faultyLineCounterRight == -1) {
			m_filterProperties.LeftUpperLeftX = 350 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
			m_filterProperties.LeftUpperRightX = 430 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
			m_filterProperties.LeftLowerLeftX = 350 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
			m_filterProperties.LeftLowerRightX = 430 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
		}
		else if (faultyLineCounterMiddle == -1) {
			m_filterProperties.LeftUpperLeftX = 350 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
			m_filterProperties.LeftUpperRightX = 430 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
			m_filterProperties.LeftLowerLeftX = 350 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
			m_filterProperties.LeftLowerRightX = 430 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
		}
		else
		{
			m_filterProperties.LeftUpperLeftX = 350;
			m_filterProperties.LeftUpperRightX = 430;
			m_filterProperties.LeftLowerLeftX = 350;
			m_filterProperties.LeftLowerRightX = 430;
			//m_filterProperties.LeftUpperLeftX = 420;
			//m_filterProperties.LeftUpperRightX = 500;
			//m_filterProperties.LeftLowerLeftX = 420;
			//m_filterProperties.LeftLowerRightX = 500;
		}
	}
	else {
		m_filterProperties.LeftUpperLeftX = 350 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;;
		m_filterProperties.LeftUpperRightX = 430 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;;
		m_filterProperties.LeftLowerLeftX = 350 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
		m_filterProperties.LeftLowerRightX = 430 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
	}
	if ((wheelCountLeft + wheelCountRight) / 2 - faultyLineCounterMiddle > m_filterProperties.faultyCounterThreshold && faultyLineCounterMiddle != -1 || lineMiddle == Vec4i(0, 0, 0, 0)) {
		if (faultyLineCounterRight == -1) {
			m_filterProperties.MidUpperLeftX = 530 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
			m_filterProperties.MidUpperRightX = 590 + sin(laneAngleRight)*m_filterProperties.FovCurveLeft + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
			m_filterProperties.MidLowerLeftX = 530 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
			m_filterProperties.MidLowerRightX = 590 + sin(laneAngleRight)*m_filterProperties.curvatureOffsetRight;
		}
		else if (faultyLineCounterLeft == -1) {
			m_filterProperties.MidUpperLeftX = 530 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
			m_filterProperties.MidUpperRightX = 590 + sin(laneAngleLeft)*m_filterProperties.FovCurveLeft + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
			m_filterProperties.MidLowerLeftX = 530 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
			m_filterProperties.MidLowerRightX = 590 + sin(laneAngleLeft)*m_filterProperties.curvatureOffsetLeft;
		}
		else
		{
			m_filterProperties.MidUpperLeftX = 530;
			m_filterProperties.MidUpperRightX = 590;
			m_filterProperties.MidLowerLeftX = 530;
			m_filterProperties.MidLowerRightX = 590;
			//m_filterProperties.MidUpperLeftX = 550;
			//m_filterProperties.MidUpperRightX = 610;
			//m_filterProperties.MidLowerLeftX = 550;
			//m_filterProperties.MidLowerRightX = 610;
		}
	}
	else {
		m_filterProperties.MidUpperLeftX = 530 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;;
		m_filterProperties.MidUpperRightX = 590 + sin(laneAngleMiddle)*m_filterProperties.FovCurveLeft + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;;
		m_filterProperties.MidLowerLeftX = 530 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
		m_filterProperties.MidLowerRightX = 590 + sin(laneAngleMiddle)*m_filterProperties.curvatureOffsetMid;
	}

	RETURN_NOERROR;
}


tResult cLaneDetection::transmitSteeringAngle(tFloat32 SteeringAngle) {
	//init mediasample
	cObjectPtr<IMediaSample> pMediaSample;
	//allocate memory to mediasample
	AllocMediaSample((tVoid**)&pMediaSample);

	//create interaction with ddl
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionSteeringAngle->GetMediaSampleSerializer(&pSerializer);

	//allocate buffer to write in mediasample
	pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());

	{
		//write lock
		__adtf_sample_write_lock_mediadescription(m_pDescriptionSteeringAngle, pMediaSample, pCoderInput);

		//Set all Ids
		if (!m_szIdOutputSteeringAngleSet)
		{
			pCoderInput->GetID("f32Value", m_szIdSteeringAngleValue);
			//pCoderInput->GetID("ui32ArduinoTimestamp", m_szIdSteeringAngleTimestamp);
			m_szIdOutputSteeringAngleSet = tTrue;
		}
		pCoderInput->Set(m_szIdSteeringAngleValue, (tVoid*)&SteeringAngle);
	}

	//pMediaSample->SetTime(_clock->GetStreamTime());
	pMediaSample->SetTime(_clock->GetStreamTime());
	m_oSteeringAngleOutputPin.Transmit(pMediaSample);

	RETURN_NOERROR;
}

tResult cLaneDetection::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
	//check if pixelformat or size has changed
	if (tInt32(outputImage.total() * outputImage.elemSize()) != m_sOutputFormat.nSize)
	{
		Mat2BmpFormat(outputImage, m_sOutputFormat);

		//LOG_INFO(adtf_util::cString::Format("Output: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sOutputFormat.nWidth, m_sOutputFormat.nHeight, m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));
		//set output format for output pin
		m_oVideoOutputPin.SetFormat(&m_sOutputFormat, NULL);
	}
	RETURN_NOERROR;
}
