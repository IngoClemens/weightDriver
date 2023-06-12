// ---------------------------------------------------------------------
//
//  weightDriver.h
//
//  Created by ingo on 9/27/13.
//  Copyright (c) 2021 Ingo Clemens. All rights reserved.
//
// ---------------------------------------------------------------------

#ifndef __SHAPESTools__weightDriver__
#define __SHAPESTools__weightDriver__

#include <iostream>

#include <maya/MGlobal.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MTypeId.h>

#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>

#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MRampAttribute.h>

#include <maya/MArrayDataBuilder.h>
#include <maya/MColor.h>
#include <maya/MDagPath.h>
#include <maya/MFloatArray.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnIkJoint.h>
#include <maya/MMatrix.h>
#include <maya/MPlugArray.h>
#include <maya/MPoint.h>
#include <maya/MQuaternion.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVectorArray.h>

// VP2.0
#if MAYA_API_VERSION >= 201400
#if MAYA_API_VERSION < 202400
#include <maya/MDrawContext.h>
#endif
#include <maya/MDrawRegistry.h>
#include <maya/MEventMessage.h>
#include "maya/MFnCamera.h"
#include "maya/MHWGeometryUtilities.h"
#include <maya/MPxDrawOverride.h>
#include <maya/MUserData.h>
#endif

#include "BRMatrix.h"
#include <vector>

class weightDriver : public MPxLocatorNode
{
public:
    weightDriver();
    virtual ~weightDriver();

    static void* creator();

    virtual bool isBounded() const;

    static MStatus initialize();

    virtual void postConstructor();

    virtual MStatus postConstructor_init_curveRamp(MObject &nodeObj,
                                                   MObject &rampObj,
                                                   int index,
                                                   float position,
                                                   float value,
                                                   int interpolation);

    virtual MStatus compute(const MPlug &plug, MDataBlock &data);

    virtual MStatus getPoseVectors(MDataBlock &data,
                                   std::vector<double> &driver,
                                   unsigned &poseCount,
                                   BRMatrix &poseData,
                                   BRMatrix &poseValues,
                                   MIntArray &poseModes,
                                   unsigned twistAxis,
                                   bool invert,
                                   unsigned driverId);
    virtual MStatus getPoseData(MDataBlock &data,
                                std::vector<double> &driver,
                                unsigned &poseCount,
                                unsigned &solveCount,
                                BRMatrix &poseData,
                                BRMatrix &poseValues,
                                MIntArray &poseModes);

    static double getTwistAngle(MQuaternion q, unsigned int axis);
    static BRMatrix getDistances(BRMatrix poseMat, double &meanDist, int distType);
    static double getPoseDelta(std::vector<double> vec1, std::vector<double> vec2, int distType);
    static double getRadius(std::vector<double> vec1, std::vector<double> vec2);
    static double getAngle(std::vector<double> vec1, std::vector<double> vec2);
    static void getActivations(BRMatrix &mat, double width, short kernelType);
    static double interpolateRbf(double value, double width, short kernelType);
    static void getPoseWeights(MDoubleArray &out,
                               BRMatrix poses,
                               std::vector<double> driver,
                               MIntArray poseModes,
                               BRMatrix weightMat,
                               double dist,
                               int distType,
                               short kernelType);

    virtual double rbfWeightBias(double value, double biasValue);
    virtual double interpolateWeight(double value, int type);
    virtual double blendCurveWeight(double value);
    virtual void setOutputValues(MDoubleArray weightsArray, MDataBlock data, bool inactive);

    void showArray(MDoubleArray array, MString name);
    void showVector(MVector vector, MString name);
    void showMatrix(MMatrix mat, MString name);

#if MAYA_API_VERSION < 201900
    virtual void draw(M3dView &view,
                      const MDagPath &path,
                      M3dView::DisplayStyle style,
                      M3dView::DisplayStatus status);
#endif

    static MTypeId id;

#if MAYA_API_VERSION >= 201400
    static MString drawDbClassification;
    static MString drawRegistrantId;
#endif

public:
    // vector angle attributes (sorted)
    static MObject active;
    static MObject angle;
    static MObject centerAngle;
    static MObject color;
    static MObject colorR;
    static MObject colorG;
    static MObject colorB;
    static MObject curveRamp;
    static MObject direction;
    static MObject drawCenter;
    static MObject drawCone;
    static MObject drawWeight;
    static MObject driverMatrix;
    static MObject grow;
    static MObject interpolate;
    static MObject invert;
    static MObject outWeight;
    static MObject readerMatrix;
    static MObject size;
    static MObject translateMax;
    static MObject translateMin;
    static MObject twist;
    static MObject twistAngle;
    static MObject useRotate;
    static MObject useTranslate;

    // rbf attributes (sorted)
    static MObject allowNegative;
    static MObject bias;
    static MObject colorDriver;
    static MObject colorDriverR;
    static MObject colorDriverG;
    static MObject colorDriverB;
    static MObject controlNode;
    static MObject distanceType;
    static MObject drawDriver;
    static MObject drawIndices;
    static MObject drawOrigin;
    static MObject drawPoses;
    static MObject drawTwist;
    static MObject driverIndex;
    static MObject driverInput;
    static MObject driverList;
    static MObject evaluate;
    static MObject exposeData;
    static MObject indexDist;
    static MObject input;
    static MObject kernel;
    static MObject opposite;
    static MObject output;
    static MObject pose;
    static MObject poseAttributes;
    static MObject poseDrawTwist; // VP2.0 twist display
    static MObject poseDrawVector; // VP2.0 pose display
    static MObject poseInput;
    static MObject poseLength;
    static MObject poseMatrix;
    static MObject poseMode;
    static MObject poseParentMatrix;
    static MObject poseRotateOrder;
    static MObject poses;
    static MObject poseValue;
    static MObject poseValues;
    static MObject rbfMode;
    static MObject restInput;
    static MObject scale;
    static MObject type;
    static MObject twistAxis;
    static MObject useInterpolation;

    MRampAttribute curveAttr;

private:
    // vector angle
    double angleVal;
    double centerAngleVal;
    short dirVal;
    bool invVal;

    // rbf
    short distanceTypeVal;
    bool evalInput;
    bool genericMode;
    unsigned globalPoseCount;
    MIntArray poseMatrixIds;
    MVectorArray poseVectorArray; // legacy viewport pose display
    MDoubleArray poseTwistArray; // legacy viewport twist display
    short typeVal;

    BRMatrix matPoses;
    BRMatrix matValues;
    double meanDist;
    MIntArray poseModes;
    BRMatrix wMat;
};

// ---------------------------------------------------------------------
//
// Viewport 2.0
//
// ---------------------------------------------------------------------

#if MAYA_API_VERSION >= 201400
class weightDriverData : public MUserData
{
public:
#if MAYA_API_VERSION > 20200300
    weightDriverData() : MUserData() {}
#else
    weightDriverData() : MUserData(false) {}
#endif
    virtual ~weightDriverData() {}

    bool activeVal;
    double angleVal;
    double centerAngleVal;
    double colorDriverRVal;
    double colorDriverGVal;
    double colorDriverBVal;
    double colorRVal;
    double colorGVal;
    double colorBVal;
    short dirVal;
    bool drawCenterVal;
    bool drawConeVal;
    bool drawDriverVal;
    bool drawIndicesVal;
    bool drawOriginVal;
    bool drawPosesVal;
    bool drawTwistVal;
    bool drawWeightVal;
    int driverIndexVal;
    double indexDistVal;
    int invVal;
    double poseLengthVal;
    short rbfModeVal;
    double sizeVal;
    short typeVal;
    double weightVal;
};

class weightDriverOverride : public MHWRender::MPxDrawOverride
{
public:
    static MHWRender::MPxDrawOverride* Creator(const MObject& obj)
    {
        return new weightDriverOverride(obj);
    }
    virtual ~weightDriverOverride();

// -----------------------------------------------
// VP2.0 API 2016.5
// -----------------------------------------------
#if MAYA_API_VERSION >= 201650
    virtual MHWRender::DrawAPI supportedDrawAPIs() const;
#endif

    virtual bool isBounded(const MDagPath &objPath,
                           const MDagPath &cameraPath) const { return true; };

    virtual MBoundingBox boundingBox(const MDagPath &objPath,
                                     const MDagPath &cameraPath) const;

    virtual MUserData* prepareForDraw(const MDagPath &objPath,
                                      const MDagPath &cameraPath,
                                      const MHWRender::MFrameContext& frameContext,
                                      MUserData *oldData);

    virtual bool hasUIDrawables() const { return true; };

    virtual void addUIDrawables(const MDagPath &objPath,
                                MHWRender::MUIDrawManager &drawManager,
                                const MHWRender::MFrameContext &frameContext,
                                const MUserData *data);

#if MAYA_API_VERSION < 202400
    static void draw(const MHWRender::MDrawContext &context, const MUserData *data);
#endif

public:
    MVector viewVector;

private:
    weightDriverOverride(const MObject& obj);

// -----------------------------------------------
// VP2.0 API 2016.5
// -----------------------------------------------
#if MAYA_API_VERSION >= 201650
    static void OnModelEditorChanged(void *clientData);

    weightDriver*  fWeightDriver;
    MCallbackId fModelEditorChangedCbId;
#endif
};
#endif // MAYA_API_VERSION >= 201400

#endif

// ---------------------------------------------------------------------
// MIT License
//
// Copyright (c) 2021 Ingo Clemens, brave rabbit
// weightDriver is under the terms of the MIT License
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Author: Ingo Clemens    www.braverabbit.com
// ---------------------------------------------------------------------
