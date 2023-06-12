// ---------------------------------------------------------------------
//
//  weightDriver.cpp
//
//  Created by ingo on 9/27/13.
//  Copyright (c) 2021 Ingo Clemens. All rights reserved.
//
// ---------------------------------------------------------------------

#include "weightDriver.h"

#include "math.h"

#ifdef _WIN64
#define M_PI 3.1415926535897932384626433832795
#endif

#define DOUBLE_EPSILON 2.2204460492503131e-16

const float DEGTORAD = (float)(M_PI / 180);
const float RADTODEG = (float)(180 / M_PI);


MTypeId weightDriver::id(0x0011C1C5);


// -----------------------------------------------
// vector angle attributes (sorted by category)
// -----------------------------------------------

// input
MObject weightDriver::driverMatrix;
MObject weightDriver::readerMatrix;
// controls
MObject weightDriver::active;
MObject weightDriver::angle;
MObject weightDriver::centerAngle;
MObject weightDriver::curveRamp;
MObject weightDriver::direction;
MObject weightDriver::grow;
MObject weightDriver::interpolate;
MObject weightDriver::invert;
MObject weightDriver::translateMax;
MObject weightDriver::translateMin;
MObject weightDriver::twist;
MObject weightDriver::twistAngle;
MObject weightDriver::useRotate;
MObject weightDriver::useTranslate;
// display
MObject weightDriver::color;
MObject weightDriver::colorR;
MObject weightDriver::colorG;
MObject weightDriver::colorB;
MObject weightDriver::drawCenter;
MObject weightDriver::drawCone;
MObject weightDriver::drawWeight;
MObject weightDriver::size;
// output
MObject weightDriver::outWeight;


// -----------------------------------------------
// rbf attributes (sorted by category)
// -----------------------------------------------

// input
MObject weightDriver::controlNode;
MObject weightDriver::driverInput;
MObject weightDriver::driverList;
MObject weightDriver::input;
MObject weightDriver::pose;
MObject weightDriver::poseAttributes;
MObject weightDriver::poseInput;
MObject weightDriver::poseMatrix;
MObject weightDriver::poseMode;
MObject weightDriver::poseParentMatrix;
MObject weightDriver::poseRotateOrder;
MObject weightDriver::poses;
MObject weightDriver::poseValue;
MObject weightDriver::poseValues;
MObject weightDriver::restInput;
// controls
MObject weightDriver::allowNegative;
MObject weightDriver::bias;
MObject weightDriver::distanceType;
MObject weightDriver::evaluate;
MObject weightDriver::kernel;
MObject weightDriver::opposite;
MObject weightDriver::rbfMode;
MObject weightDriver::twistAxis;
MObject weightDriver::type;
MObject weightDriver::useInterpolation;
// display
MObject weightDriver::colorDriver;
MObject weightDriver::colorDriverR;
MObject weightDriver::colorDriverG;
MObject weightDriver::colorDriverB;
MObject weightDriver::drawDriver;
MObject weightDriver::drawIndices;
MObject weightDriver::drawOrigin;
MObject weightDriver::drawPoses;
MObject weightDriver::drawTwist;
MObject weightDriver::driverIndex;
MObject weightDriver::indexDist;
MObject weightDriver::poseDrawTwist;
MObject weightDriver::poseDrawVector;
MObject weightDriver::poseLength;
MObject weightDriver::scale;
// output
MObject weightDriver::output;

// special
MObject weightDriver::exposeData;

// ---------------------------------------------------------------------
// creator
// ---------------------------------------------------------------------

weightDriver::weightDriver()
{}

weightDriver::~weightDriver()
{}

void* weightDriver::creator()
{
    return new weightDriver();
}

bool weightDriver::isBounded() const
{
    return false;
}

// ---------------------------------------------------------------------
// initialize the attributes
// ---------------------------------------------------------------------

MStatus weightDriver::initialize()
{
    //
    // MFnEnumAttribute
    //

    MFnEnumAttribute eAttr;

    direction = eAttr.create("direction", "dir", 0);
    eAttr.addField("X", 0);
    eAttr.addField("Y", 1);
    eAttr.addField("Z", 2);
    eAttr.setKeyable(true);

    distanceType = eAttr.create("distanceType", "dist", 0);
    eAttr.addField("Euclidean", 0);
    eAttr.addField("Angle", 1);
    eAttr.setKeyable(true);

    interpolate = eAttr.create("interpolation", "int", 0);
    eAttr.addField("Linear", 0);
    eAttr.addField("Slow", 1);
    eAttr.addField("Fast", 2);
    eAttr.addField("Smooth1", 3);
    eAttr.addField("Smooth2", 4);
    eAttr.addField("Curve", 5);
    eAttr.setKeyable(true);

    kernel = eAttr.create("kernel", "kn", 0);
    eAttr.addField("Gaussian", 0);
    eAttr.addField("Linear", 1);
    // Set the attribute to be hidden and non-keyable because the
    // evaluation needs to get updated when switching the kernel type.
    // The automatic update is tied to the control in the attribute
    // editor. But since the channel box doesn't allow for such a
    // command execution the attribute is hidden from the channel box
    // to force the editing through the attribute editor.
    eAttr.setKeyable(false);
    eAttr.setHidden(true);

    poseMode = eAttr.create("poseMode", "pmd", 0);
    eAttr.addField("Rotate/Twist", 0);
    eAttr.addField("Rotate", 1);
    eAttr.addField("Twist", 2);

    poseRotateOrder = eAttr.create("controlPoseRotateOrder", "cpro", 0);
    eAttr.addField("xyz", 0);
    eAttr.addField("yzx", 1);
    eAttr.addField("zxy", 2);
    eAttr.addField("xzy", 3);
    eAttr.addField("yxz", 4);
    eAttr.addField("zyx", 5);

    rbfMode = eAttr.create("rbfMode", "rbfm", 0);
    eAttr.addField("Generic", 0);
    eAttr.addField("Matrix", 1);
    eAttr.setKeyable(false);
    eAttr.setHidden(true);

    twistAxis = eAttr.create("twistAxis", "tax", 0);
    eAttr.addField("X", 0);
    eAttr.addField("Y", 1);
    eAttr.addField("Z", 2);
    eAttr.setKeyable(false);

    type = eAttr.create("type", "typ", 0);
    eAttr.addField("Vector Angle", 0);
    eAttr.addField("RBF", 1);
    eAttr.setKeyable(true);

    //
    // MFnNumericAttribute
    //

    MFnNumericAttribute nAttr;

    active = nAttr.create("active", "ac", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    allowNegative = nAttr.create("allowNegativeWeights", "anw", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    angle = nAttr.create("angle", "an", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.01);
    nAttr.setMax(180.0);
    nAttr.setDefault(45.0);

    bias = nAttr.create("bias", "bias", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setDefault(0.0);
    nAttr.setSoftMin(-1.0);
    nAttr.setSoftMax(1.0);

    centerAngle = nAttr.create("centerAngle", "ca", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setMax(180.0);
    nAttr.setDefault(0.0);

    colorDriverR = nAttr.create("driverColorR", "dcr", MFnNumericData::kDouble);
    nAttr.setKeyable(false);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setDefault(0.1);

    colorDriverG = nAttr.create("driverColorG", "dcg", MFnNumericData::kDouble);
    nAttr.setKeyable(false);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setDefault(0.7);

    colorDriverB = nAttr.create("driverColorB", "dcb", MFnNumericData::kDouble);
    nAttr.setKeyable(false);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setDefault(0.0);

    colorR = nAttr.create("iconColorR", "icr", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setDefault(1.0);

    colorG = nAttr.create("iconColorG", "icg", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setDefault(0.8);

    colorB = nAttr.create("iconColorB", "icb", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setDefault(0.2);

    drawCenter = nAttr.create("drawCenterCone", "dcc", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(false);

    drawCone = nAttr.create("drawCone", "dc", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    drawDriver = nAttr.create("drawDriver", "dd", MFnNumericData::kBoolean);
    nAttr.setKeyable(false);
    nAttr.setHidden(true);
    nAttr.setDefault(false);

    drawIndices = nAttr.create("drawIndices", "did", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    drawOrigin = nAttr.create("drawOrigin", "dor", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    drawPoses = nAttr.create("drawPoses", "dp", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    drawTwist = nAttr.create("drawTwist", "dt", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(false);

    drawWeight = nAttr.create("drawWeight", "dw", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    driverIndex = nAttr.create("driverIndex", "dvi", MFnNumericData::kInt);
    nAttr.setKeyable(false);
    nAttr.setHidden(true);
    nAttr.setDefault(0);

    evaluate = nAttr.create("evaluate", "e", MFnNumericData::kBoolean);
    nAttr.setKeyable(false);
    nAttr.setHidden(true);
    nAttr.setDefault(false);

    exposeData = nAttr.create("exposeData", "exd", MFnNumericData::kInt);
    nAttr.setKeyable(true);
    nAttr.setHidden(true);
    nAttr.setDefault(0);

    grow = nAttr.create("grow", "gr", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    indexDist = nAttr.create("indexDistance", "idd", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setDefault(0.1);

    input = nAttr.create("input", "i", MFnNumericData::kDouble);
    nAttr.setWritable(true);
    nAttr.setKeyable(true);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);

    invert = nAttr.create("invert", "iv", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(false);

    opposite = nAttr.create("opposite", "op", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(false);

    output = nAttr.create("output", "o", MFnNumericData::kDouble);
    nAttr.setWritable(true);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);

    outWeight = nAttr.create("outWeight", "ow", MFnNumericData::kDouble);
    nAttr.setWritable(true);
    nAttr.setKeyable(false);
    nAttr.setDefault(0.0);

    poseDrawTwist = nAttr.create("poseDrawTwist", "pdt", MFnNumericData::kDouble);
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    nAttr.setHidden(true);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);

    poseDrawVector = nAttr.create("poseDrawVector", "pdv", MFnNumericData::k3Double);
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    nAttr.setHidden(true);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);

    poseInput = nAttr.create("poseInput", "pi", MFnNumericData::kDouble);
    nAttr.setWritable(true);
    nAttr.setKeyable(true);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);

    poseLength = nAttr.create("poseLength", "pl", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setDefault(1.0);

    poseValue = nAttr.create("poseValue", "pv", MFnNumericData::kDouble);
    nAttr.setWritable(true);
    nAttr.setKeyable(true);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);

    restInput = nAttr.create("restInput", "rin", MFnNumericData::kDouble);
    nAttr.setWritable(true);
    nAttr.setKeyable(true);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);

    scale = nAttr.create("scale", "sc", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setDefault(1.0);

    size = nAttr.create("iconSize", "is", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setSoftMax(50.0);
    nAttr.setDefault(1.0);

    translateMax = nAttr.create("translateMax", "tmax", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setDefault(0.0);

    translateMin = nAttr.create("translateMin", "tmin", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setDefault(0.0);

    twist = nAttr.create("twist", "tw", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(false);

    twistAngle = nAttr.create("twistAngle", "ta", MFnNumericData::kDouble);
    nAttr.setKeyable(true);
    nAttr.setMin(0.01);
    nAttr.setMax(180.0);
    nAttr.setDefault(90.0);

    useInterpolation = nAttr.create("useInterpolation", "uint", MFnNumericData::kBoolean);
    nAttr.setKeyable(false);
    nAttr.setHidden(true);

    useRotate = nAttr.create("useRotate", "ur", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(true);

    useTranslate = nAttr.create("useTranslate", "ut", MFnNumericData::kBoolean);
    nAttr.setKeyable(true);
    nAttr.setDefault(false);

    //
    // MFnMessageAttribute
    //

    MFnMessageAttribute msgAttr;

    controlNode = msgAttr.create("controlNode", "cn");

    //
    // MFnMatrixAttribute
    //

    MFnMatrixAttribute mAttr;

    driverInput = mAttr.create("driverInput", "di");
    mAttr.setHidden(true);
    driverMatrix = mAttr.create("driverMatrix", "dm");
    mAttr.setHidden(true);
    poseMatrix = mAttr.create("poseMatrix", "pmat");
    mAttr.setHidden(true);
    poseParentMatrix = mAttr.create("poseParentMatrix", "ppmat");
    mAttr.setHidden(true);
    readerMatrix = mAttr.create("readerMatrix", "rm");
    mAttr.setHidden(true);

    //
    // MFnTypedAttribute
    //

    MFnTypedAttribute tAttr;

    poseAttributes = tAttr.create("controlPoseAttributes", "cpa", MFnData::kStringArray);
    poseValues = tAttr.create("controlPoseValues", "cpv", MFnData::kDoubleArray);

    //
    // MFnCompoundAttribute
    //

    MFnCompoundAttribute cAttr;

    color = cAttr.create("iconColor", "ic");
    cAttr.setKeyable(true);
    cAttr.addChild(colorR);
    cAttr.addChild(colorG);
    cAttr.addChild(colorB);

    colorDriver = cAttr.create("driverColor", "dco");
    cAttr.setKeyable(false);
    cAttr.setHidden(true);
    cAttr.addChild(colorDriverR);
    cAttr.addChild(colorDriverG);
    cAttr.addChild(colorDriverB);

    pose = cAttr.create("pose", "p");
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.addChild(poseMatrix);
    cAttr.addChild(poseParentMatrix);
    cAttr.addChild(poseMode);
    cAttr.addChild(poseAttributes);
    cAttr.addChild(poseValues);
    cAttr.addChild(poseRotateOrder);

    driverList = cAttr.create("driverList", "dl");
    cAttr.setHidden(true);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.addChild(driverInput);
    cAttr.addChild(controlNode);
    cAttr.addChild(pose);

    poses = cAttr.create("poses", "ps");
    cAttr.setKeyable(true);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.addChild(poseInput);
    cAttr.addChild(poseValue);

    //
    // MRampAttribute
    //

    MRampAttribute rAttr;

    curveRamp = rAttr.createCurveRamp("blendCurve", "bc");

    // -----------------------------------------------------------------
    // add attributes (order matters)
    // -----------------------------------------------------------------

    addAttribute(active);
    addAttribute(type);
    addAttribute(direction);
    addAttribute(invert);
    addAttribute(useRotate);
    addAttribute(angle);
    addAttribute(centerAngle);
    addAttribute(twist);
    addAttribute(twistAngle);
    addAttribute(useTranslate);
    addAttribute(grow);
    addAttribute(translateMin);
    addAttribute(translateMax);
    addAttribute(interpolate);
    addAttribute(curveRamp);
    addAttribute(size);
    addAttribute(color);
    addAttribute(drawCone);
    addAttribute(drawCenter);
    addAttribute(drawWeight);
    addAttribute(outWeight);
    addAttribute(readerMatrix);
    addAttribute(driverMatrix);
    addAttribute(driverList);
    addAttribute(driverInput);
    addAttribute(pose);
    addAttribute(poseMatrix);
    addAttribute(poseParentMatrix);
    addAttribute(input);
    addAttribute(restInput);
    addAttribute(poses);
    addAttribute(poseInput);
    addAttribute(poseValue);
    addAttribute(output);
    addAttribute(poseMode);
    addAttribute(twistAxis);
    addAttribute(opposite);
    addAttribute(poseAttributes);
    addAttribute(poseValues);
    addAttribute(poseRotateOrder);
    addAttribute(rbfMode);
    addAttribute(evaluate);
    addAttribute(kernel);
    addAttribute(bias);
    addAttribute(useInterpolation);
    addAttribute(allowNegative);
    addAttribute(scale);
    addAttribute(distanceType);
    addAttribute(drawOrigin);
    addAttribute(drawDriver);
    addAttribute(drawPoses);
    addAttribute(drawIndices);
    addAttribute(drawTwist);
    addAttribute(poseLength);
    addAttribute(indexDist);
    addAttribute(driverIndex);
    addAttribute(colorDriver);
    addAttribute(controlNode);
    addAttribute(poseDrawVector);
    addAttribute(poseDrawTwist);
    addAttribute(exposeData);

    // -----------------------------------------------------------------
    // affects
    // -----------------------------------------------------------------

    attributeAffects(weightDriver::active, weightDriver::output);
    attributeAffects(weightDriver::allowNegative, weightDriver::output);
    attributeAffects(weightDriver::angle, weightDriver::output);
    attributeAffects(weightDriver::bias, weightDriver::output);
    attributeAffects(weightDriver::centerAngle, weightDriver::output);
    attributeAffects(weightDriver::curveRamp, weightDriver::output);
    attributeAffects(weightDriver::direction, weightDriver::output);
    attributeAffects(weightDriver::distanceType, weightDriver::output);
    attributeAffects(weightDriver::driverIndex, weightDriver::output);
    attributeAffects(weightDriver::driverInput, weightDriver::output);
    attributeAffects(weightDriver::driverMatrix, weightDriver::output);
    attributeAffects(weightDriver::evaluate, weightDriver::output);
    attributeAffects(weightDriver::grow, weightDriver::output);
    attributeAffects(weightDriver::input, weightDriver::output);
    attributeAffects(weightDriver::interpolate, weightDriver::output);
    attributeAffects(weightDriver::invert, weightDriver::output);
    attributeAffects(weightDriver::kernel, weightDriver::output);
    attributeAffects(weightDriver::opposite, weightDriver::output);
    attributeAffects(weightDriver::poseInput, weightDriver::output);
    attributeAffects(weightDriver::poseMatrix, weightDriver::output);
    attributeAffects(weightDriver::poseMode, weightDriver::output);
    attributeAffects(weightDriver::poseParentMatrix, weightDriver::output);
    attributeAffects(weightDriver::poseValue, weightDriver::output);
    attributeAffects(weightDriver::scale, weightDriver::output);
    attributeAffects(weightDriver::rbfMode, weightDriver::output);
    attributeAffects(weightDriver::readerMatrix, weightDriver::output);
    attributeAffects(weightDriver::restInput, weightDriver::output);
    attributeAffects(weightDriver::translateMax, weightDriver::output);
    attributeAffects(weightDriver::translateMin, weightDriver::output);
    attributeAffects(weightDriver::twist, weightDriver::output);
    attributeAffects(weightDriver::twistAngle, weightDriver::output);
    attributeAffects(weightDriver::twistAxis, weightDriver::output);
    attributeAffects(weightDriver::type, weightDriver::output);
    attributeAffects(weightDriver::useInterpolation, weightDriver::output);
    attributeAffects(weightDriver::useRotate, weightDriver::output);
    attributeAffects(weightDriver::useTranslate, weightDriver::output);

    // -----------------------------------------------------------------
    // affects also the legacy outWeight plug
    // (to not break compatibility)
    // -----------------------------------------------------------------
    attributeAffects(weightDriver::active, weightDriver::outWeight);
    attributeAffects(weightDriver::angle, weightDriver::outWeight);
    attributeAffects(weightDriver::centerAngle, weightDriver::outWeight);
    attributeAffects(weightDriver::curveRamp, weightDriver::outWeight);
    attributeAffects(weightDriver::direction, weightDriver::outWeight);
    attributeAffects(weightDriver::driverMatrix, weightDriver::outWeight);
    attributeAffects(weightDriver::interpolate, weightDriver::outWeight);
    attributeAffects(weightDriver::invert, weightDriver::outWeight);
    attributeAffects(weightDriver::grow, weightDriver::outWeight);
    attributeAffects(weightDriver::readerMatrix, weightDriver::outWeight);
    attributeAffects(weightDriver::translateMax, weightDriver::outWeight);
    attributeAffects(weightDriver::translateMin, weightDriver::outWeight);
    attributeAffects(weightDriver::twist, weightDriver::outWeight);
    attributeAffects(weightDriver::twistAngle, weightDriver::outWeight);
    attributeAffects(weightDriver::type, weightDriver::outWeight);
    attributeAffects(weightDriver::useRotate, weightDriver::outWeight);
    attributeAffects(weightDriver::useTranslate, weightDriver::outWeight);

    return MStatus::kSuccess;
}


void weightDriver::postConstructor()
{
    MObject thisNode = this->thisMObject();
    MFnDependencyNode nodeFn(thisNode);
    nodeFn.setName("weightDriverShape#");

    // initialize the curve ramp
    postConstructor_init_curveRamp(thisNode, curveRamp, 0, 0.0f, 0.0f, 3);
    postConstructor_init_curveRamp(thisNode, curveRamp, 1, 1.0f, 1.0f, 3);

    // -----------------------------------------------------------------
    // hide the default attributes
    // -----------------------------------------------------------------

    MPlug attrPlug(thisNode, weightDriver::localPositionX);
    attrPlug.setChannelBox(false);
    attrPlug.setAttribute(weightDriver::localPositionY);
    attrPlug.setChannelBox(false);
    attrPlug.setAttribute(weightDriver::localPositionZ);
    attrPlug.setChannelBox(false);
    attrPlug.setAttribute(weightDriver::localScaleX);
    attrPlug.setChannelBox(false);
    attrPlug.setAttribute(weightDriver::localScaleY);
    attrPlug.setChannelBox(false);
    attrPlug.setAttribute(weightDriver::localScaleZ);
    attrPlug.setChannelBox(false);
}


MStatus weightDriver::postConstructor_init_curveRamp(MObject &nodeObj,
                                                     MObject &rampObj,
                                                     int index,
                                                     float position,
                                                     float value,
                                                     int interpolation)
{
    MStatus status;

    MPlug rampPlug(nodeObj, rampObj);
    MPlug elementPlug = rampPlug.elementByLogicalIndex((unsigned)index, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug positionPlug = elementPlug.child(0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = positionPlug.setFloat(position);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug valuePlug = elementPlug.child(1);
    status = valuePlug.setFloat(value);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug interpolationPlug = elementPlug.child(2);
    interpolationPlug.setInt(interpolation);

    return status;
}


// ---------------------------------------------------------------------
// compute function
// ---------------------------------------------------------------------

MStatus weightDriver::compute(const MPlug &plug, MDataBlock &data)
{
    MStatus status = MStatus::kSuccess;

    MObject thisNode = this->thisMObject();
    MFnDependencyNode thisFn(thisNode);
    MString thisName = thisFn.name();

    // -----------------------------------------------------------------
    // get the attributes
    // -----------------------------------------------------------------

    MPlug activePlug(thisNode, weightDriver::active);
    MPlug allowNegativePlug(thisNode, weightDriver::allowNegative);
    MPlug anglePlug(thisNode, weightDriver::angle);
    MPlug biasPlug(thisNode, weightDriver::bias);
    MPlug centerAnglePlug(thisNode, weightDriver::centerAngle);
    MPlug dirPlug(thisNode, weightDriver::direction);
    MPlug distanceTypePlug(thisNode, weightDriver::distanceType);
    MPlug driverIndexPlug(thisNode, weightDriver::driverIndex);
    MPlug evaluatePlug(thisNode, weightDriver::evaluate);
    MPlug exposeDataPlug(thisNode, weightDriver::exposeData);
    MPlug interpolatePlug(thisNode, weightDriver::interpolate);
    MPlug invPlug(thisNode, weightDriver::invert);
    MPlug useMaxPlug(thisNode, weightDriver::grow);
    MPlug kernelPlug(thisNode, weightDriver::kernel);
    MPlug oppositePlug(thisNode, weightDriver::opposite);
    MPlug rbfModePlug(thisNode, weightDriver::rbfMode);
    MPlug scalePlug(thisNode, weightDriver::scale);
    MPlug translateMaxPlug(thisNode, weightDriver::translateMax);
    MPlug translateMinPlug(thisNode, weightDriver::translateMin);
    MPlug twistPlug(thisNode, weightDriver::twist);
    MPlug twistAnglePlug(thisNode, weightDriver::twistAngle);
    MPlug twistAxisPlug(thisNode, weightDriver::twistAxis);
    MPlug typePlug(thisNode, weightDriver::type);
    MPlug useInterpolationPlug(thisNode, weightDriver::useInterpolation);
    MPlug useRotatePlug(thisNode, weightDriver::useRotate);
    MPlug useTranslatePlug(thisNode, weightDriver::useTranslate);

    bool activeVal = activePlug.asBool();
    bool allowNegativeVal = allowNegativePlug.asBool();
    angleVal = anglePlug.asDouble();
    double biasVal = biasPlug.asDouble();
    centerAngleVal = centerAnglePlug.asDouble();
    dirVal = dirPlug.asShort();
    distanceTypeVal = distanceTypePlug.asShort();
    int driverIndexVal = driverIndexPlug.asInt();
    evalInput = evaluatePlug.asBool();
    int exposeDataVal = exposeDataPlug.asInt();
    bool growVal = useMaxPlug.asBool();
    short interVal = interpolatePlug.asShort();
    invVal = invPlug.asBool();
    short kernelVal = kernelPlug.asShort();
    bool oppositeVal = oppositePlug.asBool();
    double scaleVal = scalePlug.asDouble();
    double twistAngleVal = twistAnglePlug.asDouble();
    short twistAxisVal = twistAxisPlug.asShort();
    bool twistVal = twistPlug.asBool();
    double translateMaxVal = translateMaxPlug.asDouble();
    double translateMinVal = translateMinPlug.asDouble();
    bool useInterpolationVal = useInterpolationPlug.asBool();
    bool useRotateVal = useRotatePlug.asBool();
    bool useTranslateVal = useTranslatePlug.asBool();
    typeVal = typePlug.asShort();

    curveAttr = MRampAttribute(thisNode, curveRamp, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    if (((plug == output && typeVal != 0) || (plug == outWeight && typeVal == 0)) && activeVal)
    {
        // Deactivate the node if the state is set to HasNoEffect.
        MDataHandle stateData = data.inputValue(state, &status);
        if (stateData.asShort() == 1)
            return status;

        // -------------------------------------------------------------
        // main calculation
        // -------------------------------------------------------------

        MDoubleArray weightsArray;
        unsigned poseCount = 1;

        // -------------------------------------------------------------
        // vector angle
        // -------------------------------------------------------------

        if (typeVal == 0)
        {
            // ---------------------------------------------------------
            // get the general matrix data handles
            // ---------------------------------------------------------

            MDataHandle readerHandle = data.inputValue(readerMatrix, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);
            MMatrix readerMat = readerHandle.asMatrix();

            MDataHandle driverHandle = data.inputValue(driverMatrix, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);
            MMatrix driverMat = driverHandle.asMatrix();

            MTransformationMatrix transMatReader = readerMat;
            MTransformationMatrix transMatDriver = driverMat;

            MVector readerPos = transMatReader.getTranslation(MSpace::kWorld);
            MVector driverPos = transMatDriver.getTranslation(MSpace::kWorld);

            MVector driverMVec = driverPos - readerPos;

            weightsArray.setLength(poseCount);

            // ---------------------------------------------------------
            // define the target vector
            // ---------------------------------------------------------

            MPoint targetPos;
            MVector upMVec;

            double axis = 1.0;
            if (invVal)
                axis = -1.0;

            if (dirVal == 0)
            {
                targetPos = MPoint(axis, 0.0, 0.0);
                upMVec = MVector(0.0, 1.0, 0.0);
            }
            else if (dirVal == 1)
            {
                targetPos = MPoint(0.0, axis, 0.0);
                upMVec = MVector(1.0, 0.0, 0.0);
            }
            else
            {
                targetPos = MPoint(0.0, 0.0, axis);
                upMVec = MVector(1.0, 0.0, 0.0);
            }

            targetPos *= readerMat;

            MVector targetMVec = targetPos - readerPos;
            MMatrix relativeMat = readerMat * driverMat.inverse();

            // ---------------------------------------------------------
            // calculate the twist value
            // ---------------------------------------------------------

            double twistWeightVal = 1.0;

            if (twistVal)
            {
                MVector twistMVec = upMVec * relativeMat;
                twistMVec.normalize();

                double twistAngle = twistMVec.angle(upMVec);
                twistAngle = twistAngle * RADTODEG;

                twistWeightVal = 1 - twistAngle / twistAngleVal;
            }

            // ---------------------------------------------------------
            // calculate the translate value
            // ---------------------------------------------------------

            double translateVal = 1;

            if (useTranslateVal)
            {
                MTransformationMatrix transMatRelative = relativeMat;
                MVector transMVec = transMatRelative.getTranslation(MSpace::kWorld);
                double distance = transMVec.length();
                if (distance <= translateMinVal)
                    translateVal = 1;
                else if (distance >= translateMaxVal)
                    translateVal = 0;
                else
                {
                    translateVal = 1 - ((distance - translateMinVal)
                                   / (translateMaxVal - translateMinVal));
                }

                if (growVal)
                    translateVal = 1 - translateVal;
            }

            // ---------------------------------------------------------
            // calculate the vectors and resulting angle
            // ---------------------------------------------------------

            double weightVal = 1;

            if (useRotateVal)
            {
                double offsetAngle = targetMVec.angle(driverMVec);
                offsetAngle = offsetAngle * RADTODEG;

                weightVal = 1 - offsetAngle / angleVal;

                weightVal *= twistWeightVal;

                // Make sure that the center angle is always smaller
                // than the angle.
                if (angleVal <= centerAngleVal)
                    centerAngleVal = angleVal - 0.1;

                // Create another value from the center angle to
                // calculate an offset factor for widening the center
                // range.
                double centerVal = (angleVal - centerAngleVal) / angleVal;
                weightVal /= centerVal;
            }

            weightVal *= translateVal;

            // Clamp the value to a 0-1 range.
            if (weightVal <= 0)
                weightVal = 0;
            else if (weightVal >= 1)
                weightVal = 1;

            // ---------------------------------------------------------
            // apply the interpolation
            // ---------------------------------------------------------

            weightVal = interpolateWeight(weightVal, interVal);

            // ---------------------------------------------------------
            // set the output values
            // ---------------------------------------------------------

            // Pass the weight to the array output.
            weightsArray.set(weightVal, 0);

            // Pass the weight to the legacy outWeight plug.
            MDataHandle outWeightHandle = data.outputValue(outWeight);
            outWeightHandle.setDouble(weightsArray[0]);
        }

        // -------------------------------------------------------------
        // radial basis function
        // -------------------------------------------------------------

        else
        {
            unsigned int i, c;

            std::vector<double> driver;
            unsigned int solveCount;
            unsigned int driverCount = 0;

            // ---------------------------------------------------------
            // Check the rbf mode.
            // Any connected input assumes generic mode which is mainly
            // for switching the display of the locator.
            // ---------------------------------------------------------

            MPlug inputPlug(thisNode, weightDriver::input);
            MIntArray inputIds;
            inputPlug.getExistingArrayAttributeIndices(inputIds, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);

            // Set generic mode to be the default.
            genericMode = true;
            if (inputIds.length())
            {
                MDataHandle rbfModeHandle = data.outputValue(rbfMode);
                rbfModeHandle.set(0);
                genericMode = true;
            }
            else
            {
                MDataHandle rbfModeHandle = data.outputValue(rbfMode);
                rbfModeHandle.set(1);
                genericMode = false;
            }

            // ---------------------------------------------------------
            // get the pose data based on the mode
            // ---------------------------------------------------------

            if (genericMode)
            {
                unsigned int driverSize = inputIds.length();
                driver.resize(driverSize);

                status = getPoseData(data,
                                     driver,
                                     poseCount,
                                     solveCount,
                                     matPoses,
                                     matValues,
                                     poseModes);
                CHECK_MSTATUS_AND_RETURN_IT(status);
            }
            else
            {
                // get the driver indices
                MPlug driverPlug(thisNode, weightDriver::driverList);
                MIntArray driverIds;
                driverPlug.getExistingArrayAttributeIndices(driverIds, &status);
                CHECK_MSTATUS_AND_RETURN_IT(status);

                driverCount = driverIds.length();
                driver.resize(4 * driverCount);

                status = getPoseVectors(data,
                                        driver,
                                        poseCount,
                                        matPoses,
                                        matValues,
                                        poseModes,
                                        (unsigned)twistAxisVal,
                                        oppositeVal,
                                        (unsigned)driverIndexVal);
                CHECK_MSTATUS_AND_RETURN_IT(status);

                // Override the distance type for the matrix mode to
                // euclidean which makes it easier to calculate rotation
                // and twist.
                distanceTypeVal = 0;

                solveCount = poseCount;
            }

            if (exposeDataVal == 1 || exposeDataVal == 4)
            {
                matPoses.show(thisName, "Poses");
                matValues.show(thisName, "Values");
                //BRMatrix().showVector(driver, "driver");
            }

            // ---------------------------------------------------------
            // rbf calculation
            // ---------------------------------------------------------

            if (poseCount != 0)
            {
                // Set the default values for the output.
                weightsArray.setLength(solveCount);
                for (i = 0; i < solveCount; i ++)
                    weightsArray.set(0.0, i);

                if (evalInput)
                {
                    // -------------------------------------------------
                    // distances
                    // -------------------------------------------------

                    // Create a distance matrix from all poses and
                    // calculate the mean distance for the rbf function.
                    BRMatrix linMat;
                    meanDist = 0.0;
                    linMat = getDistances(matPoses, meanDist, distanceTypeVal);

                    if (exposeDataVal > 2)
                        linMat.show(thisName, "Distance matrix");

                    // -------------------------------------------------
                    // activations
                    // -------------------------------------------------

                    // Transform the distance matrix to include the
                    // activation values.
                    getActivations(linMat, meanDist, kernelVal);

                    if (exposeDataVal > 2)
                        linMat.show(thisName, "Activations");

                    // -------------------------------------------------
                    // prepare
                    // -------------------------------------------------

                    // create the matrix to store the weights
                    wMat = BRMatrix();
                    wMat.setSize(poseCount, solveCount);

                    // -------------------------------------------------
                    // solve for each dimension
                    // -------------------------------------------------

                    for (c = 0; c < solveCount; c ++)
                    {
                        // Get the pose values for each dimension.
                        std::vector<double> y;
                        y = matValues.getColumnVector(c);
                        //BRMatrix().showVector(y, MString(MString("values ") + c));

                        // Copy the activation matrix because it gets
                        // modified during the solving process.
                        BRMatrix solveMat = linMat;

                        double* w = new double [poseCount];
                        bool solved = solveMat.solve(y, w);
                        if (!solved)
                        {
                            MGlobal::displayError("RBF decomposition failed");
                            return MStatus::kFailure;
                        }

                        // Store the weights in the weight matrix.
                        for (i = 0; i < poseCount; i ++)
                            wMat(i, c) = w[i];

                        delete [] w;
                    }

                    if (exposeDataVal > 2)
                        wMat.show(thisName, "Weight matrix");
                }

                // -----------------------------------------------
                // final weight calculation
                // -----------------------------------------------

                getPoseWeights(weightsArray,
                               matPoses,
                               driver,
                               poseModes,
                               wMat,
                               meanDist,
                               distanceTypeVal,
                               kernelVal);

                if (exposeDataVal == 2 || exposeDataVal == 4)
                    showArray(weightsArray, thisName + " : RBF Weights");

                // -----------------------------------------------
                // define the final values
                // -----------------------------------------------

                for (i = 0; i < weightsArray.length(); i ++)
                {
                    double value = weightsArray[i];

                    if (value < 0.0 && !allowNegativeVal)
                        value = 0.0;

                    if (biasVal != 0.0)
                        value = rbfWeightBias(value, biasVal);

                    if (useInterpolationVal)
                        value = interpolateWeight(value, interVal);

                    // Set the final weight.
                    weightsArray.set(value * scaleVal, i);
                }
            }
            // In case there are no poses generate a default value at
            // the output.
            else
            {
                weightsArray.setLength(1);
                weightsArray.set(1.0, 0);
            }
        }

        // -----------------------------------------------
        // pass the pose value to the output
        // -----------------------------------------------

        setOutputValues(weightsArray, data, false);

        data.setClean(plug);
    }
    else if (plug == output && !activeVal)
    {
        setOutputValues(MDoubleArray(1, 0.0), data, true);

        data.setClean(plug);
    }

    return MStatus::kSuccess;
}


//
// Description:
//      Collect all driver and pose relevant data.
//      RBF Matrix Mode (when using SHAPES)
//
// Input Arguments:
//      data            The MPxNode dataBlock.
//      driver          The array of driver values. Each driver has four
//                      values: the vector and the twist value. The
//                      array length is numberOfDrivers * 4.
//      poseCount       The number of poses.
//      poseData        The matrix containing all poses.
//      poseVals        The matrix of pose values.
//      poseModes       The array containing the the mode per pose.
//      twistAxisVal    The twist axis.
//      invertAxes      True, if the axis should be inverted.
//      driverId        The index of the driver for drawing.
//
// Return Value:
//      MStatus
//
MStatus weightDriver::getPoseVectors(MDataBlock &data,
                                     std::vector<double> &driver,
                                     unsigned &poseCount,
                                     BRMatrix &poseData,
                                     BRMatrix &poseVals,
                                     MIntArray &poseModes,
                                     unsigned twistAxisVal,
                                     bool invertAxes,
                                     unsigned driverId)
{
    MStatus status = MStatus::kSuccess;

    MObject thisNode = this->thisMObject();

    unsigned int d, i, p;
    unsigned increment = 0;

    // -----------------------------------------------------------------
    // create the base vector
    // -----------------------------------------------------------------

    MVector baseVec(1.0, 0.0, 0.0);
    // Define the reference vector base.
    if (twistAxisVal == 1)
        baseVec = MVector(0.0, 1.0, 0.0);
    else if (twistAxisVal == 2)
        baseVec = MVector(0.0, 0.0, 1.0);

    if (invertAxes)
        baseVec *= -1;

    // -----------------------------------------------------------------
    // get the driver list handle
    // -----------------------------------------------------------------

    MArrayDataHandle driverListHandle = data.inputArrayValue(driverList, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    unsigned driverCount = driverListHandle.elementCount();

    // This plug is necessary to get the connected node for the parent
    // matrix and dag type since the MDataHandle cannot be used for
    // this.
    MPlug driverListPlug(thisNode, weightDriver::driverList);

    // -----------------------------------------------------------------
    // process for each driver
    // -----------------------------------------------------------------

    for (d = 0; d < driverCount; d ++)
    {
        status = driverListHandle.jumpToArrayElement(d);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        unsigned currentId = driverListHandle.elementIndex();

        MDataHandle driverListIdHandle = driverListHandle.inputValue();

        // -------------------------------------------------------------
        // get the attributes
        // -------------------------------------------------------------

        MDataHandle driverInputHandle = driverListIdHandle.child(driverInput);
        MMatrix driverMat = driverInputHandle.asMatrix();

        MDataHandle poseHandle = driverListIdHandle.child(pose);
        MArrayDataHandle poseArrayHandle(poseHandle, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // -------------------------------------------------------------
        // get the parent matrix and joint orientation
        // -------------------------------------------------------------

        MPlug driverListIdPlug = driverListPlug.elementByLogicalIndex(currentId);
        MPlug driverInputPlug = driverListIdPlug.child(driverInput);
        MPlug posePlug = driverListIdPlug.child(pose);

        // Check if the driver node is connected.
        // Cancel if not connected.
        MPlugArray plugConn;
        driverInputPlug.connectedTo(plugConn, true, false, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        if (!plugConn.length())
            return status;

        // Retrieve the dag path of the driver node to get the parent
        // matrix.
        MDagPath dagPath;
        MDagPath::getAPathTo(plugConn[0].node(), dagPath);
        MMatrix driverParentMatInv = dagPath.exclusiveMatrixInverse();

        // In case the driver node is a joint the joint orientation
        // needs to be considered as well.
        MMatrix jointOrientMatInv;
        if (dagPath.hasFn(MFn::kJoint))
        {
            MFnIkJoint joint(dagPath);
            MQuaternion jointOrientQuat;
            joint.getOrientation(jointOrientQuat);
            jointOrientMatInv = jointOrientQuat.asMatrix().inverse();
        }

        // Build a local transform matrix.
        MTransformationMatrix transMatDriver = driverMat * driverParentMatInv * jointOrientMatInv;

        MQuaternion quatDriver = transMatDriver.rotation();

        // -------------------------------------------------------------
        // create the driver vector
        // -------------------------------------------------------------

        MVector driverMVec = baseVec * transMatDriver.asMatrix();
        MVector driverMVecDraw = baseVec * driverMat;

        // -------------------------------------------------------------
        // set the driver vector and twist
        // -------------------------------------------------------------

        driver[0 + increment] = driverMVec.x;
        driver[1 + increment] = driverMVec.y;
        driver[2 + increment] = driverMVec.z;
        driver[3 + increment] = getTwistAngle(quatDriver, twistAxisVal);

        // -------------------------------------------------------------
        // get the pose array indices and set the matrices
        // -------------------------------------------------------------

        // Do this only for the first driver because even if there is
        // more than one driver all other drivers should have the same
        // amount of poses and data values.
        if (d == 0)
        {
            posePlug.getExistingArrayAttributeIndices(poseMatrixIds, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);

            poseCount = poseMatrixIds.length();

            if (poseCount != globalPoseCount)
            {
                globalPoseCount = poseCount;
                evalInput = true;
            }

            // ---------------------------------------------------------
            // prepare the data matrices
            // ---------------------------------------------------------

            // Prepare the matrix to hold the pose vectors.
            // Assign an empty matrix to clear pre-existing data.
            poseData = BRMatrix();
            poseData.setSize(poseCount, 4 * driverCount);

            // Prepare the matrix to hold the pose values.
            // Assign an empty matrix to clear pre-existing data.
            poseVals = BRMatrix();
            poseVals.setSize(poseCount, poseCount);
        }

        // -------------------------------------------------------------
        // get the pose matrices and define the pose vectors
        // -------------------------------------------------------------

        if (poseCount)
        {
            // ---------------------------------------------------------
            // prepare the data matrices
            // ---------------------------------------------------------

            MVectorArray poseVectors;
            poseVectors.setLength(poseCount);
            MDoubleArray poseTwist;
            poseTwist.setLength(poseCount);
            MVectorArray poseVectorsDraw;
            poseVectorsDraw.setLength(poseCount);

            // Copy the previous pose modes for comparison to see
            // if the matrices need to get updated.
            MIntArray poseModesPrev = poseModes;

            // Clear pre-existing pose modes.
            poseModes.clear();
            poseModes.setLength(poseCount);

            // ---------------------------------------------------------
            // get the pose data
            // ---------------------------------------------------------

            for (i = 0; i < poseCount; i ++)
            {
                status = poseArrayHandle.jumpToArrayElement(i);
                CHECK_MSTATUS_AND_RETURN_IT(status);

                MDataHandle poseIdHandle = poseArrayHandle.inputValue();
                MDataHandle poseMatrixHandle = poseIdHandle.child(poseMatrix);
                MMatrix poseMat = poseMatrixHandle.asMatrix();

                MDataHandle parentMatrixHandle = poseIdHandle.child(poseParentMatrix);
                MMatrix parentMat = parentMatrixHandle.asMatrix();

                MMatrix poseMatRel = poseMat * parentMat.inverse() * jointOrientMatInv;

                // -----------------------------------------------------
                // pose mode
                // -----------------------------------------------------

                MDataHandle poseModeHandle = poseIdHandle.child(poseMode);
                int poseModeValue = poseModeHandle.asShort();
                poseModes.set(poseModeValue, i);

                // Evaluation for the processing the matrices always
                // needs to be active when the pose mode for a pose
                // changes.
                if (poseModesPrev.length() && poseModeValue != poseModesPrev[i])
                    evalInput = true;

                // -----------------------------------------------------
                // pose vectors
                // -----------------------------------------------------

                MVector poseVec = baseVec * poseMatRel;
                poseVectors.set(poseVec, i);

                MVector poseVecDraw = baseVec * poseMat;
                poseVectorsDraw.set(poseVecDraw, i);

                // -----------------------------------------------------
                // pose vector and twist angle
                // -----------------------------------------------------

                MTransformationMatrix transMatPose = poseMatRel;
                MQuaternion quatPose = transMatPose.rotation();

                if (poseModes[i] != 2)
                {
                    poseData(i, 0 + increment) = poseVec.x;
                    poseData(i, 1 + increment) = poseVec.y;
                    poseData(i, 2 + increment) = poseVec.z;
                }

                poseData(i, 3 + increment) = 0.0;
                poseTwist.set(0.0, i);
                if (poseModes[i] != 1)
                {
                    double twistVal = getTwistAngle(quatPose, twistAxisVal);
                    poseData(i, 3 + increment) = twistVal;
                    poseTwist.set(twistVal, i);
                }

                // -----------------------------------------------------
                // pose values
                // -----------------------------------------------------

                // Create the vector for the pose values.
                if (d == 0)
                {
                    for (p = 0; p < poseCount; p ++)
                    {
                        poseVals(i, p) = 0;
                        if (i == p)
                            poseVals(i, p) = 1;
                    }
                }
            }

            // ---------------------------------------------------------
            // fill the array for drawing
            // ---------------------------------------------------------

            if (d == driverId)
            {
                // Copy the pose vectors and twist for the legacy
                // viewport display.
                poseVectorArray.copy(poseVectorsDraw);
                poseVectorArray.append(driverMVecDraw);
                poseTwistArray.copy(poseTwist);
                poseTwistArray.append(driver[3 + increment]);

                // Copy the pose vectors and twist values for the VP 2.0
                // display.
                MArrayDataHandle pvHandle = data.outputArrayValue(poseDrawVector);
                MArrayDataBuilder pvBuilder(&data, poseDrawVector, poseCount + 1);
                MArrayDataHandle ptHandle = data.outputArrayValue(poseDrawTwist);
                MArrayDataBuilder ptBuilder(&data, poseDrawTwist, poseCount + 1);
                for (i = 0; i < poseCount; i ++)
                {
                    MDataHandle pvIdHandle = pvBuilder.addElement((unsigned)poseMatrixIds[i]);
                    pvIdHandle.set3Double(poseVectorsDraw[i].x, poseVectorsDraw[i].y, poseVectorsDraw[i].z);
                    pvHandle.set(pvBuilder);
                    pvHandle.setAllClean();

                    MDataHandle ptIdHandle = ptBuilder.addElement((unsigned)poseMatrixIds[i]);
                    ptIdHandle.setDouble(poseData(i, 3 + increment));
                    ptHandle.set(ptBuilder);
                    ptHandle.setAllClean();
                }
                // Add the driver vector.
                MDataHandle pvIdHandle = pvBuilder.addElement((unsigned)poseMatrixIds[poseCount - 1] + 1);
                pvIdHandle.set3Double(driverMVecDraw.x, driverMVecDraw.y, driverMVecDraw.z);
                pvHandle.set(pvBuilder);
                pvHandle.setAllClean();

                // Add the driver twist.
                MDataHandle ptIdHandle = ptBuilder.addElement((unsigned)poseMatrixIds[poseCount - 1] + 1);
                ptIdHandle.setDouble(driver[3 + increment]);
                ptHandle.set(ptBuilder);
                ptHandle.setAllClean();
            }
        }

        increment += 4;
    }

    return status;
}


//
// Description:
//      Collect all driver and pose relevant data.
//      Generic Mode
//
// Input Arguments:
//      data            The MPxNode dataBlock.
//      driver          The array of driver values.
//      poseCount       The number of poses.
//      solveCount      The number of outputs to generate values for.
//      poseData        The matrix containing all poses.
//      poseVals        The matrix of pose values.
//      poseModes       The array containing the the mode per pose.
//
// Return Value:
//      MStatus
//
MStatus weightDriver::getPoseData(MDataBlock &data,
                                  std::vector<double> &driver,
                                  unsigned &poseCount,
                                  unsigned &solveCount,
                                  BRMatrix &poseData,
                                  BRMatrix &poseVals,
                                  MIntArray &poseModes)
{
    MStatus status = MStatus::kSuccess;

    MObject thisNode = this->thisMObject();

    unsigned int i, j;

    // -----------------------------------------------------------------
    // get the number of outputs
    // -----------------------------------------------------------------

    MPlug outputPlug(thisNode, weightDriver::output);
    MIntArray outputIds;
    outputPlug.getExistingArrayAttributeIndices(outputIds, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    solveCount = outputIds.length();

    // -----------------------------------------------------------------
    // get the attributes
    // -----------------------------------------------------------------

    MPlug inputPlug(thisNode, weightDriver::input);
    MPlug restInputPlug(thisNode, weightDriver::restInput);
    MPlug posesPlug(thisNode, weightDriver::poses);

    // -----------------------------------------------------------------
    // get the data handles
    // -----------------------------------------------------------------

    MArrayDataHandle inputHandle = data.inputArrayValue(input, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MArrayDataHandle restInputHandle = data.inputArrayValue(restInput, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MArrayDataHandle posesHandle = data.inputArrayValue(poses, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // -----------------------------------------------------------------
    // get the array ids
    // -----------------------------------------------------------------

    MIntArray inputIds;
    MIntArray restInputIds;
    MIntArray poseIds;

    inputPlug.getExistingArrayAttributeIndices(inputIds, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    restInputPlug.getExistingArrayAttributeIndices(restInputIds, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    posesPlug.getExistingArrayAttributeIndices(poseIds, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    unsigned inDim = inputIds.length();
    unsigned restDim = restInputIds.length();

    poseCount = poseIds.length();
    // Store the original pose count before the count gets modified
    // because of a missing 0 index.
    // The original index list is important when querying the last index
    // of the array, see below *).
    unsigned poseCountOriginal = poseCount;

    // Make sure to start at a pose index of 0.
    // Because Maya creates sparse arrays it's possible that the first
    // pose gets lost when a rest pose is present which only contain
    // zero values.
    if (poseCount != 0 && poseIds[0] != 0)
    {
        poseIds.insert(0, 0);
        poseCount ++;
    }
    // Problem: *)
    // When loading a scene with the weightDriver node the index count
    // of the poses plug (compound array attribute) matches the number
    // of poses, whereas once the scene gets evaluated the plug array
    // contains an additional empty (next available) index.
    // Since the correct number of poses needs to be known for creating
    // the matrices, the last index gets checked. If the child
    // attributes have elements in case of a freshly loaded scene, the
    // pose count doesn't need to be altered. But when the scene already
    // has been evaluated the children of the last index don't have any
    // elements and therefore can be ignored.
    status = posesHandle.jumpToArrayElement(poseCountOriginal - 1);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MDataHandle lastIdHandle = posesHandle.inputValue();
    MDataHandle lastInputHandle = lastIdHandle.child(poseInput);
    MArrayDataHandle lastInputArrayHandle(lastInputHandle);
    unsigned lastInCount = lastInputArrayHandle.elementCount();
    if (lastInCount == 0)
        poseCount --;

    // Check for any pose connections. In case the pose attributes are
    // connected all data need to get re-evaluated, which slows down the
    // calculation.
    unsigned int numConnChildren = posesPlug.numConnectedChildren();
    if (numConnChildren != 0 || poseCount != globalPoseCount)
        evalInput = true;

    // Clear the indices for setting the output array values because
    // valid indices get appended.
    poseMatrixIds.clear();

    // -----------------------------------------------------------------
    // fill the driver and rest vector
    // -----------------------------------------------------------------

    std::vector<double> rest;
    rest.resize(inDim);

    for (i = 0; i < inDim; i ++)
    {
        status = inputHandle.jumpToArrayElement(i);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        driver[i] = inputHandle.inputValue().asDouble();

        // get the rest input
        if (i < restDim)
        {
            status = restInputHandle.jumpToArrayElement(i);
            CHECK_MSTATUS_AND_RETURN_IT(status);
            rest[i] = restInputHandle.inputValue().asDouble();
        }
        else
            rest[i] = 0.0;

        if (distanceTypeVal)
            driver[i] -= rest[i];
        else
            rest[i] = 0.0;
    }

    // -----------------------------------------------------------------
    // get the pose data
    // -----------------------------------------------------------------

    if (poseCount != 0 && evalInput)
    {
        globalPoseCount = poseCount;

        // Prepare the matrix to hold the pose vectors.
        // Assign an empty matrix to clear pre-existing data.
        poseData = BRMatrix();
        poseData.setSize(poseCount, inDim);

        // Prepare the matrix to hold the pose values.
        // Assign an empty matrix to clear pre-existing data.
        poseVals = BRMatrix();
        poseVals.setSize(poseCount, solveCount);

        // Clear pre-existing mode modes.
        poseModes.clear();
        poseModes.setLength(poseCount);

        for (i = 0; i < poseCount; i ++)
        {
            poseMatrixIds.append((int)i);

            // Fill with zeros in case an array index is missing because
            // of sparse arrays.
            for (j = 0; j < inDim; j ++)
                poseData(i, j) = 0.0 - rest[j];
            for (j = 0; j < solveCount; j ++)
                poseVals(i, j) = 0.0;

            // ---------------------------------------------------------
            // pose positions
            // ---------------------------------------------------------

            status = posesHandle.jumpToArrayElement(i);
            if (status == MStatus::kSuccess)
            {
                MDataHandle posesIdHandle = posesHandle.inputValue();
                MDataHandle poseInputHandle = posesIdHandle.child(poseInput);
                MArrayDataHandle poseInputArrayHandle(poseInputHandle);

                unsigned poseInputCount = poseInputArrayHandle.elementCount();

                for (j = 0; j < inDim; j ++)
                {
                    // Handle the special case of sparse arrays which
                    // might hold less data than is needed.
                    if (poseInputCount != 0)
                    {
                        status = poseInputArrayHandle.jumpToElement(j);
                        if (status == MStatus::kSuccess)
                            poseData(i, j) = poseInputArrayHandle.inputValue().asDouble() - rest[j];
                    }
                }

                // -----------------------------------------------
                // pose values
                // -----------------------------------------------

                MDataHandle poseValueHandle = posesIdHandle.child(poseValue);
                MArrayDataHandle poseValueArrayHandle(poseValueHandle);

                unsigned valueCount = poseValueArrayHandle.elementCount();

                for (j = 0; j < solveCount; j ++)
                {
                    // Handle the special case of sparse arrays which
                    // might hold less data than is needed.
                    if (valueCount != 0)
                    {
                        status = poseValueArrayHandle.jumpToElement(j);
                        if (status == MStatus::kSuccess)
                            poseVals(i, j) = poseValueArrayHandle.inputValue().asDouble();
                    }
                }
            }

            // -----------------------------------------------
            // pose modes
            // -----------------------------------------------

            // Set the pose mode value. This is not necessary for
            // generic mode, but only to make the data for both modes
            // consistent.
            poseModes.set(0, i);
        }
    }

    return MStatus::kSuccess;
}


//
// Description:
//      Calculate the twist angle based on the given rotate order.
//
// Input Arguments:
//      q               The quaternion to get the twist angle from.
//      axis            The twist axis.
//
// Return Value:
//      double          The twist angle.
//
double weightDriver::getTwistAngle(MQuaternion q, unsigned int axis)
{
    double axisComponent = q.x;
    if (axis == 1)
        axisComponent = q.y;
    else if (axis == 2)
        axisComponent = q.z;
    return 2.0 * atan2(axisComponent, q.w);
}


//
// Description:
//      Build a matrix containing the distance values between all poses.
//      Based on all distances also calculate the general mean distance
//      which is needed for the RBF calculation.
//
// Input Arguments:
//      poseMat         The matrix containing all poses.
//      meanDist        The average distance between the poses.
//      distType        The distance type (linear/angle).
//
// Return Value:
//      BRMatrix        The twist angle.
//
BRMatrix weightDriver::getDistances(BRMatrix poseMat, double &meanDist, int distType)
{
    unsigned count = poseMat.getRowSize();

    unsigned int i, j;

    BRMatrix distMat;
    distMat.setSize(count, count);

    double sum = 0.0;

    for (i = 0; i < count; i ++)
    {
        for (j = 0; j < count; j ++)
        {
            double dist = getPoseDelta(poseMat.getRowVector(i), poseMat.getRowVector(j), distType);
            distMat(i, j) = dist;
            sum += dist;
        }
    }

    meanDist = sum / (count * count);

    return distMat;
}


//
// Description:
//      Return the distance between the two given vectors based on the
//      distance type (linear/angle) or the vector components in case
//      of the generic RBF mode.
//
// Input Arguments:
//      vec1            The first vector.
//      vec2            The second vector.
//      distType        The distance type (linear/angle).
//
// Return Value:
//      double          The distance value.
//
double weightDriver::getPoseDelta(std::vector<double> vec1, std::vector<double> vec2, int distType)
{
    double dist = 0.0;
    if (distType == 0)
        dist = getRadius(vec1, vec2);
    else
    {
        if (vec1.size() == 3 && vec2.size() == 3)
            dist = getAngle(vec1, vec2);
        else
            dist = getRadius(vec1, vec2);
    }
    return dist;
}


//
// Description:
//      Calculate the linear distance between two vectors.
//
// Input Arguments:
//      vec1            The first vector.
//      vec2            The second vector.
//
// Return Value:
//      double          The linear distance.
//
double weightDriver::getRadius(std::vector<double> vec1, std::vector<double> vec2)
{
    size_t count = vec1.size();

    double sum = 0.0;
    for (unsigned i = 0; i < count; i ++)
        sum += pow(vec1[i] - vec2[i], 2);
    return sqrt(sum);
}


//
// Description:
//      Calculate the angle between two vectors.
//
// Input Arguments:
//      vec1            The first vector.
//      vec2            The second vector.
//
// Return Value:
//      double          The angle value.
//
double weightDriver::getAngle(std::vector<double> vec1, std::vector<double> vec2)
{
    MVector v1(vec1[0], vec1[1], vec1[2]);
    MVector v2(vec2[0], vec2[1], vec2[2]);
    return v1.angle(v2);
}


//
// Description:
//      Calculate the RBF activation values.
//
// Input Arguments:
//      mat             The matrix with the activation values.
//      width           The activation width.
//      kernelType      The interpolation function.
//
// Return Value:
//      None
//
void weightDriver::getActivations(BRMatrix &mat, double width, short kernelType)
{
    unsigned count = mat.getRowSize();

    unsigned int i, j;

    for (i = 0; i < count; i ++)
    {
        for (j = 0; j < count; j ++)
            mat(i, j) = interpolateRbf(mat(i, j), width, kernelType);
    }
}


//
// Description:
//      Interpolation function for processing the weight values.
//
// Input Arguments:
//      value           The value to interpolate.
//      width           The activation width.
//      kernelType      The interpolation function.
//
// Return Value:
//      double          The new interpolated value.
//
double weightDriver::interpolateRbf(double value, double width, short kernelType)
{
    double result = 0.0;

    // gaussian
    if (kernelType == 0)
    {
        if (width == 0.0)
            width = 1.0;
        width = 1.0 / width;
        double sigma = -(width * width);
        result = exp(sigma * value);
    }
    // linear
    else if (kernelType == 1)
    {
        result = value;
    }

    return result;
}


//
// Description:
//      Calculate the individual output weights based on the current
//      driver values in relation to the stored poses. This is the main
//      part of the RBF calculation but a rather simple process as it
//      just gets the distances of the driver to the stored poses and
//      calculates the weighted output values based on the weight matrix
//      built during initialization.
//
// Input Arguments:
//      out             The array of output weight values.
//      poses           The matrix containing all poses.
//      driver          The array of driver values.
//      poseModes       The array containing the the mode per pose.
//      weightMat       The matrix with the RBF weights.
//      avgDist         The average distance between the poses.
//      distType        The distance type (linear/angle).
//      kernelType      The interpolation function.
//
// Return Value:
//      None
//
void weightDriver::getPoseWeights(MDoubleArray &out,
                                  BRMatrix poses,
                                  std::vector<double> driver,
                                  MIntArray poseModes,
                                  BRMatrix weightMat,
                                  double avgDist,
                                  int distType,
                                  short kernelType)
{
    unsigned int poseCount = poses.getRowSize();
    unsigned int valueCount = out.length();

    // Make sure that the weight matrix has the correct dimensions.
    // This has become necessary with introducing multiple drivers in
    // matrix mode.
    if (weightMat.getRowSize() != poseCount || weightMat.getColSize() != valueCount)
        return;

    unsigned int i, j;

    for (i = 0; i < poseCount; i ++)
    {
        double dist = 0.0;
        std::vector<double> dv = driver;
        std::vector<double> ps = poses.getRowVector(i);

        if (poseModes[i] == 1)
            dv[3] = 0.0;
        else if (poseModes[i] == 2)
        {
            dv[0] = 0.0;
            dv[1] = 0.0;
            dv[2] = 0.0;
        }

        dist = getPoseDelta(dv, ps, distType);

        for (j = 0; j < valueCount; j ++)
            out[j] += weightMat(i, j) * interpolateRbf(dist, avgDist, kernelType);
    }
}


//
// Description:
//      Pass the weight values to the outputs.
//
// Input Arguments:
//      weightsArray    The array of output weight values.
//      data            The MPxNode dataBlock.
//      inactive        True, if the node is enabled.
//
// Return Value:
//      None
//
void weightDriver::setOutputValues(MDoubleArray weightsArray, MDataBlock data, bool inactive)
{
    MStatus status = MStatus::kSuccess;
    
    MObject thisNode = this->thisMObject();
    
    unsigned int i;

    // In generic mode pose and output indices are not related.
    // The ordering of the output always starts at 0 with an increment
    // of 1, no matter if pose indices are missing.
    // In matrix mode pose and output indices are matching, due to the
    // square dimensions of blendshape usage.
    unsigned count = 0;
    MIntArray ids;
    if (genericMode)
    {
        if (!inactive)
        {
            count = weightsArray.length();
            ids.setLength(count);
            for (i = 0; i < count; i ++)
                ids.set((int)i, i);
        }
        else
        {
            MPlug outputPlug(thisNode, weightDriver::output);
            outputPlug.getExistingArrayAttributeIndices(ids, &status);
            if (status != MStatus::kSuccess)
                return;
            count = ids.length();
        }
    }
    else
    {
        count = poseMatrixIds.length();
        ids = poseMatrixIds;
    }

    MArrayDataHandle outputHandle = data.outputArrayValue(output);
    MArrayDataBuilder outputBuilder(&data, output, count);
    for (i = 0; i < count; i ++)
    {
        MDataHandle outputIdHandle = outputBuilder.addElement((unsigned)ids[i]);
        if (!inactive)
            outputIdHandle.setDouble(weightsArray[i]);
        else
            outputIdHandle.setDouble(0.0);

        // If the node is set up for rbf but switched back to vector
        // angle all other output weights need to be set to 0.
        if (weightsArray.length() == 1 && i > 0)
            outputIdHandle.setDouble(0.0);

        outputHandle.set(outputBuilder);
    }
    outputHandle.setAllClean();
}


//
// Description:
//      Modify the value by shifting it towards the lower or upper end
//      of the interpolation range.
//
// Input Arguments:
//      value           The value to shift.
//      biasValue       The bias value.
//
// Return Value:
//      double          The new value with bias.
//
double weightDriver::rbfWeightBias(double value, double biasValue)
{
    if (biasValue >= 0.0)
    {
        value = fabs(value) * pow(fabs(value), biasValue);
    }
    else
    {
        double baseVal = 1 - fabs(value);
        // If the value is 1 the bias transformation has zero at the
        // base and outputs NaN.
        if (fabs(baseVal) > DOUBLE_EPSILON)
            value = 1 - pow(baseVal, (1 + fabs(biasValue)));
        else
            value = 1;
    }

    return value;
}


//
// Description:
//      Modify the output weight value by the chosen interpolation type.
//
// Input Arguments:
//      value           The value to interpolate.
//      type            The type of interpolation.
//
// Return Value:
//      double          The new interpolated value.
//
double weightDriver::interpolateWeight(double value, int type)
{
    // slow - inverse quadratic
    if (type == 1)
        value = 1 - pow((1 - value), 2.0);
    // fast - quadratic
    else if (type == 2)
        value = 1 - pow((1 - value), 1 / 2.0);
    // smooth1 - smoothstep
    else if (type == 3)
        value = value * value * (3 - 2 * value);
    // smooth2 - smootherstep
    else if (type == 4)
        value = value * value * value * (value * (value * 6 - 15) + 10);
    else if (type == 5)
        value = blendCurveWeight(value);

    return value;
}


//
// Description:
//      Return the blend curve weight value at the given position.
//
// Input Arguments:
//      value           The input value of the blend curve.
//
// Return Value:
//      double          The blend curve output value.
//
double weightDriver::blendCurveWeight(double value)
{
    float curveValue;
    curveAttr.getValueAtPosition((float)value, curveValue);
    value = curveValue;

    return value;
}


// ---------------------------------------------------------------------
// Helper functions to display the various data elements of the RBF
// calculation process.
// ---------------------------------------------------------------------
void weightDriver::showArray(MDoubleArray array, MString name)
{
    unsigned int i;

    MString s(name + ":\n");

    for (i = 0; i < array.length(); i++)
        s += MString(" ") + array[i];

    MGlobal::displayInfo(s);
}

void weightDriver::showVector(MVector vector, MString name)
{
    unsigned int i;

    MString s(name + ":\n");

    for (i = 0; i < 3; i++)
        s += MString(" ") + vector[i];

    MGlobal::displayInfo(s);
}

void weightDriver::showMatrix(MMatrix mat, MString name)
{
    unsigned int i, j;

    MString s(name + ":\n");

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
            s += MString(" ") + mat[i][j];

        s += MString("\n");
    }

    MGlobal::displayInfo(s);
}


// ---------------------------------------------------------------------
//
// Draw the locator for the legacy viewport.
//
// This applies to the vector angle reader and RBF solver.
//
// ---------------------------------------------------------------------
#if MAYA_API_VERSION < 201900

void weightDriver::draw(M3dView &view,
                        const MDagPath &path,
                        M3dView::DisplayStyle style,
                        M3dView::DisplayStatus status)
{
    MObject thisNode = this->thisMObject();

    MStatus stat = MStatus::kSuccess;

    unsigned int i;

    // -----------------------------------------------------------------
    // get the attributes
    // -----------------------------------------------------------------

    MPlug activePlug(thisNode, weightDriver::active);
    MPlug colorRPlug(thisNode, weightDriver::colorR);
    MPlug colorGPlug(thisNode, weightDriver::colorG);
    MPlug colorBPlug(thisNode, weightDriver::colorB);
    MPlug drawCenterPlug(thisNode, weightDriver::drawCenter);
    MPlug drawConePlug(thisNode, weightDriver::drawCone);
    MPlug drawDriverPlug(thisNode, weightDriver::drawDriver);
    MPlug drawIndicesPlug(thisNode, weightDriver::drawIndices);
    MPlug drawOriginPlug(thisNode, weightDriver::drawOrigin);
    MPlug drawPosesPlug(thisNode, weightDriver::drawPoses);
    MPlug drawTwistPlug(thisNode, weightDriver::drawTwist);
    MPlug drawWeightPlug(thisNode, weightDriver::drawWeight);
    //MPlug driverIndexPlug(thisNode, weightDriver::driverIndex);
    MPlug indexDistPlug(thisNode, weightDriver::indexDist);
    MPlug outWeightPlug(thisNode, weightDriver::outWeight);
    MPlug poseLengthPlug(thisNode, weightDriver::poseLength);
    MPlug rbfModePlug(thisNode, weightDriver::rbfMode);
    MPlug sizePlug(thisNode, weightDriver::size);

    bool activeVal = activePlug.asBool();
    float colorRVal = (float)colorRPlug.asDouble();
    float colorGVal = (float)colorGPlug.asDouble();
    float colorBVal = (float)colorBPlug.asDouble();
    double sizeVal = sizePlug.asDouble();
    bool drawCenterVal = drawCenterPlug.asBool();
    bool drawConeVal = drawConePlug.asBool();
    bool drawDriverVal = drawDriverPlug.asBool();
    bool drawIndicesVal = drawIndicesPlug.asBool();
    bool drawOriginVal = drawOriginPlug.asBool();
    bool drawPosesVal = drawPosesPlug.asBool();
    bool drawTwistVal = drawTwistPlug.asBool();
    bool drawWeightVal = drawWeightPlug.asBool();
    //int driverIndexVal = driverIndexPlug.asInt();
    double indexDistVal = indexDistPlug.asDouble();
    double poseLengthVal = poseLengthPlug.asDouble();
    short rbfModeVal = rbfModePlug.asShort();
    double weightVal = outWeightPlug.asDouble();

    if (!activeVal)
        return;

    if (invVal && typeVal == 0)
        sizeVal *= -1;

    // -----------------------------------------------------------------
    // GL part
    // -----------------------------------------------------------------

    MColor lineColor;

    if (status == view.kLead)
        lineColor = MColor(0.26f, 1.0f, 0.64f);
    else if (status == view.kActive)
        lineColor = MColor(1.0f, 1.0f, 1.0f);
    else if (status == view.kActiveAffected)
        lineColor = MColor(0.78f, 0.0f, 0.78f);
    else if (status == view.kTemplate)
        lineColor = MColor(0.47f, 0.47f, 0.47f);
    else if (status == view.kActiveTemplate)
        lineColor = MColor(1.0f, 0.47f, 0.47f);
    else
        lineColor = MColor(colorRVal, colorGVal, colorBVal);

    view.setDrawColor(lineColor);

    view.beginGL();

    glPushAttrib(GL_CURRENT_BIT);

    // -----------------------------------------------------------------
    // vector angle cone
    // -----------------------------------------------------------------

    if (typeVal == 0)
    {
        // -------------------------------------------------------------
        // get the driver node name
        // -------------------------------------------------------------

        MString driverName;
        MPlug driverPlug(thisNode, weightDriver::driverMatrix);
        if (driverPlug.isConnected())
        {
            MPlugArray plugConn;
            driverPlug.connectedTo(plugConn, true, false, &stat);
            if (stat == MStatus::kSuccess)
            {
                driverName = plugConn[0].name();
                MStringArray items;
                driverName.split('.', items);
                driverName = items[0];
            }
        }

        // -------------------------------------------------------------
        // draw lines
        // -------------------------------------------------------------

        if (drawConeVal)
        {
            glBegin(GL_LINES);

            unsigned int steps = 16;
            double angleStep = 2.0 * M_PI / steps;
            double increment = 0.0;

            MFloatArray firstPoint;
            MFloatArray coords;
            int skip = 0;

            // Modify the angle and size values if the cone should be
            // drawn past 90 degrees.
            double drawAngle = angleVal;
            double drawSize = sizeVal;
            if (angleVal > 90.0)
            {
                drawAngle = 180.0 - angleVal + 0.000000001;
                drawSize *= -1.0;
            }

            double angleRadians = drawAngle * DEGTORAD;
            double radius = sin(angleRadians);
            float height = (float)((radius / tan(angleRadians)) * drawSize);

            for (i = 0; i < steps; i ++)
            {
                float ptX = (float)(radius * cos(increment) * drawSize);
                float ptY = (float)(radius * sin(increment) * drawSize);

                MFloatArray dirReorder;
                if (dirVal == 0)
                {
                    dirReorder.append(height);
                    dirReorder.append(ptX);
                    dirReorder.append(ptY);
                }
                else if (dirVal == 1)
                {
                    dirReorder.append(ptX);
                    dirReorder.append(height);
                    dirReorder.append(ptY);
                }
                else
                {
                    dirReorder.append(ptX);
                    dirReorder.append(ptY);
                    dirReorder.append(height);
                }

                coords.append(dirReorder[0]);
                coords.append(dirReorder[1]);
                coords.append(dirReorder[2]);

                // Store the first point which is needed for the last
                // segment as the end point.
                if (firstPoint.length() == 0)
                {
                    firstPoint.append(dirReorder[0]);
                    firstPoint.append(dirReorder[1]);
                    firstPoint.append(dirReorder[2]);
                }

                // If there are six values draw the segment.
                if (coords.length() == 6)
                {
                    glVertex3f(coords[0], coords[1], coords[2]);
                    glVertex3f(coords[3], coords[4], coords[5]);
                    coords.remove(0);
                    coords.remove(0);
                    coords.remove(0);
                }

                // Draw a line from the cone tip every other segment.
                if (skip == 0)
                {
                    glVertex3f(0.0, 0.0, 0.0);
                    glVertex3f(coords[0], coords[1], coords[2]);
                    skip = 1;
                }
                else
                    skip = 0;

                increment += angleStep;
            }

            // Draw the last segment.
            coords.append(firstPoint[0]);
            coords.append(firstPoint[1]);
            coords.append(firstPoint[2]);
            glVertex3f(coords[0], coords[1], coords[2]);
            glVertex3f(coords[3], coords[4], coords[5]);

            glEnd();
        }

        if (drawCenterVal && drawConeVal)
        {
            glBegin(GL_LINES);

            unsigned int steps = 16;
            double angleStep = 2.0 * M_PI / steps;
            double increment = 0.0;

            MFloatArray firstPoint;
            MFloatArray coords;
            int skip = 0;

            // Modify the angle and size values if the cone should be
            // drawn past 90 degrees.
            double drawAngle = centerAngleVal;
            double drawSize = sizeVal;
            if (centerAngleVal > 90.0)
            {
                drawAngle = 180.0 - centerAngleVal;
                drawSize *= -1.0;
            }

            double angleRadians = (drawAngle + 0.000000001) * DEGTORAD;
            double radius = sin(angleRadians);
            float height = (float)((radius / tan(angleRadians)) * drawSize);

            for (i = 0; i < steps; i ++)
            {
                float ptX = (float)(radius * cos(increment) * drawSize);
                float ptY = (float)(radius * sin(increment) * drawSize);

                MFloatArray dirReorder;
                if (dirVal == 0)
                {
                    dirReorder.append(height);
                    dirReorder.append(ptX);
                    dirReorder.append(ptY);
                }
                else if (dirVal == 1)
                {
                    dirReorder.append(ptX);
                    dirReorder.append(height);
                    dirReorder.append(ptY);
                }
                else
                {
                    dirReorder.append(ptX);
                    dirReorder.append(ptY);
                    dirReorder.append(height);
                }

                coords.append(dirReorder[0]);
                coords.append(dirReorder[1]);
                coords.append(dirReorder[2]);

                // Store the first point which is needed for the last
                // segment as the end point.
                if (firstPoint.length() == 0)
                {
                    firstPoint.append(dirReorder[0]);
                    firstPoint.append(dirReorder[1]);
                    firstPoint.append(dirReorder[2]);
                }

                // If there are six values draw the segment.
                if (coords.length() == 6)
                {
                    glVertex3f(coords[0], coords[1], coords[2]);
                    glVertex3f(coords[3], coords[4], coords[5]);
                    coords.remove(0);
                    coords.remove(0);
                    coords.remove(0);
                }

                // Draw a line from the cone tip every other segment.
                if (skip == 0)
                {
                    glVertex3f(0.0, 0.0, 0.0);
                    glVertex3f(coords[0], coords[1], coords[2]);
                    skip = 1;
                }
                else
                    skip = 0;

                increment += angleStep;
            }

            // Draw the last segment.
            coords.append(firstPoint[0]);
            coords.append(firstPoint[1]);
            coords.append(firstPoint[2]);
            glVertex3f(coords[0], coords[1], coords[2]);
            glVertex3f(coords[3], coords[4], coords[5]);

            glEnd();
        }

        // -------------------------------------------------------------
        // draw weight value
        // -------------------------------------------------------------

        if (drawWeightVal)
        {
            MPoint drawPoint;
            if (dirVal == 0)
                drawPoint = MPoint(sizeVal, 0.0, 0.0);
            else if (dirVal == 1)
                drawPoint = MPoint(0.0, sizeVal, 0.0);
            else
                drawPoint = MPoint(0.0, 0.0, sizeVal);

            char info[512];
        #ifdef _WIN64
            sprintf_s(info, "%s %.3f", driverName.asChar(), weightVal);
        #else
            sprintf(info, "%s %.3f", driverName.asChar(), weightVal);
        #endif
            if (!invVal)
                view.drawText(info, drawPoint, M3dView::kLeft);
            else
                view.drawText(info, drawPoint, M3dView::kRight);
        }
    }

    // -----------------------------------------------------------------
    // rbf sphere
    // -----------------------------------------------------------------

    // Draw the rbf elements only when in transform mode.
    else if (rbfModeVal == 1)
    {
        // Remove the driver vector at the end of the array for the
        // pose count.
        unsigned int poseVectorArraySize = poseVectorArray.length();
        unsigned int poseCount = poseVectorArraySize - 1;
        double lineSize = poseLengthVal * sizeVal;

        if (poseCount < 1)
        {
            glPopAttrib();
            view.endGL();
            return;
        }

        if (drawOriginVal)
        {
            // In order to draw the circle in 3d space but oriented to
            // the view get the model view matrix and reset the
            // translation and scale. The points to draw are then
            // multiplied by the inverse matrix.
            MMatrix modelViewMat;
            view.modelViewMatrix(modelViewMat);
            MTransformationMatrix transMat(modelViewMat);
            transMat.setTranslation(MVector(0.0, 0.0, 0.0), MSpace::kWorld);
            const double scale[3] = {1.0, 1.0, 1.0};
            transMat.setScale(scale, MSpace::kWorld);
            modelViewMat = transMat.asMatrix();

            glBegin(GL_LINE_LOOP);

            for (i = 0; i < 360; i +=2)
            {
                double degInRad = i * DEGTORAD;
                MPoint point(cos(degInRad) * sizeVal, sin(degInRad) * sizeVal, 0.0);
                point *= modelViewMat.inverse();
                glVertex3f((float)point.x, (float)point.y, (float)point.z);
            }

            glEnd();
        }

        if (drawDriverVal)
        {
            glBegin(GL_LINES);

            MVector dv = poseVectorArray[poseVectorArraySize - 1];
            dv.normalize();
            MPoint point(dv.x * lineSize, dv.y * lineSize, dv.z * lineSize);
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f((float)point.x, (float)point.y, (float)point.z);

            glEnd();

            if (drawTwistVal)
            {
                dv *= 0.9 + indexDistVal;

                point = MPoint(dv.x * lineSize, dv.y * lineSize, dv.z * lineSize);

                char info[64];
            #ifdef _WIN64
                sprintf_s(info, "%.2f", poseTwistArray[poseVectorArraySize - 1] * RADTODEG);
            #else
                sprintf(info, "%.2f", poseTwistArray[poseVectorArraySize - 1] * RADTODEG);
            #endif

                view.drawText(info, point, M3dView::kRight);
            }
        }

        if (drawPosesVal)
        {
            if (poseCount != 0)
            {
                for (i = 0; i < (unsigned)poseCount; i ++)
                {
                    glBegin(GL_LINES);

                    MVector pv = poseVectorArray[i];
                    pv.normalize();
                    MPoint point(pv.x * lineSize, pv.y * lineSize, pv.z * lineSize);
                    glVertex3f(0.0, 0.0, 0.0);
                    glVertex3f((float)point.x, (float)point.y, (float)point.z);

                    glEnd();

                    if (drawTwistVal)
                    {
                        pv *= 0.9 + indexDistVal;

                        point = MPoint(pv.x * lineSize, pv.y * lineSize, pv.z * lineSize);

                        char info[64];
                    #ifdef _WIN64
                        sprintf_s(info, "%.2f", poseTwistArray[i] * RADTODEG);
                    #else
                        sprintf(info, "%.2f", poseTwistArray[i] * RADTODEG);
                    #endif

                        view.drawText(info, point, M3dView::kRight);
                    }
                }
            }
        }

        if (drawIndicesVal)
        {
            if (poseCount != 0)
            {
                for (i = 0; i < (unsigned)poseCount; i ++)
                {
                    MVector pv = poseVectorArray[i];
                    pv.normalize();
                    pv *= 1.03 + indexDistVal;

                    MPoint point(pv.x * lineSize, pv.y * lineSize, pv.z * lineSize);

                    char info[64];
                #ifdef _WIN64
                    sprintf_s(info, "%i", poseMatrixIds[i]);
                #else
                    sprintf(info, "%i", poseMatrixIds[i]);
                #endif

                    view.drawText(info, point, M3dView::kCenter);
                }
            }
        }
    }

    glPopAttrib();
    view.endGL();
}
#endif


// ---------------------------------------------------------------------
//
// Viewport 2.0
//
// ---------------------------------------------------------------------
#if MAYA_API_VERSION >= 201400

MString weightDriver::drawDbClassification("drawdb/geometry/weightDriver");
MString weightDriver::drawRegistrantId("weightDriverNodePlugin");

// VP2.0 API for Maya 2016.5 and later.
//
// The overrides for Maya 2016.5 and later are different than for
// earlier versions of Maya.
#if MAYA_API_VERSION >= 201650

// By setting isAlwaysDirty to false in MPxDrawOverride constructor, the
// draw override will be updated (via prepareForDraw()) only when the
// node is marked dirty via DG evaluation or dirty propagation.
// Additional callback is also added to explicitly mark the node as
// being dirty (via MRenderer::setGeometryDrawDirty()) for certain
// circumstances.
// Note that the draw callback in MPxDrawOverride constructor is set to
// NULL in order to achieve better performance.

weightDriverOverride::weightDriverOverride(const MObject &obj)
: MHWRender::MPxDrawOverride(obj, NULL, true)
{
    fModelEditorChangedCbId = MEventMessage::addEventCallback("modelEditorChanged",
                                                              OnModelEditorChanged, this);

    MStatus status;
    MFnDependencyNode node(obj, &status);
    fWeightDriver = status ? dynamic_cast<weightDriver*>(node.userNode()) : NULL;
}


weightDriverOverride::~weightDriverOverride()
{
    fWeightDriver = NULL;

    if (fModelEditorChangedCbId != 0)
    {
        MMessage::removeCallback(fModelEditorChangedCbId);
        fModelEditorChangedCbId = 0;
    }
}


void weightDriverOverride::OnModelEditorChanged(void *clientData)
{
    weightDriverOverride *ovr = static_cast<weightDriverOverride*>(clientData);
    if (ovr && ovr->fWeightDriver)
    {
        MHWRender::MRenderer::setGeometryDrawDirty(ovr->fWeightDriver->thisMObject());
    }
}


MHWRender::DrawAPI weightDriverOverride::supportedDrawAPIs() const
{
    return (MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile);
}


//
// VP2.0 API for Maya 2016 and earlier.
//
#else
weightDriverOverride::weightDriverOverride(const MObject &obj)
: MHWRender::MPxDrawOverride(obj, weightDriverOverride::draw)
{
}


weightDriverOverride::~weightDriverOverride()
{
}
#endif


MBoundingBox weightDriverOverride::boundingBox(const MDagPath &objPath,
                                               const MDagPath &cameraPath) const
{
    MStatus status;
    MObject thisNode = objPath.node(&status);
    MPlug sizePlug(thisNode, weightDriver::size);
    double sizeMult = sizePlug.asDouble();
    MPlug typePlug(thisNode, weightDriver::type);
    short typeVal = typePlug.asShort();

    int xCorner = 0;
    if (typeVal == 1)
        xCorner = -1;

    MPoint corner1 = MPoint(xCorner, -1, -1);
    MPoint corner2 = MPoint(1, 1, 1);

    corner1 = corner1 * sizeMult;
    corner2 = corner2 * sizeMult;

    return MBoundingBox(corner1, corner2);
}


MUserData* weightDriverOverride::prepareForDraw(const MDagPath &objPath,
                                                const MDagPath &cameraPath,
                                                const MHWRender::MFrameContext &frameContext,
                                                MUserData *oldData)
{
    weightDriverData* data = dynamic_cast<weightDriverData*>(oldData);
    if (!data)
        data = new weightDriverData();

    // -----------------------------------------------
    // get the attributes
    // -----------------------------------------------

    MStatus status;
    MObject thisNode = objPath.node(&status);

    MPlug activePlug(thisNode, weightDriver::active);
    MPlug anglePlug(thisNode, weightDriver::angle);
    MPlug centerAnglePlug(thisNode, weightDriver::centerAngle);
    MPlug colorDriverRPlug(thisNode, weightDriver::colorDriverR);
    MPlug colorDriverGPlug(thisNode, weightDriver::colorDriverG);
    MPlug colorDriverBPlug(thisNode, weightDriver::colorDriverB);
    MPlug colorRPlug(thisNode, weightDriver::colorR);
    MPlug colorGPlug(thisNode, weightDriver::colorG);
    MPlug colorBPlug(thisNode, weightDriver::colorB);
    MPlug dirPlug(thisNode, weightDriver::direction);
    MPlug drawCenterPlug(thisNode, weightDriver::drawCenter);
    MPlug drawConePlug(thisNode, weightDriver::drawCone);
    MPlug drawDriverPlug(thisNode, weightDriver::drawDriver);
    MPlug drawIndicesPlug(thisNode, weightDriver::drawIndices);
    MPlug drawOriginPlug(thisNode, weightDriver::drawOrigin);
    MPlug drawPosesPlug(thisNode, weightDriver::drawPoses);
    MPlug drawTwistPlug(thisNode, weightDriver::drawTwist);
    MPlug drawWeightPlug(thisNode, weightDriver::drawWeight);
    MPlug driverIndexPlug(thisNode, weightDriver::driverIndex);
    MPlug indexDistPlug(thisNode, weightDriver::indexDist);
    MPlug invertPlug(thisNode, weightDriver::invert);
    MPlug poseLengthPlug(thisNode, weightDriver::poseLength);
    MPlug rbfModePlug(thisNode, weightDriver::rbfMode);
    MPlug sizePlug(thisNode, weightDriver::size);
    MPlug typePlug(thisNode, weightDriver::type);
    MPlug weightPlug(thisNode, weightDriver::outWeight);

    data->activeVal = activePlug.asBool();
    data->angleVal = anglePlug.asDouble();
    data->centerAngleVal = centerAnglePlug.asDouble();
    data->dirVal = dirPlug.asShort();
    data->drawCenterVal = drawCenterPlug.asBool();
    data->drawConeVal = drawConePlug.asBool();
    data->drawDriverVal = drawDriverPlug.asBool();
    data->drawIndicesVal = drawIndicesPlug.asBool();
    data->drawOriginVal = drawOriginPlug.asBool();
    data->drawPosesVal = drawPosesPlug.asBool();
    data->drawTwistVal = drawTwistPlug.asBool();
    data->drawWeightVal = drawWeightPlug.asBool();
    data->driverIndexVal = driverIndexPlug.asInt();
    data->indexDistVal = indexDistPlug.asDouble();
    data->invVal = invertPlug.asBool();
    data->poseLengthVal = poseLengthPlug.asDouble();
    data->rbfModeVal = rbfModePlug.asShort();
    data->sizeVal = sizePlug.asDouble();
    data->typeVal = typePlug.asShort();
    data->weightVal = weightPlug.asDouble();

    MHWRender::DisplayStatus stat = MHWRender::MGeometryUtilities::displayStatus(objPath);

    MColor lineColor;
    if (stat == MHWRender::kLead)
        lineColor = MColor(0.26f, 1.0f, 0.64f);
    else if (stat == MHWRender::kActive)
        lineColor = MColor(1.0f, 1.0f, 1.0f);
    else if (stat == MHWRender::kActiveAffected)
        lineColor = MColor(0.78f, 0.0f, 0.78f);
    else if (stat == MHWRender::kTemplate)
        lineColor = MColor(0.47f, 0.47f, 0.47f);
    else if (stat == MHWRender::kActiveTemplate)
        lineColor = MColor(1.0f, 0.47f, 0.47f);
    else
        lineColor = MColor((float)colorRPlug.asDouble(), (float)colorGPlug.asDouble(), (float)colorBPlug.asDouble());

    data->colorRVal = lineColor.r;
    data->colorGVal = lineColor.g;
    data->colorBVal = lineColor.b;

    data->colorDriverRVal = colorDriverRPlug.asDouble();
    data->colorDriverGVal = colorDriverGPlug.asDouble();
    data->colorDriverBVal = colorDriverBPlug.asDouble();

    // Make sure that the center angle is always smaller then the angle.
    if (data->angleVal <= data->centerAngleVal)
        data->centerAngleVal = data->angleVal - 0.1;

    MFnCamera camFn(cameraPath);
    viewVector = camFn.viewDirection(MSpace::kWorld);

    return data;
}


void weightDriverOverride::addUIDrawables(const MDagPath &objPath,
                                          MHWRender::MUIDrawManager &drawManager,
                                          const MHWRender::MFrameContext &frameContext,
                                          const MUserData *data)
{
    MStatus status;

    MObject thisNode = objPath.node(&status);

    // Get the user draw data.
    const weightDriverData* wdData = dynamic_cast<const weightDriverData*>(data);
    if (!wdData)
        return;

    if (!wdData->activeVal)
        return;

    unsigned int i;

    MColor lineColor((float)wdData->colorRVal, (float)wdData->colorGVal, (float)wdData->colorBVal, 1.0f);
    MColor driverColor((float)wdData->colorDriverRVal, (float)wdData->colorDriverGVal, (float)wdData->colorDriverBVal, 1.0f);

    // -----------------------------------------------------------------
    // vector angle cone
    // -----------------------------------------------------------------

    if (wdData->typeVal == 0)
    {
        // -------------------------------------------------------------
        // get the driver node name
        // -------------------------------------------------------------

        MString driverName;
        MPlug driverPlug(thisNode, weightDriver::driverMatrix);
        if (driverPlug.isConnected())
        {
            MPlugArray sourcePlug;
            driverPlug.connectedTo(sourcePlug, true, false);
            driverName = sourcePlug[0].name();
            MStringArray items;
            driverName.split('.', items);
            driverName = items[0];
        }

        // -------------------------------------------------------------
        // draw lines
        // -------------------------------------------------------------

        if (wdData->drawConeVal)
        {
            drawManager.beginDrawable();

            drawManager.setColor(lineColor);

            MPoint base;
            MVector direction;

            // Modify the angle and size values if the cone should be
            // drawn past 90 degrees.
            double drawAngle = wdData->angleVal;
            double drawPos = wdData->sizeVal;
            int drawDir = -1;
            if (wdData->invVal)
            {
                drawPos *= -1.0;
                drawDir *= -1;
            }
            if (wdData->angleVal > 90.0)
            {
                drawAngle = 180.0 - wdData->angleVal + 0.000000001;
                drawPos *= -1.0;
                drawDir *= -1;
            }

            double angleRadians = drawAngle * DEGTORAD;
            double radius = sin(angleRadians);
            double height = (radius / tan(angleRadians)) * drawPos;

            if (wdData->dirVal == 0)
            {
                base = MPoint(height, 0.0, 0.0, 1.0);
                direction = MVector(drawDir, 0.0, 0.0);
            }
            else if (wdData->dirVal == 1)
            {
                base = MPoint(0.0, height, 0.0, 1.0);
                direction = MVector(0.0, drawDir, 0.0);
            }
            else
            {
                base = MPoint(0.0, 0.0, height, 1.0);
                direction = MVector(0.0, 0.0, drawDir);
            }

            drawManager.cone(base, direction, radius * wdData->sizeVal, height * (drawDir * -1.0));

            drawManager.endDrawable();
        }

        if (wdData->drawCenterVal && wdData->drawConeVal)
        {
            drawManager.beginDrawable();

            drawManager.setColor(lineColor);

            MPoint base;
            MVector direction;

            // Modify the angle and size values if the cone should be
            // drawn past 90 degrees.
            double drawAngle = wdData->centerAngleVal;
            double drawPos = wdData->sizeVal;
            int drawDir = -1;
            if (wdData->invVal)
            {
                drawPos *= -1.0;
                drawDir *= -1;
            }
            if (wdData->centerAngleVal > 90.0)
            {
                drawAngle = 180.0 - wdData->centerAngleVal;
                drawPos *= -1.0;
                drawDir *= -1;
            }

            double angleRadians = (drawAngle + 0.000000001) * DEGTORAD;
            double radius = sin(angleRadians);
            double height = (radius / tan(angleRadians)) * drawPos;

            if (wdData->dirVal == 0)
            {
                base = MPoint(height, 0.0, 0.0, 1.0);
                direction = MVector(drawDir, 0.0, 0.0);
            }
            else if (wdData->dirVal == 1)
            {
                base = MPoint(0.0, height, 0.0, 1.0);
                direction = MVector(0.0, drawDir, 0.0);
            }
            else
            {
                base = MPoint(0.0, 0.0, height, 1.0);
                direction = MVector(0.0, 0.0, drawDir);
            }

            drawManager.cone(base, direction, radius * wdData->sizeVal, height * (drawDir * -1.0));

            drawManager.endDrawable();
        }

        // -------------------------------------------------------------
        // draw weight value
        // -------------------------------------------------------------

        if (wdData->drawWeightVal)
        {
            drawManager.beginDrawable();

            drawManager.setColor(lineColor);

            double drawPos = wdData->sizeVal;
            if (wdData->invVal)
                drawPos *= -1.0;

            MPoint drawPoint;
            if (wdData->dirVal == 0)
                drawPoint = MPoint(drawPos, 0.0, 0.0);
            else if (wdData->dirVal == 1)
                drawPoint = MPoint(0.0, drawPos, 0.0);
            else
                drawPoint = MPoint(0.0, 0.0, drawPos);

            char info[512];
    #ifdef _WIN64
            sprintf_s(info, "%s %.3f", driverName.asChar(), wdData->weightVal);
    #else
            sprintf(info, "%s %.3f", driverName.asChar(), wdData->weightVal);
    #endif

            if (wdData->invVal == false)
                drawManager.text(drawPoint, info, MHWRender::MUIDrawManager::kLeft);
            else
                drawManager.text(drawPoint, info, MHWRender::MUIDrawManager::kRight);

            drawManager.endDrawable();
        }
    }

    // -----------------------------------------------------------------
    // rbf sphere
    // -----------------------------------------------------------------

    // draw the rbf elements only when in transform mode
    else if (wdData->rbfModeVal == 1)
    {
        double lineSize = wdData->poseLengthVal * wdData->sizeVal;

        // -------------------------------------------------------------
        // get the pose vectors
        // -------------------------------------------------------------

        MPlug poseDrawVectorPlug(thisNode, weightDriver::poseDrawVector);
        MPlug poseDrawTwistPlug(thisNode, weightDriver::poseDrawTwist);
        MIntArray poseIds;
        poseDrawVectorPlug.getExistingArrayAttributeIndices(poseIds, &status);
        if (status != MStatus::kSuccess || !poseIds.length())
            return;

        // Remove the driver vector at the end of the array for the pose
        // count.
        unsigned int poseIdsSize = poseIds.length();
        unsigned int poseCount = poseIdsSize - 1;
        MVectorArray poseVectors;
        poseVectors.setLength(poseCount);
        MDoubleArray poseTwist;
        poseTwist.setLength(poseCount);

        for (i = 0; i < poseCount; i ++)
        {
            MPlug posePlug = poseDrawVectorPlug.elementByLogicalIndex((unsigned)poseIds[i]);
            MDataHandle poseHandle = posePlug.asMDataHandle();
            double3 &poseValues = poseHandle.asDouble3();
            poseVectors.set(MVector(poseValues[0], poseValues[1], poseValues[2]), i);

            MPlug twistPlug = poseDrawTwistPlug.elementByLogicalIndex((unsigned)poseIds[i]);
            MDataHandle twistHandle = twistPlug.asMDataHandle();
            poseTwist.set(twistHandle.asDouble(), i);
        }

        // Get the values for the driver vector.
        MPlug posePlug = poseDrawVectorPlug.elementByLogicalIndex((unsigned)poseIds[poseIdsSize - 1]);
        MDataHandle poseHandle = posePlug.asMDataHandle();
        double3 &driverValues = poseHandle.asDouble3();

        MPlug twistPlug = poseDrawTwistPlug.elementByLogicalIndex((unsigned)poseIds[poseIdsSize - 1]);
        MDataHandle twistHandle = twistPlug.asMDataHandle();
        double driverTwist = twistHandle.asDouble();

        // -------------------------------------------------------------
        // draw
        // -------------------------------------------------------------

        if (wdData->drawOriginVal)
        {
            drawManager.beginDrawable();
            drawManager.setColor(lineColor);

            drawManager.circle(MPoint(0.0, 0.0, 0.0), viewVector, 1.0 * wdData->sizeVal);

            drawManager.endDrawable();
        }

        if (wdData->drawDriverVal)
        {
            drawManager.beginDrawable();
            drawManager.setColor(driverColor);

            MVector dv = MVector(driverValues[0], driverValues[1], driverValues[2]);
            dv.normalize();
            MPoint point(dv.x * lineSize, dv.y * lineSize, dv.z * lineSize);
            drawManager.line(MPoint(0.0, 0.0, 0.0), point);

            drawManager.circle(point, viewVector, 0.05 * wdData->sizeVal, true);

            if (wdData->drawTwistVal)
            {
                dv *= 0.9 + wdData->indexDistVal;

                point = MPoint(dv.x * lineSize, dv.y * lineSize, dv.z * lineSize);

                char info[64];
            #ifdef _WIN64
                sprintf_s(info, "%.2f", driverTwist * RADTODEG);
            #else
                sprintf(info, "%.2f", driverTwist * RADTODEG);
            #endif

                drawManager.text(point, info, MHWRender::MUIDrawManager::kRight);
            }

            drawManager.endDrawable();
        }

        if (wdData->drawPosesVal)
        {
            if (poseCount != 0)
            {
                drawManager.beginDrawable();
                drawManager.setColor(lineColor);

                for (i = 0; i < poseCount; i ++)
                {
                    MVector pv = poseVectors[i];
                    pv.normalize();
                    MPoint point(pv.x * lineSize, pv.y * lineSize, pv.z * lineSize);
                    drawManager.line(MPoint(0.0, 0.0, 0.0), point);

                    drawManager.circle(point, viewVector, 0.03 * wdData->sizeVal, true);

                    if (wdData->drawTwistVal)
                    {
                        pv *= 0.9 + wdData->indexDistVal;

                        point = MPoint(pv.x * lineSize, pv.y * lineSize, pv.z * lineSize);

                        char info[64];
                    #ifdef _WIN64
                        sprintf_s(info, "%.2f", poseTwist[i] * RADTODEG);
                    #else
                        sprintf(info, "%.2f", poseTwist[i] * RADTODEG);
                    #endif

                        drawManager.text(point, info, MHWRender::MUIDrawManager::kRight);
                    }
                }

                drawManager.endDrawable();
            }
        }

        if (wdData->drawIndicesVal)
        {
            if (poseCount != 0)
            {
                drawManager.beginDrawable();
                drawManager.setColor(lineColor);

                for (i = 0; i < poseCount; i ++)
                {
                    MVector pv = poseVectors[i];
                    pv.normalize();
                    pv *= 1.03 + wdData->indexDistVal;

                    MPoint point(pv.x * lineSize, pv.y * lineSize, pv.z * lineSize);

                    char info[64];
                #ifdef _WIN64
                    sprintf_s(info, "%i", poseIds[i]);
                #else
                    sprintf(info, "%i", poseIds[i]);
                #endif

                    drawManager.text(point, info, MHWRender::MUIDrawManager::kCenter);
                }

                drawManager.endDrawable();
            }
        }
    }
}

#if MAYA_API_VERSION < 202400
void weightDriverOverride::draw(const MHWRender::MDrawContext &context, const MUserData *data)
{
}
#endif

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
