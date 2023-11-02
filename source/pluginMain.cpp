// ---------------------------------------------------------------------
//
//  pluginMain.cpp
//
//  Created by ingo on 10/5/13.
//  Copyright (c) 2013-2023 Ingo Clemens. All rights reserved.
//
// ---------------------------------------------------------------------

#include <string>
#include <maya/MDrawRegistry.h>
#include <maya/MFnPlugin.h>

static const std::string kVERSION = "4.0.1";

#include "weightDriver.h"

// ---------------------------------------------------------------------
// initialization
// ---------------------------------------------------------------------

MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, "Ingo Clemens", kVERSION.c_str(), "Any");

    status = plugin.registerNode("weightDriver",
                                 weightDriver::id,
                                 &weightDriver::creator,
                                 &weightDriver::initialize,
                                 MPxNode::kLocatorNode,
                                 &weightDriver::drawDbClassification);
    if (status != MStatus::kSuccess)
        status.perror("Register weightDriver command failed");

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(weightDriver::drawDbClassification,
                                                                   weightDriver::drawRegistrantId,
                                                                   weightDriverOverride::Creator);
    if (status != MStatus::kSuccess)
        status.perror("Register DrawOverrideCreator for weightDriver command failed");

    return status;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, "Ingo Clemens", kVERSION.c_str(), "Any");

    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(weightDriver::drawDbClassification,
                                                                     weightDriver::drawRegistrantId);
    if (status != MStatus::kSuccess)
        status.perror("Deregister DrawOverrideCreator for weightDriver command failed");

    status = plugin.deregisterNode(weightDriver::id);

    if (status != MStatus::kSuccess)
        status.perror("Deregister weightDriver command failed");

    return status;
}

// ---------------------------------------------------------------------
// MIT License
//
// Copyright (c) 2021-2023 Ingo Clemens, brave rabbit
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
