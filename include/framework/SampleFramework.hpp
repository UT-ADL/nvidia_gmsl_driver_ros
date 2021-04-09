/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_COMMON_SAMPLEFRAMEWORK_HPP_
#define SAMPLES_COMMON_SAMPLEFRAMEWORK_HPP_





// ######################################################################################
//
//  NOTE: This framework is deprecated. Use DriveWorksSample.hpp instead
//
// ######################################################################################











#include "Checks.hpp"
#include "WindowGLFW.hpp"
#include "ProgramArguments.hpp"
#include "Log.hpp"

#include <signal.h>
#include <cstring> // for memset
#include <iostream>

#include <dw/visualization/Renderer.h>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

extern void (*gUserKeyPressCallback)(int);
extern ProgramArguments gArguments;
extern WindowBase *gWindow;
extern bool gRun;
extern bool gPause;

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

// signal handling
void sig_int_handler(int sig);

// key press event
void keyPressCallback(int key, int scancode, int action, int mods);

// draw box
// Required renderbuffer with the following properties:
// - Renderbuffer should be initialized with DW_RENDER_PRIM_LINELIST
// - Expected position format: DW_RENDER_FORMAT_R32G32_FLOAT
// - Expected position semantic: DW_RENDER_SEMANTIC_POS_XY
void drawBoxes(const std::vector<dwBox2D> &boxes, const std::vector<uint32_t> *boxIds,
               float32_t normalizationWidth, float32_t normalizationHeight,
               dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);

// draw box with labels
// Required renderbuffer with the following properties:
// - Renderbuffer should be initialized with DW_RENDER_PRIM_LINELIST
// - Expected position format: DW_RENDER_FORMAT_R32G32_FLOAT
// - Expected position semantic: DW_RENDER_SEMANTIC_POS_XY
void drawBoxesWithLabels(const std::vector<std::pair<dwBox2D, std::string> > &boxesWithLabels,
                         float32_t normalizationWidth, float32_t normalizationHeight,
                         dwRenderBufferHandle_t renderBuffer, dwRendererHandle_t renderer);


// init sample application
bool initSampleApp(int argc, const char **argv,
                   const ProgramArguments* arguments,
                   void (*userKeyPressCallback)(int),
                   uint32_t width, uint32_t height,
                   bool offscreen = false,
                   int samples = 0);

void releaseSampleApp();


#endif // SAMPLES_COMMON_SAMPLEFRAMEWORK_HPP_
