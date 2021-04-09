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
// Copyright (c) 2015-2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_COMMON_MAT3_HPP_
#define SAMPLES_COMMON_MAT3_HPP_

#include <string.h>
#include <math.h>

//Note all 3x3 matrices here are in column-major ordering

//#######################################################################################
inline void Mat3_AxB(float res[9], const float A[9], const float B[9])
{
    res[0 + 0 * 3] = A[0 + 0 * 3] * B[0 + 0 * 3] + A[0 + 1 * 3] * B[1 + 0 * 3] + A[0 + 2 * 3] * B[2 + 0 * 3];
    res[1 + 0 * 3] = A[1 + 0 * 3] * B[0 + 0 * 3] + A[1 + 1 * 3] * B[1 + 0 * 3] + A[1 + 2 * 3] * B[2 + 0 * 3];
    res[2 + 0 * 3] = A[2 + 0 * 3] * B[0 + 0 * 3] + A[2 + 1 * 3] * B[1 + 0 * 3] + A[2 + 2 * 3] * B[2 + 0 * 3];

    res[0 + 1 * 3] = A[0 + 0 * 3] * B[0 + 1 * 3] + A[0 + 1 * 3] * B[1 + 1 * 3] + A[0 + 2 * 3] * B[2 + 1 * 3];
    res[1 + 1 * 3] = A[1 + 0 * 3] * B[0 + 1 * 3] + A[1 + 1 * 3] * B[1 + 1 * 3] + A[1 + 2 * 3] * B[2 + 1 * 3];
    res[2 + 1 * 3] = A[2 + 0 * 3] * B[0 + 1 * 3] + A[2 + 1 * 3] * B[1 + 1 * 3] + A[2 + 2 * 3] * B[2 + 1 * 3];

    res[0 + 2 * 3] = A[0 + 0 * 3] * B[0 + 2 * 3] + A[0 + 1 * 3] * B[1 + 2 * 3] + A[0 + 2 * 3] * B[2 + 2 * 3];
    res[1 + 2 * 3] = A[1 + 0 * 3] * B[0 + 2 * 3] + A[1 + 1 * 3] * B[1 + 2 * 3] + A[1 + 2 * 3] * B[2 + 2 * 3];
    res[2 + 2 * 3] = A[2 + 0 * 3] * B[0 + 2 * 3] + A[2 + 1 * 3] * B[1 + 2 * 3] + A[2 + 2 * 3] * B[2 + 2 * 3];
}

//#######################################################################################
inline void Mat3_Axp(float res[3], const float A[9], const float B[3])
{
    res[0] = A[0 + 0 * 3] * B[0] + A[0 + 1 * 3] * B[1] + A[0 + 2 * 3] * B[2];
    res[1] = A[1 + 0 * 3] * B[0] + A[1 + 1 * 3] * B[1] + A[1 + 2 * 3] * B[2];
    res[2] = A[2 + 0 * 3] * B[0] + A[2 + 1 * 3] * B[1] + A[2 + 2 * 3] * B[2];
}

//#######################################################################################
inline void Mat3_Transpose(float dst[9], const float src[9])
{
    std::copy(src, src + 9, dst);
    std::swap(dst[0 + 1 * 3], dst[1 + 0 * 3]);
    std::swap(dst[0 + 2 * 3], dst[2 + 0 * 3]);
    std::swap(dst[1 + 2 * 3], dst[2 + 1 * 3]);
}

#endif // SAMPLES_COMMON_MAT3_HPP_
