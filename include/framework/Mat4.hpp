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
// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_COMMON_MAT4_HPP_
#define SAMPLES_COMMON_MAT4_HPP_

#include <string.h>
#include <math.h>
#include <dw/core/Types.h>

//Note all 4x4 matrices here are in column-major ordering

////////////////////////////////////////////////////////////
inline void Mat4_identity(float M[16])
{
    memset(M, 0, 16*sizeof(float));
    M[0] = 1.0f;
    M[5] = 1.0f;
    M[10] = 1.0f;
    M[15] = 1.0f;
}

////////////////////////////////////////////////////////////
inline void Mat4_Axp(float dst[3], const float M[16], const float p[3])
{
    dst[0] = M[0 + 0 * 4] * p[0] + M[0 + 1 * 4] * p[1] + M[0 + 2 * 4] * p[2] + M[0 + 3 * 4];
    dst[1] = M[1 + 0 * 4] * p[0] + M[1 + 1 * 4] * p[1] + M[1 + 2 * 4] * p[2] + M[1 + 3 * 4];
    dst[2] = M[2 + 0 * 4] * p[0] + M[2 + 1 * 4] * p[1] + M[2 + 2 * 4] * p[2] + M[2 + 3 * 4];
}

////////////////////////////////////////////////////////////
inline void Mat4_Rxp(float dst[3], const float M[16], const float p[3])
{
    dst[0] = M[0 + 0 * 4] * p[0] + M[0 + 1 * 4] * p[1] + M[0 + 2 * 4] * p[2];
    dst[1] = M[1 + 0 * 4] * p[0] + M[1 + 1 * 4] * p[1] + M[1 + 2 * 4] * p[2];
    dst[2] = M[2 + 0 * 4] * p[0] + M[2 + 1 * 4] * p[1] + M[2 + 2 * 4] * p[2];
}

////////////////////////////////////////////////////////////
inline void Mat4_Rtxp(float dst[3], const float M[16], const float p[3])
{
    dst[0] = M[0 + 0 * 4] * p[0] + M[1 + 0 * 4] * p[1] + M[2 + 0 * 4] * p[2];
    dst[1] = M[0 + 1 * 4] * p[0] + M[1 + 1 * 4] * p[1] + M[2 + 1 * 4] * p[2];
    dst[2] = M[0 + 2 * 4] * p[0] + M[1 + 2 * 4] * p[1] + M[2 + 2 * 4] * p[2];
}

////////////////////////////////////////////////////////////
inline void Mat4_IsoInv(float res[16], const float A[16])
{
    //Transpose R
    res[0 + 0 * 4] = A[0 + 0 * 4];
    res[1 + 0 * 4] = A[0 + 1 * 4];
    res[2 + 0 * 4] = A[0 + 2 * 4];

    res[0 + 1 * 4] = A[1 + 0 * 4];
    res[1 + 1 * 4] = A[1 + 1 * 4];
    res[2 + 1 * 4] = A[1 + 2 * 4];

    res[0 + 2 * 4] = A[2 + 0 * 4];
    res[1 + 2 * 4] = A[2 + 1 * 4];
    res[2 + 2 * 4] = A[2 + 2 * 4];

    //ti = -Rt
    const float tx = A[0 + 3 * 4];
    const float ty = A[1 + 3 * 4];
    const float tz = A[2 + 3 * 4];
    res[0 + 3 * 4] = -(A[0 + 0 * 4] * tx + A[1 + 0 * 4] * ty + A[2 + 0 * 4] * tz);
    res[1 + 3 * 4] = -(A[0 + 1 * 4] * tx + A[1 + 1 * 4] * ty + A[2 + 1 * 4] * tz);
    res[2 + 3 * 4] = -(A[0 + 2 * 4] * tx + A[1 + 2 * 4] * ty + A[2 + 2 * 4] * tz);

    //Empty row
    res[3 + 0 * 4] = 0;
    res[3 + 1 * 4] = 0;
    res[3 + 2 * 4] = 0;
    res[3 + 3 * 4] = 1;
}

////////////////////////////////////////////////////////////
inline void Mat4_AxB(float res[16], const float A[16], const float B[16])
{
    res[0 + 0 * 4] = A[0 + 0 * 4] * B[0 + 0 * 4] + A[0 + 1 * 4] * B[1 + 0 * 4] + A[0 + 2 * 4] * B[2 + 0 * 4] + A[0 + 3 * 4] * B[3 + 0 * 4];
    res[1 + 0 * 4] = A[1 + 0 * 4] * B[0 + 0 * 4] + A[1 + 1 * 4] * B[1 + 0 * 4] + A[1 + 2 * 4] * B[2 + 0 * 4] + A[1 + 3 * 4] * B[3 + 0 * 4];
    res[2 + 0 * 4] = A[2 + 0 * 4] * B[0 + 0 * 4] + A[2 + 1 * 4] * B[1 + 0 * 4] + A[2 + 2 * 4] * B[2 + 0 * 4] + A[2 + 3 * 4] * B[3 + 0 * 4];
    res[3 + 0 * 4] = A[3 + 0 * 4] * B[0 + 0 * 4] + A[3 + 1 * 4] * B[1 + 0 * 4] + A[3 + 2 * 4] * B[2 + 0 * 4] + A[3 + 3 * 4] * B[3 + 0 * 4];

    res[0 + 1 * 4] = A[0 + 0 * 4] * B[0 + 1 * 4] + A[0 + 1 * 4] * B[1 + 1 * 4] + A[0 + 2 * 4] * B[2 + 1 * 4] + A[0 + 3 * 4] * B[3 + 1 * 4];
    res[1 + 1 * 4] = A[1 + 0 * 4] * B[0 + 1 * 4] + A[1 + 1 * 4] * B[1 + 1 * 4] + A[1 + 2 * 4] * B[2 + 1 * 4] + A[1 + 3 * 4] * B[3 + 1 * 4];
    res[2 + 1 * 4] = A[2 + 0 * 4] * B[0 + 1 * 4] + A[2 + 1 * 4] * B[1 + 1 * 4] + A[2 + 2 * 4] * B[2 + 1 * 4] + A[2 + 3 * 4] * B[3 + 1 * 4];
    res[3 + 1 * 4] = A[3 + 0 * 4] * B[0 + 1 * 4] + A[3 + 1 * 4] * B[1 + 1 * 4] + A[3 + 2 * 4] * B[2 + 1 * 4] + A[3 + 3 * 4] * B[3 + 1 * 4];

    res[0 + 2 * 4] = A[0 + 0 * 4] * B[0 + 2 * 4] + A[0 + 1 * 4] * B[1 + 2 * 4] + A[0 + 2 * 4] * B[2 + 2 * 4] + A[0 + 3 * 4] * B[3 + 2 * 4];
    res[1 + 2 * 4] = A[1 + 0 * 4] * B[0 + 2 * 4] + A[1 + 1 * 4] * B[1 + 2 * 4] + A[1 + 2 * 4] * B[2 + 2 * 4] + A[1 + 3 * 4] * B[3 + 2 * 4];
    res[2 + 2 * 4] = A[2 + 0 * 4] * B[0 + 2 * 4] + A[2 + 1 * 4] * B[1 + 2 * 4] + A[2 + 2 * 4] * B[2 + 2 * 4] + A[2 + 3 * 4] * B[3 + 2 * 4];
    res[3 + 2 * 4] = A[3 + 0 * 4] * B[0 + 2 * 4] + A[3 + 1 * 4] * B[1 + 2 * 4] + A[3 + 2 * 4] * B[2 + 2 * 4] + A[3 + 3 * 4] * B[3 + 2 * 4];

    res[0 + 3 * 4] = A[0 + 0 * 4] * B[0 + 3 * 4] + A[0 + 1 * 4] * B[1 + 3 * 4] + A[0 + 2 * 4] * B[2 + 3 * 4] + A[0 + 3 * 4] * B[3 + 3 * 4];
    res[1 + 3 * 4] = A[1 + 0 * 4] * B[0 + 3 * 4] + A[1 + 1 * 4] * B[1 + 3 * 4] + A[1 + 2 * 4] * B[2 + 3 * 4] + A[1 + 3 * 4] * B[3 + 3 * 4];
    res[2 + 3 * 4] = A[2 + 0 * 4] * B[0 + 3 * 4] + A[2 + 1 * 4] * B[1 + 3 * 4] + A[2 + 2 * 4] * B[2 + 3 * 4] + A[2 + 3 * 4] * B[3 + 3 * 4];
    res[3 + 3 * 4] = A[3 + 0 * 4] * B[0 + 3 * 4] + A[3 + 1 * 4] * B[1 + 3 * 4] + A[3 + 2 * 4] * B[2 + 3 * 4] + A[3 + 3 * 4] * B[3 + 3 * 4];
}

inline void Mat4_Transpose(float dst[16], const float src[16]) {
    memcpy(dst, src, sizeof(float)*16);
    float  t;
    for (int i = 1 ; i < 4; ++i)
        for (int j = 0; j < i; ++j)
        {
            t = dst[i * 4 + j];
            dst[i * 4 + j] = dst[i + 4 * j];
            dst[i + 4 * j] = t;
        }

}
////////////////////////////////////////////////////////////
inline void Mat4_RenormR(float res[16])
{
    float x, y, z;
    float norm;
    //R1
    x = res[0 + 0 * 4];
    y = res[1 + 0 * 4];
    z = res[2 + 0 * 4];
    norm = sqrt(x*x+y*y+z*z);
    res[0 + 0 * 4] /= norm;
    res[1 + 0 * 4] /= norm;
    res[2 + 0 * 4] /= norm;

    //R2
    x = res[0 + 1 * 4];
    y = res[1 + 1 * 4];
    z = res[2 + 1 * 4];
    norm = sqrt(x*x+y*y+z*z);
    res[0 + 1 * 4] /= norm;
    res[1 + 1 * 4] /= norm;
    res[2 + 1 * 4] /= norm;

    //R3
    x = res[0 + 2 * 4];
    y = res[1 + 2 * 4];
    z = res[2 + 2 * 4];
    norm = sqrt(x*x+y*y+z*z);
    res[0 + 2 * 4] /= norm;
    res[1 + 2 * 4] /= norm;
    res[2 + 2 * 4] /= norm;
}

////////////////////////////////////////////////////////////
inline void Mat4_AxBinv(float res[16], const float A[16], const float B[16])
{
    float Binv[16];
    Mat4_IsoInv(Binv, B);
    Mat4_AxB(res, A, Binv);
    Mat4_RenormR(res);
}

////////////////////////////////////////////////////////////
inline dwTransformation dwMakeTransformation(const dwMatrix4f &m)
{
    dwTransformation res;
    for(uint32_t i=0; i<4; i++)
        for(uint32_t j=0; j<3; j++)
        {
            res.array[i*4+j] = m.array[i*4+j];
        }
    // Last row of transformation is always 0,0,0,1
    res.array[0*4+3] = 0;
    res.array[1*4+3] = 0;
    res.array[2*4+3] = 0;
    res.array[3*4+3] = 1;
    return res;

}

////////////////////////////////////////////////////////////
inline dwMatrix4f dwMakeMatrix4f(const dwTransformation &m)
{
    dwMatrix4f res;
    for(uint32_t i=0; i<4; i++)
        for(uint32_t j=0; j<4; j++)
        {
            res.array[i*4+j] = m.array[i*4+j];
        }
    return res;

}

#endif // SAMPLES_COMMON_MAT4_HPP_
