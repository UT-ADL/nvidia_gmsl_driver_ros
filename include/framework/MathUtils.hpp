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

#ifndef SAMPLES_COMMON_MATHUTILS_HPP_
#define SAMPLES_COMMON_MATHUTILS_HPP_

#include <math.h>
#include <algorithm>
#include <dw/core/Types.h>
#include <fstream>

#define DEG2RAD(x) (static_cast<float>(x) * 0.01745329251994329575f)
#define RAD2DEG(x) (static_cast<float>(x) * 57.29577951308232087721f)

//Note all 4x4 matrices here are in column-major ordering

////////////////////////////////////////////////////////////
void cross(float dst[3], const float x[3], const float y[3]);

////////////////////////////////////////////////////////////
void normalize(float dst[3]);
void lookAt(float M[16], const float eye[3], const float center[3], const float up[3]);
void frustum(float M[16], const float l, const float r,
                          const float b, const float t,
                          const float n, const float f);

void perspective(float M[16], float fovy, float aspect, float n, float f);
void ortho(float M[16], const float l, const float r, const float b, const float t, const float n, const float f);
void ortho(float M[16], float fovy, float aspect, float n, float f);

void quaternionToRotationMatrix(float rotation[16], const float quaternion[4]);
void quaternionToRotationMatrix(dwMatrix3d& mat, const float64_t* quat);
void positionToTranslateMatrix(float translate[16], const float position[3]);
void rotationToTransformMatrix(float transform[16], const float rotation[9]);
void quaternionToEulerianAngle(const float32_t q[4], float32_t& roll, float32_t& pitch, float32_t& yaw);


dwVector3f pos2DTo3D(const dwVector2f &in);
dwVector2f pos3DTo2D(const dwVector3f &in);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** Calculates focal length of pinhole camera based on horizontal and vertical Field Of View angle (in radians)
*   and size of the image
**/
dwVector2f focalFromFOV(const dwVector2f fov, dwVector2ui imageSize);

//------------------------------------------------------------------------------
// This is a wrapped around the Mat4.hpp ::  Mat4_AxB for easy usage.
// Returns a * b
dwTransformation operator*(const dwTransformation& a, const dwTransformation& b);

//------------------------------------------------------------------------------
// This is a wrapped around the Mat4.hpp ::  Mat4_AxB for easy usage.
// Sets a -> a * b
dwTransformation& operator*=(dwTransformation& a, const dwTransformation& b);

//------------------------------------------------------------------------------
// Transform point by transformation matrix
dwVector4f operator*(const dwTransformation& T, const dwVector4f& p);

//------------------------------------------------------------------------------
// This is a wrapped around the Mat4.hpp ::  Mat4_AxBinv for easy usage.
// Returns a * inv(b)
dwTransformation operator/(const dwTransformation& a, const dwTransformation& b);

//------------------------------------------------------------------------------
// This is a wrapped around the Mat4.hpp ::  Mat4_AxBinv for easy usage.
// Sets a -> a * inv(b)
dwTransformation& operator/=(dwTransformation& a, const dwTransformation& b);

//------------------------------------------------------------------------------
// Pretty printer for a dwTransformation object.
std::ostream& operator << (std::ostream& o, const dwTransformation& tx);

//------------------------------------------------------------------------------
// Returns the magnitude of the translation of T
float32_t getTranslationMagnitude(const dwTransformation &T);

//------------------------------------------------------------------------------
// Returns the magnitude of the rotation of T, as used in angle/axis representation
// It is computed as acos( (trace(R) - 1) / 2)
float32_t getRotationMagnitude(const dwTransformation &T);

//------------------------------------------------------------------------------
// Returns the rotation matrix based on roll, pitch and yaw in degrees
void getRotationMatrix(dwMatrix3f *R, float32_t rollInDegrees, float32_t pitchInDegrees, float32_t yawInDegrees);

//------------------------------------------------------------------------------
// Returns the homography matrix based on the input camera transformation matrix, output camera rotation and translation matrix
void computeHomography(dwMatrix3f* homographyOut, dwTransformation transformationIn, dwMatrix3f camOutRotationMatrix, float32_t camOutTranslation[], float32_t normal[], float32_t distanceToPlane);

#endif // SAMPLES_COMMON_MATHUTILS_HPP_
