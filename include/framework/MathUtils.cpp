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

#include "MathUtils.hpp"

#include <framework/Mat4.hpp>
#include <framework/Mat3.hpp>

#include <iomanip>

//Note all 4x4 matrices here are in column-major ordering

////////////////////////////////////////////////////////////
void cross(float dst[3], const float x[3], const float y[3])
{
    dst[0] = x[1] * y[2] - x[2] * y[1];
    dst[1] = -(x[0] * y[2] - x[2] * y[0]);
    dst[2] = x[0] * y[1] - x[1] * y[0];
}

////////////////////////////////////////////////////////////
void normalize(float dst[3])
{
    float normSqr = dst[0] * dst[0] + dst[1] * dst[1] + dst[2] * dst[2];

    if (normSqr > 0.0)
    {
        float invNorm = 1.0f / sqrt(normSqr);

        dst[0] = dst[0] * invNorm;
        dst[1] = dst[1] * invNorm;
        dst[2] = dst[2] * invNorm;
    }
}

void lookAt(float M[16], const float eye[3], const float center[3], const float up[3])
{
    float x[3], y[3], z[3];

    // make rotation matrix

    // Z vector
    z[0] = eye[0] - center[0];
    z[1] = eye[1] - center[1];
    z[2] = eye[2] - center[2];
    normalize(z);

    // Y vector
    y[0] = up[0];
    y[1] = up[1];
    y[2] = up[2];

    // X vector = Y cross Z
    cross(x, y, z);

    // Recompute Y = Z cross X
    cross(y, z, x);

    // cross product gives area of parallelogram, which is < 1.0 for
    // non-perpendicular unit-length vectors; so normalize x, y here
    normalize(x);
    normalize(y);

    M[0] = x[0];
    M[1] = y[0];
    M[2] = z[0];
    M[3] = 0.0;

    M[4] = x[1];
    M[5] = y[1];
    M[6] = z[1];
    M[7] = 0.0;

    M[8]  = x[2];
    M[9]  = y[2];
    M[10] = z[2];
    M[11] = 0.0;

    M[12] = -x[0] * eye[0] - x[1] * eye[1] - x[2] * eye[2];
    M[13] = -y[0] * eye[0] - y[1] * eye[1] - y[2] * eye[2];
    M[14] = -z[0] * eye[0] - z[1] * eye[1] - z[2] * eye[2];
    M[15] = 1.0;
}

void frustum(float M[16], const float l, const float r, const float b, const float t, const float n, const float f)
{
    M[0] = ((float)(2.0)) * n / (r - l);
    M[1] = 0.0;
    M[2] = 0.0;
    M[3] = 0.0;

    M[4] = 0.0;
    M[5] = ((float)(2.0)) * n / (t - b);
    M[6] = 0.0;
    M[7] = 0.0;

    M[8]  = (r + l) / (r - l);
    M[9]  = (t + b) / (t - b);
    M[10] = -(f + n) / (f - n);
    M[11] = -1.0;

    M[12] = 0.0;
    M[13] = 0.0;
    M[14] = -(((float)(2.0)) * f * n) / (f - n);
    M[15] = 0.0;
}

void perspective(float M[16], float fovy, float aspect, float n, float f)
{
    float xmin, xmax, ymin, ymax;

    ymax = n * (float)tan(fovy * 0.5f);
    ymin = -ymax;

    xmin = ymin * aspect;
    xmax = ymax * aspect;

    frustum(M, xmin, xmax, ymin, ymax, n, f);
}

void ortho(float M[16], const float l, const float r, const float b, const float t, const float n, const float f)
{
    // https://msdn.microsoft.com/ru-ru/library/windows/desktop/dd373965.aspx

    M[0] = (2.0f) / (r - l);
    M[1] = 0.0f;
    M[2] = 0.0f;
    M[3] = 0.0f;

    M[4] = 0.0f;
    M[5] = (2.0f) / (t - b);
    M[6] = 0.0f;
    M[7] = 0.0f;

    M[8]  = 0.0f;
    M[9]  = 0.0f;
    M[10] = (-2.0f) / (f - n);
    M[11] = 0.0;

    M[12] = -(r + l) / (r - l);
    M[13] = -(t + b) / (t - b);
    M[14] = -(f + n) / (f - n);
    M[15] = 1.0;
}

void ortho(float M[16], float fovy, float aspect, float n, float f)
{
    float xmin, xmax, ymin, ymax;

    ymax = n * (float)tan(fovy * 0.5f);
    ymin = -ymax;

    xmin = ymin * aspect;
    xmax = ymax * aspect;

    ortho(M, xmin, xmax, ymin, ymax, n, f);
}

void quaternionToRotationMatrix(float rotation[16], const float quaternion[4])
{
    float q[4] = {quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
    {
        float dist2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if (fabs(dist2 - 1) > 1e-6)
        {
            float dist = sqrt(dist2);
            q[0] /= dist;
            q[1] /= dist;
            q[2] /= dist;
            q[3] /= dist;
        }
    }

    rotation[0] = 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2];
    rotation[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
    rotation[2] = 2 * (q[0] * q[2] + q[1] * q[3]);

    rotation[4] = 2 * (q[0] * q[1] - q[2] * q[3]);
    rotation[5] = 1 - 2 * q[0] * q[0] - 2 * q[2] * q[2];
    rotation[6] = 2 * (q[1] * q[2] + q[0] * q[3]);

    rotation[8]  = 2 * (q[0] * q[2] + q[1] * q[3]);
    rotation[9]  = 2 * (q[1] * q[2] - q[0] * q[3]);
    rotation[10] = 1 - 2 * q[0] * q[0] - 2 * q[1] * q[1];

    rotation[15] = 1; // homogeneous component
}

void quaternionToRotationMatrix(dwMatrix3d& mat, const float64_t* quat)
{
    mat.array[0] = 1 - 2 * quat[1] * quat[1] - 2 * quat[2] * quat[2];
    mat.array[1] = 2 * quat[0] * quat[1] + 2 * quat[2] * quat[3];
    mat.array[2] = 2 * quat[0] * quat[2] - 2 * quat[1] * quat[3];
    mat.array[3] = 2 * quat[0] * quat[1] - 2 * quat[2] * quat[3];
    mat.array[4] = 1 - 2 * quat[0] * quat[0] - 2 * quat[2] * quat[2];
    mat.array[5] = 2 * quat[1] * quat[2] + 2 * quat[0] * quat[3];
    mat.array[6] = 2 * quat[0] * quat[2] + 2 * quat[1] * quat[3];
    mat.array[7] = 2 * quat[1] * quat[2] - 2 * quat[0] * quat[3];
    mat.array[8] = 1 - 2 * quat[0] * quat[0] - 2 * quat[1] * quat[1];
}

void positionToTranslateMatrix(float translate[16], const float position[3])
{
    translate[3 * 4]     = position[0];
    translate[3 * 4 + 1] = position[1];
    translate[3 * 4 + 2] = position[2];

    translate[15] = 1; // homogeneous component
}

void rotationToTransformMatrix(float transform[16], const float rotation[9])
{
    transform[0]   = rotation[0];
    transform[1]   = rotation[1];
    transform[2]   = rotation[2];
    transform[1+3] = rotation[3];
    transform[1+4] = rotation[4];
    transform[1+5] = rotation[5];    
    transform[2+6] = rotation[6];
    transform[2+7] = rotation[7];
    transform[2+8] = rotation[8];

    transform[3]  = 0;
    transform[7]  = 0;
    transform[11] = 0;
    transform[15] = 1; // homogeneous component
}

dwVector2f focalFromFOV(const dwVector2f fov, dwVector2ui imageSize)
{
    return dwVector2f{imageSize.x / (2.0f * (float32_t)tan(fov.x * 0.5f)),
                      imageSize.y / (2.0f * (float32_t)tan(fov.y * 0.5f))};
}

// see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void quaternionToEulerianAngle(const float32_t q[4], float32_t& roll, float32_t& pitch, float32_t& yaw)
{
    float32_t ysqr = q[1] * q[1];

    // roll (x-axis rotation)
    float32_t t0 = +2.0 * (q[3] * q[0] + q[1] * q[2]);
    float32_t t1 = +1.0 - 2.0 * (q[0] * q[0] + ysqr);
    roll         = atan2(t0, t1);

    // pitch (y-axis rotation)
    float32_t t2 = +2.0 * (q[3] * q[1] - q[2] * q[0]);
    t2           = ((t2 > 1.0) ? 1.0 : t2);
    t2           = ((t2 < -1.0) ? -1.0 : t2);
    pitch        = asin(t2);

    // yaw (z-axis rotation)
    float32_t t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    float32_t t4 = +1.0 - 2.0 * (ysqr + q[2] * q[2]);
    yaw          = atan2(t3, t4);
}

dwVector3f pos2DTo3D(const dwVector2f& in)
{
    dwVector3f out;
    out.x = in.x;
    out.y = in.y;
    out.z = 0.0f; // padding zero in the extra dimension

    return out;
}

dwVector2f pos3DTo2D(const dwVector3f& in)
{
    dwVector2f out;
    out.x = in.x;
    out.y = in.y;

    return out;
}

std::ostream& operator<<(std::ostream& o, const dwTransformation& tx)
{
    o << '\n';
    for (int32_t r = 0; r < 4; ++r)
    {
        for (int32_t c = 0; c < 4; ++c)
        {
            o << '\t' << std::setw(16) << std::right
              << std::fixed << std::setprecision(12) << tx.array[r + c * 4];
        }
        o << '\n';
    }
    return o;
}

dwTransformation operator*(const dwTransformation& a, const dwTransformation& b)
{
    dwTransformation temp;
    Mat4_AxB(temp.array, a.array, b.array);
    Mat4_RenormR(temp.array);
    return temp;
}

dwTransformation& operator*=(dwTransformation& a, const dwTransformation& b)
{
    dwTransformation temp = a;
    a                     = temp * b;
    return a;
}

dwTransformation operator/(const dwTransformation& a, const dwTransformation& b)
{
    dwTransformation temp;
    Mat4_AxBinv(temp.array, a.array, b.array);
    Mat4_RenormR(temp.array);
    return temp;
}

dwTransformation& operator/=(dwTransformation& a, const dwTransformation& b)
{
    dwTransformation temp = a;
    a                     = temp / b;
    return a;
}

dwVector4f operator*(const dwTransformation& T, const dwVector4f& p)
{
    dwVector4f q;
    Mat4_Axp(reinterpret_cast<float32_t*>(&q), T.array, reinterpret_cast<const float32_t*>(&p));
    q.w = p.w;
    return q;
}

float32_t getTranslationMagnitude(const dwTransformation& T)
{
    return sqrtf(T.array[0 + 3 * 4] * T.array[0 + 3 * 4] +
                 T.array[1 + 3 * 4] * T.array[1 + 3 * 4] +
                 T.array[2 + 3 * 4] * T.array[2 + 3 * 4]);
}

float32_t getRotationMagnitude(const dwTransformation& T)
{
    float32_t traceR = std::min(T.array[0 + 0 * 4] + T.array[1 + 1 * 4] + T.array[2 + 2 * 4], 3.0f);
    return acosf(0.5f * (traceR - 1.0f));
}

void getRotationMatrix(dwMatrix3f* R, float32_t rollInDegrees, float32_t pitchInDegrees, float32_t yawInDegrees)
{
    // Convert to radians
    const float32_t roll  = rollInDegrees * M_PI / 180.0f;
    const float32_t pitch = pitchInDegrees * M_PI / 180.0f;
    const float32_t yaw   = yawInDegrees * M_PI / 180.0f;

    const float32_t croll  = cos(roll);
    const float32_t sroll  = sin(roll);
    const float32_t cpitch = cos(pitch);
    const float32_t spitch = sin(pitch);
    const float32_t cyaw   = cos(yaw);
    const float32_t syaw   = sin(yaw);

    dwMatrix3f Rx = {{1.0f, 0.0f, 0.0f,
                      0.0f, croll, sroll,
                      0.0f, -sroll, croll}};
    dwMatrix3f Ry = {{cpitch, 0.0f, -spitch,
                      0.0f, 1.0f, 0.0f,
                      spitch, 0.0f, cpitch}};
    dwMatrix3f Rz = {{cyaw, syaw, 0.0f,
                      -syaw, cyaw, 0.0f,
                      0.0f, 0.0f, 1.0f}};

    dwMatrix3f Ryx;
    Mat3_AxB(Ryx.array, Ry.array, Rx.array);
    Mat3_AxB(R->array, Rz.array, Ryx.array);
}

void computeHomography(dwMatrix3f* homographyOut, dwTransformation transformationIn, dwMatrix3f camOutRotationMatrix,
                       float32_t camOutTranslation[3], float32_t normal[3], float32_t distanceToPlane)
{
    // Extract rotation matrix from camera matrix
    dwMatrix3f camInRotationMatrix;
    for (uint32_t row = 0U; row < 3U; row++)
    {
        for (uint32_t col = 0U; col < 3U; ++col)
        {
            camInRotationMatrix.array[col * 3 + row] = transformationIn.array[col * 4 + row];
        }
    }

    // Estimate rotation from cameraIn to cameraOut
    dwMatrix3f rotationInToOut;
    Mat3_AxB(rotationInToOut.array, camOutRotationMatrix.array, camInRotationMatrix.array);

    // Extract translation vector from camera matrix
    float32_t camInTranslation[3];
    for (uint32_t row = 0U; row < 3U; row++)
    {
        camInTranslation[row] = transformationIn.array[3 * 4 + row];
    }

    // Estimate translation from cameraIn to cameraOut
    float32_t translation[3];
    translation[0] = camInTranslation[0] - camOutTranslation[0];
    translation[1] = camInTranslation[1] - camOutTranslation[1];
    translation[2] = camInTranslation[2] - camOutTranslation[2];

    // Calculate homography
    // H = Ra * Rb^(-1) - Ra * ((tb - ta) * n^t)/d * Rb^(-1)
    // where Rb, tb are rotation matrix and translation vector of cameraIn,
    // Ra, ta are rotation matrix and translation vector of cameraOut
    // and n is the normal of the ground plane.
    {
        dwMatrix3f translationMatrix;
        for (uint32_t row = 0U; row < 3; ++row)
        {
            for (uint32_t col = 0U; col < 3; ++col)
            {
                translationMatrix.array[col * 3 + row] = (translation[row] * normal[col]) / distanceToPlane;
            }
        }

        dwMatrix3f rotatedTranslationMatrix;
        Mat3_AxB(rotatedTranslationMatrix.array, camOutRotationMatrix.array, translationMatrix.array);
        Mat3_AxB(translationMatrix.array, rotatedTranslationMatrix.array, camInRotationMatrix.array);

        dwMatrix3f homography;
        for (uint32_t row = 0U; row < 3; ++row)
        {
            for (uint32_t col = 0U; col < 3; ++col)
            {
                homography.array[row * 3 + col] = rotationInToOut.array[row * 3 + col] -
                                                  translationMatrix.array[row * 3 + col];
            }
        }

        *homographyOut = homography;
    }
}
