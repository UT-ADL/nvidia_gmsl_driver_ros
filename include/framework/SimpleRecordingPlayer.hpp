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

#ifndef COMMON_SIMPLERECORDINGPLAYER_HPP_
#define COMMON_SIMPLERECORDINGPLAYER_HPP_

// Driveworks
#include <dw/core/Context.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/imu/IMU.h>
#include <dw/sensors/gps/GPS.h>
#include <dw/sensors/lidar/Lidar.h>
// Common
#include <framework/SimpleCamera.hpp>

namespace dw_samples
{
namespace common
{

/// Abstract class implemented by users that will handle sensor events
class ISensorEventHandler
{
public:
    virtual void handleCAN(const dwCANMessage &) {throw std::runtime_error("CAN requested but not processed");}
    virtual void handleIMU(const dwIMUFrame &) {throw std::runtime_error("IMU requested but not processed");}
    virtual void handleGPS(const dwGPSFrame &) {throw std::runtime_error("GPS requested but not processed");}
    virtual void handleLidar(const dwLidarDecodedPacket *packet) {(void)packet; throw std::runtime_error("Lidar requested but not processed");}
    virtual void handleCamera(uint32_t idx, dwImageHandle_t frame) {(void)idx; (void)frame; throw std::runtime_error("Camera requested but not processed");}
    virtual void handleEndOfStream() {}

protected:
    ISensorEventHandler() {}
};

//-------------------------------------------------------------------------------
/**
* Simple class to play back a recording. It reads from all sensors and checks which timestamp is next.
* The player will call the handleXXX methods of the handler to signal which event is next.
*/
class SimpleRecordingPlayer
{
public:
    SimpleRecordingPlayer(ISensorEventHandler *handler)
        : m_handler(handler)
    {
    }

    void addCamera(SimpleCamera *camera)
    {
        m_cameras.emplace_back();
        m_cameras.back().camera = camera;
        m_cameras.back().pendingImage = nullptr;

        m_lastImages.push_back(nullptr);
    }

    void setCAN(dwSensorHandle_t can)
    {
        m_canSensor = can;
    }

    void setIMU(dwSensorHandle_t imu)
    {
        m_imuSensor = imu;
    }

    void setGPS(dwSensorHandle_t gps)
    {
        m_gpsSensor = gps;
    }

    void setLidar(dwSensorHandle_t lidar)
    {
        m_lidarSensor = lidar;
    }

    /// Determines whether only a single sensor is being played back.
    /// When only a single sensor is active, missing timestamps will not trigger an exception
    bool isSingleSensorPlayback() const
    {
        size_t count = 0;
        if(m_canSensor != DW_NULL_HANDLE)
            count++;
        if(m_imuSensor != DW_NULL_HANDLE)
            count++;
        if(m_gpsSensor != DW_NULL_HANDLE)
            count++;
        if(m_lidarSensor != DW_NULL_HANDLE)
            count++;
        count += m_cameras.size();
        return count==1;
    }

    /// Resets all sensors and restarts playback
    void restart();

    /// Main method to play the recording
    /// Each call to stepForward will generate one event and will overwrite one
    /// the data returned by one of the getLastXXX() functions.
    void stepForward();

    const dwCANMessage &getLastCAN() const {return m_lastCANMsg;}
    const dwIMUFrame &getLastIMU() const {return m_lastIMUMsg;}
    const dwGPSFrame &getLastGPS() const {return m_lastGPSMsg;}
    const dwLidarDecodedPacket &getLastLidar() const {return *m_lastLidarPacket;}
    dwImageHandle_t getLastImage(uint32_t idx) const {return m_lastImages[idx];}

protected:
    ISensorEventHandler *m_handler = nullptr;

    struct CameraData
    {
        SimpleCamera *camera;
        dwImageHandle_t pendingImage;
    };

    // Sensors used in replay
    dwSensorHandle_t m_canSensor = DW_NULL_HANDLE;
    dwSensorHandle_t m_imuSensor = DW_NULL_HANDLE;
    dwSensorHandle_t m_gpsSensor = DW_NULL_HANDLE;
    dwSensorHandle_t m_lidarSensor = DW_NULL_HANDLE;
    std::vector<CameraData> m_cameras;

    // Data read but not used yet
    bool m_isPendingCANMsgValid = false;
    bool m_isPendingIMUMsgValid = false;
    bool m_isPendingGPSMsgValid = false;
    bool m_isPendingLidarPacketValid = false;
    dwCANMessage m_pendingCANMsg = {};
    dwIMUFrame m_pendingIMUMsg = {};
    dwGPSFrame m_pendingGPSMsg = {};
    const dwLidarDecodedPacket *m_pendingLidarPacket = nullptr;
    // Data that was passed in the last handleXXX call.
    // Can also be retrieved by the user from here later before a new event overwrites it.
    dwCANMessage m_lastCANMsg = {};
    dwIMUFrame m_lastIMUMsg = {};
    dwGPSFrame m_lastGPSMsg = {};
    const dwLidarDecodedPacket *m_lastLidarPacket = nullptr;
    std::vector<dwImageHandle_t > m_lastImages;
};

}
}

#endif
