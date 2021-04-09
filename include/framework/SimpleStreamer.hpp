/* Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef SAMPLES_COMMON_SIMPLESTREAMER_HPP_
#define SAMPLES_COMMON_SIMPLESTREAMER_HPP_

#include <dw/image/ImageStreamer.h>
#include <framework/Checks.hpp>

namespace dw_samples
{
namespace common
{

template<typename T>
inline T getTyped(dwImageHandle_t img);

template<>
inline dwImageHandle_t getTyped<dwImageHandle_t>(dwImageHandle_t img) {
    return img;
}

template<>
inline dwImageCPU* getTyped<dwImageCPU*>(dwImageHandle_t img) {
    dwImageCPU* imgCPU;
    dwImage_getCPU(&imgCPU, img);
    return imgCPU;
}

template<>
inline dwImageCUDA* getTyped<dwImageCUDA*>(dwImageHandle_t img) {
    dwImageCUDA* imgCUDA;
    dwImage_getCUDA(&imgCUDA, img);
    return imgCUDA;
}

template<>
inline dwImageGL* getTyped<dwImageGL*>(dwImageHandle_t img) {
    dwImageGL* imgGL;
    dwImage_getGL(&imgGL, img);
    return imgGL;
}

#ifdef VIBRANTE
template<>
inline  dwImageNvMedia* getTyped<dwImageNvMedia*>(dwImageHandle_t img) {
    dwImageNvMedia* imgNvMedia;
    dwImage_getNvMedia(&imgNvMedia, img);
    return imgNvMedia;
}
#endif

/**
 * A wrapper for the streamer classes. It sacrifices performance to provide a very simple interface.
 * Posting an image blocks and directly returns the image.
 *
 * Usage:
 * \code
 * SimpleImageStreamer streamer(propsIn, DW_IMAGE_GL, 66000, ctx);
 *
 * dwImageCUDA inputImg = getImgFromSomewhere();
 *
 * dwImageGL *outputImg = streamer.post(&inputImg);
 * ...do GL stuff...
 * streamer.release();
 *
 * \endcode
 *
 * NOTE: we strongly encourage to use the real dwImageStreamer, please see the samples in Image for a
 *       complete tutorial
 */
template <typename T = dwImageHandle_t>
class SimpleImageStreamer
{
public:
    SimpleImageStreamer(const dwImageProperties &imageProps, dwImageType typeOut, dwTime_t timeout, dwContextHandle_t ctx)
        : m_timeout(timeout)
        , m_pendingReturn(nullptr)
    {
        CHECK_DW_ERROR( dwImageStreamer_initialize(&m_streamer, &imageProps, typeOut, ctx) );
    }

    ~SimpleImageStreamer()
    {
#ifndef DW_USE_NVMEDIA
        if (m_pendingReturn)
            release();
#endif
        dwImageStreamer_release(m_streamer);
    }

    /// Posts the input image, blocks until the output image is available, returns the output image.

    typename std::conditional<std::is_same<T, dwImageHandle_t>::value, T, T*>::type post(dwImageHandle_t imgS)
    {
        if (m_pendingReturn)
            release();

        CHECK_DW_ERROR(dwImageStreamer_producerSend(imgS, m_streamer));
        CHECK_DW_ERROR(dwImageStreamer_consumerReceive(&m_pendingReturn, m_timeout, m_streamer));

        if (!m_pendingReturn)
            throw std::runtime_error("Cannot receive image");

        return getTyped<typename std::conditional<std::is_same<T, dwImageHandle_t>::value, T, T*>::type>(m_pendingReturn);
    }

    /// Returns the previously received image to the real streamer.
    /// This method is optional. Either post() or the destructor will also return the image.
    void release()
    {
        if (!m_pendingReturn)
            throw std::runtime_error("Nothing to release");

        CHECK_DW_ERROR(dwImageStreamer_consumerReturn(&m_pendingReturn, m_streamer));

        m_pendingReturn = nullptr;

        CHECK_DW_ERROR(dwImageStreamer_producerReturn(nullptr, m_timeout, m_streamer));
    }

private:
    dwImageStreamerHandle_t m_streamer;
    dwTime_t m_timeout;
    
    dwImageHandle_t m_pendingReturn;
};

}
}

#endif
