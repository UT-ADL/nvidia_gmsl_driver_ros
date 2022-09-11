// Created by Maxandre Ogeret.
// (c) 2022 University of Tartu - Autonomous Driving Lab.

#pragma once

#include <nvmedia_surface.h>

namespace camera_common
{
constexpr int DEFAULT_FRAMERATE = 30;
constexpr int DEFAULT_WIDTH = 1920;
constexpr int DEFAULT_HEIGHT = 1208;

inline dwImageProperties get_rgba_pitch_img_prop(uint32_t width = DEFAULT_WIDTH, uint32_t height = DEFAULT_HEIGHT)
{
  return { DW_IMAGE_NVMEDIA, width, height, DW_IMAGE_FORMAT_RGBA_UINT8, {}, DW_IMAGE_MEMORY_TYPE_PITCH };
}

inline dwImageProperties get_yuv420_pitch_img_prop(uint32_t width = DEFAULT_WIDTH, uint32_t height = DEFAULT_HEIGHT)
{
  return { DW_IMAGE_NVMEDIA, width, height, DW_IMAGE_FORMAT_YUV420_UINT8_SEMIPLANAR, {}, DW_IMAGE_MEMORY_TYPE_PITCH };
}

inline dwImageProperties get_yuv420_block_img_prop(uint32_t width = DEFAULT_WIDTH, uint32_t height = DEFAULT_HEIGHT)
{
  return { DW_IMAGE_NVMEDIA, width, height, DW_IMAGE_FORMAT_YUV420_UINT8_SEMIPLANAR, {}, DW_IMAGE_MEMORY_TYPE_BLOCK };
}

inline NvMediaSurfaceType get_yuv420_pitch_img_surface()
{
  NvMediaSurfFormatAttr attrs_[NVM_SURF_FMT_ATTR_MAX];
  NVM_SURF_FMT_SET_ATTR_YUV(attrs_, YUV, 420, SEMI_PLANAR, UINT, 8, PL)
  return NvMediaSurfaceFormatGetType(attrs_, NVM_SURF_FMT_ATTR_MAX);
}

inline NvMediaSurfaceType get_yuv420_block_img_surface()
{
  NvMediaSurfFormatAttr attrs_[NVM_SURF_FMT_ATTR_MAX];
  NVM_SURF_FMT_SET_ATTR_YUV(attrs_, YUV, 420, SEMI_PLANAR, UINT, 8, BL)
  return NvMediaSurfaceFormatGetType(attrs_, NVM_SURF_FMT_ATTR_MAX);
}

}  // namespace camera_common