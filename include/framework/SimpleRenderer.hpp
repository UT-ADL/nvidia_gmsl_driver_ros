/* Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef SAMPLES_COMMON_SIMPLERENDERER_HPP_
#define SAMPLES_COMMON_SIMPLERENDERER_HPP_

#include <framework/Checks.hpp>

#include <dw/visualization/Renderer.h>
#include <dw/image/Image.h>

#include <vector>

namespace dw_samples
{
namespace common
{

class SimpleRenderer
{
public :
    SimpleRenderer(dwRendererHandle_t renderer, dwContextHandle_t ctx);
    ~SimpleRenderer();

    /// Specifies a 2D line segment between point a and point b, in floating point coordinates
    typedef struct {
        dwVector2f a;
        dwVector2f b;
    } dwLineSegment2Df;

    void setScreenRect(dwRect screenRect);

    void setRenderBufferNormCoords(float32_t width, float32_t height, dwRenderBufferPrimitiveType type);

    void setColor(const dwVector4f color);

    void render(float32_t const *vertices2D, uint32_t numVertices, dwRenderBufferPrimitiveType type);

    void renderQuad(dwImageGL *input);

    void renderText(uint32_t textX, uint32_t textY, const dwVector4f color, std::string text,
    dwRendererFonts font = DW_RENDER_FONT_VERDANA_20);

    void setRectangleCoords(float32_t* coords, dwRect rectangle, uint32_t vertexStride);

    void renderRectangle(const dwRect &rectangle, const dwVector4f color);

    void renderRectangles(const dwRect* rectangles, uint32_t numBoxes);

    void renderRectangles(const std::vector<dwRect> &rectangles);

    void renderRectanglesWithLabels(const std::vector<std::pair<dwRect, std::string>> &rectanglesWithLabels,
                                    float32_t normalizationWidth, float32_t normalizationHeight);

    void renderLineSegments(const std::vector<dwLineSegment2Df> &segments, float32_t lineWidth, const dwVector4f color);

    void renderPolyline(const std::vector<dwVector2f> &points, float32_t lineWidth, const dwVector4f color);

    void renderPoints(const std::vector<dwVector2f> &points, float32_t pointSize, const dwVector4f color);

private:
    static constexpr auto m_maxVertexCount = 20000u;

    // one render buffer per primitive
    void fillBuffer(float32_t const *vertices2D, uint32_t numVertices, dwRenderBufferPrimitiveType type) const;
    dwRenderBufferHandle_t m_renderBuffer[5];

    // handle to a renderer, not owned by this class
    dwRendererHandle_t m_renderer;
};

} // namespace common
} // namespace dw_samples

#endif

