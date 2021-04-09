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

#include "SimpleRenderer.hpp"

namespace dw_samples
{
namespace common
{

SimpleRenderer::SimpleRenderer(dwRendererHandle_t renderer, dwContextHandle_t ctx)
    : m_renderer(renderer)
{
    dwRenderBufferVertexLayout layout;
    layout.posFormat   = DW_RENDER_FORMAT_R32G32_FLOAT;
    layout.posSemantic = DW_RENDER_SEMANTIC_POS_XY;
    layout.colFormat   = DW_RENDER_FORMAT_NULL;
    layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
    layout.texFormat   = DW_RENDER_FORMAT_NULL;
    layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;

    CHECK_DW_ERROR( dwRenderBuffer_initialize(&m_renderBuffer[DW_RENDER_PRIM_POINTLIST], layout, DW_RENDER_PRIM_POINTLIST, m_maxVertexCount,
                                              ctx) );
    CHECK_DW_ERROR( dwRenderBuffer_initialize(&m_renderBuffer[DW_RENDER_PRIM_LINELIST], layout, DW_RENDER_PRIM_LINELIST, m_maxVertexCount,
                                              ctx) );
    CHECK_DW_ERROR( dwRenderBuffer_initialize(&m_renderBuffer[DW_RENDER_PRIM_TRIANGLELIST], layout, DW_RENDER_PRIM_TRIANGLELIST, m_maxVertexCount,
                                              ctx) );
    CHECK_DW_ERROR( dwRenderBuffer_initialize(&m_renderBuffer[DW_RENDER_PRIM_LINESTRIP], layout, DW_RENDER_PRIM_LINESTRIP, m_maxVertexCount,
                                              ctx) );
    CHECK_DW_ERROR( dwRenderBuffer_initialize(&m_renderBuffer[DW_RENDER_PRIM_LINELOOP], layout, DW_RENDER_PRIM_LINELOOP, m_maxVertexCount,
                                              ctx) );

    glDepthFunc(GL_ALWAYS);
}

SimpleRenderer::~SimpleRenderer()
{
    for (int i=0; i < 5; i++)
        dwRenderBuffer_release(m_renderBuffer[i]);
}

void SimpleRenderer::setColor(const dwVector4f color)
{
    CHECK_DW_ERROR(dwRenderer_setColor(color, m_renderer));
}

void SimpleRenderer::render(float32_t const *vertices2D, uint32_t numVertices, dwRenderBufferPrimitiveType type)
{
    fillBuffer(vertices2D, numVertices, type);
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_renderBuffer[type], m_renderer));
}

void SimpleRenderer::setScreenRect(dwRect screenRect)
{
    CHECK_DW_ERROR(dwRenderer_setRect(screenRect, m_renderer));
}

void SimpleRenderer::setRenderBufferNormCoords(float32_t width, float32_t height, dwRenderBufferPrimitiveType type)
{
    CHECK_DW_ERROR(dwRenderBuffer_set2DCoordNormalizationFactors(width, height, m_renderBuffer[type]));
}

void SimpleRenderer::renderQuad(dwImageGL *input)
{
    if (!input) {
        throw std::runtime_error("Null dwImageGL passed to renderQuad.");
    }

    // render dwImageGL
    CHECK_DW_ERROR(dwRenderer_renderTexture(input->tex, input->target, m_renderer));
}

void SimpleRenderer::renderText(uint32_t textX, uint32_t textY, const dwVector4f color, std::string text,
dwRendererFonts font)
{
    // store color
    dwVector4f oldColor;
    CHECK_DW_ERROR(dwRenderer_getColor(&oldColor, m_renderer));

    // store font
    dwRendererFonts oldFont;
    CHECK_DW_ERROR(dwRenderer_getFont(&oldFont, m_renderer));

    // overlay text
    CHECK_DW_ERROR(dwRenderer_setColor(color, m_renderer));
    CHECK_DW_ERROR(dwRenderer_setFont(font, m_renderer));
    CHECK_DW_ERROR(dwRenderer_renderText(textX, textY, text.c_str(), m_renderer));

    // restore color
    CHECK_DW_ERROR(dwRenderer_setColor(oldColor, m_renderer));

    //restore font
    CHECK_DW_ERROR(dwRenderer_setFont(oldFont, m_renderer));
}

void SimpleRenderer::setRectangleCoords(float32_t* coords, dwRect rectangle, uint32_t vertexStride)
{
    float32_t x_start = static_cast<float32_t>(rectangle.x) - 0.5f ;
    float32_t x_end   = static_cast<float32_t>(rectangle.x + rectangle.width);
    float32_t y_start = static_cast<float32_t>(rectangle.y) - 0.5f;
    float32_t y_end   = static_cast<float32_t>(rectangle.y + rectangle.height);
    coords[0]  = x_start;
    coords[1]  = y_start;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_start;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0]  = x_end;
    coords[1]  = y_end;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_end;
    coords[1] = y_start;
    coords    += vertexStride;
    coords[0] = x_start;
    coords[1] = y_start;
}

void SimpleRenderer::renderRectangle(const dwRect &rectangle, const dwVector4f color)
{
    // store color
    dwVector4f oldColor;
    CHECK_DW_ERROR(dwRenderer_getColor(&oldColor, m_renderer));

    // render rectangle
    CHECK_DW_ERROR(dwRenderer_setColor(color, m_renderer));

    renderRectangles(&rectangle, 1);

    // restore color
    CHECK_DW_ERROR(dwRenderer_setColor(oldColor, m_renderer));
}

void SimpleRenderer::renderRectangles(const dwRect* rectangles, uint32_t numBoxes)
{

    float32_t* coords = nullptr;
    uint32_t maxVertices = 0;
    uint32_t vertexStride = 0;
    CHECK_DW_ERROR(dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, m_renderBuffer[DW_RENDER_PRIM_LINELIST]));

    uint32_t n_boxes    = numBoxes;
    uint32_t n_verts    = 8 * n_boxes;
    if (n_verts > maxVertices) {
        n_boxes = maxVertices / 8;
        n_verts = 8 * n_boxes;
    }

    for (uint32_t i = 0U; i < n_boxes; ++i) {
        setRectangleCoords(coords + 8*i*vertexStride, rectangles[i], vertexStride);
    }

    CHECK_DW_ERROR(dwRenderBuffer_unmap(n_verts, m_renderBuffer[DW_RENDER_PRIM_LINELIST]));
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_renderBuffer[DW_RENDER_PRIM_LINELIST], m_renderer));
}

void SimpleRenderer::renderRectangles(const std::vector<dwRect> &rectangles)
{
    if (rectangles.size() == 0)
        return;

    renderRectangles(rectangles.data(), rectangles.size());

}

void SimpleRenderer::renderRectanglesWithLabels(const std::vector<std::pair<dwRect, std::string>> &rectanglesWithLabels,
                                                               float32_t normalizationWidth, float32_t normalizationHeight)
{
    if (rectanglesWithLabels.size() == 0)
        return;

    dwVector4f color;
    CHECK_DW_ERROR(dwRenderer_getColor(&color, m_renderer));

    float32_t* coords = nullptr;
    uint32_t maxVertices = 0;
    uint32_t vertexStride = 0;
    CHECK_DW_ERROR(dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, m_renderBuffer[DW_RENDER_PRIM_LINELIST]));

    uint32_t n_boxes    = static_cast<uint32_t>(rectanglesWithLabels.size());
    uint32_t n_verts    = 8 * n_boxes;
    if (n_verts > maxVertices)
    {
        n_boxes = maxVertices / 8;
        n_verts = 8 * n_boxes;
    }

    dwRect screenRectangle;
    CHECK_DW_ERROR(dwRenderer_getRect(&screenRectangle, m_renderer));
    float32_t screenWidth = static_cast<float32_t>(screenRectangle.width);
    float32_t screenHeight = static_cast<float32_t>(screenRectangle.height);

    for (uint32_t i = 0U; i < n_boxes; ++i)
    {
        std::pair<dwBox2D, std::string> boxClassIdPair = rectanglesWithLabels[i];
        dwBox2D &box = boxClassIdPair.first;
        std::string classLabel = boxClassIdPair.second;

        float32_t x_start = static_cast<float32_t>(box.x) - 0.5f;
        float32_t y_start = static_cast<float32_t>(box.y) - 0.5f;
        float32_t x_end   = static_cast<float32_t>(box.width) + x_start;
        float32_t y_end   = static_cast<float32_t>(box.height) + y_start;

        coords[0] = x_start;
        coords[1] = y_start;
        coords += vertexStride;

        coords[0] = x_start;
        coords[1] = y_end;
        coords += vertexStride;

        coords[0] = x_start;
        coords[1] = y_end;
        coords += vertexStride;

        coords[0] = x_end;
        coords[1] = y_end;
        coords += vertexStride;

        coords[0] = x_end;
        coords[1] = y_end;
        coords += vertexStride;

        coords[0] = x_end;
        coords[1] = y_start;
        coords += vertexStride;

        coords[0] = x_end;
        coords[1] = y_start;
        coords += vertexStride;

        coords[0] = x_start;
        coords[1] = y_start;
        coords += vertexStride;

        renderText(static_cast<int32_t>(((box.x - 0.5f) * screenWidth) / normalizationWidth),
                   screenRectangle.height -
                   static_cast<int32_t>(((box.y - 0.5f) * screenHeight) / normalizationHeight),
                   color, classLabel.c_str());
    }

    CHECK_DW_ERROR(dwRenderBuffer_unmap(n_verts, m_renderBuffer[DW_RENDER_PRIM_LINELIST]));
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_renderBuffer[DW_RENDER_PRIM_LINELIST], m_renderer));
}

void SimpleRenderer::renderLineSegments(const std::vector<dwLineSegment2Df> &segments,
                                                       float32_t lineWidth, const dwVector4f color)
{
    // store
    dwVector4f oldColor;
    CHECK_DW_ERROR(dwRenderer_getColor(&oldColor, m_renderer));
    float32_t oldLineWidth;
    CHECK_DW_ERROR(dwRenderer_getLineWidth(&oldLineWidth, m_renderer));

    // change
    CHECK_DW_ERROR(dwRenderer_setColor(color, m_renderer));
    CHECK_DW_ERROR(dwRenderer_setLineWidth(lineWidth, m_renderer));

    // if alpha channel is not 1.0, enable transparency
    if (color.z < 1.0f) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    float32_t* coords = nullptr;
    uint32_t maxVertices = 0;
    uint32_t vertexStride = 0;
    CHECK_DW_ERROR(dwRenderBuffer_map(&coords, &maxVertices, &vertexStride, m_renderBuffer[DW_RENDER_PRIM_LINELIST]));

    dwRect screenRectangle;
    CHECK_DW_ERROR(dwRenderer_getRect(&screenRectangle, m_renderer));

    for (uint32_t i = 0U; i < segments.size(); ++i)
    {
        // transform pixel coords into rendered rectangle
        float32_t x_start = static_cast<float32_t>(segments[i].a.x) - 0.5f;
        float32_t y_start = static_cast<float32_t>(segments[i].a.y) - 0.5f;
        float32_t x_end   = static_cast<float32_t>(segments[i].b.x) - 0.5f;
        float32_t y_end   = static_cast<float32_t>(segments[i].b.y) - 0.5f;

        coords[0] = x_start;
        coords[1] = y_start;
        coords += vertexStride;

        coords[0] = x_end;
        coords[1] = y_end;
        coords += vertexStride;
    }

    CHECK_DW_ERROR(dwRenderBuffer_unmap(static_cast<uint32_t>(segments.size() * 2), m_renderBuffer[DW_RENDER_PRIM_LINELIST]));
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_renderBuffer[DW_RENDER_PRIM_LINELIST], m_renderer));

    if (color.z < 1.0f) {
        glDisable(GL_BLEND);
    }

    // restore
    CHECK_DW_ERROR(dwRenderer_setColor(oldColor, m_renderer));
    CHECK_DW_ERROR(dwRenderer_setLineWidth(oldLineWidth, m_renderer));
}

void SimpleRenderer::renderPolyline(const std::vector<dwVector2f> &points, float32_t lineWidth,
                                    const dwVector4f color)
{
    // store
    dwVector4f oldColor;
    CHECK_DW_ERROR(dwRenderer_getColor(&oldColor, m_renderer));
    float32_t oldLineWidth;
    CHECK_DW_ERROR(dwRenderer_getLineWidth(&oldLineWidth, m_renderer));

    // change
    CHECK_DW_ERROR(dwRenderer_setColor(color, m_renderer));
    CHECK_DW_ERROR(dwRenderer_setLineWidth(lineWidth, m_renderer));

    // if alpha channel is not 1.0, enable transparency
    if (color.z < 1.0f) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    float32_t* coords = nullptr;
    uint32_t maxVertices = 0;
    uint32_t n_verts = 0;
    uint32_t vertexStride = 0;
    CHECK_DW_ERROR(dwRenderBuffer_map(&coords, &maxVertices, &vertexStride,
                                      m_renderBuffer[DW_RENDER_PRIM_LINELIST]));

    for (uint32_t i = 1U; i < points.size(); ++i)
    {
        // transform pixel coords into rendered rectangle
        float32_t xPrev = points[i - 1].x - 0.5f;
        float32_t yPrev = points[i - 1].y - 0.5f;
        float32_t xCurr = points[i].x - 0.5f;
        float32_t yCurr = points[i].y - 0.5f;

        n_verts += 2;

        coords[0] = xPrev;
        coords[1] = yPrev;
        coords += vertexStride;

        coords[0] = xCurr;
        coords[1] = yCurr;
        coords += vertexStride;
    }

    CHECK_DW_ERROR(dwRenderBuffer_unmap((static_cast<uint32_t>(points.size()-1)*2),
                                        m_renderBuffer[DW_RENDER_PRIM_LINELIST]));
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_renderBuffer[DW_RENDER_PRIM_LINELIST], m_renderer));

    if (color.z < 1.0f) {
        glDisable(GL_BLEND);
    }

    // restore
    CHECK_DW_ERROR(dwRenderer_setColor(oldColor, m_renderer));
    CHECK_DW_ERROR(dwRenderer_setLineWidth(oldLineWidth, m_renderer));
}

void SimpleRenderer::renderPoints(const std::vector<dwVector2f> &points, float32_t pointSize,
                                  const dwVector4f color)
{
    // store
    dwVector4f oldColor;
    CHECK_DW_ERROR(dwRenderer_getColor(&oldColor, m_renderer));
    float32_t oldPointSize;
    CHECK_DW_ERROR(dwRenderer_getPointSize(&oldPointSize, m_renderer));

    // change
    CHECK_DW_ERROR(dwRenderer_setColor(color, m_renderer));
    CHECK_DW_ERROR(dwRenderer_setPointSize(pointSize, m_renderer));

    // if alpha channel is not 1.0, enable transparency
    if (color.z < 1.0f) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    float32_t* coords = nullptr;
    uint32_t maxVertices = 0;
    uint32_t n_verts = 0;
    uint32_t vertexStride = 0;
    CHECK_DW_ERROR(dwRenderBuffer_map(&coords, &maxVertices, &vertexStride,
                                      m_renderBuffer[DW_RENDER_PRIM_POINTLIST]));

    for (uint32_t i = 0U; i < points.size(); ++i)
    {
        // transform pixel coords into rendered rectangle
        float32_t xCurr = points[i].x - 0.5f;
        float32_t yCurr = points[i].y - 0.5f;

        n_verts++;

        coords[0] = xCurr;
        coords[1] = yCurr;
        coords += vertexStride;
    }

    CHECK_DW_ERROR(dwRenderBuffer_unmap(static_cast<uint32_t>(points.size()),
                                        m_renderBuffer[DW_RENDER_PRIM_POINTLIST]));
    CHECK_DW_ERROR(dwRenderer_renderBuffer(m_renderBuffer[DW_RENDER_PRIM_POINTLIST], m_renderer));

    if (color.z < 1.0f) {
        glDisable(GL_BLEND);
    }

    // restore
    CHECK_DW_ERROR(dwRenderer_setColor(oldColor, m_renderer));
    CHECK_DW_ERROR(dwRenderer_setPointSize(oldPointSize, m_renderer));
}

void SimpleRenderer::fillBuffer(float32_t const *vertices2D, uint32_t numVertices,
                                dwRenderBufferPrimitiveType type) const
{
    float32_t *coords = nullptr;
    auto maxVertices  = uint32_t{0};
    auto vertexStride = uint32_t{0};
    CHECK_DW_ERROR(dwRenderBuffer_map(&coords, &maxVertices, &vertexStride,
                                      m_renderBuffer[type]));
    auto i = 0u;
    for (; i < numVertices; ++i) {
        if (i == m_maxVertexCount) {
            std::cout << "SimpleRenderer: render buffer size insufficient, discarding remaining vertices"
                      << std::endl;
            break;
        }
        coords[0] = vertices2D[2*i];
        coords[1] = vertices2D[2*i+1];
        coords += vertexStride;
    }
    CHECK_DW_ERROR(dwRenderBuffer_unmap(i, m_renderBuffer[type]));
}

}
}
