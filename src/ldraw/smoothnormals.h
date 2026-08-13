// Copyright (C) 2004-2026 Robert Griebl
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QList>


namespace LDraw {

struct SmoothBuffer
{
    QByteArray *vertexData; // interleaved floats, position at offset 0, normal at offset 3
    int stride;             // in bytes
};

// Replaces the flat per-face normals in the vertex buffers with smoothed per-vertex normals:
//  * vertices are welded position-wise (epsilon based), across all buffers, so smoothing
//    works across color and sub-part boundaries
//  * an edge shared by exactly two triangles is kept hard if a type 2 edge line runs along
//    it, and smoothed if a type 5 conditional line does (the LDraw library marks all curved
//    surface seams this way); unmarked edges fall back to a 30 degree crease angle
//  * normals are accumulated angle-weighted per smoothing cluster around each vertex, so
//    creases stay sharp right up to the corner vertices
// lineBuffer holds QQuick3DInstancing::InstanceTableEntry records as built by
// QmlRenderLineInstancing::add*LineToBuffer().
void smoothNormals(const QList<SmoothBuffer> &buffers, const QByteArray &lineBuffer);

} // namespace LDraw
