// Copyright (C) 2004-2026 Robert Griebl
// SPDX-License-Identifier: GPL-3.0-only

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <QtCore/QVarLengthArray>
#include <QtGui/QVector3D>
#include <QtQuick3D/QQuick3DInstancing>

#include "smoothnormals.h"


namespace LDraw {

namespace {

// Welds positions within Epsilon via a uniform grid. The epsilon ball is checked against
// all overlapping cells, so two matching positions cannot be separated by an unlucky cell
// boundary (the failure mode of plain coordinate quantization).
struct VertexWelder
{
    static constexpr float Epsilon = 0.01f; // LDU
    static constexpr float CellSize = 0.1f;

    std::vector<QVector3D> positions;
    std::unordered_map<quint64, std::vector<quint32>> grid;

    static quint64 cellKey(int cx, int cy, int cz)
    {
        auto u = [](int c) { return quint64(quint32(c)) & 0x1fffff; };
        return (u(cx) << 42) | (u(cy) << 21) | u(cz);
    }

    quint32 weld(const QVector3D &p)
    {
        int c0[3], c1[3];
        for (int i = 0; i < 3; ++i) {
            c0[i] = int(std::floor((p[i] - Epsilon) / CellSize));
            c1[i] = int(std::floor((p[i] + Epsilon) / CellSize));
        }
        for (int cx = c0[0]; cx <= c1[0]; ++cx) {
            for (int cy = c0[1]; cy <= c1[1]; ++cy) {
                for (int cz = c0[2]; cz <= c1[2]; ++cz) {
                    auto it = grid.find(cellKey(cx, cy, cz));
                    if (it == grid.end())
                        continue;
                    for (quint32 idx : it->second) {
                        if ((positions[idx] - p).lengthSquared() <= (Epsilon * Epsilon))
                            return idx;
                    }
                }
            }
        }
        auto id = quint32(positions.size());
        positions.push_back(p);
        grid[cellKey(int(std::floor(p.x() / CellSize)), int(std::floor(p.y() / CellSize)),
                     int(std::floor(p.z() / CellSize)))].push_back(id);
        return id;
    }
};

struct Face
{
    quint32 vid[3];       // welded corner ids
    float *normals[3];    // write-back locations in the vertex buffers
    QVector3D normal;     // flat face normal
    float angle[3];       // corner angles, used as smoothing weights
    bool edgeSmooth[3];   // edge e connects vid[e] and vid[(e + 1) % 3]
};

quint64 edgeKey(quint32 a, quint32 b)
{
    if (a > b)
        std::swap(a, b);
    return (quint64(a) << 32) | b;
}

} // namespace

void smoothNormals(const QList<SmoothBuffer> &buffers, const QByteArray &lineBuffer)
{
    qsizetype totalVertexCount = 0;
    for (const auto &buffer : buffers)
        totalVertexCount += (buffer.vertexData->size() / buffer.stride);
    if (!totalVertexCount)
        return;

    // gather faces, welding all corners
    VertexWelder welder;
    welder.positions.reserve(size_t(totalVertexCount) / 3);
    welder.grid.reserve(size_t(totalVertexCount) / 3);

    std::vector<Face> faces;
    faces.reserve(size_t(totalVertexCount) / 3);

    for (const auto &buffer : buffers) {
        char *base = buffer.vertexData->data();
        const int stride = buffer.stride;
        const qsizetype n = buffer.vertexData->size() / stride;

        for (qsizetype i = 0; (i + 2) < n; i += 3) {
            Face face;
            QVector3D p[3];
            for (int j = 0; j < 3; ++j) {
                auto *fp = reinterpret_cast<float *>(base + (i + j) * stride);
                p[j] = QVector3D(fp[0], fp[1], fp[2]);
                face.vid[j] = welder.weld(p[j]);
                face.normals[j] = fp + 3;
            }
            face.normal = QVector3D(face.normals[0][0], face.normals[0][1], face.normals[0][2]);
            for (int j = 0; j < 3; ++j) {
                const QVector3D e1 = (p[(j + 1) % 3] - p[j]).normalized();
                const QVector3D e2 = (p[(j + 2) % 3] - p[j]).normalized();
                face.angle[j] = std::acos(std::clamp(QVector3D::dotProduct(e1, e2), -1.f, 1.f));
                face.edgeSmooth[j] = false;
            }
            faces.push_back(face);
        }
    }
    if (faces.empty())
        return;

    // type 2 lines mark hard edges, type 5 conditional lines mark smooth ones
    std::unordered_set<quint64> hardLines, smoothLines;
    {
        const auto *entry = reinterpret_cast<const QQuick3DInstancing::InstanceTableEntry *>(
            lineBuffer.constData());
        const qsizetype n = lineBuffer.size() / qsizetype(sizeof(*entry));
        hardLines.reserve(size_t(n));
        smoothLines.reserve(size_t(n));

        for (qsizetype i = 0; i < n; ++i, ++entry) {
            const quint32 a = welder.weld(entry->row0.toVector3D());
            const quint32 b = welder.weld(entry->row1.toVector3D());
            if (a == b)
                continue;
            ((entry->instanceData.w() > 0.f) ? smoothLines : hardLines).insert(edgeKey(a, b));
        }
    }

    // classify all edges shared by exactly two faces
    std::vector<std::pair<quint64, quint32>> edgeList; // (edge key, face * 4 + edge index)
    edgeList.reserve(faces.size() * 3);
    for (size_t f = 0; f < faces.size(); ++f) {
        const Face &face = faces[f];
        for (quint32 e = 0; e < 3; ++e) {
            if (face.vid[e] != face.vid[(e + 1) % 3])
                edgeList.emplace_back(edgeKey(face.vid[e], face.vid[(e + 1) % 3]),
                                      quint32(f) * 4 + e);
        }
    }
    std::sort(edgeList.begin(), edgeList.end());

    // 16-gon primitives have 22.5 degree facets, so the fallback crease angle must stay
    // above that; steeper smooth seams (e.g. lo-res primitives) are covered by their
    // conditional lines
    static constexpr float CosCreaseAngle = 0.8660254f; // cos(30 deg)

    for (size_t i = 0; i < edgeList.size(); ) {
        size_t j = i + 1;
        while ((j < edgeList.size()) && (edgeList[j].first == edgeList[i].first))
            ++j;
        if ((j - i) == 2) { // ignore open (1) and non-manifold (3+) edges
            const quint64 key = edgeList[i].first;
            Face &f0 = faces[edgeList[i].second / 4];
            Face &f1 = faces[edgeList[i + 1].second / 4];

            bool smooth;
            if (hardLines.contains(key))
                smooth = false;
            else if (smoothLines.contains(key))
                smooth = true;
            else
                smooth = (QVector3D::dotProduct(f0.normal, f1.normal) >= CosCreaseAngle);

            if (smooth) {
                f0.edgeSmooth[edgeList[i].second % 4] = true;
                f1.edgeSmooth[edgeList[i + 1].second % 4] = true;
            }
        }
        i = j;
    }

    // per-vertex incidence in CSR form: (face * 4 + corner) per welded vertex
    const auto vertexCount = quint32(welder.positions.size());
    std::vector<quint32> offsets(vertexCount + 1, 0);
    for (const Face &face : faces) {
        for (int j = 0; j < 3; ++j)
            ++offsets[face.vid[j] + 1];
    }
    for (quint32 v = 0; v < vertexCount; ++v)
        offsets[v + 1] += offsets[v];

    std::vector<quint32> incidence(faces.size() * 3);
    {
        std::vector<quint32> cursor(offsets.begin(), offsets.end() - 1);
        for (size_t f = 0; f < faces.size(); ++f) {
            for (quint32 j = 0; j < 3; ++j)
                incidence[cursor[faces[f].vid[j]]++] = quint32(f) * 4 + j;
        }
    }

    // around each vertex, cluster the incident faces via their smooth edges and average
    // the face normals per cluster, weighted by each face's corner angle
    for (quint32 v = 0; v < vertexCount; ++v) {
        const quint32 begin = offsets[v];
        const int k = int(offsets[v + 1] - begin);
        if (k < 2) // nothing to smooth, keep the flat normal
            continue;

        QVarLengthArray<int, 32> parent(k);
        for (int i = 0; i < k; ++i)
            parent[i] = i;
        auto findRoot = [&parent](int x) {
            while (parent[x] != x) {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        };

        // group by the other endpoint of each smooth edge through v: two incident faces
        // sharing such an edge belong to the same cluster
        QVarLengthArray<std::pair<quint32, int>, 64> spokes; // (other vid, local index)
        for (int i = 0; i < k; ++i) {
            const quint32 enc = incidence[begin + i];
            const Face &face = faces[enc / 4];
            const int corner = int(enc % 4);
            for (int e : { corner, (corner + 2) % 3 }) { // the two edges at this corner
                if (!face.edgeSmooth[e])
                    continue;
                const quint32 other = (face.vid[e] == v) ? face.vid[(e + 1) % 3] : face.vid[e];
                spokes.emplace_back(other, i);
            }
        }
        std::sort(spokes.begin(), spokes.end());
        for (int s = 1; s < spokes.size(); ++s) {
            if (spokes[s].first == spokes[s - 1].first) {
                const int r0 = findRoot(spokes[s - 1].second);
                const int r1 = findRoot(spokes[s].second);
                if (r0 != r1)
                    parent[r1] = r0;
            }
        }

        QVarLengthArray<QVector3D, 32> clusterNormal(k);
        for (int i = 0; i < k; ++i)
            clusterNormal[i] = { };
        for (int i = 0; i < k; ++i) {
            const quint32 enc = incidence[begin + i];
            const Face &face = faces[enc / 4];
            clusterNormal[findRoot(i)] += (face.normal * face.angle[enc % 4]);
        }
        for (int i = 0; i < k; ++i) {
            const quint32 enc = incidence[begin + i];
            const Face &face = faces[enc / 4];
            QVector3D n = clusterNormal[findRoot(i)].normalized();
            if (n.isNull())
                n = face.normal; // degenerate cluster, keep the flat normal
            float *out = face.normals[enc % 4];
            out[0] = n.x();
            out[1] = n.y();
            out[2] = n.z();
        }
    }
}

} // namespace LDraw
