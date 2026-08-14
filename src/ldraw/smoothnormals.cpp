// Copyright (C) 2004-2026 Robert Griebl
// SPDX-License-Identifier: GPL-3.0-only

#include <algorithm>
#include <array>
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

constexpr int MaxStrideFloats = 8;

// The tolerance for "vertex lies on an edge" has to cover the sagitta between a fine arc
// and the coarse chord subdividing it: 48-in-16 vertices sit at 4.4% of the edge length
// off the chord (measured on part 12885). It only gates which candidates are considered;
// the chain verification in findChain() is what rejects unrelated geometry.
float splitTolerance(float edgeLength)
{
    return std::min(0.05f + (0.065f * edgeLength), 0.5f);
}

// Split targets and candidates both come from open edges (owned by a single triangle).
// Candidates near the interior of an open edge only qualify if they chain from one edge
// endpoint to the other via open edges: the fine side of a real T-junction seam replaces
// the coarse edge completely. Unrelated geometry that merely passes nearby (wall rims at
// 2 LDU offsets, logo strokes) has no such chain and is left alone.
struct TJunctionContext
{
    static constexpr float CellSize = 2.f;

    const std::vector<QVector3D> &positions;
    const std::unordered_set<quint64> &openEdges;
    std::unordered_map<quint64, std::vector<quint32>> grid;

    void addCandidate(quint32 vid)
    {
        const QVector3D &p = positions[vid];
        grid[VertexWelder::cellKey(int(std::floor(p.x() / CellSize)),
                                   int(std::floor(p.y() / CellSize)),
                                   int(std::floor(p.z() / CellSize)))].push_back(vid);
    }

    // fills chain with the candidate vids on the interior of (vA, vB), sorted along the
    // edge, but only if they form a complete open-edge chain from vA to vB
    bool findChain(quint32 vA, quint32 vB, QVarLengthArray<quint32, 4> *chain) const
    {
        const QVector3D p0 = positions[vA];
        const QVector3D d = positions[vB] - p0;
        const float len2 = d.lengthSquared();
        const float len = std::sqrt(len2);
        const float tol = splitTolerance(len);
        if (len < (3 * tol)) // no room for an interior point
            return false;

        QVarLengthArray<std::pair<float, quint32>, 8> hits;

        const int steps = int(len / CellSize) + 1;
        for (int k = 0; k <= steps; ++k) {
            const QVector3D s = p0 + d * (float(k) / float(steps));
            const int cx = int(std::floor(s.x() / CellSize));
            const int cy = int(std::floor(s.y() / CellSize));
            const int cz = int(std::floor(s.z() / CellSize));

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        auto it = grid.find(VertexWelder::cellKey(cx + dx, cy + dy, cz + dz));
                        if (it == grid.end())
                            continue;
                        for (quint32 vid : it->second) {
                            if ((vid == vA) || (vid == vB))
                                continue;
                            bool seen = false;
                            for (const auto &hit : hits)
                                seen = seen || (hit.second == vid);
                            if (seen) // the grid walk can visit a cell more than once
                                continue;
                            const QVector3D &c = positions[vid];
                            const float t = QVector3D::dotProduct(c - p0, d) / len2;
                            if ((t <= 0.f) || (t >= 1.f))
                                continue;
                            if ((c - (p0 + d * t)).lengthSquared() > (tol * tol))
                                continue;
                            if (((c - p0).lengthSquared() <= (tol * tol))
                                    || ((c - positions[vB]).lengthSquared() <= (tol * tol)))
                                continue;
                            hits.append({ t, vid });
                        }
                    }
                }
            }
        }
        if (hits.isEmpty())
            return false;
        std::sort(hits.begin(), hits.end());

        quint32 prev = vA;
        for (const auto &hit : hits) {
            if ((hit.second == prev) || !openEdges.contains(edgeKey(prev, hit.second)))
                return false;
            prev = hit.second;
        }
        if (!openEdges.contains(edgeKey(prev, vB)))
            return false;

        for (const auto &hit : hits)
            chain->append(hit.second);
        return true;
    }
};

// Splits the triangle along its open edges at verified T-junction chains and appends the
// resulting fan triangles to out. The split vertices take the exact candidate positions,
// so the later welding connects them with the other side of the seam.
void emitSplitTriangles(const float *const rec[3], const quint32 vid[3], const bool open[3],
                        int strideFloats, const TJunctionContext &ctx,
                        int depth, std::vector<float> &out)
{
    if (depth < 4) { // each level consumes one of the (at most three) original open edges
        for (int e = 0; e < 3; ++e) {
            if (!open[e])
                continue;

            QVarLengthArray<quint32, 4> chain;
            if (!ctx.findChain(vid[e], vid[(e + 1) % 3], &chain))
                continue;

            const float *a = rec[e];
            const float *b = rec[(e + 1) % 3];
            const float *c = rec[(e + 2) % 3];
            const QVector3D p0(a[0], a[1], a[2]);
            const QVector3D d = QVector3D(b[0], b[1], b[2]) - p0;
            const float len2 = d.lengthSquared();

            // interpolated records for the chain points
            QVarLengthArray<std::array<float, MaxStrideFloats>, 4> mids;
            for (quint32 cv : chain) {
                const QVector3D &cp = ctx.positions[cv];
                const float t = QVector3D::dotProduct(cp - p0, d) / len2;
                std::array<float, MaxStrideFloats> m;
                for (int k = 0; k < strideFloats; ++k)
                    m[size_t(k)] = a[k] + ((b[k] - a[k]) * t);
                m[0] = cp.x();
                m[1] = cp.y();
                m[2] = cp.z();
                mids.append(m);
            }

            QVarLengthArray<const float *, 6> fanRec;
            QVarLengthArray<quint32, 6> fanVid;
            fanRec.append(a);
            fanVid.append(vid[e]);
            for (int i = 0; i < chain.size(); ++i) {
                fanRec.append(mids[i].data());
                fanVid.append(chain[i]);
            }
            fanRec.append(b);
            fanVid.append(vid[(e + 1) % 3]);

            for (int i = 0; i < (fanRec.size() - 1); ++i) {
                const float *prec[3] = { fanRec[i], fanRec[i + 1], c };
                const quint32 pvid[3] = { fanVid[i], fanVid[i + 1], vid[(e + 2) % 3] };
                // only the outermost fan triangles carry the remaining original open edges
                const bool popen[3] = { false,
                                        (i == (fanRec.size() - 2)) && open[(e + 1) % 3],
                                        (i == 0) && open[(e + 2) % 3] };
                emitSplitTriangles(prec, pvid, popen, strideFloats, ctx, depth + 1, out);
            }
            return;
        }
    }
    out.insert(out.end(), rec[0], rec[0] + strideFloats);
    out.insert(out.end(), rec[1], rec[1] + strideFloats);
    out.insert(out.end(), rec[2], rec[2] + strideFloats);
}

// Where hi-res meets lo-res geometry (48 vs 16 sided primitives), the fine side's vertices
// lie in the interior of the coarse side's edges: nothing welds, the seam neither closes
// nor smooths. This pass splits those edges at those vertices. Both the split targets and
// the candidate vertices can only come from open edges (edges owned by a single triangle),
// which keeps the search space tiny.
void splitTJunctions(const QList<SmoothBuffer> &buffers)
{
    qsizetype totalVertexCount = 0;
    for (const auto &buffer : buffers)
        totalVertexCount += (buffer.vertexData->size() / buffer.stride);
    if (!totalVertexCount)
        return;

    VertexWelder welder;
    welder.positions.reserve(size_t(totalVertexCount) / 3);
    welder.grid.reserve(size_t(totalVertexCount) / 3);

    std::vector<quint32> cornerVids;
    cornerVids.reserve(size_t(totalVertexCount));

    for (const auto &buffer : buffers) {
        const char *base = buffer.vertexData->constData();
        const qsizetype n = buffer.vertexData->size() / buffer.stride;
        for (qsizetype i = 0; i < n; ++i) {
            const auto *fp = reinterpret_cast<const float *>(base + i * buffer.stride);
            cornerVids.push_back(welder.weld(QVector3D(fp[0], fp[1], fp[2])));
        }
    }

    std::vector<quint64> edgeKeys;
    edgeKeys.reserve(cornerVids.size());
    for (size_t c = 0; (c + 2) < cornerVids.size(); c += 3) {
        for (int e = 0; e < 3; ++e) {
            const quint32 a = cornerVids[c + size_t(e)];
            const quint32 b = cornerVids[c + size_t((e + 1) % 3)];
            if (a != b)
                edgeKeys.push_back(edgeKey(a, b));
        }
    }
    std::sort(edgeKeys.begin(), edgeKeys.end());

    std::unordered_set<quint64> openEdges;
    std::unordered_set<quint32> candidateVids;
    for (size_t i = 0; i < edgeKeys.size(); ) {
        size_t j = i + 1;
        while ((j < edgeKeys.size()) && (edgeKeys[j] == edgeKeys[i]))
            ++j;
        if ((j - i) == 1) {
            openEdges.insert(edgeKeys[i]);
            candidateVids.insert(quint32(edgeKeys[i] >> 32));
            candidateVids.insert(quint32(edgeKeys[i] & 0xffffffff));
        }
        i = j;
    }
    if (openEdges.empty())
        return;

    TJunctionContext ctx { welder.positions, openEdges, { } };
    ctx.grid.reserve(candidateVids.size());
    for (quint32 vid : candidateVids)
        ctx.addCandidate(vid);

    size_t cornerBase = 0;
    for (const auto &buffer : buffers) {
        const int strideFloats = buffer.stride / int(sizeof(float));
        Q_ASSERT(strideFloats <= MaxStrideFloats);
        const qsizetype n = buffer.vertexData->size() / buffer.stride;
        const auto *base = reinterpret_cast<const float *>(buffer.vertexData->constData());

        std::vector<float> out;
        out.reserve(size_t(n) * size_t(strideFloats));
        bool changed = false;

        for (qsizetype tri = 0; (tri + 2) < n; tri += 3) {
            const float *a = base + (tri * strideFloats);

            bool open[3];
            bool anyOpen = false;
            for (int e = 0; e < 3; ++e) {
                const quint32 va = cornerVids[cornerBase + size_t(tri) + size_t(e)];
                const quint32 vb = cornerVids[cornerBase + size_t(tri) + size_t((e + 1) % 3)];
                open[e] = (va != vb) && openEdges.contains(edgeKey(va, vb));
                anyOpen = anyOpen || open[e];
            }

            if (anyOpen) {
                const float *rec[3] = { a, a + strideFloats, a + (2 * strideFloats) };
                const quint32 vid[3] = { cornerVids[cornerBase + size_t(tri)],
                                         cornerVids[cornerBase + size_t(tri) + 1],
                                         cornerVids[cornerBase + size_t(tri) + 2] };
                const size_t before = out.size();
                emitSplitTriangles(rec, vid, open, strideFloats, ctx, 0, out);
                changed = changed || (out.size() != (before + (3 * size_t(strideFloats))));
            } else {
                out.insert(out.end(), a, a + (3 * strideFloats));
            }
        }
        cornerBase += size_t(n);

        if (changed) {
            *buffer.vertexData = QByteArray(reinterpret_cast<const char *>(out.data()),
                                            qsizetype(out.size() * sizeof(float)));
        }
    }
}

} // namespace

void smoothNormals(const QList<SmoothBuffer> &buffers, const QByteArray &lineBuffer)
{
    splitTJunctions(buffers);

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
    // above that. A type 5 conditional line widens the allowed angle (lo-res primitives
    // have 45 degree facets), but must not force smoothness: partial cylinder primitives
    // also carry condlines on their angular boundary edges, and those can end up on real
    // corners (the wall-to-flat-side edges of 25269 measure 94 degrees).
    static constexpr float CosCreaseAngle = 0.8660254f;         // cos(30 deg)
    static constexpr float CosCondlineCreaseAngle = 0.5f;       // cos(60 deg)

    for (size_t i = 0; i < edgeList.size(); ) {
        size_t j = i + 1;
        while ((j < edgeList.size()) && (edgeList[j].first == edgeList[i].first))
            ++j;
        if ((j - i) == 2) { // ignore open (1) and non-manifold (3+) edges
            const quint64 key = edgeList[i].first;
            Face &f0 = faces[edgeList[i].second / 4];
            Face &f1 = faces[edgeList[i + 1].second / 4];

            bool smooth;
            if (hardLines.contains(key)) {
                smooth = false;
            } else {
                const float cosAngle = QVector3D::dotProduct(f0.normal, f1.normal);
                smooth = (cosAngle >= (smoothLines.contains(key) ? CosCondlineCreaseAngle
                                                                 : CosCreaseAngle));
            }

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
