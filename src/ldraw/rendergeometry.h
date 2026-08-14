// Copyright (C) 2004-2026 Robert Griebl
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <QtGui/QColor>
#include <QtGui/QVector3D>
#include <QQmlEngine>
#include <QtQuick3D/QQuick3DGeometry>
#include <QtQuick3D/QQuick3DInstancing>

#include "bricklink/color.h"

QT_FORWARD_DECLARE_CLASS(QQuick3DTextureData)


namespace LDraw {

class Part;

class QmlRenderGeometry : public QQuick3DGeometry
{
    Q_OBJECT
    QML_NAMED_ELEMENT(RenderGeometry)
    QML_UNCREATABLE("")
    Q_PROPERTY(QColor color READ color NOTIFY colorChanged FINAL)
    Q_PROPERTY(float luminance READ luminance NOTIFY colorChanged FINAL)
    Q_PROPERTY(bool isChrome READ isChrome NOTIFY colorChanged FINAL)
    Q_PROPERTY(bool isMetallic READ isMetallic NOTIFY colorChanged FINAL)
    Q_PROPERTY(bool isPearl READ isPearl NOTIFY colorChanged FINAL)
    Q_PROPERTY(bool isTwoSided READ isTwoSided CONSTANT FINAL)
    Q_PROPERTY(QQuick3DTextureData *textureData READ textureData NOTIFY colorChanged FINAL)
    Q_PROPERTY(QVector3D center READ center CONSTANT FINAL)
    Q_PROPERTY(float radius READ radius CONSTANT FINAL)

public:
    // a surface built from LDraw color 16 inherits the model color: it is re-colored via
    // setModelColor() instead of rebuilding the geometry. twoSided surfaces (BFC uncertified
    // or NOCLIP geometry) must not be back-face culled.
    QmlRenderGeometry(const BrickLink::Color *color, bool inheritsModelColor, bool twoSided);

    QColor color() const        { return m_color->ldrawColor(); }
    float luminance() const     { return m_color->luminance(); }
    bool isChrome() const       { return m_color->isChrome() || (m_color->id() == 0); }
    bool isMetallic() const     { return m_color->isMetallic(); }
    bool isPearl() const        { return m_color->isPearl(); }
    bool isTwoSided() const     { return m_twoSided; }
    QQuick3DTextureData *textureData() const     { return m_texture; }
    void setTextureData(QQuick3DTextureData *td) { m_texture = td; }
    QVector3D center() const                     { return m_center; }
    void setCenter(const QVector3D &center)      { m_center = center; }
    float radius() const                         { return m_radius; }
    void setRadius(float radius)                 { m_radius = radius; }

    bool inheritsModelColor() const              { return m_inheritsModelColor; }
    // takes ownership of textureData (may be nullptr) and deletes the old one
    void setModelColor(const BrickLink::Color *color, QQuick3DTextureData *textureData);

signals:
    void colorChanged();

private:
    const BrickLink::Color *m_color;
    QQuick3DTextureData *m_texture = nullptr;
    QVector3D m_center;
    float m_radius = 0;
    bool m_inheritsModelColor = false;
    bool m_twoSided = false;
};

class QmlRenderLineInstancing : public QQuick3DInstancing
{
    Q_OBJECT

public:
    QmlRenderLineInstancing();
    QByteArray getInstanceBuffer(int *instanceCount) override;

    void clear();
    void setBuffer(const QByteArray &ba);

    // an invalid color c means "the model's edge color": the line is flagged in the instance
    // table and the customEdgeColor uniform is substituted in the line shader
    static void addLineToBuffer(QByteArray &buffer, const QColor &c, const QVector3D &p0,
                                const QVector3D &p1);
    static void addConditionalLineToBuffer(QByteArray &buffer, const QColor &c, const QVector3D &p0,
                                           const QVector3D &p1, const QVector3D &p2, const QVector3D &p3);

private:
    QByteArray m_buffer;
};

} // namespace LDraw

Q_DECLARE_METATYPE(LDraw::QmlRenderGeometry *)
