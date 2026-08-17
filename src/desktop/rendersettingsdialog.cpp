// Copyright (C) 2004-2026 Robert Griebl
// SPDX-License-Identifier: GPL-3.0-only

#include <QMetaMethod>
#include <QMetaProperty>

#include "ldraw/rendersettings.h"
#include "rendersettingsdialog.h"
#include "ui_rendersettingsdialog.h"


RenderSettingsDialog *RenderSettingsDialog::s_inst = nullptr;


RenderSettingsDialog::RenderSettingsDialog()
    : QDialog(nullptr, Qt::Tool | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowCloseButtonHint | Qt::WindowStaysOnTopHint)
    , ui(new Ui::RenderSettingsDialog)
{
    ui->setupUi(this);

    connectComboBox(ui->antiAliasing, "antiAliasing");
    connectToggleButton(ui->smoothNormals, "smoothNormals");
    connectToggleButton(ui->showLines, "renderLines");
    connectSliderAndSpinBox(ui->lineWidthSlider,  ui->lineWidth, "lineThickness", 10);

    connectToggleButton(ui->enableLighting, "lighting");
    connectSliderAndSpinBox(ui->brightnessSlider, ui->brightness, "additionalLight");
    connectSliderAndSpinBox(ui->aoStrengthSlider, ui->aoStrength, "aoStrength");
    connectSliderAndSpinBox(ui->aoSoftnessSlider, ui->aoSoftness, "aoSoftness");
    connectSliderAndSpinBox(ui->aoDistanceSlider, ui->aoDistance, "aoDistance");

    connectSliderAndSpinBox(ui->plainMetalnessSlider,    ui->plainMetalness,    "plainMetalness");
    connectSliderAndSpinBox(ui->plainRoughnessSlider,    ui->plainRoughness,    "plainRoughness");
    connectSliderAndSpinBox(ui->chromeMetalnessSlider,   ui->chromeMetalness,   "chromeMetalness");
    connectSliderAndSpinBox(ui->chromeRoughnessSlider,   ui->chromeRoughness,   "chromeRoughness");
    connectSliderAndSpinBox(ui->metallicMetalnessSlider, ui->metallicMetalness, "metallicMetalness");
    connectSliderAndSpinBox(ui->metallicRoughnessSlider, ui->metallicRoughness, "metallicRoughness");
    connectSliderAndSpinBox(ui->pearlMetalnessSlider,    ui->pearlMetalness,    "pearlMetalness");
    connectSliderAndSpinBox(ui->pearlRoughnessSlider,    ui->pearlRoughness,    "pearlRoughness");

    connect(ui->buttonBox, &QDialogButtonBox::clicked,
            this, [this](QAbstractButton *button) {
        switch (ui->buttonBox->standardButton(button)) {
        case QDialogButtonBox::Save:
            LDraw::RenderSettings::inst()->save();
            break;
        case QDialogButtonBox::RestoreDefaults:
            LDraw::RenderSettings::inst()->resetToDefaults();
            break;
        default:
            break;
        }
    });

    resize(minimumSizeHint());
}

RenderSettingsDialog *RenderSettingsDialog::inst()
{
    if (!s_inst)
        s_inst = new RenderSettingsDialog();
    return s_inst;
}

RenderSettingsDialog::~RenderSettingsDialog()
{
    delete ui;
}

void RenderSettingsDialog::connectToggleButton(QAbstractButton *checkBox, const QByteArray &propName)
{
    auto rs = LDraw::RenderSettings::inst();

    connect(checkBox, &QCheckBox::toggled,
            rs, [=](bool b) {
        rs->setProperty(propName, b);
    });

    updateOnChange(propName, [=]() {
        checkBox->setChecked(rs->property(propName).toBool());
    });
}

void RenderSettingsDialog::connectComboBox(QComboBox *comboBox, const QByteArray &propName)
{
    auto rs = LDraw::RenderSettings::inst();

    connect(comboBox, &QComboBox::currentIndexChanged,
            rs, [=](int i) {
        rs->setProperty(propName, i);
    });

    updateOnChange(propName, [=]() {
        comboBox->setCurrentIndex(rs->property(propName).toInt());
    });
}

void RenderSettingsDialog::connectSliderAndSpinBox(QSlider *slider, QDoubleSpinBox *spinBox,
                                                   const QByteArray &propName, int factor)
{
    auto rs = LDraw::RenderSettings::inst();

    connect(slider, &QSlider::valueChanged,
            spinBox, [=](int v) {
        double dv = double(v) / factor;
        if (!qFuzzyCompare(dv, spinBox->value()))
            spinBox->setValue(dv);
    });
    connect(spinBox, &QDoubleSpinBox::valueChanged,
            slider, [=](double v) {
        int iv = qRound(v * factor);
        if (iv != slider->value())
            slider->setValue(iv);

        rs->setProperty(propName, v);
    });

    updateOnChange(propName, [=]() {
        spinBox->setValue(rs->property(propName).toDouble());
    });
}

void RenderSettingsDialog::updateOnChange(const QByteArray &propName, const std::function<void()> &updateUi)
{
    auto rs = LDraw::RenderSettings::inst();
    const QMetaObject *rsmo = rs->metaObject();
    const int propIndex = rsmo->indexOfProperty(propName.constData());
    Q_ASSERT(propIndex >= 0);
    const QMetaMethod notify = rsmo->property(propIndex).notifySignal();
    Q_ASSERT(notify.isValid());
#if QT_VERSION < QT_VERSION_CHECK(6, 10, 0)
    static const QMetaMethod dispatch = staticMetaObject.method(
        staticMetaObject.indexOfSlot("propertyChanged()"));
    Q_ASSERT(dispatch.isValid());

    connect(rs, notify, this, dispatch);
    m_updateUi.insert(notify.methodIndex(), updateUi);
#else
    QMetaObject::connect(rs, notify, this, [updateUi]() { updateUi(); });
#endif
    updateUi();
}

#if QT_VERSION < QT_VERSION_CHECK(6, 10, 0)
void RenderSettingsDialog::propertyChanged()
{
    if (const auto &updateUi = m_updateUi.value(senderSignalIndex()))
        updateUi();
}
#endif
