// Copyright (C) 2004-2026 Robert Griebl
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <functional>

#include <QDialog>
#include <QHash>

QT_FORWARD_DECLARE_CLASS(QSlider)
QT_FORWARD_DECLARE_CLASS(QDoubleSpinBox)
QT_FORWARD_DECLARE_CLASS(QAbstractButton)
QT_FORWARD_DECLARE_CLASS(QComboBox)

namespace Ui {
class RenderSettingsDialog;
}

namespace LDraw {
class RenderSettings;
}

class RenderSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    static RenderSettingsDialog *inst();
    ~RenderSettingsDialog() override;

private:
    explicit RenderSettingsDialog();
    void connectToggleButton(QAbstractButton *checkBox, const QByteArray &propName);
    void connectComboBox(QComboBox *comboBox, const QByteArray &propName);
    void connectSliderAndSpinBox(QSlider *slider, QDoubleSpinBox *spinBox, const QByteArray &propName,
                                 int factor = 100);
    void updateOnChange(const QByteArray &propName, const std::function<void()> &updateUi);

    Ui::RenderSettingsDialog *ui;
    static RenderSettingsDialog *s_inst;

#if QT_VERSION < QT_VERSION_CHECK(6, 10, 0)
    Q_SLOT void propertyChanged();
    QHash<int, std::function<void()>> m_updateUi; // notify signal index -> UI updater
#endif
};
