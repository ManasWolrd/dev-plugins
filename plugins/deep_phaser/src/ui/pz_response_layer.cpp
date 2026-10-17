#include "pz_response_layer.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "pluginshared/color_preset.hpp"

// ------------------------------------------------------------
// PzResponseLayer
// ------------------------------------------------------------

PzResponseLayer::PzResponseLayer() {
    setInterceptsMouseClicks(false, false);  // 背景层不抢鼠标事件
}

void PzResponseLayer::SetData(
    std::array<global::PolarPoint, global::kMaxSections> const& poles,
    std::array<global::PolarPoint, global::kMaxSections> const& zeros) {
    poles_ = poles;
    zeros_ = zeros;

    // 峰值归一到 0dB，并钳制到显示窗口（低于地板曲线被削平）
    auto mags = ComputeMagnitudes(poles_, zeros_);
    float peak = 0.0f;
    for (float m : mags) {
        peak = std::max(peak, m);
    }
    db_.resize(kGrid);
    if (peak <= 1.0e-12f) {
        std::fill(db_.begin(), db_.end(), kDbFloor);
    }
    else {
        for (int i = 0; i < kGrid; ++i) {
            float const rel = std::max(mags[i] / peak, 1.0e-9f);
            float const db = 20.0f * std::log10(rel);
            db_[i] = std::clamp(db, kDbFloor, kDbCeil);
        }
    }
    repaint();
}

std::vector<float> PzResponseLayer::ComputeMagnitudes(
    std::array<global::PolarPoint, global::kMaxSections> const& poles,
    std::array<global::PolarPoint, global::kMaxSections> const& zeros) {
    // 极坐标 → 上半平面复数 → 共轭对称全集合（double 精度）；被禁用的节不参与
    std::vector<std::complex<double>> pf, zf;
    pf.reserve(global::kMaxSections * 2);
    zf.reserve(global::kMaxSections * 2);
    for (size_t i = 0; i < global::kMaxSections; ++i) {
        if (global::IsPoleDisabled(poles[i])) {
            continue; // 非法极点连同同索引零点整节退出分解，曲线随之不含它
        }
        auto const pc = global::ToComplex(poles[i]);
        auto const zc = global::ToComplex(zeros[i]);
        std::complex<double> const p{pc.real(), pc.imag()};
        std::complex<double> const z{zc.real(), zc.imag()};
        pf.push_back(p);
        pf.push_back(std::conj(p));
        zf.push_back(z);
        zf.push_back(std::conj(z));
    }
    // 全部节被禁用：输出为直通（|H| ≡ 1）
    if (pf.empty()) {
        std::vector<float> flat(kGrid, 1.0f);
        return flat;
    }

    std::vector<float> mags(kGrid);
    for (int i = 0; i < kGrid; ++i) {
        float const x = kXMin + (kXMax - kXMin) * static_cast<float>(i) / (kGrid - 1);
        std::complex<double> const q{
            std::cos(std::numbers::pi_v<double> * x),
            -std::sin(std::numbers::pi_v<double> * x)};
        std::complex<double> num{1.0, 0.0}, den{1.0, 0.0};
        for (auto const& z : zf) {
            num *= (1.0 - z * q);
        }
        for (auto const& p : pf) {
            den *= (1.0 - p * q);
        }
        double const m = std::abs(num / den);
        mags[i] = std::isfinite(m) ? static_cast<float>(m) : 0.0f;
    }
    return mags;
}

void PzResponseLayer::paint(juce::Graphics& g) {
    auto const b = getLocalBounds().toFloat();

    // 曲线（峰值归一 dB，左右两端延伸到底部的填充）
    if (db_.empty()) {
        return;
    }
    juce::Path curve;
    for (int i = 0; i < kGrid; ++i) {
        auto const px = b.getX() + b.getWidth() * static_cast<float>(i) / (kGrid - 1);
        auto const norm = (db_[i] - kDbFloor) / (kDbCeil - kDbFloor);
        auto const py = b.getBottom() - b.getHeight() * std::clamp(norm, 0.0f, 1.0f);
        if (i == 0) {
            curve.startNewSubPath(px, py);
        }
        else {
            curve.lineTo(px, py);
        }
    }
    curve.lineTo(b.getRight(), b.getBottom());
    curve.lineTo(b.getX(), b.getBottom());
    curve.closeSubPath();
    g.setColour(ui::line_fore.withAlpha(0.10f));
    g.fillPath(curve);

    g.setColour(ui::line_fore);
    g.strokePath(curve, juce::PathStrokeType(1.5f));
}
