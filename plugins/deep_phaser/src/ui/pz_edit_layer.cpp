#include "pz_edit_layer.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "pluginshared/color_preset.hpp"

// ------------------------------------------------------------
// PzEditLayer
// ------------------------------------------------------------

PzEditLayer::PzEditLayer() = default;

void PzEditLayer::SetData(
    std::array<global::PolarPoint, global::kMaxSections> const& poles,
    std::array<global::PolarPoint, global::kMaxSections> const& zeros) {
    poles_ = poles;
    zeros_ = zeros;
    repaint();
}

juce::Point<float> PzEditLayer::ToScreen(float x, float r) const {
    auto const b = getLocalBounds().toFloat();
    auto const nx = (x - kXMin) / (kXMax - kXMin);
    auto const ny = r / kRMax;
    return {b.getX() + b.getWidth() * nx, b.getBottom() - b.getHeight() * ny};
}

void PzEditLayer::FromScreen(juce::Point<float> pos, float& x_out, float& r_out) const {
    auto const b = getLocalBounds().toFloat();
    auto const nx = std::clamp((pos.x - b.getX()) / b.getWidth(), 0.0f, 1.0f);
    auto const ny = std::clamp((b.getBottom() - pos.y) / b.getHeight(), 0.0f, 1.0f);
    x_out = kXMin + (kXMax - kXMin) * nx;
    r_out = kRMax * ny;
}

bool PzEditLayer::HitTest(juce::Point<int> pos, bool pole, int& out_idx) const {
    float const radius_sq = 64.0f;  // 8px 命中半径^2
    float best = radius_sq;
    bool hit = false;
    for (size_t i = 0; i < global::kMaxSections; ++i) {
        auto const& z = pole ? poles_[i] : zeros_[i];
        auto const sp = ToScreen(z.x, z.r);
        float const dx = pos.x - sp.x;
        float const dy = pos.y - sp.y;
        float const d2 = dx * dx + dy * dy;
        if (d2 < best) {
            best = d2;
            out_idx = static_cast<int>(i);
            hit = true;
        }
    }
    return hit;
}

void PzEditLayer::ClampAndCommit(size_t idx, bool pole, float x, float r) {
    // 只做显示范围钳制；不再强制虚部地板，允许把点拖入非法区。
    // 落入非法区的极点由引擎整节禁用（见 global::IsPoleDisabled），不在此处拦截。
    x = std::clamp(x, kXMin, kXMax);
    float const r_max = pole ? kPoleRMax : kZeroRMax;
    r = std::clamp(r, kRMin, r_max);
    if (on_commit_) {
        on_commit_(idx, pole, global::PolarPoint{x, r});
    }
}

void PzEditLayer::mouseDown(juce::MouseEvent const& e) {
    int idx = -1;
    if (e.mods.isLeftButtonDown()) {
        if (HitTest(e.getPosition(), true, idx)) {
            dragging_pole_idx_ = idx;
            float x, r;
            FromScreen(e.getPosition().toFloat(), x, r);
            ClampAndCommit(static_cast<size_t>(idx), true, x, r);
        }
        else if (HitTest(e.getPosition(), false, idx)) {
            dragging_zero_idx_ = idx;
            float x, r;
            FromScreen(e.getPosition().toFloat(), x, r);
            ClampAndCommit(static_cast<size_t>(idx), false, x, r);
        }
    }
    else if (e.mods.isRightButtonDown()) {
        // 右键：命中最近点 → 请求容器复位默认布局
        if (HitTest(e.getPosition(), true, idx)) {
            if (on_reset_) {
                on_reset_(static_cast<size_t>(idx), true);
            }
        }
        else if (HitTest(e.getPosition(), false, idx)) {
            if (on_reset_) {
                on_reset_(static_cast<size_t>(idx), false);
            }
        }
    }
}

void PzEditLayer::mouseDrag(juce::MouseEvent const& e) {
    if (dragging_pole_idx_ >= 0 && e.mods.isLeftButtonDown()) {
        float x, r;
        FromScreen(e.getPosition().toFloat(), x, r);
        ClampAndCommit(static_cast<size_t>(dragging_pole_idx_), true, x, r);
    }
    else if (dragging_zero_idx_ >= 0 && e.mods.isLeftButtonDown()) {
        float x, r;
        FromScreen(e.getPosition().toFloat(), x, r);
        ClampAndCommit(static_cast<size_t>(dragging_zero_idx_), false, x, r);
    }
}

void PzEditLayer::mouseUp(juce::MouseEvent const& e) {
    std::ignore = e;
    dragging_pole_idx_ = -1;
    dragging_zero_idx_ = -1;
}

void PzEditLayer::paint(juce::Graphics& g) {
    auto const b = getLocalBounds().toFloat();

    // 非法区：Im(z) = r·sin(πx) < global::kMinPoleImag 的点。
    // 落入其中的极点会被引擎禁用，这里先以警告色作背景标出。
    {
        constexpr int kSeg = 96;
        juce::Path boundary;   // 边界曲线（用于描边）
        juce::Path region;     // 边界 + 底部围成的区域（用于填充）
        for (int i = 0; i <= kSeg; ++i) {
            float const x = kXMin + (kXMax - kXMin) * static_cast<float>(i) / static_cast<float>(kSeg);
            float const s = std::sin(std::numbers::pi_v<float> * x);
            float r_b = (s > 1.0e-6f) ? (global::kMinPoleImag / s) : kRMax;
            r_b = std::min(r_b, kRMax);
            auto const p = ToScreen(x, r_b);
            if (i == 0) {
                boundary.startNewSubPath(p);
                region.startNewSubPath(p);
            }
            else {
                boundary.lineTo(p);
                region.lineTo(p);
            }
        }
        region.lineTo(ToScreen(kXMax, 0.0f));
        region.lineTo(ToScreen(kXMin, 0.0f));
        region.closeSubPath();
        g.setColour(ui::warning_fore.withAlpha(0.13f));
        g.fillPath(region);
        g.setColour(ui::warning_fore.withAlpha(0.40f));
        g.strokePath(boundary, juce::PathStrokeType(1.0f));
    }

    // 单位圆参考线（|z| = 1 水平虚线，位于 y 轴顶端）
    auto const line_y = ToScreen(0.0f, 1.0f).getY();
    float const dashes[] = {6.0f, 4.0f};
    g.setColour(ui::white_fore.withAlpha(0.35f));
    g.drawDashedLine(juce::Line<float>{b.getX(), line_y, b.getRight(), line_y},
                     dashes, 2, 0.0f, 1.0f);

    // 节编号：每个 section 的极点与零点共用同一编号（1..kMaxSections），
    // 便于在 pzmap 上把成对的极点/零点对应起来（类似 EQ8 的 band 编号）。
    auto const draw_number = [&](juce::Point<float> c, juce::Colour col, size_t idx) {
        g.setColour(col);
        g.setFont(juce::Font{juce::FontOptions{kMarkerFontH}});
        auto const box = juce::Rectangle<float>(c.x - kMarkerR, c.y - kMarkerR,
                                                kMarkerR * 2.0f, kMarkerR * 2.0f);
        g.drawText(juce::String(static_cast<int>(idx) + 1), box, juce::Justification::centred, false);
    };

    // 零点（空圈 + 编号）；同索引极点被禁用时，该零点也随之退出分解 → 变暗
    for (size_t i = 0; i < global::kMaxSections; ++i) {
        auto const p = ToScreen(zeros_[i].x, zeros_[i].r);
        bool const section_disabled = global::IsPoleDisabled(poles_[i]);
        auto const col = ui::white_fore.withAlpha(section_disabled ? 0.25f : 1.0f);
        g.setColour(col);
        g.drawEllipse(p.x - kMarkerR, p.y - kMarkerR, kMarkerR * 2.0f, kMarkerR * 2.0f, 1.5f);
        draw_number(p, col, i);
    }
    // 极点（实心 + 编号）；落入非法区 = 禁用：警告色填充/描边
    for (size_t i = 0; i < global::kMaxSections; ++i) {
        auto const p = ToScreen(poles_[i].x, poles_[i].r);
        if (global::IsPoleDisabled(poles_[i])) {
            g.setColour(ui::warning_fore.withAlpha(0.28f));
            g.fillEllipse(p.x - kMarkerR, p.y - kMarkerR, kMarkerR * 2.0f, kMarkerR * 2.0f);
            g.setColour(ui::warning_fore);
            g.drawEllipse(p.x - kMarkerR, p.y - kMarkerR, kMarkerR * 2.0f, kMarkerR * 2.0f, 2.0f);
            draw_number(p, ui::white_fore, i);
        }
        else {
            g.setColour(ui::active_bg);
            g.fillEllipse(p.x - kMarkerR, p.y - kMarkerR, kMarkerR * 2.0f, kMarkerR * 2.0f);
            draw_number(p, ui::black_bg, i);
        }
    }
}
