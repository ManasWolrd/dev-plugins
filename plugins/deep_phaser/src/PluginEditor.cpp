#include "PluginEditor.h"
#include "PluginProcessor.h"

#include <algorithm>
#include <cmath>
#include <numbers>

// ---------------------------------------- pzmap view ----------------------------------------

PzmapView::PzmapView(DeepPhaserAudioProcessor& p)
    : p_(p) {
    addAndMakeVisible(title_);
    addAndMakeVisible(response_layer_);
    addAndMakeVisible(edit_layer_);

    edit_layer_.on_commit_ = [this](size_t idx, bool pole, global::PolarPoint z) { Commit(idx, pole, z); };
    edit_layer_.on_reset_ = [this](size_t idx, bool pole) { ResetToDefault(idx, pole); };
    ReloadFromProcessor();
}

bool PzmapView::PoleSeparated(size_t idx, global::PolarPoint z) const {
    if (global::IsPoleDisabled(z)) {
        return true; // 非法极点会被整节禁用、不参与分解，无需间距校验
    }
    auto const zc = global::ToComplex(z);
    for (size_t i = 0; i < global::kMaxSections; ++i) {
        if (i == idx) {
            continue;
        }
        global::PolarPoint other;
        {
            juce::SpinLock::ScopedLockType lock(p_.params_.poles_zeros_lock_);
            other = p_.params_.poles_[i];
        }
        if (global::IsPoleDisabled(other)) {
            continue; // 已禁用的极点不参与分解，不参与间距校验
        }
        if (std::abs(zc - global::ToComplex(other)) < kMinSep) {
            return false;
        }
    }
    return true;
}

void PzmapView::Commit(size_t idx, bool pole, global::PolarPoint z) {
    if (pole && !PoleSeparated(idx, z)) {
        return; // 间距不足：拒绝本次提交，保持原布局
    }
    if (pole) {
        p_.params_.SetPole(idx, z);
    }
    else {
        p_.params_.SetZero(idx, z);
    }
    ReloadFromProcessor();
}

void PzmapView::ResetToDefault(size_t idx, bool pole) {
    // 默认布局：上半平面均匀分布（与 Params::InitDefaultPZ 一致）
    float const denom = static_cast<float>(global::kMaxSections + 1);
    global::PolarPoint const z{static_cast<float>(idx + 1) / denom, pole ? 0.8f : 1.0f};
    Commit(idx, pole, z);
}

void PzmapView::ReloadFromProcessor() {
    {
        juce::SpinLock::ScopedLockType lock(p_.params_.poles_zeros_lock_);
        std::copy_n(p_.params_.poles_.begin(), global::kMaxSections, cached_poles_.begin());
        std::copy_n(p_.params_.zeros_.begin(), global::kMaxSections, cached_zeros_.begin());
    }
    response_layer_.SetData(cached_poles_, cached_zeros_);
    edit_layer_.SetData(cached_poles_, cached_zeros_);
    repaint();
}

void PzmapView::SetNumSections(int /*n*/) {
    // 始终展示全部可编辑零极点，与 state(= 重复级数 N) 解耦
    repaint();
}

void PzmapView::paint(juce::Graphics& g) {
    g.fillAll(ui::green_bg);

    auto b = getLocalBounds();
    b.reduce(4, 4);
    g.setColour(ui::black_bg);
    g.fillRect(b);

    auto plot = b;
    plot.removeFromLeft(kInsetLeft);
    plot.removeFromRight(kInsetRight);
    plot.removeFromTop(kInsetTop);
    plot.removeFromBottom(kInsetBottom);
    auto const pf = plot.toFloat();

    // 网格：0dB（峰值参考线）与 -30dB
    auto const grid_y = [&](float db) {
        float const norm = (db - PzResponseLayer::kDbFloor) / (PzResponseLayer::kDbCeil - PzResponseLayer::kDbFloor);
        return pf.getBottom() - pf.getHeight() * std::clamp(norm, 0.0f, 1.0f);
    };
    g.setColour(ui::line_fore.withAlpha(0.12f));
    g.drawLine(pf.getX(), grid_y(-30.0f), pf.getRight(), grid_y(-30.0f), 1.0f);
    g.setColour(ui::line_fore.withAlpha(0.30f));
    g.drawLine(pf.getX(), grid_y(0.0f), pf.getRight(), grid_y(0.0f), 1.0f);

    // 三组轴刻度：左 dB、右模长、底部 x（归一化角频率）
    g.setFont(juce::Font{juce::FontOptions{10.0f}});
    g.setColour(ui::inactive_bg);

    auto const ticks_left = [&](float db) {
        float const y = grid_y(db);
        g.drawLine(static_cast<float>(plot.getX()) - 4.0f, y, static_cast<float>(plot.getX()), y, 1.0f);
        juce::String const label = juce::String::formatted("%d", static_cast<int>(db));
        g.drawFittedText(label, {b.getX() + 2, static_cast<int>(y) - 6, plot.getX() - b.getX() - 8, 12},
                         juce::Justification::centredRight, 1);
    };
    ticks_left(PzResponseLayer::kDbFloor);
    ticks_left(-30.0f);
    ticks_left(0.0f);

    auto const ticks_right = [&](float r) {
        float const y = pf.getBottom() - pf.getHeight() * (r / PzEditLayer::kRMax);
        g.drawLine(static_cast<float>(plot.getRight()), y, static_cast<float>(plot.getRight()) + 4.0f, y, 1.0f);
        juce::String const label = juce::String::formatted("%g", r);
        g.drawFittedText(label, {plot.getRight() + 6, static_cast<int>(y) - 6, b.getRight() - plot.getRight() - 10, 12},
                         juce::Justification::centredLeft, 1);
    };
    // 模长刻度：0 / 0.5 / 1（kRMax = 1，故 1.0 即顶端，不再重复标一次）
    ticks_right(0.0f);
    ticks_right(0.5f);
    ticks_right(PzEditLayer::kRMax);

    auto const ticks_bottom = [&](float x) {
        float const px =
            pf.getX() + pf.getWidth() * (x - PzEditLayer::kXMin) / (PzEditLayer::kXMax - PzEditLayer::kXMin);
        g.drawLine(px, pf.getBottom(), px, pf.getBottom() + 4.0f, 1.0f);
        juce::String const label = juce::String::formatted("%.2g", x);
        g.drawFittedText(label, {static_cast<int>(px) - 20, plot.getBottom() + 2, 40, 12}, juce::Justification::centred,
                         1);
    };
    ticks_bottom(PzEditLayer::kXMin);
    ticks_bottom(0.25f);
    ticks_bottom(0.5f);
    ticks_bottom(0.75f);
    ticks_bottom(PzEditLayer::kXMax);

    // x 轴名称（左下角）
    g.setColour(ui::inactive_bg);
    g.drawFittedText("rad", {b.getX() + 2, plot.getBottom() + 2, 30, 12}, juce::Justification::centredLeft, 1);
}

void PzmapView::resized() {
    auto b = getLocalBounds();
    auto const title_b = b.removeFromTop(static_cast<int>(title_.getFont().getHeight() * 1.2f));
    title_.setBounds(title_b);
    b.reduce(4, 4);

    // 两层共用同一 plotting bound（共享 bound 要求）
    auto plot = b;
    plot.removeFromLeft(kInsetLeft);
    plot.removeFromRight(kInsetRight);
    plot.removeFromTop(kInsetTop);
    plot.removeFromBottom(kInsetBottom);
    response_layer_.setBounds(plot);
    edit_layer_.setBounds(plot);
}

// ---------------------------------------- editor ----------------------------------------

DeepPhaserAudioProcessorEditor::DeepPhaserAudioProcessorEditor(DeepPhaserAudioProcessor& p)
    : AudioProcessorEditor(&p)
    , p_(p)
    , preset_panel_(*p.preset_manager_)
    , pzmap_(p) {
    addAndMakeVisible(preset_panel_);

    addAndMakeVisible(allpass_title_);
    addAndMakeVisible(panic_);
    panic_.setButtonText("panic");
    panic_.onClick = [&p] { p.reset(); };

    state_.BindParam(p.params_.state.ptr_);
    addAndMakeVisible(state_);
    drywet_.BindParam(p.params_.drywet.ptr_);
    addAndMakeVisible(drywet_);

    addAndMakeVisible(blend_title_);
    blend_.BindParam(p.params_.blend.ptr_);
    addAndMakeVisible(blend_);

    addAndMakeVisible(barber_title_);
    barber_enable_.BindParam(p.params_.barber_enable.ptr_);
    addAndMakeVisible(barber_enable_);
    barber_phase_.BindParam(p.params_.barber_phase.ptr_);
    addAndMakeVisible(barber_phase_);
    barber_speed_.BindParam(p.params_.barber_lfo);
    addAndMakeVisible(barber_speed_);
    barber_stereo_.BindParam(p.params_.barber_stereo.ptr_);
    addAndMakeVisible(barber_stereo_);
    barber_reset_phase_.setButtonText("reset phase");
    barber_reset_phase_.onClick = [this] {
        juce::ScopedLock _{p_.getCallbackLock()};
        p_.params_.barber_lfo.TryResetPhase(0.0f);
    };
    addAndMakeVisible(barber_reset_phase_);

    pzmap_.SetNumSections(static_cast<int>(global::kMaxSections));
    addAndMakeVisible(pzmap_);

    // 控制行(110px) + pzmap 约 400px 见方，避免 pzmap 被挤成一条裂缝
    setSize(780, 540);

    startTimerHz(30);
}

DeepPhaserAudioProcessorEditor::~DeepPhaserAudioProcessorEditor() {}

//==============================================================================
void DeepPhaserAudioProcessorEditor::paint(juce::Graphics& g) {
    g.fillAll(ui::black_bg);

    g.setColour(ui::green_bg);
    auto b = getLocalBounds();
    g.fillRect(b.removeFromTop(30));
    b.removeFromTop(2);
    g.fillRect(basic_bound_);
    g.fillRect(blend_bound_);
    g.fillRect(barber_bound_);
}

void DeepPhaserAudioProcessorEditor::resized() {
    auto b = getLocalBounds();
    preset_panel_.setBounds(b.removeFromTop(30));
    b.removeFromTop(2);

    // 单一控制行：basic + blend + barberpole，均布全宽
    {
        auto topblock = b.removeFromTop(110);
        // --- basic 区 ---
        auto basic_block = topblock.removeFromLeft(190);
        basic_bound_ = basic_block;
        {
            auto blk = basic_block;
            auto blk_title = blk.removeFromTop(25);
            allpass_title_.setBounds(blk_title);
            state_.setBounds(blk.removeFromLeft(70));
            drywet_.setBounds(blk.removeFromLeft(60));
            panic_.setBounds(blk);
        }
        topblock.removeFromLeft(8);

        // --- blend 区（单一参数） ---
        blend_bound_ = topblock.removeFromLeft(190);
        {
            auto blk = blend_bound_;
            auto blk_title = blk.removeFromTop(25);
            blend_title_.setBounds(blk_title);
            blend_.setBounds(blk.removeFromLeft(70));
        }
        topblock.removeFromLeft(8);

        // --- barberpole 区 ---
        barber_bound_ = topblock;
        {
            auto blk = barber_bound_;
            auto blk_title = blk.removeFromTop(25);
            barber_reset_phase_.setBounds(blk_title.removeFromRight(90).reduced(2));
            barber_enable_.setBounds(blk_title.removeFromRight(70).reduced(2));
            barber_title_.setBounds(blk_title);
            barber_phase_.setBounds(blk.removeFromLeft(70));
            barber_speed_.setBounds(blk.removeFromLeft(70));
            barber_stereo_.setBounds(blk.removeFromLeft(70));
        }
    }
    b.removeFromTop(8);

    // 剩余区域：pzmap
    pzmap_.setBounds(b);
}

void DeepPhaserAudioProcessorEditor::UpdateGui() {
    pzmap_.ReloadFromProcessor();
    pzmap_.SetNumSections(static_cast<int>(global::kMaxSections));
}

void DeepPhaserAudioProcessorEditor::timerCallback() {
    if (p_.engine_.ExchangeNewCoeff()) {
        UpdateGui();
    }
}
