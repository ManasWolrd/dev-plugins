#pragma once
#include <complex>
#include <vector>

#include "pluginshared/bpm_sync_ui.hpp"
#include "pluginshared/component.hpp"
#include "pluginshared/preset_panel.hpp"

#include "global.hpp"
#include "params.hpp"
#include "ui/pz_edit_layer.hpp"
#include "ui/pz_response_layer.hpp"

class DeepPhaserAudioProcessor;

// ---------------------------------------- pzmap edit view ----------------------------------------
// 双层编辑器容器：背景层 = 零极点原始幅度响应曲线（PzResponseLayer），
// 交互层 = 零极点拖拽（PzEditLayer，置顶）。两层用同一绘图 bound，
// x 轴均为归一化角频率 ω/π ∈ [PzEditLayer::kXMin, kXMax]；
// 边距内画三组轴刻度：左侧 dB、右侧模长、底部 x。
class PzmapView : public juce::Component {
public:
    explicit PzmapView(DeepPhaserAudioProcessor& p);

    void paint(juce::Graphics& g) override;
    void resized() override;

    void ReloadFromProcessor();
    void SetNumSections(int n);
private:
    // 极点间距校验（其它已放置极点），供提交回调使用
    bool PoleSeparated(size_t idx, global::PolarPoint z) const;
    // 提交 / 右键复位共用路径（含间距校验后写入 params）
    void Commit(size_t idx, bool pole, global::PolarPoint z);
    void ResetToDefault(size_t idx, bool pole);

    static constexpr float kMinSep = 0.04f; // 极点最小间距（复平面距离）
    static constexpr int kInsetLeft = 30;   // 左轴 dB 刻度边距
    static constexpr int kInsetRight = 30;  // 右轴模长刻度边距
    static constexpr int kInsetTop = 8;
    static constexpr int kInsetBottom = 16; // 底部 x 刻度边距

    DeepPhaserAudioProcessor& p_;
    juce::Label title_{"", "pzmap"};
    PzResponseLayer response_layer_;
    PzEditLayer edit_layer_;
    std::array<global::PolarPoint, global::kMaxSections> cached_poles_{};
    std::array<global::PolarPoint, global::kMaxSections> cached_zeros_{};
};

// ---------------------------------------- editor ----------------------------------------

//==============================================================================
class DeepPhaserAudioProcessorEditor final
    : public juce::AudioProcessorEditor
    , public juce::Timer {
public:
    explicit DeepPhaserAudioProcessorEditor(DeepPhaserAudioProcessor&);
    ~DeepPhaserAudioProcessorEditor() override;

    //==============================================================================
    void paint(juce::Graphics&) override;
    void resized() override;

    void UpdateGui();
    void timerCallback() override;
private:
    DeepPhaserAudioProcessor& p_;
    pluginshared::PresetPanel preset_panel_;

    // control section
    juce::Rectangle<int> basic_bound_;
    juce::Label allpass_title_{"basic", "basic"};
    ui::Dial state_{"state"};
    ui::Dial drywet_{"drywet"};
    ui::FlatButton panic_;

    // blend（单一中心参数，无 LFO 调制）
    juce::Rectangle<int> blend_bound_;
    juce::Label blend_title_{"", "blend"};
    ui::Dial blend_{"blend"};

    // barberpole
    juce::Rectangle<int> barber_bound_;
    juce::Label barber_title_{"barberpole", "barberpole"};
    ui::Switch barber_enable_{"enable"};
    ui::Dial barber_phase_{"phase"};
    ui::BpmSyncDial barber_speed_{"speed"};
    ui::Dial barber_stereo_{"stereo"};
    ui::FlatButton barber_reset_phase_;

    // pzmap
    PzmapView pzmap_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DeepPhaserAudioProcessorEditor)
};
