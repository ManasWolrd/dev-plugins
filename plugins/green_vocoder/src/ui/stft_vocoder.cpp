#include "stft_vocoder.hpp"
#include "PluginProcessor.h"

namespace green_vocoder::ui {

// 顶部控件区高度（左侧两个下拉框 1 列 2 行 + 右侧 dial 行）
constexpr int kTopHeight = 65;

STFTVocoder::STFTVocoder(AudioPluginAudioProcessor& processor)
    : processor_(processor) {
    bandwidth_.BindParam(processor.params_.stft_bandwidth.ptr_);
    addAndMakeVisible(bandwidth_);

    attack_.BindParam(processor.params_.stft_attack.ptr_);
    addAndMakeVisible(attack_);

    release_.BindParam(processor.params_.stft_release.ptr_);
    addAndMakeVisible(release_);

    blend_.BindParam(processor.params_.stft_blend.ptr_);
    addAndMakeVisible(blend_);

    size_.BindParam(processor.params_.stft_size.ptr_);
    addAndMakeVisible(size_);

    detail_.BindParam(processor.params_.stft_detail.ptr_);
    addAndMakeVisible(detail_);

    smooth_type_.BindParam(processor.params_.stft_smooth_erb.ptr_);
    addAndMakeVisible(smooth_type_);

    smooth_.BindParam(processor.params_.stft_smooth.ptr_);
    addAndMakeVisible(smooth_);

    mfcc_size_.BindParam(processor.params_.mfcc_nbands.ptr_);
    addAndMakeVisible(mfcc_size_);

    mode_.BindParam(processor.params_.stft_type.ptr_);
    mode_.onChange = [this] { OnModeChanged(); };
    addAndMakeVisible(mode_);

    OnModeChanged();
}

void STFTVocoder::resized() {
    using enum green_vocoder::dsp::STFTMode;
    auto mode = static_cast<green_vocoder::dsp::STFTMode>(mode_.getSelectedItemIndex());

    auto b = getLocalBounds();
    auto top = b.removeFromTop(kTopHeight);

    // 左侧 1 列 2 行：块大小 / 算法两个下拉框
    auto left = top.removeFromLeft(100);
    size_.setBounds(left.removeFromTop(30).withSizeKeepingCentre(100, 30));
    mode_.setBounds(left.withHeight(30).withSizeKeepingCentre(100, 30));

    // 右侧：逐一放置可见控件（隐藏控件不占空间）
    // dial 统一 50×65；switch 按内容排版
    auto place = [&top](juce::Component& c, int width, int height) {
        if (c.isVisible())
            c.setBounds(top.removeFromLeft(width).withSizeKeepingCentre(width, height));
    };
    auto place_dial = [&place](juce::Component& c) { place(c, 50, 65); };
    auto place_switch = [&place](juce::Component& c) { place(c, 70, 30); };

    place_dial(attack_);
    place_dial(release_);
    if (mode == SincSmear) {
        place_dial(bandwidth_);
        place_dial(blend_);
    }
    else if (mode == Cepstrum) {
        place_dial(detail_);
        place_dial(blend_);
    }
    else if (mode == MelAvg) {
        place_dial(mfcc_size_);
        place_dial(blend_);
    }
    else if (mode == Smooth) {
        place_switch(smooth_type_);
        place_dial(smooth_);
        place_dial(blend_);
    }
}

// -------------------- private --------------------

void STFTVocoder::OnModeChanged() {
    using enum green_vocoder::dsp::STFTMode;
    auto mode = static_cast<green_vocoder::dsp::STFTMode>(mode_.getSelectedItemIndex());
    bandwidth_.setVisible(mode == SincSmear);
    detail_.setVisible(mode == Cepstrum);
    smooth_type_.setVisible(mode == Smooth);
    smooth_.setVisible(mode == Smooth);
    mfcc_size_.setVisible(mode == MelAvg);
    if (!getBounds().isEmpty()) {
        resized();
    }
}

} // namespace green_vocoder::ui
