#pragma once
#include <array>
#include <atomic>
#include <complex>
#include <numbers>
#include <span>

#include "global.hpp"
#include "pluginshared/bpm_sync_lfo.hpp"
#include "pluginshared/wrap_parameters.hpp"

// ------------------------------------------------------------
// 参数：抽象为 pluginshared 包装 + atomic bool 驱动的系数重建回调
// 采用 steep_flanger 的 atomic bool 模式：UI/参数改动只写原子标志，
// 音频线程在 Update 中消费标志并重建 IIR 系数。
// ------------------------------------------------------------
namespace deepphaser {

class Params : public juce::AudioProcessorParameter::Listener {
public:
    pluginshared::IntParam state{"state", 1, static_cast<int>(global::kMaxStages), 4};
    pluginshared::FloatParam blend{
        "blend", {0.0f, 1.0f, 0.0001f},
         0.5f
    };
    pluginshared::FloatParam drywet{
        "drywet", {0.0f, 1.0f, 0.001f},
         1.0f
    };

    // -------------------- barberpole --------------------
    pluginshared::FloatParam barber_phase{
        "barber_phase", {0.0f, 1.0f, 0.01f},
         0.0f
    };
    pluginshared::FloatParam barber_stereo{
        "barber_stereo", {0.0f, 1.0f, 0.01f},
         0.0f
    };
    pluginshared::BoolParam barber_enable{"barber_enable", true};

    // -------------------- bpm sync lfo --------------------
    pluginshared::BpmSyncLFO barber_lfo{"barber_speed", -10.0f,  10.0f, 0.01f, 0.4f, true,
                                        "-1/64T",       "1/64T", 0.2f,  "1",   false};

    // -------------------- IIR 系数重建控制（atomic bool 驱动） --------------------
    std::atomic<bool> should_update_iir_{};

    // 用户拖拽的零极点（UI 线程写入，音频线程在 RebuildCoeffs 中读取）
    // 只存上半平面 (imag>0)，共轭由设计自动处理；active count = state
    juce::SpinLock poles_zeros_lock_;
    std::array<global::PolarPoint, global::kMaxSections> poles_{};
    std::array<global::PolarPoint, global::kMaxSections> zeros_{};

    // 状态恢复暂存（setStateInformation 写入，音频线程消费后清除）
    std::atomic<bool> has_pz_from_state_{};
    std::array<global::PolarPoint, global::kMaxSections> poles_from_state_{};
    std::array<global::PolarPoint, global::kMaxSections> zeros_from_state_{};

    // 默认零极点布局：上半平面内 (0, π) 均匀分布，避免落在实轴上（实轴极点的
    // 共轭是其自身，会与展开集合中的副本重复，导致部分分式校验失败）
    static void InitDefaultPZ(std::span<global::PolarPoint, global::kMaxSections> poles,
                              std::span<global::PolarPoint, global::kMaxSections> zeros) noexcept {
        float const denom = static_cast<float>(global::kMaxSections + 1);
        for (size_t i = 0; i < global::kMaxSections; ++i) {
            float const x = static_cast<float>(i + 1) / denom; // 归一化极角 ω/π
            poles[i] = {x, 0.8f};
            zeros[i] = {x, 1.0f}; // 零点在单位圆上（镜像 → 全通感）
        }
    }

    void BuildLayout(juce::AudioProcessorValueTreeState::ParameterLayout& layout) {
        barber_lfo += layout;
        layout += state;
        layout += blend;
        layout += drywet;
        layout += barber_phase;
        layout += barber_stereo;
        layout += barber_enable;
    }

    void BeginListening() {
        state.ptr_->addListener(this);
        blend.ptr_->addListener(this);
        drywet.ptr_->addListener(this);
        barber_phase.ptr_->addListener(this);
        barber_stereo.ptr_->addListener(this);
        barber_enable.ptr_->addListener(this);

        state_idx_ = state.ptr_->getParameterIndex();
    }

    void EndListening() {
        state.ptr_->removeListener(this);
        blend.ptr_->removeListener(this);
        drywet.ptr_->removeListener(this);
        barber_phase.ptr_->removeListener(this);
        barber_stereo.ptr_->removeListener(this);
        barber_enable.ptr_->removeListener(this);
    }

    // 供 UI 调用：写入零极点（极坐标）并触发音频线程重建系数
    void SetPole(size_t idx, global::PolarPoint v) {
        juce::SpinLock::ScopedLockType lock(poles_zeros_lock_);
        poles_[idx] = v;
        should_update_iir_.store(true, std::memory_order_release);
    }

    void SetZero(size_t idx, global::PolarPoint v) {
        juce::SpinLock::ScopedLockType lock(poles_zeros_lock_);
        zeros_[idx] = v;
        should_update_iir_.store(true, std::memory_order_release);
    }
private:
    void parameterValueChanged(int parameterIndex, float newValue) override {
        juce::ignoreUnused(newValue);
        if (parameterIndex == state_idx_) {
            should_update_iir_.store(true, std::memory_order_release);
        }
    }

    void parameterGestureChanged(int parameterIndex, bool gestureIsStarting) override {
        juce::ignoreUnused(parameterIndex, gestureIsStarting);
    }

    int state_idx_{-1};
};

} // namespace deepphaser
