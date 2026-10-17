#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <numbers>
#include <vector>

#include "global.hpp"
#include "iir_design.hpp"
#include "iir_filter_cheap.hpp"
#include "params.hpp"
#include "pluginshared/dsp/stereo_iir_hilbert_cpx.hpp"
#include "pluginshared/simd/simd.hpp"
#include "qwqdsp/misc/smoother.hpp"

// ------------------------------------------------------------
// DeepPhaser2 音频引擎（标量实现）
// 核心：
//   H(z) 由 pzmap 拖拽零极点经部分分式分解为并行一阶节 R_k/(1 - p_k q)。
//   以「级联一阶全通」A(z)^N 扭曲每个并行节的变量 q=A(z)^N，得到
//   H(A(z)^N) —— H(z) 的响应被 A(z)^N 重复/拉伸，N 为重复次数(state)。
//   每节 = WarpedSectionCheap（见 iir_filter_cheap.hpp），把延迟块替换成
//   "延迟 + 全通级联"，消除零延迟反馈回路，无需代数求解。
//   输出求和 Σ y_k，再叠加前馈项，最后做 dry/wet、反馈、barberpole 旋转。
// ------------------------------------------------------------
namespace deepphaser {

class Engine {
public:
    struct Snapshot {
        float fs{};
        float drywet{1.0f};
        float barber_phase{};
        float barber_stereo_phase{};
        bool barber_enable{false};
        int state{4};
        float blend{0.5f};
        float barber_speed{};
    };

    void Init(float fs) {
        fs_ = fs;
        barber_phase_smoother_.SetSmoothTime(20.0f, fs);
        sections_.clear();
        sections_.reserve(global::kMaxSections * 2);
        for (size_t i = 0; i < global::kMaxSections * 2; ++i) {
            sections_.emplace_back();
        }
        num_sections_ = 0;
        Reset();
    }

    void Reset() noexcept {
        barber_phase_ = 0;
        hilbert_complex_.Reset();
        for (auto& s : sections_) {
            s.Reset();
        }
    }

    void Update(Params& p) {
        if (p.has_pz_from_state_.exchange(false, std::memory_order_acq_rel)) {
            auto lock = juce::SpinLock::ScopedLockType(p.poles_zeros_lock_);
            p.poles_ = p.poles_from_state_;
            p.zeros_ = p.zeros_from_state_;
        }

        size_t num_stages = static_cast<size_t>(p.state.Get());
        num_stages = std::clamp(num_stages, size_t{1}, global::kMaxStages);

        if (p.should_update_iir_.exchange(false, std::memory_order_acq_rel) || num_stages != rebuild_stages_) {
            RebuildCoeffs(p, num_stages);
        }
    }

    void RebuildCoeffs(Params& p, size_t num_stages) {
        PoleZeroSet pz;
        unsigned disabled_mask = 0;
        {
            auto lock = juce::SpinLock::ScopedLockType(p.poles_zeros_lock_);
            for (size_t i = 0; i < global::kMaxSections; ++i) {
                // 落入非法区（太靠近实轴）的极点，连同同索引零点整节禁用、不参与分解
                if (global::IsPoleDisabled(p.poles_[i])) {
                    disabled_mask |= (1u << i);
                    continue;
                }
                pz.poles.push_back(global::ToComplex(p.poles_[i]));
                pz.zeros.push_back(global::ToComplex(p.zeros_[i]));
            }
        }

        // 启用集合变化会改变并行节与零极点的索引对应关系：清空全部节状态，
        // 避免旧状态套到新节上产生咔哒（仅在增/删禁用节时发生，不是每次拖拽）
        if (disabled_mask != disabled_mask_) {
            for (auto& s : sections_) {
                s.Reset();
            }
            disabled_mask_ = disabled_mask;
        }

        // 全部节被禁用：退化为直通（无滤波），避免保留过期系数
        if (pz.poles.empty()) {
            num_sections_ = 0;
            iir_fir_k_real_ = 1.0f;
            rebuild_stages_ = num_stages;
            have_new_coeff_.store(true, std::memory_order_release);
            return;
        }

        DesignResult design;
        ComputeResidues(pz, design);
        if (!design.valid) {
            return; // 无效布局（极点太近/越界/失稳）：保持上一组系数
        }

        // 展开后的节数 = 共轭对称全集合 (2 * kMaxSections)
        size_t const expanded = design.sections.size();
        if (expanded == 0 || expanded > sections_.size()) {
            return;
        }
        num_sections_ = expanded;

        // 峰值增益归一化：以 A(z)^N 的重复次数 N 为"延迟基数"，评估 |H| 峰值
        float const peak = EvaluatePeak(design, static_cast<float>(num_stages));
        float const norm = (peak > 1.0e-5f) ? (1.0f / peak) : 1.0f;

        // 前馈项
        iir_fir_k_real_ = design.fir_k.real() * norm;

        // 重复次数 = state（一阶全通级联数）
        for (size_t i = 0; i < expanded; ++i) {
            sections_[i].Set(design.sections[i].residual * norm, design.sections[i].pole);
            sections_[i].ResizeStages(num_stages);
        }
        rebuild_stages_ = num_stages;
        have_new_coeff_.store(true, std::memory_order_release);
    }

    bool ExchangeNewCoeff() noexcept {
        return have_new_coeff_.exchange(false, std::memory_order_acq_rel);
    }

    void ProcessBlock(float* left, float* right, int num_samples, Snapshot const& s) {
        size_t cando = static_cast<size_t>(num_samples);
        while (cando != 0) {
            size_t const num_process = std::min<size_t>(512, cando);
            cando -= num_process;
            ProcessSamples(left, right, num_process, s);
            left += num_process;
            right += num_process;
        }
    }
private:
    void SetUniformAllpassCoeff(float c) noexcept {
        coeff_ = c;
        for (size_t i = 0; i < num_sections_; ++i) {
            sections_[i].SetCoeff(c);
        }
    }

    void ProcessSamples(float* left, float* right, size_t num_process, Snapshot const& s) {
        float const wet_mix = s.drywet;
        float const dry_mix = 1.0f - wet_mix;

        barber_phase_smoother_.SetTarget(s.barber_phase * std::numbers::pi_v<float> * 2);
        float const barber_omega = s.barber_speed / s.fs * std::numbers::pi_v<float> * 2;

        float const inv_n = 1.0f / static_cast<float>(num_process);

        // blend 直接控制一阶全通系数：c = 2·blend - 1（blend=0 → c≈-1，0.5 → c=0，1 → c≈+1）
        float const target_coeff = std::clamp(2.0f * s.blend - 1.0f, -kBlendCoeffMax, kBlendCoeffMax);

        float curr_coeff = last_coeff_;
        float const delta_coeff = (target_coeff - curr_coeff) * inv_n;

        size_t const expanded = num_sections_;

        if (!s.barber_enable) {
            for (size_t j = 0; j < num_process; ++j) {
                curr_coeff += delta_coeff;

                SetUniformAllpassCoeff(curr_coeff);
                float const left_in = *left;
                float left_acc = left_in * iir_fir_k_real_;
                for (size_t i = 0; i < expanded; ++i) {
                    left_acc += sections_[i].TickLeft(left_in).real();
                }

                float const right_in = *right;
                float right_acc = right_in * iir_fir_k_real_;
                for (size_t i = 0; i < expanded; ++i) {
                    right_acc += sections_[i].TickRight(right_in).real();
                }

                *left = left_acc * wet_mix + left_in * dry_mix;
                *right = right_acc * wet_mix + right_in * dry_mix;

                ++left;
                ++right;
            }
        }
        else {
            for (size_t j = 0; j < num_process; ++j) {
                curr_coeff += delta_coeff;

                // barberpole：q 域旋转 H(q -> q*exp(-i*phi))，即把每节极点
                // p_k -> p_k*exp(-i*phi)。响应绕 q 平面旋转，单位时间穿越的 notch
                // 数与 state(=N) 无关；phi 走 2π 恰好回到原位，回卷无缝。
                float const smoothed = barber_phase_smoother_.Tick();
                barber_phase_ = std::fmod(barber_phase_ + barber_omega, std::numbers::pi_v<float> * 2);
                float const phi = barber_phase_ + smoothed * std::numbers::pi_v<float> * 2;
                auto const rot_l = std::polar(1.0f, -phi);
                auto const rot_r = std::polar(1.0f, -(phi + s.barber_stereo_phase));

                SetUniformAllpassCoeff(curr_coeff);
                for (size_t i = 0; i < expanded; ++i) {
                    sections_[i].SetPoleRotation(rot_l, rot_r);
                }

                // 解析信号（正频谱）：实输入经 Hilbert 得到 x_a = x + j*H(x)
                float const left_in = *left;
                float const right_in = *right;
                auto const analytic = hilbert_complex_.Tick(
                    simd::Float128{left_in, 0.0f, right_in, 0.0f});
                std::complex<float> const left_a{analytic[0], analytic[1]};
                std::complex<float> const right_a{analytic[2], analytic[3]};

                // 复滤波（极点已旋转）+ 前馈，取实部得到实值输出
                std::complex<float> left_sum = left_a * iir_fir_k_real_;
                std::complex<float> right_sum = right_a * iir_fir_k_real_;
                for (size_t i = 0; i < expanded; ++i) {
                    left_sum += sections_[i].TickLeftCpx(left_a);
                    right_sum += sections_[i].TickRightCpx(right_a);
                }

                *left = left_sum.real() * wet_mix + left_in * dry_mix;
                *right = right_sum.real() * wet_mix + right_in * dry_mix;

                ++left;
                ++right;
            }
        }

        last_coeff_ = curr_coeff;
    }

    // |c| 上限：端点 |c|=1 时全通退化为 A(z)=±1、内部状态不衰减（Nyquist 处振荡），禁止
    static constexpr float kBlendCoeffMax = 0.999f;

    float fs_{};
    float iir_fir_k_real_{1.0f};
    size_t num_sections_{0};
    size_t rebuild_stages_{0};
    unsigned disabled_mask_{0};   // 被禁用（非法）节的位掩码，用于检测启用集合变化
    float coeff_{0.0f};
    std::atomic<bool> have_new_coeff_{};

    std::vector<WarpedSectionCheap> sections_;

    // barberpole
    qwqdsp_misc::ExpSmoother barber_phase_smoother_;
    float barber_phase_{};

    // allpass coeff sweep（跨块线性平滑）
    float last_coeff_{0.0f};

    pluginshared::dsp::StereoIIRHilbertDeeperCpx hilbert_complex_;
};

} // namespace deepphaser
