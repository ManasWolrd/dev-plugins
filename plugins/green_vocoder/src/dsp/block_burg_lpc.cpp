#include "block_burg_lpc.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <span>

#include <qwqdsp/misc/smoother.hpp>
#include <qwqdsp/window/hann.hpp>

namespace green_vocoder::dsp {

using global::kMaxPoles;

using PackFloat2 = qwqdsp_simd_element::PackFloat<2>;

void BlockBurgLPC::Init(float fs) {
    sample_rate_ = fs;
    SetParam(Params{.block_size = 1024});
}

void BlockBurgLPC::SetParam(const Params& p) {
    // 仅当 block_size 改变时才重建内部缓冲（分配开销大）
    if (p.block_size != fft_size_) {
        fft_size_ = p.block_size;
        eb_.resize(static_cast<size_t>(p.block_size));
        ef_.resize(static_cast<size_t>(p.block_size));

        // 分析 hann 窗；window_norm_ = sqrt(Σw²)，用于把加窗后的残差能量换算回原始 RMS
        hann_window_.resize(static_cast<size_t>(p.block_size));
        qwqdsp_window::Hann::Window(hann_window_, true);
        float window_energy = 0.0f;
        for (float const w : hann_window_) window_energy += w * w;
        window_norm_ = std::sqrt(window_energy);
    }

    update_rate_ = sample_rate_ / (static_cast<float>(fft_size_) / 4.0f);

    num_poles_ = p.poles;

    // 衰减平滑自动跟随块大小：时间常数取一个分析块（fft_size 个采样），
    // 使不同块大小下的时间平滑行为一致
    float const block_ms = 1000.0f * static_cast<float>(fft_size_) / sample_rate_;
    attack_factor_ = qwqdsp_misc::ExpSmoother::ComputeSmoothFactor(block_ms, update_rate_);

    // formant_shift 为半音（st），归一化到全通系数范围（st=±12 → ±0.99）
    fir_allpass_coeff_ = std::clamp(-p.formant_shift / 12.0f, -0.99f, 0.99f);
}

std::span<PackFloat2 const> BlockBurgLPC::operator()(std::span<PackFloat2 const> main,
                                                     std::span<PackFloat2 const> side) {
    // 逐帧去均值（直流）后加 hann 分析窗；再注入微小噪声，防止反射系数
    // k = -2*up/down 在静音时病态（同 LeakyBurgLPC）
    PackFloat2 mean{};
    for (auto const& x : main) mean += x;
    mean *= 1.0f / static_cast<float>(main.size());
    for (int i = 0; i < static_cast<int>(main.size()); ++i) {
        auto x = main[static_cast<size_t>(i)];
        x -= mean;
        x *= hann_window_[static_cast<size_t>(i)];
        x += noise_.Next() * global::kNoiseGain;
        ef_[static_cast<size_t>(i)] = x;
        eb_[static_cast<size_t>(i)] = x;
    }

    // forward fir lattice
    std::array<PackFloat2, kMaxPoles> latticek{};
    for (int kidx = 0; kidx < num_poles_; ++kidx) {
        auto& k = latticek[static_cast<size_t>(kidx)];

        PackFloat2 up{};
        PackFloat2 down{};
        PackFloat2 s_iir{};
        for (int i = 0; i < static_cast<int>(ef_.size()); ++i) {
            auto y = fir_allpass_coeff_ * eb_[static_cast<size_t>(i)] + s_iir;
            s_iir = eb_[static_cast<size_t>(i)] - fir_allpass_coeff_ * y;
            eb_[static_cast<size_t>(i)] = y;
            up += ef_[static_cast<size_t>(i)] * y;
            down += ef_[static_cast<size_t>(i)] * ef_[static_cast<size_t>(i)];
            down += y * y;
        }
        k = -2.0f * up / down;

        for (int i = 0; i < static_cast<int>(ef_.size()); ++i) {
            auto upgo = ef_[static_cast<size_t>(i)] + eb_[static_cast<size_t>(i)] * k;
            auto downgo = eb_[static_cast<size_t>(i)] + ef_[static_cast<size_t>(i)] * k;
            ef_[static_cast<size_t>(i)] = upgo;
            eb_[static_cast<size_t>(i)] = downgo;
        }
    }
    // the FIR and IIR lattice coeffient are reversed
    std::reverse_copy(latticek.begin(), latticek.begin() + num_poles_, latticek_.begin());
    // eval gain
    PackFloat2 gain{};
    for (int i = 0; i < static_cast<int>(ef_.size()); ++i) {
        gain += ef_[static_cast<size_t>(i)] * ef_[static_cast<size_t>(i)];
    }
    gain = qwqdsp_simd_element::PackOps::Sqrt(gain);
    PackFloat2 gain_side{};
    gain_side.Broadcast(window_norm_);
    auto atten = gain / (gain_side);
    // atten smooth
    gain_lag_ *= attack_factor_;
    gain_lag_ += (1.0f - attack_factor_) * atten;
    // iir lattice
    std::array<PackFloat2, kMaxPoles + 1> l_iir{};
    for (int j = 0; j < static_cast<int>(ef_.size()); ++j) {
        auto x0 = side[static_cast<size_t>(j)] * gain_lag_;
        for (int idx = 0; idx < num_poles_; idx += 2) {
            auto x1 = x0 - latticek_[static_cast<size_t>(idx)] * l_iir[static_cast<size_t>(idx) + 1];
            auto x2 = x1 - latticek_[static_cast<size_t>(idx) + 1] * l_iir[static_cast<size_t>(idx) + 2];
            auto l0 = l_iir[static_cast<size_t>(idx) + 1] + latticek_[static_cast<size_t>(idx)] * x1;
            auto l1 = l_iir[static_cast<size_t>(idx) + 2] + latticek_[static_cast<size_t>(idx) + 1] * x2;
            x0 = x2;
            l_iir[static_cast<size_t>(idx)] = l0;
            l_iir[static_cast<size_t>(idx) + 1] = l1;
        }
        l_iir[static_cast<size_t>(num_poles_)] = x0;
        ef_[static_cast<size_t>(j)] = x0;
    }
    return std::span<PackFloat2 const>{ef_};
}

void BlockBurgLPC::CopyLatticeCoeffient(std::span<float> buffer, int order) {
    auto const backup = latticek_;
    auto reverse_iir_it = backup.begin() + order;
    for (int i = 0; i < order; ++i) {
        buffer[static_cast<size_t>(i)] = (*(--reverse_iir_it))[0];
    }
}

} // namespace green_vocoder::dsp
