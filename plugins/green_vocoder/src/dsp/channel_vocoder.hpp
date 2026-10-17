#pragma once
#include <array>
#include <cstddef>
#include <qwqdsp/filter/iir_design.hpp>
#include <qwqdsp/filter/iir_design_extra.hpp>
#include <qwqdsp/simd_element/simd_pack.hpp>

#include "../global.hpp"

namespace green_vocoder::dsp {

// ------------------------------------------------------------
// TDF2 双二阶节（4 个频带并行）
// ------------------------------------------------------------
// 左右声道各一份状态；系数按 4 个 SIMD lane 分别存放。
// 相比 SVF，TDF2 直接使用 b/a 系数，便于配合 IIR 设计的
// 增益重分配（NormalizeSectionGains）控制状态范围。
struct BiquadTDF2 {
    using Pack4 = qwqdsp_simd_element::PackFloat<4>;

    void Set(std::array<qwqdsp_filter::BiquadCoeff, 4> const& c) noexcept {
        b0_ = {c[0].b0, c[1].b0, c[2].b0, c[3].b0};
        b1_ = {c[0].b1, c[1].b1, c[2].b1, c[3].b1};
        b2_ = {c[0].b2, c[1].b2, c[2].b2, c[3].b2};
        a1_ = {c[0].a1, c[1].a1, c[2].a1, c[3].a1};
        a2_ = {c[0].a2, c[1].a2, c[2].a2, c[3].a2};
    }

    void Reset() noexcept {
        s1_l_.Broadcast(0);
        s2_l_.Broadcast(0);
        s1_r_.Broadcast(0);
        s2_r_.Broadcast(0);
    }

    void Tick(Pack4& l, Pack4& r) noexcept {
        auto const yl = l * b0_ + s1_l_;
        s1_l_ = l * b1_ - yl * a1_ + s2_l_;
        s2_l_ = l * b2_ - yl * a2_;
        l = yl;

        auto const yr = r * b0_ + s1_r_;
        s1_r_ = r * b1_ - yr * a1_ + s2_r_;
        s2_r_ = r * b2_ - yr * a2_;
        r = yr;
    }
private:
    Pack4 b0_{};
    Pack4 b1_{};
    Pack4 b2_{};
    Pack4 a1_{};
    Pack4 a2_{};
    Pack4 s1_l_{};
    Pack4 s2_l_{};
    Pack4 s1_r_{};
    Pack4 s2_r_{};
};

// 一条级联：最多 6 个 TDF2 节；output_gain 为设计器给出的末端增益
struct CascadeBPTDF2 {
    using Pack4 = qwqdsp_simd_element::PackFloat<4>;

    template <size_t kNumFilters>
    void Tick(Pack4& l, Pack4& r) noexcept {
        for (size_t i = 0; i < kNumFilters; ++i) {
            sections_[i].Tick(l, r);
        }
        l *= output_gain_;
        r *= output_gain_;
    }

    void Reset() noexcept {
        for (auto& s : sections_) {
            s.Reset();
        }
    }

    void SetSection(size_t idx, std::array<qwqdsp_filter::BiquadCoeff, 4> const& c) noexcept {
        sections_[idx].Set(c);
    }

    void SetOutputGain(Pack4 const& gain) noexcept {
        output_gain_ = gain;
    }

    std::array<BiquadTDF2, 6> sections_{};
    Pack4 output_gain_{};
};

enum eChannelVocoderMap {
    eChannelVocoderMap_Linear = 0,
    eChannelVocoderMap_Mel,
    eChannelVocoderMap_Log,
    eChannelVocoderMap_NumEnums
};

class ChannelVocoder {
public:
    enum class FilterBankMode {
        Bandpass12,
        StackButterworth24,
        StackButterworth36,
        FlatButterworth24,
        FlatButterworth36,
        Chebyshev24,
        Chebyshev36,
        Elliptic24,
        Elliptic36,
    };

    struct Params {
        int num_bands{20};
        float freq_begin{40.0f};
        float freq_end{12000.0f};
        float attack{10.0f};
        float release{150.0f};
        float modulator_scale{1.0f};
        float carry_scale{1.0f};
        eChannelVocoderMap map{eChannelVocoderMap_Mel};
        FilterBankMode filter_bank_mode{FilterBankMode::StackButterworth24};
        float gate{-100.0f};
        float formant_shift{0.0f};
        float ripple{1.0f};
    };

    void Init(float sample_rate, int block_size);
    void Process(qwqdsp_simd_element::PackFloat<2>* main, qwqdsp_simd_element::PackFloat<2>* side, int num_samples);

    void SetParam(const Params& p);

    int GetNumBins() const {
        return num_bans_;
    }
    qwqdsp_simd_element::PackFloat<2> GetBinPeak(int idx) const {
        auto v = main_peaks_[static_cast<size_t>(idx) / 4];
        return {v[0][static_cast<size_t>(idx & 3)], v[1][static_cast<size_t>(idx & 3)]};
    }
private:
    void UpdateFilters();
    // 计算当前滤波器模式/纹波对应的 1rad/s 原型（每次滤波器重建只算一次）
    void PreparePrototype();

    template <class AssignMap>
    void _UpdateFilters();

    template <class AssignMap, class Designer>
    void _UpdateFilters2();

    template <size_t kFilterNumbers>
    void _ProcessBlock(qwqdsp_simd_element::PackFloat<2>* main, qwqdsp_simd_element::PackFloat<2>* side,
                       int num_samples);

    float carry_w_mul_{1.0f};
    float gate_peak_{0.0f};
    float gain_{1.0f};
    FilterBankMode filter_bank_mode_{FilterBankMode::StackButterworth24};
    float sample_rate_{48000.0f};
    float freq_begin_{40.0f};
    float freq_end_{12000.0f};
    int num_bans_{20};
    int num_filters_{5};
    float attack_{1.0f};
    float release_{150.0f};
    float scale_{1.0f};
    float carry_scale_{1.0f};
    float attack_ms_{};
    float release_ms_{};
    float filter_ripple_{1.0f};
    eChannelVocoderMap map_{eChannelVocoderMap_Mel};
    bool need_filter_update_{true};
    // 1rad/s 模拟原型；各带通滤波器只需复制它再做频率映射
    std::array<qwqdsp_filter::IIRDesign::ZPK, 3> prototype_{};
    size_t prototype_sections_{};
    std::array<std::pair<CascadeBPTDF2, CascadeBPTDF2>, global::kMaxOrder> filters_;
    std::array<qwqdsp_simd_element::PackFloat<4>[2], global::kMaxOrder> main_peaks_{};
    std::array<qwqdsp_simd_element::PackFloat<2>, 256> output_{};
};

} // namespace green_vocoder::dsp
