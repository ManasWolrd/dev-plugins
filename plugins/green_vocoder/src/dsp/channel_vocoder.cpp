#include "channel_vocoder.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <span>
#include <qwqdsp/convert.hpp>

namespace green_vocoder::dsp {

using Pack4 = qwqdsp_simd_element::PackFloat<4>;
using Pack4CRef = qwqdsp_simd_element::PackFloatCRef<4>;
using ZPK = qwqdsp_filter::IIRDesign::ZPK;
using BiquadCoeff = qwqdsp_filter::BiquadCoeff;

namespace {
bool ParamChanged(float a, float b) noexcept {
    return std::abs(a - b) > 0.0f;
}

// TPT SVF 带通（MakeBandpass）的 TDF2 等效系数：
// g = tan(pi*fc/fs)（这里由设计函数给出），R = 1/Q
std::array<BiquadCoeff, 4> MakeBandpassCoeffs(Pack4 const& g, Pack4 const& Q) noexcept {
    auto const r2 = 1.0f / Q;
    auto const d = 1.0f / (1.0f + r2 * g + g * g);
    auto const b0 = d * g * r2;
    auto const a1 = -2.0f * d * (1.0f - g * g);
    auto const a2 = -1.0f + 2.0f * d * (1.0f + g * g);
    std::array<BiquadCoeff, 4> coeff;
    for (size_t i = 0; i < 4; ++i) {
        coeff[i] = {b0[i], 0.0f, -b0[i], a1[i], a2[i]};
    }
    return coeff;
}
} // namespace

void ChannelVocoder::Init(float sample_rate, [[maybe_unused]] int block_size) {
    sample_rate_ = sample_rate;
    need_filter_update_ = true;
    SetParam(Params{});
}

void ChannelVocoder::SetParam(const Params& p) {
    assert(p.num_bands % 4 == 0);

    float const new_carry_w_mul = std::exp2(p.formant_shift / 12.0f);
    bool const filters_changed =
        need_filter_update_ || filter_bank_mode_ != p.filter_bank_mode || num_bans_ != p.num_bands
        || ParamChanged(freq_begin_, p.freq_begin) || ParamChanged(freq_end_, p.freq_end)
        || ParamChanged(scale_, p.modulator_scale) || ParamChanged(carry_scale_, p.carry_scale)
        || ParamChanged(filter_ripple_, p.ripple) || ParamChanged(carry_w_mul_, new_carry_w_mul)
        || map_ != p.map;

    filter_bank_mode_ = p.filter_bank_mode;
    num_bans_ = p.num_bands;
    num_filters_ = p.num_bands / 4;
    freq_begin_ = p.freq_begin;
    freq_end_ = p.freq_end;

    attack_ms_ = p.attack;
    attack_ = qwqdsp::convert::Ms2DecayDb(p.attack, sample_rate_, -60.0f);
    release_ms_ = p.release;
    release_ = qwqdsp::convert::Ms2DecayDb(p.release + p.attack, sample_rate_, -60.0f);

    scale_ = p.modulator_scale;
    carry_scale_ = p.carry_scale;
    map_ = p.map;
    gate_peak_ = qwqdsp::convert::Db2Gain(p.gate);
    carry_w_mul_ = new_carry_w_mul;
    filter_ripple_ = p.ripple;

    if (filters_changed) {
        need_filter_update_ = false;
        UpdateFilters();
        for (auto& f : filters_) {
            f.first.Reset();
            f.second.Reset();
        }
    }
}

// -------------------- frequency maps --------------------
struct LogMap {
    static float FromFreq(float freq) {
        return std::log(freq);
    }

    static Pack4 ToFreq(Pack4 log) {
        return qwqdsp_simd_element::PackOps::Exp(log);
    }

    static Pack4 FromFreq(Pack4 log) {
        return qwqdsp_simd_element::PackOps::Log(log);
    }

    static float ToFreq(float log) {
        return std::exp(log);
    }
};

struct MelMap {
    static Pack4 FromFreq(Pack4 freq) {
        return 1127.0f * qwqdsp_simd_element::PackOps::Log(1.0f + freq / 700.0f);
    }

    static float FromFreq(float freq) {
        return 1127.0f * std::log(1.0f + freq / 700.0f);
    }

    static Pack4 ToFreq(Pack4 mel) {
        return 700.0f * (qwqdsp_simd_element::PackOps::Exp(mel / 1127.0f) - 1.0f);
    }

    static float ToFreq(float mel) {
        return 700.0f * (std::exp(mel / 1127.0f) - 1.0f);
    }
};

struct LinearMap {
    static Pack4 FromFreq(Pack4 freq) {
        return freq;
    }

    static float FromFreq(float freq) {
        return freq;
    }

    static Pack4 ToFreq(Pack4 freq) {
        return freq;
    }

    static float ToFreq(float freq) {
        return freq;
    }
};

void ChannelVocoder::PreparePrototype() {
    prototype_sections_ = 0;
    auto const proto2 = std::span<ZPK>{prototype_.data(), 2};
    auto const proto3 = std::span<ZPK>{prototype_.data(), 3};
    switch (filter_bank_mode_) {
        case FilterBankMode::Bandpass12:
        case FilterBankMode::StackButterworth24:
        case FilterBankMode::StackButterworth36:
            break;
        case FilterBankMode::FlatButterworth24:
            qwqdsp_filter::IIRDesignExtra::ButterworthAttenGain(proto2, 2, 0.5f);
            prototype_sections_ = 2;
            break;
        case FilterBankMode::FlatButterworth36:
            qwqdsp_filter::IIRDesignExtra::ButterworthAttenGain(proto3, 3, std::sqrt(0.793700526f));
            prototype_sections_ = 3;
            break;
        case FilterBankMode::Chebyshev24:
            qwqdsp_filter::IIRDesign::Chebyshev1(proto2, 2, filter_ripple_, false);
            prototype_sections_ = 2;
            break;
        case FilterBankMode::Chebyshev36:
            qwqdsp_filter::IIRDesign::Chebyshev1(proto3, 3, filter_ripple_, false);
            prototype_sections_ = 3;
            break;
        case FilterBankMode::Elliptic24:
            if (!qwqdsp_filter::IIRDesign::Elliptic(proto2, 2, filter_ripple_, 100.0, false)) {
                // 规格过陡时退回 4 阶 Butterworth，保证节有效
                qwqdsp_filter::IIRDesignExtra::ButterworthAttenGain(proto2, 2, 0.5f);
            }
            prototype_sections_ = 2;
            break;
        case FilterBankMode::Elliptic36:
            if (!qwqdsp_filter::IIRDesign::Elliptic(proto3, 3, filter_ripple_, 100.0, false)) {
                qwqdsp_filter::IIRDesignExtra::ButterworthAttenGain(proto3, 3, std::sqrt(0.793700526f));
            }
            prototype_sections_ = 3;
            break;
    }
}

void ChannelVocoder::UpdateFilters() {
    PreparePrototype();
    switch (map_) {
        case eChannelVocoderMap_Log:
            this->template _UpdateFilters<LogMap>();
            break;
        case eChannelVocoderMap_Linear:
            this->template _UpdateFilters<LinearMap>();
            break;
        case eChannelVocoderMap_Mel:
            this->template _UpdateFilters<MelMap>();
            break;
        case eChannelVocoderMap_NumEnums:
        default:
            assert(false);
            break;
    }
}

// -------------------- filter designs --------------------
struct Bandpass12 {
    static void Design(CascadeBPTDF2& cascade, [[maybe_unused]] std::span<ZPK const> prototype, Pack4CRef w1,
                       Pack4CRef w2, float analog_w_mul,
                       [[maybe_unused]] float ripple, [[maybe_unused]] float sample_rate) noexcept {
        auto const f1 = qwqdsp_simd_element::PackOps::Tan(w1 * 0.5f) * analog_w_mul;
        auto const f2 = qwqdsp_simd_element::PackOps::Tan(w2 * 0.5f) * analog_w_mul;
        auto const f0 = qwqdsp_simd_element::PackOps::Sqrt(f1 * f2);
        auto const Q = f0 / qwqdsp_simd_element::PackOps::Abs(f2 - f1);
        auto const coeff = MakeBandpassCoeffs(f0, Q);
        cascade.SetSection(0, coeff);
        cascade.SetSection(1, coeff);
        cascade.SetOutputGain(Pack4::vBroadcast(1.0f));
    }
};

struct StackButterworth24 {
    static void Design(CascadeBPTDF2& cascade, [[maybe_unused]] std::span<ZPK const> prototype, Pack4CRef w1,
                       Pack4CRef w2, float analog_w_mul,
                       [[maybe_unused]] float ripple, [[maybe_unused]] float sample_rate) noexcept {
        auto const f1 = qwqdsp_simd_element::PackOps::Tan(w1 * 0.5f) * analog_w_mul;
        auto const f2 = qwqdsp_simd_element::PackOps::Tan(w2 * 0.5f) * analog_w_mul;
        auto const f0 = qwqdsp_simd_element::PackOps::Sqrt(f1 * f2);
        constexpr float half_power = std::numbers::sqrt2_v<float> * 0.5f;
        auto const w_pow_2 = qwqdsp_simd_element::PackOps::X2(f1 / f0);
        auto const Q = qwqdsp_simd_element::PackOps::Sqrt(w_pow_2 / half_power - w_pow_2)
                     / qwqdsp_simd_element::PackOps::Abs(w_pow_2 - 1.0f);
        auto const coeff = MakeBandpassCoeffs(f0, Q);
        for (size_t i = 0; i < 4; ++i) {
            cascade.SetSection(i, coeff);
        }
        cascade.SetOutputGain(Pack4::vBroadcast(1.0f));
    }
};

struct StackButterworth36 {
    static void Design(CascadeBPTDF2& cascade, [[maybe_unused]] std::span<ZPK const> prototype, Pack4CRef w1,
                       Pack4CRef w2, float analog_w_mul,
                       [[maybe_unused]] float ripple, [[maybe_unused]] float sample_rate) noexcept {
        auto const f1 = qwqdsp_simd_element::PackOps::Tan(w1 * 0.5f) * analog_w_mul;
        auto const f2 = qwqdsp_simd_element::PackOps::Tan(w2 * 0.5f) * analog_w_mul;
        auto const f0 = qwqdsp_simd_element::PackOps::Sqrt(f1 * f2);
        constexpr float half_power = 0.793700526f;
        auto const w_pow_2 = qwqdsp_simd_element::PackOps::X2(f1 / f0);
        auto const Q = qwqdsp_simd_element::PackOps::Sqrt(w_pow_2 / half_power - w_pow_2)
                     / qwqdsp_simd_element::PackOps::Abs(w_pow_2 - 1.0f);
        auto const coeff = MakeBandpassCoeffs(f0, Q);
        for (size_t i = 0; i < 6; ++i) {
            cascade.SetSection(i, coeff);
        }
        cascade.SetOutputGain(Pack4::vBroadcast(1.0f));
    }
};

struct PrototypedDesigner {
    // prototype 为 1rad/s 模拟原型（每次滤波器重建只计算一次），
    // 这里只对每个频带做 bandpass 映射、双线性变换和增益重分配。
    static void Design(CascadeBPTDF2& cascade, std::span<ZPK const> prototype, Pack4CRef w1, Pack4CRef w2,
                       float analog_w_mul, [[maybe_unused]] float ripple, float sample_rate) {
        size_t const num_prototype = prototype.size();
        std::array<ZPK, 6> prototype_buffer;
        std::copy(prototype.begin(), prototype.end(), prototype_buffer.begin());

        std::array<std::array<BiquadCoeff, 4>, 6> section_coeffs;
        std::array<float, 4> output_gain{};

        auto const w1_analog =
            qwqdsp_simd_element::PackOps::Tan(w1 * 0.5f) * analog_w_mul * (2.0f * sample_rate);
        auto const w2_analog =
            qwqdsp_simd_element::PackOps::Tan(w2 * 0.5f) * analog_w_mul * (2.0f * sample_rate);
        for (int lane = 0; lane < 4; ++lane) {
            std::array<ZPK, 6> zpk_buffer = prototype_buffer;
            auto const sections = std::span<ZPK>{zpk_buffer.data(), num_prototype * 2};
            qwqdsp_filter::IIRDesign::ProtyleToBandpass2(sections, num_prototype,
                                                         w1_analog[static_cast<size_t>(lane)],
                                                         w2_analog[static_cast<size_t>(lane)]);
            qwqdsp_filter::IIRDesign::Bilinear(sections, sample_rate);
            // 重新分配各 SOS 的增益项，避免高 Q/窄带时出现 1e±N 的节增益
            output_gain[static_cast<size_t>(lane)] = static_cast<float>(
                qwqdsp_filter::IIRDesign::NormalizeSectionGains(sections));

            std::array<BiquadCoeff, 6> biquad;
            qwqdsp_filter::IIRDesign::TfToBiquad(sections, std::span<BiquadCoeff>{biquad.data(), num_prototype * 2});
            for (size_t i = 0; i < num_prototype * 2; ++i) {
                section_coeffs[i][static_cast<size_t>(lane)] = biquad[i];
            }
        }

        for (size_t i = 0; i < num_prototype * 2; ++i) {
            cascade.SetSection(i, section_coeffs[i]);
        }
        cascade.SetOutputGain({output_gain[0], output_gain[1], output_gain[2], output_gain[3]});
    }
};

template <class AssignMap>
void ChannelVocoder::_UpdateFilters() {
    switch (filter_bank_mode_) {
        case FilterBankMode::Bandpass12:
            _UpdateFilters2<AssignMap, Bandpass12>();
            break;
        case FilterBankMode::StackButterworth24:
            _UpdateFilters2<AssignMap, StackButterworth24>();
            break;
        case FilterBankMode::StackButterworth36:
            _UpdateFilters2<AssignMap, StackButterworth36>();
            break;
        case FilterBankMode::FlatButterworth24:
        case FilterBankMode::FlatButterworth36:
        case FilterBankMode::Chebyshev24:
        case FilterBankMode::Chebyshev36:
        case FilterBankMode::Elliptic24:
        case FilterBankMode::Elliptic36:
            _UpdateFilters2<AssignMap, PrototypedDesigner>();
            break;
    }
    gain_ = 5;
    if (num_bans_ < 48) {
        float norm = static_cast<float>(num_bans_) / 48.0f;
        float atten = std::lerp(0.3f, 1.0f, norm);
        gain_ *= atten;
    }
}

template <class AssignMap, class Designer>
void ChannelVocoder::_UpdateFilters2() {
    float pitch_begin = AssignMap::FromFreq(freq_begin_);
    float pitch_end = AssignMap::FromFreq(freq_end_);
    float pitch_interval = (pitch_end - pitch_begin) / static_cast<float>(num_bans_);
    float begin = qwqdsp::convert::Freq2W(freq_begin_, sample_rate_);

    int filter_idx = 0;
    auto const prototype = std::span<ZPK const>{prototype_.data(), prototype_sections_};
    auto const min_w = Pack4::vBroadcast(qwqdsp::convert::Freq2W(10.0f, sample_rate_));
    auto const max_w = Pack4::vBroadcast(qwqdsp::convert::Freq2W(sample_rate_ * 0.5f - 100.0f, sample_rate_));
    for (int i = 0; i < num_bans_; i += 4) {
        Pack4 end_omega{
            pitch_begin + pitch_interval * (i + 1), pitch_begin + pitch_interval * (i + 2),
            pitch_begin + pitch_interval * (i + 3), pitch_begin + pitch_interval * (i + 4)};
        end_omega = AssignMap::ToFreq(end_omega);
        end_omega = std::numbers::pi_v<float> * 2.0f * end_omega / sample_rate_;
        Pack4 prev_omega{begin, end_omega[0], end_omega[1], end_omega[2]};
        begin = end_omega[3];

        auto center = (prev_omega + end_omega) * 0.5f;
        auto half_bw = end_omega - center;
        auto main_half_bw = half_bw * scale_;
        auto main_w1 = center - main_half_bw;
        auto main_w2 = center + main_half_bw;
        auto side_half_bw = half_bw * carry_scale_;
        auto side_w1 = center - side_half_bw;
        auto side_w2 = center + side_half_bw;
        main_w1 = qwqdsp_simd_element::PackOps::Clamp(main_w1, min_w, max_w);
        main_w2 = qwqdsp_simd_element::PackOps::Clamp(main_w2, min_w, max_w);
        side_w1 = qwqdsp_simd_element::PackOps::Clamp(side_w1, min_w, max_w);
        side_w2 = qwqdsp_simd_element::PackOps::Clamp(side_w2, min_w, max_w);
        auto& main_filter = filters_[static_cast<size_t>(filter_idx)].first;
        auto& side_filter = filters_[static_cast<size_t>(filter_idx)].second;
        Designer::Design(main_filter, prototype, main_w1, main_w2, 1.0f, filter_ripple_, sample_rate_);
        Designer::Design(side_filter, prototype, side_w1, side_w2, carry_w_mul_, filter_ripple_, sample_rate_);
        ++filter_idx;
    }
}

void ChannelVocoder::Process(qwqdsp_simd_element::PackFloat<2>* main, qwqdsp_simd_element::PackFloat<2>* side,
                             int num_samples) {
    switch (filter_bank_mode_) {
        case FilterBankMode::Bandpass12:
            _ProcessBlock<2>(main, side, num_samples);
            break;
        case FilterBankMode::StackButterworth24:
            _ProcessBlock<4>(main, side, num_samples);
            break;
        case FilterBankMode::StackButterworth36:
            _ProcessBlock<6>(main, side, num_samples);
            break;
        case FilterBankMode::FlatButterworth24:
        case FilterBankMode::Chebyshev24:
        case FilterBankMode::Elliptic24:
            _ProcessBlock<4>(main, side, num_samples);
            break;
        case FilterBankMode::FlatButterworth36:
        case FilterBankMode::Chebyshev36:
        case FilterBankMode::Elliptic36:
            _ProcessBlock<6>(main, side, num_samples);
            break;
    }
}

template <size_t kFilterNumbers>
void ChannelVocoder::_ProcessBlock(qwqdsp_simd_element::PackFloat<2>* main, qwqdsp_simd_element::PackFloat<2>* side,
                                   int num_samples) {
    std::fill_n(output_.begin(), num_samples, qwqdsp_simd_element::PackFloat<2>{});
    auto vgate_peak = Pack4::vBroadcast(gate_peak_);
    for (int filter_idx = 0; filter_idx < num_filters_; ++filter_idx) {
        for (int sample_idx = 0; sample_idx < num_samples; ++sample_idx) {
            // filtering
            Pack4 main_l;
            Pack4 main_r;
            Pack4 side_l;
            Pack4 side_r;
            main_l.Broadcast(main[sample_idx][0]);
            main_r.Broadcast(main[sample_idx][1]);
            side_l.Broadcast(side[sample_idx][0]);
            side_r.Broadcast(side[sample_idx][1]);
            main_l *= gain_;
            main_r *= gain_;
            side_l *= gain_;
            side_r *= gain_;
            auto& filter_bind = filters_[static_cast<size_t>(filter_idx)];
            filter_bind.first.Tick<kFilterNumbers>(main_l, main_r);
            filter_bind.second.Tick<kFilterNumbers>(side_l, side_r);
            // envelope follower
            auto curr = main_peaks_[static_cast<size_t>(filter_idx)];
            main_l = qwqdsp_simd_element::PackOps::Abs(main_l);
            main_r = qwqdsp_simd_element::PackOps::Abs(main_r);
            main_l = qwqdsp_simd_element::PackOps::Select(main_l > vgate_peak, main_l, Pack4::vBroadcast(0.0f));
            main_r = qwqdsp_simd_element::PackOps::Select(main_r > vgate_peak, main_r, Pack4::vBroadcast(0.0f));
            auto lag_l = curr[0];
            auto lag_r = curr[1];
            auto coeff_mask_l = main_l > lag_l;
            auto coeff_mask_r = main_r > lag_r;
            auto coeff_l = qwqdsp_simd_element::PackOps::Select(coeff_mask_l, Pack4::vBroadcast(attack_),
                                                                Pack4::vBroadcast(release_));
            auto coeff_r = qwqdsp_simd_element::PackOps::Select(coeff_mask_r, Pack4::vBroadcast(attack_),
                                                                Pack4::vBroadcast(release_));
            lag_l *= coeff_l;
            lag_r *= coeff_r;
            lag_l += (1.0f - coeff_l) * main_l;
            lag_r += (1.0f - coeff_r) * main_r;
            main_peaks_[static_cast<size_t>(filter_idx)][0] = lag_l;
            main_peaks_[static_cast<size_t>(filter_idx)][1] = lag_r;
            // output
            float l = qwqdsp_simd_element::PackOps::ReduceAdd(lag_l * side_l);
            float r = qwqdsp_simd_element::PackOps::ReduceAdd(lag_r * side_r);
            output_[static_cast<size_t>(sample_idx)][0] += l;
            output_[static_cast<size_t>(sample_idx)][1] += r;
        }
    }
    std::copy_n(output_.begin(), num_samples, main);
}

} // namespace green_vocoder::dsp
