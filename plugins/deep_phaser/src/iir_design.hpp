#pragma once
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <vector>

// ------------------------------------------------------------
// Pole/Zero 设计 → 并行谐振器残差 (partial-fraction)
// 将用户给出的上半平面极点/零点展开为完整的共轭对称集合，然后对全部
// 极点做部分分式分解，得到每一个极点上的残差。这样 Σ R_k/(1 - p_k q)
// 严格等于 num(q)/den(q)。极点不能重复或过近，否则分母 → 0 残差爆炸。
// ------------------------------------------------------------
namespace deepphaser {

struct PoleZeroSet {
    // 上半平面极点 (imag > 0)，|z| < 1 保证稳定；共轭自动存在
    std::vector<std::complex<float>> poles;
    // 上半平面零点 (imag > 0)；共轭自动存在
    std::vector<std::complex<float>> zeros;

    bool empty() const noexcept {
        return poles.empty();
    }
    size_t size() const noexcept {
        return poles.size();
    }
};

// 单个"节"的系数：pole 与其残差（残差按全共轭集合计算）
struct SectionCoeff {
    std::complex<float> pole;
    std::complex<float> residual;
};

struct DesignResult {
    // 展开后的全共轭对称节（每个「上半极点」对应 pole 与 conj(pole) 两条）
    std::vector<SectionCoeff> sections;
    // 前馈增益 iir_fir_k（常数项 K）
    std::complex<float> fir_k{1.0f, 0.0f};
    bool valid{false};

    void Reset() {
        sections.clear();
        fir_k = {1.0f, 0.0f};
        valid = false;
    }
};

// 对给定的(可能含共轭的)极点集合做重复/过近/失稳校验
inline bool ValidatePoles(const std::vector<std::complex<float>>& poles, float min_sep = 0.04f) noexcept {
    if (poles.empty()) {
        return false;
    }
    for (auto const& p : poles) {
        if (!std::isfinite(p.real()) || !std::isfinite(p.imag()) || std::abs(p) >= 0.999f) {
            return false;
        }
    }
    for (size_t i = 0; i < poles.size(); ++i) {
        for (size_t j = i + 1; j < poles.size(); ++j) {
            if (std::abs(poles[i] - poles[j]) < min_sep) {
                return false;
            }
        }
    }
    return true;
}

// 展开上半平面极点为全共轭对称集合
inline void ExpandPoles(const std::vector<std::complex<float>>& upper, std::vector<std::complex<float>>& out) {
    out.clear();
    out.reserve(upper.size() * 2);
    for (auto const& p : upper) {
        out.push_back(p);
        out.push_back(std::conj(p));
    }
}

inline void ExpandZeros(const std::vector<std::complex<float>>& upper, std::vector<std::complex<float>>& out) {
    ExpandPoles(upper, out); // 与极点展开逻辑相同
}

// 部分分式：H(q) = K + Σ_k R_k/(1 - p_k q) 精确等于
//   g ∏_z (1 - z_j q) / ∏_p (1 - p_k q)   (#zeros == #poles)
// 其中 R_k = g ∏_z (1 - z_j / p_k) / ∏_{j≠k} (1 - p_j / p_k)
inline void ComputeResidues(const PoleZeroSet& pz, DesignResult& out) {
    out.Reset();

    std::vector<std::complex<float>> poles, zeros;
    ExpandPoles(pz.poles, poles);
    ExpandZeros(pz.zeros, zeros);
    const size_t n = poles.size();
    if (n == 0) {
        return;
    }
    if (!ValidatePoles(poles)) {
        return;
    }

    // 保证零点个数与极点一致：多则忽略，少则用"原点零"(1 - 0*q)=1 补齐（无效果）
    size_t const nz = std::min(zeros.size(), n);

    constexpr float kGain = 1.0f;
    std::vector<std::complex<float>> residuals(n, std::complex<float>{});
    for (size_t k = 0; k < n; ++k) {
        std::complex<float> num = kGain;
        for (size_t z = 0; z < nz; ++z) {
            num *= (1.0f - zeros[z] / poles[k]);
        }
        std::complex<float> den{1.0f, 0.0f};
        for (size_t j = 0; j < n; ++j) {
            if (j == k) {
                continue;
            }
            den *= (1.0f - poles[j] / poles[k]);
        }
        residuals[k] = num / den;
    }

    // 求常数项 K = g - Σ R_k (因为 H(0) = g，且 H(0) = K + Σ R_k)
    std::complex<float> k = kGain;
    for (auto const& r : residuals) {
        k -= r;
    }

    out.sections.reserve(n);
    for (size_t k = 0; k < n; ++k) {
        out.sections.push_back({poles[k], residuals[k]});
    }
    out.fir_k = k;
    out.valid = true;
}

// 以 base_delay 为延迟变量在 q = e^{-j w D} 单位圆上评估 |H|，求峰值（用于增益归一化）。
// 注意：base_delay(=state) 为 1 时也必须正常评估，否则 state=1 得不到归一化，
// 会导致尖峰布局下 state=1 音量远大于 state>1（bug）。此处只拦截无效设计。
inline float EvaluatePeak(const DesignResult& d, float base_delay, int num_bins = 4096) noexcept {
    if (!d.valid || d.sections.empty() || base_delay <= 0.0f) {
        return 1.0f;
    }
    float peak = 0.0f;
    for (int b = 0; b < num_bins; ++b) {
        float const w = static_cast<float>(b) * std::numbers::pi_v<float> / static_cast<float>(num_bins - 1);
        std::complex<float> q = std::polar(1.0f, -w * base_delay);
        std::complex<float> h = d.fir_k;
        for (auto const& s : d.sections) {
            h += s.residual / (1.0f - s.pole * q);
        }
        float const mag = std::abs(h);
        if (mag > peak) {
            peak = mag;
        }
    }
    return peak;
}

} // namespace deepphaser
