#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

#include "global.hpp"

// ------------------------------------------------------------
// WarpedSection
// 单个并行一阶节：H_k(q) = R_k / (1 - p_k * A(z)^N) ，其中 q = A(z)^N。
// A(z) = (g + z^-1)/(1 + g z^-1) 为一阶全通（统一系数 g，逐采样被调制）。
// 反馈回路穿过 N 阶一阶全通级联：
//   y = R*x + p*w,  w = A^N(y)
// 采用 TPT-ZDF 直通/转置形式：每个全通级只保存一个状态 s（单状态）。
// 标准一阶全通的 TPT 形式为：
//   out = g*in + s
//   s   = in - g*out            （等价 s = in*(1-g^2) - g*s）
// 注意：不能用 s = v + g*s，那对应的是非全通传递函数，会发散。
// 级联的对 v 的直通增益为 g^N，零输入响应 Q 仅依赖历史状态：
//   Q = Σ_{j=1..N} g^{N-j} s_j
// 反馈回路 y = R*x + p*(g^N * y + Q) 解出：
//   y = (R*x + p*Q)/(1 - p*g^N)
// 分母 |1 - p*g^N| >= 1 - |p||g|^N > 0 （|p|<1、|g|<=1），对任意合法布局无 NaN。
// 传入 g 的角度/clamp 保证 |g| < 1。
// ------------------------------------------------------------
namespace deepphaser {

class WarpedSection {
public:
    void Set(std::complex<float> residual, std::complex<float> pole) noexcept {
        residual_ = residual;
        pole_ = pole;
    }

    void ResizeStages(size_t n) {
        if (n == stage_count_)
            return;
        stage_count_ = n;
        L_.assign(n, std::complex<float>{});
        R_.assign(n, std::complex<float>{});
        RecomputeDirectGain();
    }

    void SetCoeff(float g) noexcept {
        coeff_ = g;
        RecomputeDirectGain();
    }

    void Reset() noexcept {
        std::fill(L_.begin(), L_.end(), std::complex<float>{});
        std::fill(R_.begin(), R_.end(), std::complex<float>{});
    }

    // 完整节处理（左/右声道），返回复数值 y_k，由调用方求和 / 取实部
    std::complex<float> TickLeft(float x) noexcept {
        return Tick(x, L_);
    }
    std::complex<float> TickRight(float x) noexcept {
        return Tick(x, R_);
    }
private:
    // 零延迟回路求解：y = R*x + p*A^N(y)，其中 A^N(v) = g^N v + Q。
    // 先由历史状态算出 Q，解出无 NaN 的 y，再按级联顺序更新单状态。
    std::complex<float> Tick(float x, std::vector<std::complex<float>>& s) noexcept {
        if (stage_count_ == 0) {
            return residual_ * x;
        }
        // Q = g^{N-1} s_1 + g^{N-2} s_2 + ... + g^0 s_N（仅历史状态）
        std::complex<float> q{0.0f, 0.0f};
        float gk = 1.0f;
        for (size_t i = stage_count_; i-- > 0;) {
            q += gk * s[i];
            gk *= coeff_;
        }
        auto const den = std::complex<float>{1.0f, 0.0f} - pole_ * direct_gain_;
        auto const y = (residual_ * x + pole_ * q) / den;

        // 级联顺序更新：stage i 输入 = u，out_i = g*u + s_i，s_i = u - g*out_i
        std::complex<float> u = y;
        for (size_t i = 0; i < stage_count_; ++i) {
            auto const out_i = coeff_ * u + s[i];
            s[i] = u - coeff_ * out_i;   // s' = in - g*out（标准一阶全通状态）
            u = out_i;
        }
        return y;
    }

    void RecomputeDirectGain() noexcept {
        direct_gain_ = static_cast<float>(std::pow(static_cast<double>(coeff_), static_cast<int>(stage_count_)));
    }

    float coeff_{0.0f};
    size_t stage_count_{};
    float direct_gain_{0.0f};
    std::complex<float> pole_{};
    std::complex<float> residual_{};
    std::vector<std::complex<float>> L_;
    std::vector<std::complex<float>> R_;
};

} // namespace deepphaser
