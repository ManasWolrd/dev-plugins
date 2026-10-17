#pragma once
#include <algorithm>
#include <complex>
#include <cstddef>
#include <vector>

// ------------------------------------------------------------
// WarpedSectionCheap
// 单个并行一阶节"廉价"实现：H_k(q) = R_k / (1 - p_k * q)，q = z^-1 * A(z)^N。
// A(z) = (g + z^-1)/(1 + g z^-1) 为一阶全通（统一系数 g，逐采样被调制）。
//
// 说明：不采用 ZDF 技巧解出完全正确的形式 R/(1 - p A^N)（那需要算 g^N 直通
// 增益并做一次复数除法），而是把原一阶滤波器里的延迟块 z^-1 替换成
// "延迟块 + 全通级联"：
//   w = A^N(y) 再被额外延迟一个采样（w[n] = A^N(y)[n-1]）
//   y = R*x + p*w
// 由于反馈路径里永远夹着一个真正的 z^-1，反馈信号只依赖上一采样及更早，
// 因此不会产生零延迟反馈回路，也无需任何代数求解。
// 得到的传递函数是 R/(1 - p z^-1 A^N)，DC 处 z^-1→1、A(1)=1，故与精确版
// R/(1 - p A^N) 在直流一致，频响仅多一个相位偏移，属于廉价近似。
//
// 全通级状态（同正确 TPT 形式）：out = g*in + s ；s = in - g*out
//
// barberpole：采用 q 域旋转 H(q -> q*exp(-i*phi))，实现上即把节极点
// p_k -> p_k*exp(-i*phi)（见 SetPoleRotation），单位时间穿越的 notch 数与
// state(=N) 无关；且 phi 走 2π 恰好回到原位，回卷无缝。
// 解析信号（正频谱）经复滤波后取实部得到实值输出。
// ------------------------------------------------------------
namespace deepphaser {

class WarpedSectionCheap {
public:
    void Set(std::complex<float> residual, std::complex<float> pole) noexcept {
        residual_ = residual;
        base_pole_ = pole;
        L_pole_ = pole;
        R_pole_ = pole;
    }

    void ResizeStages(size_t n) {
        if (n == stage_count_)
            return;
        stage_count_ = n;
        L_.assign(n, std::complex<float>{});
        R_.assign(n, std::complex<float>{});
        L_delayed_ = {0.0f, 0.0f};
        R_delayed_ = {0.0f, 0.0f};
    }

    void SetCoeff(float g) noexcept {
        coeff_ = g;
    }

    // barberpole q 域旋转：rot = exp(-i*phi)，逐声道独立（右声道可加立体声偏移）。
    // 旋转的是节的极点，等价于 H(q -> q*rot)，响应绕 q 平面旋转，与 N 无关。
    void SetPoleRotation(std::complex<float> rot_l, std::complex<float> rot_r) noexcept {
        L_pole_ = base_pole_ * rot_l;
        R_pole_ = base_pole_ * rot_r;
    }

    void Reset() noexcept {
        std::fill(L_.begin(), L_.end(), std::complex<float>{});
        std::fill(R_.begin(), R_.end(), std::complex<float>{});
        L_delayed_ = {0.0f, 0.0f};
        R_delayed_ = {0.0f, 0.0f};
        L_pole_ = base_pole_;
        R_pole_ = base_pole_;
    }

    // 实输入（非 barber 路径）：用未旋转极点
    std::complex<float> TickLeft(float x) noexcept {
        return Tick(std::complex<float>{x, 0.0f}, L_, L_delayed_, base_pole_);
    }
    std::complex<float> TickRight(float x) noexcept {
        return Tick(std::complex<float>{x, 0.0f}, R_, R_delayed_, base_pole_);
    }

    // 复输入（barber 路径：解析信号）：用旋转后的极点
    std::complex<float> TickLeftCpx(std::complex<float> x) noexcept {
        return Tick(x, L_, L_delayed_, L_pole_);
    }
    std::complex<float> TickRightCpx(std::complex<float> x) noexcept {
        return Tick(x, R_, R_delayed_, R_pole_);
    }
private:
    // 每采样：y = R*x + p*w（w 为上一采样全通级联的输出，天然带一个 z^-1），
    // 再用当前 y 跑一遍级联更新状态，其输出留到下一采样作为 w。
    std::complex<float> Tick(std::complex<float> x, std::vector<std::complex<float>>& s,
                             std::complex<float>& delayed, std::complex<float> pole) noexcept {
        std::complex<float> const y = residual_ * x + pole * delayed;
        std::complex<float> u = y;
        for (size_t i = 0; i < stage_count_; ++i) {
            std::complex<float> const out = coeff_ * u + s[i];
            s[i] = u - coeff_ * out;   // s' = in - g*out
            u = out;
        }
        delayed = u;   // 供下一采样当作反馈用
        return y;
    }

    float coeff_{0.0f};
    size_t stage_count_{};
    std::complex<float> residual_{};
    std::complex<float> base_pole_{};
    std::complex<float> L_pole_{};
    std::complex<float> R_pole_{};
    std::vector<std::complex<float>> L_;
    std::vector<std::complex<float>> R_;
    std::complex<float> L_delayed_{0.0f, 0.0f};
    std::complex<float> R_delayed_{0.0f, 0.0f};
};

} // namespace deepphaser
