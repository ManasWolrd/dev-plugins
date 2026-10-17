#pragma once
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>

// ------------------------------------------------------------
// DeepPhaser2 全局常量
// ------------------------------------------------------------
namespace global {

// 最大并行谐振节数（= 上半平面极点数，每个对应一个共轭对）
static constexpr size_t kMaxSections = 8;

// 一阶全通重复级数 N 上限（H(z) 被 A(z)^N 拉伸的次数），即旧版 deep_phaser 的 "state"
// N 越大，并行滤波器被 A(z)^N 重复/拉伸的份数越多（高 N 主要靠编译器/SIMD 优化摊平成本）
static constexpr size_t kMaxStages = 128;

static constexpr size_t kFFTSize = 1024;

// ------------------------------------------------------------
// 控制点表示：权威状态为 (归一化极角 x = ω/π, 模长 r)
// 拖拽、约束、绘制都直接在极坐标下进行；只有喂给引擎/算响应时才转成复数
// (re, im)，该方向无损，因此全程无需 atan2 从复数反推角度。
// ------------------------------------------------------------
struct PolarPoint {
    float x{};  // 归一化极角 ω/π
    float r{};  // 模长 |z|
};

// 极坐标 → 复数（x ∈ (0,1) 时落在上半平面）
inline std::complex<float> ToComplex(PolarPoint p) noexcept {
    return std::polar(p.r, std::numbers::pi_v<float> * p.x);
}

// 非法极点判定：|Im| = r·|sin(πx)| 低于阈值时，共轭对间距 2·|Im| 会小于
// iir_design 的 ValidatePoles min_sep(0.04)，部分分式分解随之失败。
// GUI 用此阈值标出"非法区"；引擎把落入该区的极点（连同同索引零点）整节
// 禁用、不参与分解。取 0.05 留余量（对应共轭对间距 0.10 > 0.04）。
static constexpr float kMinPoleImag = 0.05f;

inline bool IsPoleDisabled(PolarPoint p) noexcept {
    return std::abs(p.r * std::sin(std::numbers::pi_v<float> * p.x)) < kMinPoleImag;
}

} // namespace global
