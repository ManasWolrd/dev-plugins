#pragma once
#include <array>
#include <complex>
#include <vector>

#include <juce_gui_basics/juce_gui_basics.h>

#include "global.hpp"

// ------------------------------------------------------------
// PzResponseLayer — pzmap 双层编辑器的背景层（不接收鼠标）
// 绘制当前零极点（只存上半平面，共轭自动存在）产生的原始幅度响应
//   |H(q)| = |∏(1 - z_j q) / ∏(1 - p_k q)|，q = e^{-jω}，x = ω/π
// 控制点以极坐标 (x, r) 传入，内部转换为复数求值。
// 峰值归一到 0dB，显示窗口 [kDbFloor, kDbCeil]。
// x / dB 映射常量与 PzEditLayer / PzmapView（容器）保持一致，
// 三层使用同一 plotting bound。
// ------------------------------------------------------------
class PzResponseLayer : public juce::Component {
public:
    PzResponseLayer();

    // 上半平面零极点（各 kMaxSections 个，极坐标），拷贝后重算曲线
    void SetData(std::array<global::PolarPoint, global::kMaxSections> const& poles,
                 std::array<global::PolarPoint, global::kMaxSections> const& zeros);

    void paint(juce::Graphics& g) override;

    // ----- 共享映射常量（与 PzEditLayer 一致）-----
    static constexpr float kXMin = 0.01f;   // 归一化角频率 ω/π 下界
    static constexpr float kXMax = 0.99f;   // 归一化角频率 ω/π 上界
    static constexpr int kGrid = 256;       // 响应采样点数

    // ----- 背景层 y 轴（dB，左轴）-----
    static constexpr float kDbFloor = -60.0f;
    static constexpr float kDbCeil = 3.0f;

private:
    // 产品形式在 x 网格上的幅度（原始未归一化）
    static std::vector<float> ComputeMagnitudes(
        std::array<global::PolarPoint, global::kMaxSections> const& poles,
        std::array<global::PolarPoint, global::kMaxSections> const& zeros);

    std::array<global::PolarPoint, global::kMaxSections> poles_{};
    std::array<global::PolarPoint, global::kMaxSections> zeros_{};
    std::vector<float> db_;  // 峰值归一后 dB（kGrid 个）
};
