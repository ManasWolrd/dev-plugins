#pragma once
#include <array>
#include <functional>

#include <juce_gui_basics/juce_gui_basics.h>

#include "global.hpp"

// ------------------------------------------------------------
// PzEditLayer — pzmap 双层编辑器的交互层（置顶，接收鼠标）
// 控制点以极坐标 (x = ω/π, r = |z|) 为权威状态存储与处理：
// 左键拖拽直接修改 (x, r)，右键复位最近点为默认布局（由容器生成）。
// x 轴范围与 PzResponseLayer 完全一致（共享同一 bound）；
// y 轴为模长，量程 [0, 1]（kRMax = 1，单位圆虚线在顶端），圆心可拖到 r = 0。
// 全程不出现 atan2：角度只来自拖拽/默认布局，从不由复数反推。
// 拖拽只做显示范围钳制（x、半径），允许拖入"非法区"
// （|Im(p)| < global::kMinPoleImag）：该区用警告色标出，落入其中的极点
// 会被引擎整节禁用（不参与分解），并在此处以警告色叉圈显示。
// ------------------------------------------------------------
class PzEditLayer : public juce::Component {
public:
    PzEditLayer();

    void SetData(std::array<global::PolarPoint, global::kMaxSections> const& poles,
                 std::array<global::PolarPoint, global::kMaxSections> const& zeros);

    void paint(juce::Graphics& g) override;
    void mouseDown(juce::MouseEvent const& e) override;
    void mouseDrag(juce::MouseEvent const& e) override;
    void mouseUp(juce::MouseEvent const& e) override;

    // idx：节序号；pole：true 为极点；z：最终约束后的极坐标
    std::function<void(size_t, bool, global::PolarPoint)> on_commit_;
    // 右键复位：请求容器生成默认布局并走提交路径
    std::function<void(size_t, bool)> on_reset_;

    // ----- 共享映射常量（与 PzResponseLayer 一致）-----
    static constexpr float kXMin = 0.01f;
    static constexpr float kXMax = 0.99f;

    // ----- 拖拽层 y 轴（模长，右轴）-----
    static constexpr float kRMax = 1.0f;       // 显示量程上界（单位圆在顶端）
    static constexpr float kRMin = 0.0f;       // 半径下界（允许圆心落到 r=0）
    static constexpr float kPoleRMax = 0.998f; // 极点半径上界（须 < 0.999 才有效）
    static constexpr float kZeroRMax = 1.0f;   // 零点半径上界（允许落在单位圆上）
    static constexpr float kMarkerR = 7.0f;    // 零点/极点标记圆半径（含编号）
    static constexpr float kMarkerFontH = 10.0f; // 编号字号

private:
    // (x, r) → 像素；像素 → (x, r)
    juce::Point<float> ToScreen(float x, float r) const;
    void FromScreen(juce::Point<float> pos, float& x_out, float& r_out) const;

    bool HitTest(juce::Point<int> pos, bool pole, int& out_idx) const;
    void ClampAndCommit(size_t idx, bool pole, float x, float r);

    std::array<global::PolarPoint, global::kMaxSections> poles_{};
    std::array<global::PolarPoint, global::kMaxSections> zeros_{};
    int dragging_pole_idx_{-1};
    int dragging_zero_idx_{-1};
};
