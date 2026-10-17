## 重要
**遇到不确定内容询问用户**

## 命名

| 类别 | 风格 | 示例 |
|------|------|------|
| 命名空间 | `lower_snake_case` | `qwqdsp_cephes`, `qwqdsp_oscillator` |
| 结构体/类 | `PascalCase` | `struct Bessel`, `class EllipticSineOsc` |
| 函数/方法 | `PascalCase` | `GetWorkDir()`, `OutputFile()`, `i0()` |
| 普通变量 | `snake_case` | `bf_i0`, `total_errs` |
| 成员变量 | `snake_case_` | `sample_rate_`, `phase_` |
| 编译期常量 | `k` 前缀 + PascalCase | `kNum`, `kA`, `kPIO2`, `kTableSize` |
| 宏定义 | `UPPER_SNAKE_CASE` | `QWQDSP_WORK_DIR`, `QWQDSP_HAVE_IPP` |

## C++ 标准

- C++20
- 系数数组使用 `static constexpr`
- 头文件使用 `#pragma once`
