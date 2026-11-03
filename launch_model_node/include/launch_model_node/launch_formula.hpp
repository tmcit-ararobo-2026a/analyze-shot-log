// 自動生成: launch_model.py export-cpp --style formula  (手で編集しない)
// 学習 22 発 / LOO誤差 1.29 m / 採用項: v
// 着弾位置 x [m] = c0 + cv*v + cv2*v^2 + cV*V + cI*I
//   v: 実ベルト速度[m/s]  V: バッテリー電圧[V]  I: ピーク電流[A]
#pragma once

namespace launch_model {
constexpr float kC0  = -7.91095920e-01f;
constexpr float kCv  = 1.26233930e+00f;
constexpr float kCv2 = 0.00000000e+00f;
constexpr float kCV  = 0.00000000e+00f;
constexpr float kCI  = 0.00000000e+00f;
constexpr float kVmin = 3.000f;     // 速度の下限 [m/s]
constexpr float kVmax = 7.750f;     // 速度の上限 [m/s]
constexpr float kVref = 18.990f;     // 電圧の既定値 [V] (学習時の中央値)
constexpr float kIref = 87.400f;     // 電流の既定値 [A]
constexpr float kTrainMin = 2.160f;   // 学習した速度範囲 (外は外挿)
constexpr float kTrainMax = 7.620f;
constexpr float kErrorBand90 = 2.124f;   // 予測の90%範囲 [m]
constexpr float kRatioR0 = 1.01000f;       // 実速度 / 指令速度
constexpr float kRatioR1 = 0.00000f;       // 電圧1Vあたりの比の変化
constexpr float kRatioVref = 18.990f;

/** 実速度 [m/s] -> 着弾位置 [m] */
inline float distance_from_velocity(float v, float voltage = kVref, float current = kIref) {
    return kC0 + kCv * v + kCv2 * v * v + kCV * voltage + kCI * current;
}

/** 着弾したい位置 [m] -> 必要な実速度 [m/s]  (範囲を走査し、最も遅い解を返す。届かなければ端の速度) */
inline float velocity_from_distance(float d, float voltage = kVref, float current = kIref) {
    constexpr float kStep = 0.01f;
    float prev_v = kVmin;
    float prev = distance_from_velocity(prev_v, voltage, current) - d;
    if (prev >= 0.0f) return kVmin;
    for (float v = kVmin + kStep; v <= kVmax + 1e-6f; v += kStep) {
        const float cur = distance_from_velocity(v, voltage, current) - d;
        if (cur >= 0.0f) {
            return prev_v + prev / (prev - cur) * (v - prev_v);
        }
        prev_v = v;
        prev = cur;
    }
    return kVmax;
}

/** 実速度 -> 指令すべき目標速度 (バッテリー電圧 [V] で補正) */
inline float command_velocity(float v, float voltage = kVref) {
    return v / (kRatioR0 + kRatioR1 * (voltage - kRatioVref));
}
}  // namespace launch_model
