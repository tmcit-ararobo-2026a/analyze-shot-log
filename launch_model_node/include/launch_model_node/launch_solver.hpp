// 着弾モデルの計算 (ROS 非依存)。launch_model.py の predict / solve と同じ計算をする。
//   着弾位置 x [m] = c0 + cv*v + cv2*v^2 + cV*V + cI*I
//   v: 実ベルト速度[m/s]  V: バッテリー電圧[V]  I: ピーク電流[A]
//   実速度 = 指令速度 * (ratio_r0 + ratio_r1*(V - ratio_v_ref))
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "launch_model_node/launch_formula.hpp"

namespace launch_model_node {

struct ModelParams {
  double c0, cv, cv2, cV, cI;
  double v_min, v_max;            // 指令できる速度範囲 [m/s]
  double voltage_ref, current_ref;
  double train_v_min, train_v_max;  // 学習した速度範囲(この外は外挿)
  double error_band_90;           // 予測の90%範囲 [m]
  double ratio_r0, ratio_r1, ratio_v_ref;
  double launcher_x;              // 発射位置の x [m] (距離をここ基準にする)
};

struct SolveResult {
  double velocity;            // 必要な実ベルト速度 [m/s]
  double command_unclamped;   // 指令すべき目標速度 (丸める前) [m/s]
  double command;             // 指令すべき目標速度 ([v_min, v_max] に丸めた値) [m/s]
  double predicted_distance;  // 上の velocity で撃ったときの予測着弾 [m]
  double reach_min;           // 範囲内で届く着弾位置の最小・最大 [m]
  double reach_max;
  bool reachable;             // 目標距離に一致する速度が範囲内にある
  bool clamped;               // command を範囲に丸めた
  bool extrapolated;          // velocity が学習範囲の外
};

/** 生成ヘッダ(launch_formula.hpp)の値を既定値にする。再fit後は YAML で上書きできる。 */
inline ModelParams default_params() {
  namespace lm = launch_model;
  ModelParams p{};
  p.c0 = lm::kC0;  p.cv = lm::kCv;  p.cv2 = lm::kCv2;  p.cV = lm::kCV;  p.cI = lm::kCI;
  p.v_min = lm::kVmin;  p.v_max = lm::kVmax;
  p.voltage_ref = lm::kVref;  p.current_ref = lm::kIref;
  p.train_v_min = lm::kTrainMin;  p.train_v_max = lm::kTrainMax;
  p.error_band_90 = lm::kErrorBand90;
  p.ratio_r0 = lm::kRatioR0;  p.ratio_r1 = lm::kRatioR1;  p.ratio_v_ref = lm::kRatioVref;
  p.launcher_x = 0.0;
  return p;
}

/** 実速度 v -> 着弾位置 [m] (発射位置 launcher_x 基準) */
inline double predict_distance(const ModelParams& p, double v, double voltage, double current) {
  return p.c0 + p.cv * v + p.cv2 * v * v + p.cV * voltage + p.cI * current - p.launcher_x;
}

/** 実速度 / 指令速度。電圧で変わる。異常値のときは r0 → 1.0 にフォールバックする。 */
inline double ratio_at(const ModelParams& p, double voltage) {
  double r = p.ratio_r0 + p.ratio_r1 * (voltage - p.ratio_v_ref);
  if (!(r > 0.1)) r = p.ratio_r0;
  if (!(r > 0.1)) r = 1.0;
  return r;
}

/**
 * 着弾したい位置 distance [m] -> 必要な速度と、指令すべき目標速度。
 * [v_min, v_max] を 0.005 m/s 刻みで調べ、一番遅い解を線形補間で返す。
 * 解が無ければ最も近い速度を返し reachable=false にする。
 */
inline SolveResult solve(const ModelParams& p, double distance, double voltage, double current) {
  constexpr double kStep = 0.005;
  const double goal = distance;  // predict_distance 側で launcher_x を引いているので同じ基準

  SolveResult r{};
  double prev_v = p.v_min;
  double prev = predict_distance(p, prev_v, voltage, current) - goal;
  double best_v = prev_v, best_abs = std::fabs(prev);
  double fmin = prev + goal, fmax = fmin;
  double sol = std::numeric_limits<double>::quiet_NaN();
  bool found = false;
  if (prev == 0.0) { sol = prev_v; found = true; }

  auto visit = [&](double v) {
    const double cur = predict_distance(p, v, voltage, current) - goal;
    const double f = cur + goal;
    fmin = std::min(fmin, f);
    fmax = std::max(fmax, f);
    if (std::fabs(cur) < best_abs) { best_abs = std::fabs(cur); best_v = v; }
    if (!found && prev * cur <= 0.0) {
      const double t = (prev != cur) ? prev / (prev - cur) : 0.0;
      sol = prev_v + t * (v - prev_v);
      found = true;
    }
    prev_v = v;
    prev = cur;
  };
  const int n = static_cast<int>(std::floor((p.v_max - p.v_min) / kStep + 1e-9));
  for (int i = 1; i <= n; ++i) visit(p.v_min + i * kStep);
  if (p.v_min + n * kStep < p.v_max - 1e-9) visit(p.v_max);

  r.reachable = found;
  r.velocity = found ? sol : best_v;
  r.command_unclamped = r.velocity / ratio_at(p, voltage);
  r.command = std::clamp(r.command_unclamped, p.v_min, p.v_max);
  r.clamped = std::fabs(r.command - r.command_unclamped) > 1e-9;
  r.predicted_distance = predict_distance(p, r.velocity, voltage, current);
  r.reach_min = fmin;
  r.reach_max = fmax;
  r.extrapolated = r.velocity < p.train_v_min - 1e-9 || r.velocity > p.train_v_max + 1e-9;
  return r;
}

}  // namespace launch_model_node
