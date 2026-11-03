#include <gtest/gtest.h>

#include <cmath>

#include "launch_model_node/launch_solver.hpp"
#include "launch_model_node/topic_name.hpp"

using launch_model_node::ModelParams;
using launch_model_node::SolveResult;
using launch_model_node::default_params;
using launch_model_node::predict_distance;
using launch_model_node::ratio_at;
using launch_model_node::solve;

namespace {
// 手計算できる単純なモデル: x = 1.0 * v
ModelParams linear_model() {
  ModelParams p{};
  p.cv = 1.0;
  p.v_min = 3.0;
  p.v_max = 7.75;
  p.voltage_ref = 18.0;
  p.current_ref = 80.0;
  p.train_v_min = 2.0;
  p.train_v_max = 7.6;
  p.ratio_r0 = 1.0;
  p.ratio_v_ref = 18.0;
  return p;
}
}  // namespace

TEST(Solver, LinearModelSolvesExactly) {
  const auto r = solve(linear_model(), 5.0, 18.0, 80.0);
  EXPECT_TRUE(r.reachable);
  EXPECT_NEAR(r.velocity, 5.0, 1e-6);
  EXPECT_NEAR(r.command, 5.0, 1e-6);
  EXPECT_FALSE(r.clamped);
  EXPECT_FALSE(r.extrapolated);
}

TEST(Solver, VoltageRatioScalesCommand) {
  auto p = linear_model();
  p.ratio_r1 = 0.01;  // 1V 下がると実速度が 1% 落ちる
  const auto r = solve(p, 5.0, 17.0, 80.0);  // ratio = 0.99
  EXPECT_NEAR(r.velocity, 5.0, 1e-6);
  EXPECT_NEAR(r.command, 5.0 / 0.99, 1e-6);
}

TEST(Solver, UnreachableFarDistanceIsClampedToMax) {
  const auto r = solve(linear_model(), 30.0, 18.0, 80.0);
  EXPECT_FALSE(r.reachable);
  EXPECT_NEAR(r.command, 7.75, 1e-6);
  EXPECT_NEAR(r.reach_max, 7.75, 1e-6);
}

TEST(Solver, UnreachableShortDistanceIsClampedToMin) {
  const auto r = solve(linear_model(), 0.5, 18.0, 80.0);
  EXPECT_FALSE(r.reachable);
  EXPECT_NEAR(r.command, 3.0, 1e-6);
  EXPECT_NEAR(r.reach_min, 3.0, 1e-6);
}

TEST(Solver, LauncherOffsetShiftsDistance) {
  auto p = linear_model();
  p.launcher_x = 1.0;  // 距離は発射位置基準: x - 1.0
  const auto r = solve(p, 4.0, 18.0, 80.0);
  EXPECT_NEAR(r.velocity, 5.0, 1e-6);
  EXPECT_NEAR(predict_distance(p, 5.0, 18.0, 80.0), 4.0, 1e-9);
}

TEST(Solver, CommandIsClampedToRange) {
  auto p = linear_model();
  p.ratio_r0 = 0.5;  // 指令の半分しか出ない -> 必要指令が上限を超える
  const auto r = solve(p, 5.0, 18.0, 80.0);
  EXPECT_NEAR(r.command_unclamped, 10.0, 1e-6);
  EXPECT_NEAR(r.command, 7.75, 1e-9);
  EXPECT_TRUE(r.clamped);
}

TEST(Solver, BadRatioFallsBack) {
  auto p = linear_model();
  p.ratio_r1 = 1.0;
  EXPECT_NEAR(ratio_at(p, 0.0), 1.0, 1e-9);  // 1.0 + 1.0*(0-18) < 0.1 -> r0
}

// 生成ヘッダの係数が変わっても成り立つ性質: 解いた速度で撃てば、目標距離に戻る
TEST(Solver, DefaultModelRoundTrip) {
  const auto p = default_params();
  const auto probe = solve(p, 0.0, p.voltage_ref, p.current_ref);
  const double mid = 0.5 * (probe.reach_min + probe.reach_max);
  const auto r = solve(p, mid, p.voltage_ref, p.current_ref);
  ASSERT_TRUE(r.reachable);
  EXPECT_NEAR(predict_distance(p, r.velocity, p.voltage_ref, p.current_ref), mid, 1e-3);
  EXPECT_GE(r.command, p.v_min);
  EXPECT_LE(r.command, p.v_max);
}

TEST(TopicName, ValidNames) {
  using launch_model_node::is_valid_topic_name;
  EXPECT_TRUE(is_valid_topic_name("/test/belt_launcher/target_distance"));
  EXPECT_TRUE(is_valid_topic_name("/test/belt_launcher/auto_target_velocity"));
  EXPECT_TRUE(is_valid_topic_name("target_distance"));
}

TEST(TopicName, HyphenIsRejected) {  // 今回の起動エラーの原因
  using launch_model_node::is_valid_topic_name;
  EXPECT_FALSE(is_valid_topic_name("/test/belt-launcher/target-distance"));
  EXPECT_FALSE(is_valid_topic_name("/test/belt_launcher/auto-target-velocity"));
}

TEST(TopicName, OtherInvalidNames) {
  using launch_model_node::is_valid_topic_name;
  EXPECT_FALSE(is_valid_topic_name(""));
  EXPECT_FALSE(is_valid_topic_name("/"));
  EXPECT_FALSE(is_valid_topic_name("/a//b"));
  EXPECT_FALSE(is_valid_topic_name("/a/b/"));
  EXPECT_FALSE(is_valid_topic_name("/a/1b"));
  EXPECT_FALSE(is_valid_topic_name("/a b"));
}
