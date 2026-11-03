// launch_model_node
//   /test/belt_launcher/target_distance      (std_msgs/Float32)  目標着弾距離 [m]
//     -> /test/belt_launcher/auto_target_velocity (std_msgs/Float32)  ベルト直動に指令する目標速度 [m/s]
//
// 計算は launch_model.py が出力した式 (launch_solver.hpp)。係数は YAML パラメータで上書きでき、
// 起動中でも `ros2 param set` で変えられる (再ビルド不要)。
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "launch_model_node/launch_solver.hpp"
#include "launch_model_node/topic_name.hpp"

namespace {

using launch_model_node::ModelParams;
using Field = double ModelParams::*;

struct FieldDef {
  const char* name;
  Field field;
};

// YAML / ros2 param のパラメータ名 <-> ModelParams のメンバ
const std::vector<FieldDef>& field_defs() {
  static const std::vector<FieldDef> defs = {
      {"coef_c0", &ModelParams::c0},
      {"coef_v", &ModelParams::cv},
      {"coef_v2", &ModelParams::cv2},
      {"coef_V", &ModelParams::cV},
      {"coef_I", &ModelParams::cI},
      {"v_min", &ModelParams::v_min},
      {"v_max", &ModelParams::v_max},
      {"voltage_ref", &ModelParams::voltage_ref},
      {"current_ref", &ModelParams::current_ref},
      {"train_v_min", &ModelParams::train_v_min},
      {"train_v_max", &ModelParams::train_v_max},
      {"error_band_90", &ModelParams::error_band_90},
      {"ratio_r0", &ModelParams::ratio_r0},
      {"ratio_r1", &ModelParams::ratio_r1},
      {"ratio_v_ref", &ModelParams::ratio_v_ref},
      {"launcher_x", &ModelParams::launcher_x},
  };
  return defs;
}

}  // namespace

class LaunchModelNode : public rclcpp::Node {
 public:
  LaunchModelNode() : rclcpp::Node("launch_model_node") {
    // --- モデルのパラメータ (既定値は生成ヘッダの値。YAML があればそれが優先される) ---
    const ModelParams defaults = launch_model_node::default_params();
    for (const auto& d : field_defs()) {
      declare_parameter(d.name, defaults.*(d.field));
    }
    // --- トピック等 ---
    declare_parameter("input_topic", std::string("/test/belt_launcher/target_distance"));
    declare_parameter("output_topic", std::string("/test/belt_launcher/auto_target_velocity"));
    declare_parameter("voltage_topic", std::string(""));  // 空 = 電圧を購読せず voltage_ref を使う
    declare_parameter("voltage_timeout_s", 2.0);          // この秒数より古い電圧は使わない
    declare_parameter("publish_if_unreachable", true);    // 届かない距離でも、最も近い速度を出す

    params_ = load_params();
    publish_if_unreachable_ = get_parameter("publish_if_unreachable").as_bool();
    voltage_timeout_s_ = get_parameter("voltage_timeout_s").as_double();

    const std::string in_topic = get_parameter("input_topic").as_string();
    const std::string out_topic = get_parameter("output_topic").as_string();
    const std::string vol_topic = get_parameter("voltage_topic").as_string();

    // トピック名が ROS 2 の規則に合うか先に確認する (ハイフン '-' などは使えない)
    const std::vector<std::pair<std::string, std::string>> topics = {
        {"input_topic", in_topic}, {"output_topic", out_topic}, {"voltage_topic", vol_topic}};
    for (const auto& t : topics) {
      if (t.first == "voltage_topic" && t.second.empty()) continue;
      if (!launch_model_node::is_valid_topic_name(t.second)) {
        RCLCPP_ERROR(get_logger(),
                     "%s '%s' はトピック名として使えません。英数字・_ ・/ だけが使えます (ハイフン - は不可)。"
                     "例: /test/belt_launcher/target_distance",
                     t.first.c_str(), t.second.c_str());
        throw std::invalid_argument(t.first + " is not a valid ROS 2 topic name: " + t.second);
      }
    }

    pub_ = create_publisher<std_msgs::msg::Float32>(out_topic, rclcpp::QoS(10));
    sub_distance_ = create_subscription<std_msgs::msg::Float32>(
        in_topic, rclcpp::QoS(10),
        std::bind(&LaunchModelNode::on_distance, this, std::placeholders::_1));
    if (!vol_topic.empty()) {
      sub_voltage_ = create_subscription<std_msgs::msg::Float32>(
          vol_topic, rclcpp::QoS(10),
          std::bind(&LaunchModelNode::on_voltage, this, std::placeholders::_1));
    }

    // ros2 param set で係数を変えたとき、その場で反映する
    param_cb_ = add_on_set_parameters_callback(
        std::bind(&LaunchModelNode::on_set_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "in: %s  ->  out: %s", in_topic.c_str(), out_topic.c_str());
    RCLCPP_INFO(get_logger(), "voltage: %s", vol_topic.empty() ? "(購読なし。voltage_ref を使用)" : vol_topic.c_str());
    RCLCPP_INFO(get_logger(),
                "model: x = %.4g + %.4g*v + %.4g*v^2 + %.4g*V + %.4g*I   (90%%範囲 ±%.2f m, 学習 %.2f〜%.2f m/s)",
                params_.c0, params_.cv, params_.cv2, params_.cV, params_.cI, params_.error_band_90,
                params_.train_v_min, params_.train_v_max);
  }

 private:
  ModelParams load_params() {
    ModelParams p = launch_model_node::default_params();
    for (const auto& d : field_defs()) {
      p.*(d.field) = get_parameter(d.name).as_double();
    }
    return p;
  }

  ModelParams snapshot() {
    std::lock_guard<std::mutex> lk(mtx_);
    return params_;
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    ModelParams next = snapshot();
    bool publish_if_unreachable = publish_if_unreachable_;
    double voltage_timeout_s = voltage_timeout_s_;
    for (const auto& p : params) {
      for (const auto& d : field_defs()) {
        if (p.get_name() == d.name) {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason = std::string(d.name) + " は double で指定してください (例: 1.0)";
            return result;
          }
          next.*(d.field) = p.as_double();
        }
      }
      if (p.get_name() == "publish_if_unreachable") publish_if_unreachable = p.as_bool();
      if (p.get_name() == "voltage_timeout_s") voltage_timeout_s = p.as_double();
    }
    if (!(next.v_min < next.v_max)) {
      result.successful = false;
      result.reason = "v_min < v_max である必要があります";
      return result;
    }
    {
      std::lock_guard<std::mutex> lk(mtx_);
      params_ = next;
    }
    publish_if_unreachable_ = publish_if_unreachable;
    voltage_timeout_s_ = voltage_timeout_s;
    return result;
  }

  void on_voltage(const std_msgs::msg::Float32& msg) {
    if (!std::isfinite(msg.data)) return;
    last_voltage_ = msg.data;
    last_voltage_time_ = now();
    has_voltage_ = true;
  }

  double current_voltage(const ModelParams& p) {
    if (has_voltage_ && (now() - last_voltage_time_).seconds() <= voltage_timeout_s_) {
      return last_voltage_;
    }
    return p.voltage_ref;
  }

  void on_distance(const std_msgs::msg::Float32& msg) {
    const double distance = msg.data;
    if (!std::isfinite(distance)) {
      RCLCPP_WARN(get_logger(), "目標距離が不正な値のため無視しました");
      return;
    }
    const ModelParams p = snapshot();
    const double voltage = current_voltage(p);
    const auto r = launch_model_node::solve(p, distance, voltage, p.current_ref);

    if (!r.reachable) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "目標 %.2f m は範囲内(%.2f〜%.2f m/s)では届きません (届くのは %.2f〜%.2f m)。最も近い %.2f m/s %s",
          distance, p.v_min, p.v_max, r.reach_min, r.reach_max, r.command,
          publish_if_unreachable_ ? "を出力します" : "ですが出力しません");
      if (!publish_if_unreachable_) return;
    }
    if (r.extrapolated) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "必要な実速度 %.2f m/s は学習範囲(%.2f〜%.2f m/s)の外です。外挿のため精度が落ちます",
          r.velocity, p.train_v_min, p.train_v_max);
    }

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(r.command);
    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "target %.2f m -> 実速度 %.3f m/s -> 指令 %.3f m/s%s  (電圧 %.2f V, 予測誤差の目安 ±%.2f m)",
        distance, r.velocity, r.command, r.clamped ? " [範囲に丸めた]" : "", voltage, p.error_band_90);
  }

  std::mutex mtx_;
  ModelParams params_{};
  bool publish_if_unreachable_{true};
  double voltage_timeout_s_{2.0};

  bool has_voltage_{false};
  double last_voltage_{0.0};
  rclcpp::Time last_voltage_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_distance_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_voltage_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaunchModelNode>());
  rclcpp::shutdown();
  return 0;
}
