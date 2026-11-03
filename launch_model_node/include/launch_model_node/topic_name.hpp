// ROS 2 のトピック名の簡易チェック (ROS 非依存)。
// 使えるのは英数字・'_'・'/'・'~'・'{'・'}' だけ。ハイフン '-' は使えない。
#pragma once

#include <cctype>
#include <string>

namespace launch_model_node {

inline bool is_valid_topic_name(const std::string& name) {
  if (name.empty() || name == "/") return false;
  if (name.back() == '/') return false;  // 末尾の / は不可
  bool token_start = true;
  char prev = 0;
  for (const char c : name) {
    if (c == '/') {
      if (prev == '/') return false;  // "//" は不可
      token_start = true;
      prev = c;
      continue;
    }
    const auto u = static_cast<unsigned char>(c);
    const bool ok = std::isalnum(u) || c == '_' || c == '~' || c == '{' || c == '}';
    if (!ok) return false;
    if (token_start && std::isdigit(u)) return false;  // 各階層は数字で始められない
    token_start = false;
    prev = c;
  }
  return true;
}

}  // namespace launch_model_node
