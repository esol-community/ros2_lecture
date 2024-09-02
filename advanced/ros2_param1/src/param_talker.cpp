/*
 * Copyright (C) 2023 eSOL Co.,Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE FREEBSD PROJECT ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE FREEBSD PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamTalker : public rclcpp::Node
{
public:
  ParamTalker()
  : Node("param_talker")
  {
    auto desc1 = rcl_interfaces::msg::ParameterDescriptor();
    // パラメータ1の説明を追加
    desc1.description = "sample parameter 1";
    // パラメータ1は書き込み可能とする
    desc1.read_only = false;
    // パラメータ1は動的型な変換を許可する
    desc1.dynamic_typing = true;

    auto desc2 = rcl_interfaces::msg::ParameterDescriptor();
    // パラメータ2の説明を追加
    desc2.description = "sample parameter 2";
    // パラメータ2は読み込み専用とする
    desc2.read_only = true;
    // パラメータ2は動的型な変換を許可しない
    desc2.dynamic_typing = false;

    // このノードが持つパラメータparameter1, parameter2を宣言
    // デフォルト値は"foo"
    this->declare_parameter<std::string>("parameter1", "foo", desc1);
    this->declare_parameter<int32_t>("parameter2", 100, desc2);

    // タイマの生成
    // 1000msごとに、timer_callback()関数が呼ばれるようにする
    timer_ = this->create_wall_timer(1000ms, std::bind(&ParamTalker::timer_callback, this));
  }

private:
  // タイマによって呼び出される関数
  void timer_callback()
  {
    const std::string param_name("parameter1");

    // パラメータを取得
    if (this->has_parameter(param_name)) {
      this->get_parameter(param_name, param_value_);
      auto descriptor = this->describe_parameter(param_name);

      // ターミナルへ情報表示
      RCLCPP_INFO(this->get_logger(), "%-12s: %s", descriptor.name.c_str(), param_value_.c_str());
      RCLCPP_INFO(this->get_logger(), "%-12s: %d", "type", descriptor.type);
      RCLCPP_INFO(this->get_logger(), "%-12s: %s", "description", descriptor.description.c_str());
    } else {
      // パラメータが存在しない場合、warningを表示
      RCLCPP_WARN(this->get_logger(), "No declared parameter: %s", param_name.c_str());
    }
  }

  // パラメータ文字列
  std::string param_value_;

  // 一定周期で処理を実行するタイマ
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamTalker>());
  rclcpp::shutdown();

  return 0;
}
