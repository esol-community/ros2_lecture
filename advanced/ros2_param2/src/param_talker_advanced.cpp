/*
 * Copyright (C) 2023 eSOL Co.,Ltd. All rights reserved.
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
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamTalkerAdvanced : public rclcpp::Node
{
public:
  ParamTalkerAdvanced()
  : Node("param_talker_advanced")
  {
    // パラメータの宣言
    this->declare_parameter<int64_t>("integer_param", 100);
    this->declare_parameter<std::string>("string_param", "foo");
    this->declare_parameter<std::vector<bool>>("boolean_params", std::vector<bool>{true});
    this->declare_parameter<double>("list_param.double_param", 0.0);

    // タイマの生成
    // 1000msごとに、timer_callback()関数が呼ばれるようにする
    timer_ = this->create_wall_timer(1000ms, std::bind(&ParamTalkerAdvanced::timer_callback, this));
  }

private:
  // タイマによって呼び出される関数
  void timer_callback()
  {
    // 整数型パラメータの取得と表示
    int64_t int_param;
    this->get_parameter("integer_param", int_param);
    RCLCPP_INFO(this->get_logger(), "integer_param: %ld", int_param);

    // 文字列型パラメータの取得と表示
    std::string str_param;
    this->get_parameter("string_param", str_param);
    RCLCPP_INFO(this->get_logger(), "string_param: %s", str_param.c_str());

    // 真偽型パラメータの取得と表示
    std::vector<bool> boolean_params;
    std::stringstream ss;
    std::string del = "";
    this->get_parameter("boolean_params", boolean_params);
    ss << "[";
    for (auto v : boolean_params) {
      ss << del << v;
      del = ", ";
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "boolean_params: %s", ss.str().c_str());

    // 実数型パラメータの取得と表示
    double double_param;
    this->get_parameter("list_param.double_param", double_param);
    RCLCPP_INFO(this->get_logger(), "list_param.double_param: %lf\n", double_param);
  }

  // 一定周期で処理を実行するタイマ
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamTalkerAdvanced>());
  rclcpp::shutdown();

  return 0;
}
