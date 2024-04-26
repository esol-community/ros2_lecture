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
// C++標準ライブラリ
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROSのC++クライアントライブラリ
#include "rclcpp/rclcpp.hpp"
// 文字列のメッセージ型
#include "std_msgs/msg/string.hpp"

// 名前空間の省略表記
// std::chrono::milliseconds(100) を 100ms と表記できるようにする
using namespace std::chrono_literals;

// rclcpp::Nodeを継承したクラスParamTalkerの宣言
class ParamTalker : public rclcpp::Node
{
public:
  // コンストラクタ
  ParamTalker()
  : Node("param_talker")   // ノード名をparam_talkerで初期化
  {
    // このノードが持つ、param_talker_parameterという名前のパラメータを宣言
    // デフォルト値は"hello"
    this->declare_parameter<std::string>("param_talker_parameter", "hello");

    // publisherの生成
    // 第一引数はトピック名、第二引数はバッファサイズ
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // タイマの生成
    // 100msごとに、timer_callback()関数が呼ばれるようにする
    timer_ = this->create_wall_timer(100ms, std::bind(&ParamTalker::timer_callback, this));
  }

private:
  // タイマによって呼び出される関数
  void timer_callback()
  {
    // パラメータを取得
    this->get_parameter("param_talker_parameter", parameter_string_);

    // メッセージを定義し、中身のデータを設定
    std_msgs::msg::String msg;
    msg.data = parameter_string_;

    // ターミナルへメッセージ表示
    RCLCPP_INFO(this->get_logger(), "publish: %s", msg.data.c_str());

    // メッセージ送信
    publisher_->publish(msg);
  }

  // パラメータ文字列
  std::string parameter_string_;

  // 一定周期で処理を実行するタイマ
  rclcpp::TimerBase::SharedPtr timer_;

  // std_msgs::msg::String型のトピックを送信するpublisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  // rcl(ros client library)の初期化
  rclcpp::init(argc, argv);

  // ノードを作成し、スピン（実行）
  rclcpp::spin(std::make_shared<ParamTalker>());

  // rclの終了
  rclcpp::shutdown();

  return 0;
}
