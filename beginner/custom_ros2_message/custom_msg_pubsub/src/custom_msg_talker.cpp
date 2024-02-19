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
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "custom_message/msg/custom.hpp" // 今回作成したメッセージ型に変更

using namespace std::chrono_literals;

class CustomMsgTalker : public rclcpp::Node // クラス名変更
{
public:
  CustomMsgTalker()               // クラス名に合わせてコンストラクタ名変更
  : Node("custom_msg_talker")     // ノード名をcustom_msg_talkerに変更
  {

    // 今回作成したメッセージ型を指定
    publisher_ = this->create_publisher<custom_message::msg::Custom>("chatter", 10); // 変更

    timer_ = this->create_wall_timer(1000ms, std::bind(&CustomMsgTalker::timer_callback, this));
  }

private:
  void timer_callback()
  {

    // メッセージを定義
    custom_message::msg::Custom data; // 変更

    // custom_ros2_msg::msg::Custom型の要素を初期化
    data.word = "Hello.world!";
    data.num = 10;

    // ターミナルにpublsih内容を出力
    RCLCPP_INFO(
      this->get_logger(), "publish: string/%s , num/%ld",
      data.word.c_str(), data.num);           // 変更

    // メッセージ送信
    publisher_->publish(data); // 変更
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<custom_message::msg::Custom>::SharedPtr publisher_; // メッセージ型変更
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CustomMsgTalker>()); // 変更

  rclcpp::shutdown();

  return 0;
}
