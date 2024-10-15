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
// C++標準ライブラリ
#include <memory>

// ROSのC++クライアントライブラリ
#include "rclcpp/rclcpp.hpp"
// 文字列のメッセージ型
#include "std_msgs/msg/string.hpp"

// 省略表記
using std::placeholders::_1;

// rclcpp::Nodeを継承したクラスSimpleListenerの宣言
class SimpleListener : public rclcpp::Node
{
public:
  // コンストラクタ
  SimpleListener()
  : Node("simple_listener")   // ノード名をsimple_listenerで初期化
  {
    // subscriberの作成
    // 第一引数はトピック名
    // 第二引数はメッセージのバッファサイズ、
    // 第三引数は受信したときに呼ばれる関数
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&SimpleListener::chatter_callback, this, _1)
    );
  }

private:
  // トピックを受信したときに呼ばれるコールバック関数
  void chatter_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    // ターミナルへの文字列出力
    RCLCPP_INFO(this->get_logger(), "subscribe: %s", msg->data.c_str());
  }

  // std_msgs::msg::String型のトピックを受信するsubscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // rcl(ros client library)の初期化
  rclcpp::init(argc, argv);

  // ノードを作成し、スピン（実行）
  rclcpp::spin(std::make_shared<SimpleListener>());

  // rclの終了
  rclcpp::shutdown();

  return 0;
}
