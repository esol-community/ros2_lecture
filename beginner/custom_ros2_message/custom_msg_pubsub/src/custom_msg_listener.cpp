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
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "custom_message/msg/custom.hpp" // 今回作成したメッセージ型に変更

using std::placeholders::_1;

class CustomMsgListener : public rclcpp::Node
{
public:
  CustomMsgListener()
  : Node("custom_msg_listener")     // 変更
  {
    subscription_ = this->create_subscription<custom_message::msg::Custom>(
      "chatter", 10, std::bind(&CustomMsgListener::chatter_callback, this, _1)   // 変更
    );
  }

private:
  void chatter_callback(const custom_message::msg::Custom::SharedPtr data) const // 変更
  {
    RCLCPP_INFO(this->get_logger(), "subscribe: %s, %ld", data->word.c_str(), data->num); // 変更
  }

  rclcpp::Subscription<custom_message::msg::Custom>::SharedPtr subscription_; // 変更
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CustomMsgListener>()); // 変更

  rclcpp::shutdown();

  return 0;
}
