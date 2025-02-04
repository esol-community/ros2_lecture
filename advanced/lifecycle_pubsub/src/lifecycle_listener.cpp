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
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "std_msgs/msg/string.hpp"

class LifecycleListener : public rclcpp::Node
{
public:
  LifecycleListener()
  : rclcpp::Node(std::string("lifecycle_listener"))
  {
    RCLCPP_INFO(this->get_logger(), "LifecycleListener Constructor.");

    // Subscriberの作成
    sub_data_ = this->create_subscription<std_msgs::msg::String>(
      "lc_chatter", rclcpp::SystemDefaultsQoS(),
      std::bind(&LifecycleListener::callbackMsg, this, std::placeholders::_1));
    sub_state_transition_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "lifecycle_talker/transition_event",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&LifecycleListener::callbackTransition, this, std::placeholders::_1));
  }

private:
  // Publisherから送られてきたデータのコールバック関数
  void callbackMsg(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Subscribe message. data: %s", msg->data.c_str());
  }

  // Publisherのライフサイクルの状態遷移トピックのコールバック関数
  void callbackTransition(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Subscribe transition event. ");
    RCLCPP_INFO(
      this->get_logger(), "Start State: %s  --> Goal State: %s",
      msg->start_state.label.c_str(), msg->goal_state.label.c_str());
  }

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_data_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr sub_state_transition_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LifecycleListener>());
  rclcpp::shutdown();
  return 0;
}
