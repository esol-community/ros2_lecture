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
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStatePublisherExample : public rclcpp::Node
{
public:
  JointStatePublisherExample()
  : Node("joint_state_publisher_example"), count_(0)
  {
    pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states",
      rclcpp::SystemDefaultsQoS());
    timer_ =
      this->create_wall_timer(100ms, std::bind(&JointStatePublisherExample::TimerCallback, this));
  }

private:
  void TimerCallback()
  {
    // JointState型のメッセージ
    sensor_msgs::msg::JointState joint_state;

    // タイムスタンプを格納
    const auto stamp = this->now();
    joint_state.header.stamp = stamp;

    // 各要素の配列のサイズを揃える
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.effort.resize(2);

    // joint名を格納
    joint_state.name[0] = "body1_joint";
    joint_state.name[1] = "body2_joint";

    // jointの値を格納
    joint_state.position[0] = 1.5 * std::sin(count_ / 10.0);
    joint_state.position[1] = 0.2 * (std::cos(count_ / 5.0) - 1);

    // joint_stateをpublish
    pub_joint_state_->publish(joint_state);

    // 状態をターミナルに出力
    RCLCPP_INFO(
      this->get_logger(), "%s : %4lf [rad], %s : %4lf [rad]", joint_state.name[0].c_str(),
      joint_state.position[0], joint_state.name[1].c_str(), joint_state.position[1]);

    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  uint32_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisherExample>());
  rclcpp::shutdown();
  return 0;
}
