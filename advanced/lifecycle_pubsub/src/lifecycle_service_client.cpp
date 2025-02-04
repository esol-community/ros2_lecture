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
#include <vector>
#include <chrono>
#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

// 通常のノードとして、サービスクライアントを示すノードを実装する
class LifecycleServiceClient : public rclcpp::Node
{
public:
  LifecycleServiceClient()
  : rclcpp::Node("lc_service_client")
  {
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      "lifecycle_talker/get_state");
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "lifecycle_talker/change_state");
  }

  // lifecycle_talkerのLifecycleを順番に変化させるデモを行う関数
  void run()
  {
    // サービス通信のタイムアウト時間
    const auto timeout = std::chrono::seconds(3);

    // 遷移させる状態間の時間
    const auto sleep_between_states = std::chrono::seconds(5);

    // 遷移させていく状態の配列
    const std::vector<uint8_t> transition_state{
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP
    };

    RCLCPP_INFO(this->get_logger(), "Start running lifecycle service client demos.");

    // transition_stateの要素の順番に、lifecycle_talkerの状態を遷移させていく
    for (const auto next_state : transition_state) {
      // 状態を遷移させる(失敗したら終了する)
      if (!this->change_state(next_state, timeout)) {
        RCLCPP_WARN(this->get_logger(), "Demo failed while changing state.");
        return;
      }
      // 現在の状態を取得する(失敗したら終了する)
      if (!this->get_state(timeout)) {
        RCLCPP_WARN(this->get_logger(), "Demo failed while getting state.");
        return;
      }

      // 次の状態に遷移させるまで待つ
      RCLCPP_INFO(this->get_logger(), "Waiting %ld seconds...", sleep_between_states.count());
      rclcpp::sleep_for(sleep_between_states);
    }

    RCLCPP_INFO(this->get_logger(), "Demo finished.");
  }

private:
  // サービス通信によって現在の状態を取得する関数
  bool get_state(std::chrono::seconds time_out_s)
  {
    if (!client_get_state_->wait_for_service(time_out_s)) {
      RCLCPP_ERROR(
        this->get_logger(), "Service is not available: %s", client_get_state_->get_service_name());
      return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    auto result = client_get_state_->async_send_request(request);

    auto return_code = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result
    );

    if (return_code == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(
        this->get_logger(), "Success to get current state. Current state: %s",
        result.get()->current_state.label.c_str());
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get current state.");
      return false;
    }
  }

  // サービス通信によって状態を変化させる関数
  bool change_state(uint8_t transition_id, std::chrono::seconds time_out_s)
  {
    if (!client_change_state_->wait_for_service(time_out_s)) {
      RCLCPP_ERROR(
        this->get_logger(), "Service is not available: %s",
        client_change_state_->get_service_name());
      return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    auto result = client_change_state_->async_send_request(request);

    auto return_code = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result
    );

    if (return_code == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Transition succeeded.");
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Transition failed.");
      return false;
    }
  }

  // サービスクライアントの宣言
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LifecycleServiceClient>();
  node->run();

  rclcpp::shutdown();
  return 0;
}
