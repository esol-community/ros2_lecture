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
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class SimpleActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  SimpleActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("simple_action_client", options)
  {
    // Actionクライアントの作成
    client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

    // タイマの作成
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SimpleActionClient::send_goal, this));
  }

  // タイマによって呼ばれる、goalを送信する関数
  void send_goal()
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    // goal optionsの作成
    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SimpleActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&SimpleActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&SimpleActionClient::result_callback, this, _1);

    // goalを送信
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(this->get_logger(), "Goal was sent");

    // Goalを送信したので、タイマを止める
    timer_->cancel();
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Actionサーバがgoalの受け取りを完了したときに実行される関数
  void goal_response_callback(GoalHandleFibonacci::SharedPtr future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  // Actionサーバからfeedbackを受け取ったときに実行される関数
  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    // feedbackの数列を表示
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  // Actionサーバからresultを返されたときに実行される関数
  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    // 結果を判定
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    // 結果を出力
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleActionClient>());
  rclcpp::shutdown();

  return 0;
}
