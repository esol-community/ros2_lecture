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
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;

class SimpleActionServer : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  SimpleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("simple_action_server", options)
  {
    using namespace std::placeholders;

    // Actionサーバの作成
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
      std::bind(&SimpleActionServer::handle_cancel, this, _1),
      std::bind(&SimpleActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  // ActionサーバがGoalを受け取ったときに呼ばれる関数
  // Goalを受け取る(Accept)か、拒否する(Reject)かを決定する
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // ActionサーバがCancelを受け取ったときに呼ばれる関数
  // Cancelを受け取る(Accept)か、拒否する(Reject)かを決定する
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci>)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // ActionサーバがGoalをAcceptしたときに呼ばれる関数
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted. Start execution.");
    std::thread{std::bind(&SimpleActionServer::execute, this, _1), goal_handle}.detach();
  }

  // Actionサーバが行う処理本体の実装
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // 受け取ったgoalを取得
    const auto goal = goal_handle->get_goal();

    // feedbackを用意
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    // resultを用意
    auto result = std::make_shared<Fibonacci::Result>();

    // 1sごとに、フィボナッチ数列を計算する
    rclcpp::Rate loop_rate(1);
    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // cancelを受け取っていないかどうかを確認する
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // フィボナッチ数列を更新する
      sequence.push_back(sequence[i] + sequence[i - 1]);

      // feedbackをPublishする
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      // 結果をpublishする
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class SimpleActionServer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleActionServer>());
  rclcpp::shutdown();

  return 0;
}
