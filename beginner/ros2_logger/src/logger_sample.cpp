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

#include "rclcpp/rclcpp.hpp"

class LoggerSample : public rclcpp::Node
{
public:
  LoggerSample()
  : Node("logger_sample"), count_(0)
  {
    // タイマの設定
    // 1000ms(1秒)ごとに、print_all_log_level()関数を実行する
    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&LoggerSample::print_all_log_level, this));
  }

private:
  // カウンタ
  uint32_t count_;

  // タイマの変数
  rclcpp::TimerBase::SharedPtr timer_;

  void print_all_log_level()
  {
    // それぞれのログレベルでメッセージを表示
    RCLCPP_FATAL(this->get_logger(), "print_all_log_level FATAL: %d", count_);
    RCLCPP_ERROR(this->get_logger(), "print_all_log_level ERROR: %d", count_);
    RCLCPP_WARN(this->get_logger(), "print_all_log_level WARN: %d", count_);
    RCLCPP_INFO(this->get_logger(), "print_all_log_level INFO: %d", count_);
    RCLCPP_DEBUG(this->get_logger(), "print_all_log_level DEBUG: %d", count_);

    count_++;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggerSample>());
  rclcpp::shutdown();
  return 0;
}
