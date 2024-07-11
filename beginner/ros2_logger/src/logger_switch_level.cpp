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

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

class SwitchLoggerLevelSample : public rclcpp::Node
{
public:
  SwitchLoggerLevelSample()
  : Node("switch_logger_level"), count_(0), logger_debug_flag_(false)
  {
    // 一定時間ごとにログ情報を出力するためのタイマ
    print_log_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&SwitchLoggerLevelSample::print_all_log_level, this));

    // 一定時間ごとにログレベルを切り替えるためのタイマ
    switch_level_timer_ =
      create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&SwitchLoggerLevelSample::change_log_level, this));
  }

private:
  // カウンタ
  uint32_t count_;

  // ログレベルがdebugであるかどうか
  bool logger_debug_flag_;

  // タイマ
  rclcpp::TimerBase::SharedPtr print_log_timer_;
  rclcpp::TimerBase::SharedPtr switch_level_timer_;

  void print_all_log_level()
  {
    RCLCPP_FATAL(this->get_logger(), "print_all_log_level FATAL: %d", count_);
    RCLCPP_ERROR(this->get_logger(), "print_all_log_level ERROR: %d", count_);
    RCLCPP_WARN(this->get_logger(), "print_all_log_level WARN: %d", count_);
    RCLCPP_INFO(this->get_logger(), "print_all_log_level INFO: %d", count_);
    RCLCPP_DEBUG(this->get_logger(), "print_all_log_level DEBUG: %d", count_);

    count_++;
  }

  void change_log_level()
  {
    enum RCUTILS_LOG_SEVERITY next_log_level;

    // 現在のログレベルから、次のログレベルを判定
    if (logger_debug_flag_) {
      next_log_level = RCUTILS_LOG_SEVERITY_INFO;
      logger_debug_flag_ = false;

      RCLCPP_INFO(this->get_logger(), "Setting log level to INFO\n");
    } else {
      next_log_level = RCUTILS_LOG_SEVERITY_DEBUG;
      logger_debug_flag_ = true;

      RCLCPP_INFO(this->get_logger(), "Setting log level to DEBUG\n");
    }

    // ログレベルを設定
    auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), next_log_level);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(this->get_logger(), "Error while setting logger level");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwitchLoggerLevelSample>());
  rclcpp::shutdown();
  return 0;
}
