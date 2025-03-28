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
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// listenerを扱うファイル
#include "tf2_ros/transform_listener.h"
// バッファを扱うファイル
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class Listener : public rclcpp::Node
{
public:
  Listener()
  : Node("listener")
  {
    // バッファの生成
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // listenerの生成
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = this->create_wall_timer(
      1s, std::bind(&Listener::on_timer, this));
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped t;

    try {
      // 座標変換結果の取得
      // 第一引数は"target frame"
      // 第二引数は"source frame"
      // 第三引数は変換するタイミング
      // tf2::TimePointZeroは変換可能な最新の時刻
      t = tf_buffer_->lookupTransform(
        "dynamic_frame", "base_link",
        tf2::TimePointZero);
    }
    // 変換を行えなかった時のための例外処理
    catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform base_link to dynamic_frame: %s",
        ex.what());
      return;
    }

    // 座標変換結果の設定
    auto & trans = t.transform.translation;

    // 座標変換結果の位置情報の出力
    RCLCPP_INFO(
      this->get_logger(), "base_link->dynamic_frame: %f %f %f", trans.x, trans.y,
      trans.z);
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  // listenerの宣言
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // バッファの宣言
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
}
