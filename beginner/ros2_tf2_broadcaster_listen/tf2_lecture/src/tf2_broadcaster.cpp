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

// 座標情報を格納する要素を扱うメッセージ型ファイル
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// 四元数の計算を扱うファイル
#include "tf2/LinearMath/Quaternion.h"
// staticなbroadcasterを扱うファイル
#include "tf2_ros/static_transform_broadcaster.h"
// dynamicなbroadcasterを扱うファイル
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

const double PI = 3.141592653589793238463;

class Broadcaster : public rclcpp::Node
{
public:
  Broadcaster()
  : Node("static_broadcaster")
  {
    // staticなbroadcasterの生成
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // dynamicなbroadcasterの生成
    dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // staticなframeのbroadcastは一度で良いため、コンストラクタで対象関数を呼び出す。
    this->static_broadcast();

    // dynamicなframeは常にbroadcastを行う必要があるため、タイマによって対象関数を呼び出す。
    timer_ = this->create_wall_timer(
      100ms, std::bind(&Broadcaster::dynamic_broadcast_timer_callback, this));
  }

private:
  // static_frameのbroadcastをまとめた関数
  void static_broadcast()
  {
    // 座標データなどを格納するメッセージを定義
    geometry_msgs::msg::TransformStamped static_t;

    // 時間の設定
    static_t.header.stamp = this->get_clock()->now();

    // 親フレーム名の設定
    static_t.header.frame_id = "base_link";
    // 子フレーム名の設定
    static_t.child_frame_id = "static_frame";

    // 子フレームの位置の設定
    static_t.transform.translation.x = 0.0;
    static_t.transform.translation.y = 0.0;
    static_t.transform.translation.z = 1.0;

    // 子フレームの姿勢の設定
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    static_t.transform.rotation.x = q.x();
    static_t.transform.rotation.y = q.y();
    static_t.transform.rotation.z = q.z();
    static_t.transform.rotation.w = q.w();

    // broadcast
    static_broadcaster_->sendTransform(static_t);
  }

  // dynamic_frameのbroadcastをまとめた関数
  void dynamic_broadcast_timer_callback()
  {
    // static_broadcast()内と同等の処理を行っている。

    rclcpp::Time now = this->get_clock()->now();
    double r = now.seconds() * PI;

    geometry_msgs::msg::TransformStamped dynamic_t;

    dynamic_t.header.stamp = now;
    dynamic_t.header.frame_id = "base_link";
    dynamic_t.child_frame_id = "dynamic_frame";

    // /base_linkの周りを周回させるためにx軸とy軸のオフセットを常に変化させる
    dynamic_t.transform.translation.x = 1 * cos(r);
    dynamic_t.transform.translation.y = 1 * sin(r);
    dynamic_t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    dynamic_t.transform.rotation.x = q.x();
    dynamic_t.transform.rotation.y = q.y();
    dynamic_t.transform.rotation.z = q.z();
    dynamic_t.transform.rotation.w = q.w();

    // broadcast
    dynamic_broadcaster_->sendTransform(dynamic_t);
  }

  rclcpp::TimerBase::SharedPtr timer_;

  // broadcasterの宣言
  // broadcasterの種類により、使用するクラスが異なることに注意。
  // staticなbroadcasterの宣言
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  // dynamicなbroadcasterの宣言
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Broadcaster>());
  rclcpp::shutdown();
  return 0;
}
