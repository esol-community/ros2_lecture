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
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"    // サービスのメッセージ型の定義

using namespace std::chrono_literals;

// 2つの数値を加算するサービスAddTwoIntsのクライアントを示すクラスの宣言
class AddTwoIntsClient : public rclcpp::Node
{
public:
  AddTwoIntsClient()
  : Node("add_two_ints_client")
  {
    // サービスクライアントの生成
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  }

  void run(const int64_t a, const int64_t b)
  {
    // サーバが存在しているか確認する
    while (!client_->wait_for_service(1s)) {    // サーバの存在を1s待つ
      // 実行が止められた場合、エラーメッセージを表示して終了する
      if (!rclcpp::ok()) {
        return;
      }
      // サーバがない場合、ターミナルにその旨を表示する
      RCLCPP_INFO(this->get_logger(), "Service is not available. waiting...");
    }

    // リクエストを作成し、値を代入する
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    // リクエストを送信
    auto result = client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Send request.");

    // 送信したリクエストに対する結果が返ってくるまで待つ
    auto return_code = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result);

    // 正しく結果が返ってきたかどうかを判定し、メッセージを表示
    if (return_code == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Sum: %ld", result.get()->sum);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
    }
  }

private:
  // サービスクライアントの変数宣言
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 引数の個数を確認
  if (argc != 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "USAGE: add_two_ints client <int> <int>");
    return 0;
  }

  // 引数をlong long型の数値に変換
  int64_t a = (int64_t)atoll(argv[1]);
  int64_t b = (int64_t)atoll(argv[2]);

  // AddTwoIntsClientクラスのインスタンスを生成
  auto client = std::make_shared<AddTwoIntsClient>();

  // 実行
  client->run(a, b);

  rclcpp::shutdown();

  return 0;
}
