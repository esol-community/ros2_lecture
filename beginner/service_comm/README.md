# 実習ROS 2 Service通信

## 環境

本記事は以下の環境を想定した記事である。

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

このページでは、ROS(ROS 1、ROS 2共通)の基本となる通信手段であるService通信について説明し、それをROS 2で実装する。  
このページの内容はROS 2公式チュートリアルをベースとする。（[ROS 2公式チュートリアル](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)）  
公式チュートリアルのサンプルコードはmain関数で処理の大半を実装しているが、本記事のコードではクラスの中に通信処理を実装する。

## 本記事で解説していること

- ROSのService通信の概念
- clientのソースコード(C++)
- serverのソースコード(C++)

この記事では、`example_interfaces/srv/AddTwoInts`型のサービスによるノード間通信をC++で実装する。このサービスは、2つの数値をリクエストとして受け取り、その和をレスポンスとして返すという簡単なサービスである。

## Service通信

Service通信はROSのノード間の通信方法の1つである。Service通信はリクエスト・レスポンス型の通信方法であり、[Pub&Sub通信](https://github.com/esol-community/ros2_lecture/tree/main/beginner/pub_sub_comm/README.md)とは通信方法が異なる。  
Service通信では、clientがある処理のリクエストを送信する。serverはそれを受け取り、処理を行った結果をレスポンスとして返す。clientはこの結果を受け取ることで、通信が成立する。  
この概念図を以下に示す。

![sevice_comm](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/service_comm/img/service_comm.png)

トピックと同様に、サービスにもメッセージ型がある。Service通信はメッセージ型が一致している場合にのみ成立する。  

## サービス定義ファイル

サービスのリクエストとレスポンスには様々なデータを含められる。ROS 2のデータ型を組み合わせることで、独自のサービス型を実装できる。  
[今回用いるサービス定義ファイル](https://github.com/ros2/example_interfaces/blob/humble/srv/AddTwoInts.srv)は、以下のようになる。

```example_interfaces/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

`---`の上段はリクエスト、下段はレスポンスのデータである。  
`int64`はROS 2のデータ型である。  
今回作成するService通信では、整数値aとbの値がリクエストに含まれ、レスポンスにはそれらの和であるsumが入る。  

メッセージ型の説明と、独自のメッセージ型を作成して利用する方法については、「実習ROS 2 ROS 2メッセージ」、「実習ROS 2 カスタムROSメッセージ」で解説する。

## 前準備

ROS 2のプログラムを作成するためには、[Pub&Sub通信](https://github.com/esol-community/ros2_lecture/tree/main/beginner/pub_sub_comm/README.md)のときと同様の前準備が必要である。  
ここでは、`~/ros2_lecture_ws`がROS 2のワークスペースであり、パッケージ名が`service_comm`であると仮定して説明する。パッケージは以下のコマンドで作成したと仮定する。

```sh
ros2 pkg create --build-type ament_cmake service_comm --dependencies rclcpp example_interfaces
```

`--dependencies`オプションによって、パッケージを作る時点で依存パッケージを指定できる。  
`service_comm`パッケージは`AddTwoInts`サービスのメッセージ型を使用する。  
そのため、この型が定義されている`example_interfaces`パッケージを依存パッケージとして指定する。

## ソースファイルの作成

Service通信を行うプログラムを作成するために、以下の4つを行う。

- clientのコーディング
- serverのコーディング
- 依存関係の設定
- CMakeLists.txtの設定

### clientのコーディング

以下のリンク先のソースコードを実装する。

- [clientのソースコード](https://github.com/esol-community/ros2_lecture/tree/main/beginner/service_comm/src/service_client.cpp)

clientは引数を2つ受け取り、その値をリクエストとしてサーバに送信し、結果を表示するプログラムである。引数のチェックと変換はmain関数で、リクエストの生成と送信、結果の表示はAddTwoIntsClientクラスで行う。  
実際にサービスクライアントを生成しているのは、以下の部分である。

```cpp
    // サービスクライアントの生成
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
```

リクエストを送信する処理は、送信後にレスポンスを待ち続けて処理が止まらないように、非同期(async)で行う。

```cpp
    // リクエストを送信
    auto result = client_->async_send_request(request);
```

### serverのコーディング

以下のリンク先のソースコードを実装する。

- [serverのソースコード](https://github.com/esol-community/ros2_lecture/tree/main/beginner/service_comm/src/service_server.cpp)

serverは[Pub&Sub通信におけるsubscriberのコード](https://github.com/esol-community/ros2_lecture/tree/main/beginner/pub_sub_comm/src/simple_listener.cpp)と実装が似ている。subscriberの代わりに、サービスサーバを示す変数によって、サーバを実現する。  
サービスサーバを生成している場所は以下の場所である。リクエストを受け取ったときに実行されるコールバック関数として、`AddTwoIntsServer::add`関数を指定する。

```cpp
    // サービスサーバの生成
    server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&AddTwoIntsServer::add, this, std::placeholders::_1, std::placeholders::_2));
```

サービス本体の処理になる、2つの数値の加算を行う`AddTwoIntsServer::add`関数を以下に示す。引数にリクエストとレスポンスを持っており、これらを介してメッセージ通信を行う。

```cpp
  // リクエストを受け取ったときに呼び出されるコールバック関数の定義
  void add(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request: a = %ld, b = %ld", request->a, request->b);

    // サービスの処理本体
    response->sum = request->a + request->b;

    RCLCPP_INFO(this->get_logger(), "Sending back response: sum = %ld", response->sum);
  }
```

### 依存関係の設定

[Pub&Sub通信](https://github.com/esol-community/ros2_lecture/tree/main/beginner/pub_sub_comm/README.md)のときと同様に、`package.xml`に必要なメタデータを記述する。  
以下のリンク先のように記述する。

- [package.xmlへのリンク](https://github.com/esol-community/ros2_lecture/tree/main/beginner/service_comm/package.xml)  

### CMakeLists.txtの設定

[Pub&Sub通信](https://github.com/esol-community/ros2_lecture/tree/main/beginner/pub_sub_comm/README.md)のときと同様に、`CMakeLists.txt`に必要なビルド情報を記述する。
以下のリンク先のように記述する。

- [CMakeLists.txtへのリンク](https://github.com/esol-community/ros2_lecture/tree/main/beginner/service_comm/CMakeLists.txt)  

## ビルド

[Pub&Sub通信](https://github.com/esol-community/ros2_lecture/tree/main/beginner/pub_sub_comm/README.md)のときと同様に、以下のコマンドでビルドする。

```bash
cd ~/ros2_lecture_ws
colcon build --packages-select service_comm
```

## 実行

1つ目のターミナルで、以下を実行する。

```bash
cd ~/ros2_lecture_ws              # ワークスペースへの移動
. install/setup.bash              # ワークスペース環境のセットアップ
ros2 run service_comm add_two_ints_server    # 実行
```

2つ目のターミナルで、以下を実行する。  
引数の10と20は足し算を行う2つの数字である。

```bash
cd ~/ros2_lecture_ws
. install/setup.bash
ros2 run service_comm add_two_ints_client 10 20
```

## 結果

clientを起動してからserverを起動すると、以下のような実行結果が得られる。clientで与えた10と20という値の和である30を正しく表示できている。

```bash
# clientのターミナル
$ ros2 run service_comm add_two_ints_client 10 20 
[INFO] [1659661416.499417251] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661417.499673994] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661418.499830419] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661419.500242970] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661420.500420660] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661421.500860678] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661422.501306595] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661422.695416949] [add_two_ints_client]: Send Request.
[INFO] [1659661422.695600144] [add_two_ints_client]: Sum: 30

```

```bash
# serverのターミナル
$ ros2 run service_comm add_two_ints_server 
[INFO] [1659661422.695377444] [add_two_ints_server]: Create AddTwoInts service server.
[INFO] [1659661422.695472253] [add_two_ints_server]: Received request: a = 10, b = 20
[INFO] [1659661422.695482863] [add_two_ints_server]: Sending back response: sum = 30
```

また、serverを起動せずにclientのみを起動し、途中でctrl+Cにより実行を中断すると、以下のような結果になる。

```bash
$ ros2 run service_comm add_two_ints_client 10 20 
[INFO] [1659661641.420232901] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661642.420483827] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661643.420632357] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661644.420781304] [add_two_ints_client]: Service is not available. waiting...
[INFO] [1659661645.421282922] [add_two_ints_client]: Service is not available. waiting...
^C[INFO] [1659661645.949606772] [rclcpp]: signal_handler(signal_value=2)
```

なお、clientを引数なしで実行すると、以下のように表示されて実行が終了する。

```bash
$ ros2 run service_comm add_two_ints_client 
[INFO] [1659661670.320935396] [rclcpp]: USAGE: add_two_ints client <int> <int>
```

## 補足

- ROS 1は同期型のService通信のみが可能であったが、ROS 2では同期型に加えて非同期型のService通信も可能になった。
- 本記事では`int64_t`型を出力するための書式指定子は`%ld`としたが、正式には`PRId64`を用いて以下のように記述する。

```cpp
#include <cinttypes>
RCLCPP_INFO(this->get_logger(), "Sum: %" PRId64, result.get()->sum);
```

## 参考

- [ROS 2公式チュートリアル：Service通信](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
- [example_interfaces/srv/AddTwoInts.srvの定義ファイル](https://github.com/ros2/example_interfaces/blob/humble/srv/AddTwoInts.srv)
