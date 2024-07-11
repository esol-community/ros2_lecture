# 実習ROS 2 Action通信

## 環境

本記事は以下の環境を想定して記述している。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

この記事では、ROSの通信方式の1つであるAction通信について、以下の内容を説明する。

- ROS（ROS 1, ROS 2）のAction通信の概念
- ROS 2におけるAction通信
- Action通信のコマンドインタフェース
- Actionサーバ、クライアントのC++での実装とその実行例

実装では、フィボナッチ数列の計算を行うアクションサーバと、それを利用するクライアントを作成する。  
この実装はROS 2のチュートリアルでの実装をベースとしている。（[参考：ROS 2の公式チュートリアル](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)）

最後に、ROSの通信方式のまとめとして、Topic, Service, Actionの使い分けに関して整理する。

### ROSのAction通信

Action通信は、Pub&Sub通信、Service通信と並ぶROSの通信方式の1つである。  
Service通信と同じくクライアント・サーバ型の通信方式であり、以下の特徴がある。  

1. 1つのAction通信の中に、以下の3つの通信を含む
   1. Goalの設定  
      クライアントがサーバに、処理の目標（ゴール）を送信する。
   2. Feedbackの送信  
      サーバが対象の処理を開始し、処理の途中経過をクライアントに送信する。
   3. Resultの送信  
      サーバが対象の処理を完了し、処理結果をクライアントに送信する。
2. 処理を中断することが可能  
   クライアントがGoalを処理中のサーバに対して処理のキャンセルを要求できる。

これらの特徴により、Action通信は長時間実行されるタスクに適した通信である。  

例えば、自律走行ソフトの[Navigation2](https://docs.nav2.org/)では、目標地点までの走行という1つのタスクに対するActionメッセージ型が定義されている。  
この場合、Actionを示す型のGoal、Result、Feedbackはそれぞれ以下のようになる。

- Goal：走行タスクの目標地点
- Result：走行結果やエラーコード
- Feedback：走行途中のロボットのポーズ

### ROS 2のAction通信

ROS 2では、1つのAction通信は内部的に以下の3つの通信の組み合わせで実装される。  

- Goal Service: Goalの設定に関わるService通信
- Result Service: Resultの送受信に関わるService通信
- Feedback Topic: Feedbackの送信に関わるPub&Sub通信

これら3つの通信からなるAction通信の概念図を以下に示す([ROS 2公式ドキュメント](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/images/Action-SingleActionClient.gif)より引用)。

![Action-SingleActionClient](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/advanced/action_comm/./.img/Action-SingleActionClient.gif)

上図の処理の流れは、以下のようになる。

1. クライアントがサーバに、Goal Serviceのリクエストとして、処理の目標（Goal）を送信する。
2. サーバはレスポンスとして、Goalを受け取ったことをクライアントに伝える。
3. クライアントがResult Serviceのリクエストとして、処理結果をサーバに要求する。
4. サーバが対象の処理を開始し、処理の途中経過をFeedbackとしてクライアントに送信する。
5. サーバが対象の処理を完了し、Result Serviceのレスポンスとして処理結果をクライアントに送信する。

それぞれの通信の詳細等は、[ROS 2 Designのページ](http://design.ros2.org/articles/actions.html)に詳しく記載されている。

### コマンドインターフェース

`ros2 topic`コマンドや`ros2 service`コマンドと同様に、`ros2 action`コマンドが存在する。  

使えるコマンドを以下に示す。

- `ros2 action list`：利用可能なActionの名前を表示する
- `ros2 action info <action_name>`：Actionの情報（クライアント数、サーバ数、及びそれらの名称）を表示する
- `ros2 action send_goal`：ActionのGoalをサーバに送信する

実行例は、[実行結果](#実行結果)の節で示す。

## 実装

本記事ではAction通信でフィボナッチ数列の計算を行うプログラムを作成する。  
Goal, Feedback, Resultはそれぞれ以下のように定める。

1. Goalの設定：計算するフィボナッチ数列の項数N
2. Feedbackの送信：第i項（0 <= i <= N）までのフィボナッチ数列
3. Resultの送信：第N項までのフィボナッチ数列

数列の計算は一瞬で終わってしまうため、Actionサーバでは1秒に1項ずつ計算する仕様とする。

### Actionインタフェースの定義

TopicやServiceと同様に、Actionもユーザが独自にメッセージ型を定義できる。  
本記事の実装においては独自にメッセージ型を作成せずに、`example_interfaces`パッケージで定義されている型を利用する。  

[Fibonacci.action](https://github.com/ros2/example_interfaces/blob/humble/action/Fibonacci.action)の中身を以下に示す。

```txt
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] sequence
```

Actionメッセージ型の定義ファイルでは、`---`で区切られた3つの領域に、上からそれぞれGoal, Result, Feedbackのデータ型とその変数名を記述する。  
`order`は計算するフィボナッチ数列の項数、Feedbackの`sequence`が処理途中の部分的な数列、Resultの`sequence`が`order`で設定した項数の数列である。

### パッケージの作成

まず、この記事で実装するパッケージを作成する。  
任意のROS 2ワークスペースに移動したあと、以下のコマンドでパッケージ`action_comm`を作成する。  

```sh
ros2 pkg create action_comm --build-type ament_cmake --dependencies rclcpp rclcpp_action action_tutorials_interfaces
```

このコマンドでは、3つの依存パッケージを指定している。  
これらのうち、`rclcpp_action`はAction通信のサーバ、クライアントが実装されたパッケージであり、`action_tutorials_interfaces`は`Fibonacci.action`が定義されたパッケージである。

### Actionサーバの実装

フィボナッチ数列を計算するActionサーバ（`SimpleActionServer`クラス）を実装する。  
パッケージ内の`src`ディレクトリに、[simple_action_server.cpp](https://github.com/esol-community/ros2_lecture/tree/main/advanced/action_comm/./src/simple_action_server.cpp)ファイルを作成する（コード全体はリンク先を参照）。

Actionを使う場合、名前空間が長くなるため、using宣言を用いて名前を省略することが多い。  
このソースコードでも、以下のようなusing宣言を行っており、Actionメッセージ型を`Fibonacci`、ActionのGoalハンドラ型を`GoalHandleFibonacci`としている。  
`GoalHandleFibonacci`型については後述する。

```cpp
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
```

Actionサーバのオブジェクトは、以下のように作成する。  

```cpp
    using namespace std::placeholders;

    // Actionサーバの作成
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
      std::bind(&SimpleActionServer::handle_cancel, this, _1),
      std::bind(&SimpleActionServer::handle_accepted, this, _1));
```

`rclcpp_action::create_server<Fibonacci>()`関数には、順番に以下の引数を与えている。

- ノードポインタ
- Actionの名称
- Goalを受け取ったときに実行する関数
- Cancelを受け取ったときに実行する関数
- GoalがAcceptされたときに実行する関数

上記で与えている`handle_goal()`, `handle_cancel()`, `handle_accepted()`関数には、それぞれ以下のような役割がある。

- `handle_goal()`  
  Goalを受け取る（Accept）か、拒否する（Reject）かを決定する。  
  GoalをAcceptする場合は、戻り値として`rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE`を返すようにする。  
  Rejectする場合は、`rclcpp_action::GoalResponse::REJECT`を返すようにする。  

- `handle_cancel()`  
  Cancelを受け取る（Accept）か、拒否する（Reject）かを決定する。  
  CancelをAcceptする場合は、戻り値として`rclcpp_action::CancelResponse::ACCEPT`を返すようにする。  
  Rejectする場合は、`rclcpp_action::CancelResponse::REJECT`を返すようにする。  

- `handle_accepted()`  
  GoalがAcceptされたときに実行される。  
  サーバの処理本体を実装する。  
  今回の実装では、新しくスレッドを作成し、`execute()`関数を呼び出している。

`execute()`関数では、フィボナッチ数列を1秒に1項ずつ計算している。  
ここでは、`GoalHandleFibonacci`クラスのオブジェクト`goal_handle`を介して以下の処理をしている。  

- Feedbackの送信
- Resultの送信
- GoalがCancelされていないかの確認

このオブジェクトはROS 2のライブラリ内でGoalがAcceptされたときに作成されるもので、ユーザはこれを介してROS 2のAction通信を操作する。

```cpp
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
```

なお、今回の実装では使われていないが、`goal_handle->abort()`を呼ぶことでActionを中断できる。

### Actionクライアントの実装

上記のサーバに対応するActionクライアント（`SimpleActionClient`クラス）を実装する。  
パッケージ内の`src`ディレクトリに、[simple_action_client.cpp](https://github.com/esol-community/ros2_lecture/tree/main/advanced/action_comm/./src/simple_action_client.cpp)ファイルを作成する（コード全体はリンク先を参照）。

クライアントオブジェクトは、以下のように作成している。

```cpp
    // Actionクライアントの作成
    client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
```

Goalの作成と送信は、以下のように行っている。

```cpp
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
```

上記で説明したように、Action通信はGoal, Feedback, Resultの3つの通信を含むため、それぞれに対応したコールバック関数を指定する必要がある。  
これらは`rclcpp_action::Client<Fibonacci>::SendGoalOptions`オブジェクトに格納し、Goalの中身（ここでは、フィボナッチ数列の項数）とともに送信する。  

今回実装したActionサーバとクライアントの、処理とコールバックの対応は以下のようになる。

|サーバ|クライアント|
|---|---|
|`handle_goal()`が終了する|`goal_response_callback()`が実行される|
|`goal_handle->publish_feedback()`が呼ばれる|`feedback_callback()`が実行される|
|`goal_handle->canceled()`または`goal_handle->succeed()`が呼ばれる|`result_callback()`が実行される|

なお、今回の実装では使われていないが、実行中のActionをキャンセルすることができる。  
Actionをキャンセルする場合、`client_ptr_->async_cancel_goal()`関数や`client_ptr_->async_cancel_all_goals()`関数を使う。

## ビルド

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/advanced/action_comm/./CMakeLists.txt)に適切な依存関係、ビルド設定を追加する（ファイルの全体はリンク先を参照）。  

`simple_action_client.cpp`から実行ファイル`simple_action_client`、`simple_action_server.cpp`から実行ファイル`simple_action_server`を作成する。  
また、それぞれに対して`rclcpp`、`rclcpp_action`、`action_tutorials_interfaces`パッケージをリンクする。  

ワークスペースのトップディレクトリに移動し、パッケージをビルドする。

```sh
colcon build --packages-select action_comm
```

## 実行結果

以下で説明するコマンドを実行するターミナルでは、予めこのパッケージをビルドしたワークスペースのセットアップスクリプトを実行しておく。

### ノードでAction通信を行う例

1つ目のターミナルで、`simple_action_server`を起動する。

```sh
ros2 run action_comm simple_action_server
```

この時点では、ターミナルになにも表示されない。

次に、2つ目のターミナルで、`simple_action_client`を起動する。  
すると、クライアントがサーバと通信を行い、Goalの送信、Feedbackの受信と表示、Resultの表示が行われる。  

```sh
$ ros2 run action_comm simple_action_client 
[INFO] [1684987476.442527200] [simple_action_client]: Goal was sent
[INFO] [1684987476.443960254] [simple_action_client]: Goal accepted by server, waiting for result
[INFO] [1684987476.444372174] [simple_action_client]: Next number in sequence received: 0 1 1 
[INFO] [1684987477.444577968] [simple_action_client]: Next number in sequence received: 0 1 1 2 
[INFO] [1684987478.444590752] [simple_action_client]: Next number in sequence received: 0 1 1 2 3 
[INFO] [1684987479.444577238] [simple_action_client]: Next number in sequence received: 0 1 1 2 3 5 
[INFO] [1684987480.444650498] [simple_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 
[INFO] [1684987481.444594058] [simple_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 
[INFO] [1684987482.444523051] [simple_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 
[INFO] [1684987483.444585777] [simple_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 
[INFO] [1684987484.444570394] [simple_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 55 
[INFO] [1684987485.444660122] [simple_action_client]: Goal was succeeded
[INFO] [1684987485.444778999] [simple_action_client]: Result received: 0 1 1 2 3 5 8 13 21 34 55
```

このとき、Actionサーバを起動した1つ目のターミナルには処理内容が出力される。

```sh
[INFO] [1684987476.443065286] [simple_action_server]: Received goal request with order 10
[INFO] [1684987476.443452521] [simple_action_server]: Goal accepted. Start execution.
[INFO] [1684987476.443710023] [simple_action_server]: Executing goal
[INFO] [1684987476.443934938] [simple_action_server]: Publish feedback
[INFO] [1684987477.444106142] [simple_action_server]: Publish feedback
[INFO] [1684987478.444120287] [simple_action_server]: Publish feedback
[INFO] [1684987479.444102039] [simple_action_server]: Publish feedback
[INFO] [1684987480.444160206] [simple_action_server]: Publish feedback
[INFO] [1684987481.444117107] [simple_action_server]: Publish feedback
[INFO] [1684987482.444085541] [simple_action_server]: Publish feedback
[INFO] [1684987483.444125512] [simple_action_server]: Publish feedback
[INFO] [1684987484.444094283] [simple_action_server]: Publish feedback
[INFO] [1684987485.444206602] [simple_action_server]: Goal succeeded
```

### コマンドでAction通信を行う例

1つ目のターミナルで、`simple_action_server`を起動する。

```sh
ros2 run action_comm simple_action_server
```

2つ目のターミナルで、Actionの名前の一覧を確認する。

```sh
$ ros2 action list
/fibonacci
```

Actionの名前を指定し、指定したActionの情報を確認する。

```sh
$ ros2 action info /fibonacci
Action: /fibonacci
Action clients: 0
Action servers: 1
    /simple_action_server
```

`ros2 action`コマンドを用いて、Actionを実行する。

```sh
$ ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 10}" 
Waiting for an action server to become available...
Sending goal:
     order: 10

Goal accepted with ID: fa7ab00e9ebf4fe889d2d5dadbafb4cd

Result:
    sequence:
- 0
- 1
- 1
- 2
- 3
- 5
- 8
- 13
- 21
- 34
- 55

Goal finished with status: SUCCEEDED
```

最後のコマンドでは、オプション`--feedback`を指定することで、Feedbackを受け取れる。

## Topic, Service, Actionの使い分け

ROSの3つの主要な通信方式の使い分けに関するガイドラインが[ROS 2公式ドキュメント](https://docs.ros.org/en/humble/How-To-Guides/Topics-Services-Actions.html)に記載されている。  
内容を簡単にまとめると、以下のようになる。

|通信の種類|特徴|ユースケース|
|---|---|---|
|Topic|連続的なデータの流れの表現、多対多の通信が可能|センサデータ、ロボットの状態|
|Service|短時間で終了する処理の呼び出しに向き、長時間動作する処理には向かない|ノードの状態の取得|
|Action|複雑なタスクや、長時間の動作が必要でFeedbackがほしい処理に向く|走行タスク、ロボットアームの制御タスク|

## 参考

- [ROS講座95 actionlibを使う](https://qiita.com/srs/items/a39dcd24aaeb03216026)
- [ROS 2公式ドキュメント：Understanding Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [ROS 2公式ドキュメント：Writing an action server and client (C++)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [ROS 2公式ドキュメント：Creating an Action](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html)
- [ROS 2公式ドキュメント：Topics vs Services vs Actions](https://docs.ros.org/en/humble/How-To-Guides/Topics-Services-Actions.html)
- [ROS 2 Design: Actions](http://design.ros2.org/articles/actions.html)
- [ros2/examplesリポジトリ actions](https://github.com/ros2/examples/tree/humble/rclcpp/actions)
- [ros/demosリポジトリ action_tutorials](https://github.com/ros2/demos/tree/humble/action_tutorials)
- [rclcpp_action Documentation](http://docs.ros.org/en/humble/p/rclcpp_action/generated/index.html)
