# 実習ROS 2 Lifecycleノード

## 環境

本記事は以下の環境を想定して記述している。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

ROS 2ではLifecycleという概念を持つノードを実装できる。  
Lifecycleノードは状態と状態間の遷移を持つノードのことであり、これにより複雑で高機能なアプリケーションを実現できる。例えば、ノードを実行中のプロセスを落とすことなく、休止中のノードと活動中のノードを区別できるようになる。

この記事ではLifecycleノードについて、以下の内容を扱い、実行結果を示す。

- ROS 2のLifecycleの概念
- Lifecycleを持つPublisherノードと、対応するSubscriberノードの実装
- Lifecycleを遷移させるサービスクライアントの実装
- Lifecycleに関するCLIツール

## Lifecycleノード

Lifecycleは状態遷移モデルとして扱える。  
概念図（[ROS 2 Design](http://design.ros2.org/articles/node_lifecycle.html)より引用）を以下に示す。  

![state_machine_lifecycle_node](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/advanced/lifecycle_pubsub/./img/life_cycle_sm.png)  

Lifecycleノードには、4つの主要状態と6つの遷移状態がある。

### 主要状態(Primary States, 図中の青色の状態)  

名の通り、基本的な状態であり、ノードはほとんどの間これらのいずれかの状態にある。外部からのイベントが無い限り、自動的に遷移することはない（Activate状態でエラーが発生した場合を除く）。  
それぞれの状態とその説明を以下に示す。

- Unconfigured  
  ノードが生成された直後の状態である。Inactiveと異なり、処理に必要なセットアップ処理を行っていない状態である。
- Inactive  
  ノードの処理に必要な準備は完了しているが、処理は行っていない状態である。
- Active  
  ノードの処理を行っている状態である。
- Finalized  
  ノードが破棄される直前の状態である。主にデバッグ用途のために用意されている。

### 遷移状態（Transition States, 図中の黄色の状態）  

主要状態の間を遷移中の状態である。遷移中の処理が成功すると、続く状態へと自動的に遷移する。  
それぞれの状態とその説明を以下に示す。

- Configuring  
  ノードの設定、セットアップを行うための状態である。  
  `onConfigure()`関数が呼ばれる。  
- Activating  
  ノードの処理を開始するための最後の準備を行うための状態である。  
  `onActivate()`関数が呼ばれる。  
- Deactivating  
  Activating状態と対称となる状態である。  
  `onDeactivate()`関数が呼ばれる。
- CleaningUp  
  ノードの設定をリセットし、初期化時と同じ状態(Unconfigured)にするための状態である。  
  `onCleanup()`関数が呼ばれる。
- ErrorProcessing  
  状態遷移中に生じたエラー、あるいはActivate状態時に発生したエラーに対する処理を行う状態である。  
  `onError()`関数が呼ばれる。
- ShuttingDown  
  ノードを破棄する前に行っておくべき処理を行う状態である。  
  `onShutdown()`関数が呼ばれる。

それぞれの関数が`SUCCESS`あるいは`FAILURE`を返したとき、どの状態に遷移するかは、上の状態遷移図を参照いただきたい。  

なお、ここで示した関数名はLifecycleのモデリングにおけるものであり、実装上は異なる命名表記を持つことがある。  
例えば、`onConfigure()`はrclcpp, rclpyではともに`on_configure()`として実装される。

### LifecycleノードのAPI

通常のノードにおいては、ノードを表現するクラス（rclcppでは`rclcpp::Node`）を継承して実装していた。  
Lifecycleノードでは、上記の概念をインターフェースとしてもつクラス(`rclcpp_lifecycle::LifecycleNode`)を継承し、インタフェースをオーバーライドすることで実装する。  

また、LifecycleはROS 2の標準機能であるため、コマンドラインツールでLifecycleノードに関する設定、情報の取得などを行うことができる。  

Lifecycleの遷移はコマンドラインツールから行えるほか、サービス通信としてコード中から行うこともできる。

具体的な方法は以降で示す。

## 実装

この記事では、`lifecycle_pubsub`というパッケージを作成する。  
任意のワークスペース下の`src`ディレクトリで、以下のコマンドを実行し、パッケージを作成する。　　

```sh
ros2 pkg create lifecycle_pubsub --build-type ament_cmake
```

### Lifecycle Publisherの実装

`lifecycle_talker`という名前のノードを実装する。  
このノードは、Activate状態のときのみ、1sごとに`hello, world!`という文字列をデータとして持つ`std_msgs/msg/String`型のメッセージ（トピック名：`lc_chatter`）をpublishする。

パッケージ内の`src`ディレクトリに[lifecycle_talker.cpp](https://github.com/esol-community/ros2_lecture/tree/main/advanced/lifecycle_pubsub/./src/lifecycle_talker.cpp)を作成し、実装する(ソースコードの全体はリンク先を参照)。  

これまでのノードの実装では、`rclcpp::Node`を継承していたが、Lifecycleノードの実装では`rclcpp_lifecycle::LifecycleNode`を継承する。

```cpp
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
```

また、Publisherも、Lifecycleに対応したPublisherを用いる。

```cpp
rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr lifecycle_pub_;
```

`rclcpp_lifecycle::LifecycleNode`は以下の関数を持っており、これらの関数の中をオーバーライドすることで、Lifecycle遷移時の処理を実装する。  
これらの関数の戻り値が、上図で示した状態遷移図の`SUCCESS`、`FAILURE`、`ERROR`に対応し、戻り値に応じてLifecycleノードの状態が遷移する。  

```cpp
// configuring状態のときに呼ばれる関数
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_configure(const rclcpp_lifecycle::State & state);

// activating状態のときに呼ばれる関数
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_activate(const rclcpp_lifecycle::State & state);

// deactivating状態のときに呼ばれる関数
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_deactivate(const rclcpp_lifecycle::State & state);

// cleaning up状態のときに呼ばれる関数
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_cleanup(const rclcpp_lifecycle::State & state);

// error processing状態のときに呼ばれる関数
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_error(const rclcpp_lifecycle::State & state);

// shutting down状態のときに呼ばれる関数
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_shutdown(const rclcpp_lifecycle::State & state);
```

今回の実装では、`on_confiture()`でPublisherとタイマを作成し、`on_cleanup()`でそれらを破棄する処理をしている。

### Listenerの実装

`lifecycle_talker`ノードのLifecycleが遷移したとき、`lifecycle_talker/transition_event`トピックが自動的にPublishされる。他のノードはこれをSubscribeすることで遷移を知ることができる。  

そこで、`lifecycle_talker/transition_event`トピックと、`lc_chatter`トピックをSubscribeし、その情報を表示する`lifecycle_listener`という名前のノードを実装する。これは通常のノードとして実装する。  

パッケージ内の`src`ディレクトリに[lifecycle_listener.cpp](https://github.com/esol-community/ros2_lecture/tree/main/advanced/lifecycle_pubsub/./src/lifecycle_listener.cpp)を作成し、実装する(ソースコードの全体はリンク先を参照)。  

Lifecycleの状態遷移を示すメッセージは`lifecycle_msgs::msg::TransitionEvent`型であるので、Subscriberの宣言は以下のようになる。

```cpp
rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr sub_state_transition_;
```

### Lifecycleサービスクライアントの実装

パッケージ内の`src`ディレクトリに[lifecycle_service_client.cpp](https://github.com/esol-community/ros2_lecture/tree/main/advanced/lifecycle_pubsub/./src/lifecycle_service_client.cpp)を作成し、サービス通信のクライアントとなるノードを実装する(ソースコードの全体はリンク先を参照)。  
このノードでは、サービス通信によってサーバ（上記で実装した`lifecycle_talker`ノード）のLifecycleを取得、遷移させる。  

このノードでは、5秒ごとにサービス通信によって以下の遷移を順番に起こすようにする(遷移は上図の状態遷移図の青色の線に相当)。すなわち、起動直後のUnconfigured状態のノードを、Inactive→Active→Inactive→Unconfiguredの順に遷移させる。

このメインの処理を行っている関数は`run()`関数である。この関数を以下に示す。

```cpp
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
        return;
      }
      // 現在の状態を取得する(失敗したら終了する)
      if (!this->get_state(timeout)) {
        return;
      }

      // 次の状態に遷移させるまで待つ
      RCLCPP_INFO(this->get_logger(), "Waiting %ld seconds...", sleep_between_states.count());
      rclcpp::sleep_for(sleep_between_states);
    }

    RCLCPP_INFO(this->get_logger(), "Demo finished.");
  }
```

現在のLifecycleの取得は`get_state()`関数（サービス名：`lifecycle_talker/get_state`）で行っている。  
また、現在のLifecycleの遷移は`change_state()`関数（サービス名：`lifecycle_talker/change_state`）で行っている。  
サービス通信の処理部分は[Service通信](https://github.com/esol-community/ros2_lecture/tree/main/advanced/lifecycle_pubsub/../../beginner/service_comm/README.md)と同様なので、説明は省略する。  

### ビルド

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/advanced/lifecycle_pubsub/./CMakeLists.txt)と[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/advanced/lifecycle_pubsub/./package.xml)に適切な依存関係、ビルド設定を追加する。  
Lifecycleノードの作成のために`rclcpp_lifecycle`パッケージ、Lifecycle関連のメッセージのために`lifecycle_msgs`パッケージへの依存の設定が必要である。  
ファイルの全体はリンク先を参照。

ワークスペースのトップディレクトリに移動し、パッケージをビルドする。

```sh
colcon build --packages-up-to lifecycle_pubsub
```

## 実行結果

以下で説明するコマンドを実行するターミナルでは、予めこのパッケージをビルドしたワークスペースのセットアップスクリプトを実行しておく。

### コマンドラインでのLifecycleの遷移

Lifecycleノードに関する様々な情報を取得、設定する`ros2 lifecycle`コマンドの実行結果を示す。  

1つ目のターミナルで、`lifecycle_talker`を起動する。

```sh
$ ros2 run lifecycle_pubsub lifecycle_talker 
[INFO] [1666751295.138647186] [lifecycle_talker]: LifecycleTalker Constructor.
```

2つ目のターミナルで、`lifecycle_listener`を起動する。

```sh
$ ros2 run lifecycle_pubsub lifecycle_listener 
[INFO] [1666751805.586321149] [lifecycle_listener]: LifecycleListener Constructor.
```

3つ目のターミナルで、`ros2 lifecycle`コマンドを実行し、結果を確認する。

- Lifecycleノードの一覧取得

```sh
$ ros2 lifecycle nodes
/lifecycle_talker
```

- Lifecycleの取得

```sh
$ ros2 lifecycle get /lifecycle_talker 
unconfigured [1]    # 起動直後なのでUnconfigured状態。数字はID
```

- 遷移可能なLifecycleの取得

```sh
$ ros2 lifecycle list /lifecycle_talker 
- configure [1]
        Start: unconfigured
        Goal: configuring
- shutdown [5]
        Start: unconfigured
        Goal: shuttingdown
```

- Lifecycleの遷移

```sh
$ ros2 lifecycle set  /lifecycle_talker configure
Transitioning successful
$ ros2 lifecycle set  /lifecycle_talker activate
Transitioning successful
```

この後に1つ目のターミナルを見ると、メッセージから`on_configure()`や``on_activate()``が呼ばれていることが分かる。  
また、`lifecycle_talker`ノードの状態が遷移していることがわかる。  

```sh
$ ros2 run lifecycle_pubsub lifecycle_talker 
[INFO] [1666751868.689904094] [lifecycle_talker]: LifecycleTalker Constructor.
[INFO] [1666751875.568714679] [lifecycle_talker]: on_configure() called. Previous state: unconfigured
[INFO] [1666751876.569406530] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666751877.569303078] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666751878.569395552] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666751879.569388646] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666751880.569367129] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666751881.569379574] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666751882.569352665] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666751883.569325739] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666751884.569247393] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666751885.181401785] [lifecycle_talker]: on_activate() called. Previous state: inactive
[INFO] [1666751885.569292323] [lifecycle_talker]: Lifecycle publisher is activated.
[INFO] [1666751885.569421451] [lifecycle_talker]: publish: hello, world!
[INFO] [1666751886.569306971] [lifecycle_talker]: Lifecycle publisher is activated.
[INFO] [1666751886.569458669] [lifecycle_talker]: publish: hello, world!
[INFO] [1666751887.569291133] [lifecycle_talker]: Lifecycle publisher is activated.
```

また、2つ目のターミナルでは、文字列`hello, world!`とともに、Lifecycleの遷移をトピックとして受け取れていることが確認できる。

```sh
$ ros2 run lifecycle_pubsub lifecycle_listener 
[INFO] [1666751871.628474089] [lifecycle_listener]: LifecycleListener Constructor.
[INFO] [1666751875.568893251] [lifecycle_listener]: Subscribe transition event. 
[INFO] [1666751875.568926856] [lifecycle_listener]: Start State: unconfigured  --> Goal State: configuring
[INFO] [1666751875.569188110] [lifecycle_listener]: Subscribe transition event. 
[INFO] [1666751875.569205453] [lifecycle_listener]: Start State: configuring  --> Goal State: inactive
[INFO] [1666751885.181578069] [lifecycle_listener]: Subscribe transition event. 
[INFO] [1666751885.181602936] [lifecycle_listener]: Start State: inactive  --> Goal State: activating
[INFO] [1666751885.181724712] [lifecycle_listener]: Subscribe transition event. 
[INFO] [1666751885.181745087] [lifecycle_listener]: Start State: activating  --> Goal State: active
[INFO] [1666751885.570007112] [lifecycle_listener]: Subscribe message. data: hello, world!
[INFO] [1666751886.569984614] [lifecycle_listener]: Subscribe message. data: hello, world!
[INFO] [1666751887.569941370] [lifecycle_listener]: Subscribe message. data: hello, world!
```

### サービス通信によるLifecycleの遷移

1つ目のターミナルで、`Lifecycle_talker`を起動する。

```sh
$ ros2 run lifecycle_pubsub lifecycle_talker 
[INFO] [1666752056.733023340] [lifecycle_talker]: LifecycleTalker Constructor.
```

2つ目のターミナルで、`lifecycle_service_client`を起動する。  
5秒ごとにサービス通信を行い、Lifecycleを遷移、状態を取得できていることがわかる。

```sh
 ros2 run lifecycle_pubsub lifecycle_service_client 
[INFO] [1666752150.786165100] [lc_service_client]: Start running lifecycle service client demos.
[INFO] [1666752150.857568721] [lc_service_client]: Transition succeeded.
[INFO] [1666752150.858000745] [lc_service_client]: Success to get current state. Current state: inactive
[INFO] [1666752150.858065866] [lc_service_client]: Waiting 5 seconds...
[INFO] [1666752155.859475549] [lc_service_client]: Transition succeeded.
[INFO] [1666752155.860557304] [lc_service_client]: Success to get current state. Current state: active
[INFO] [1666752155.860693179] [lc_service_client]: Waiting 5 seconds...
[INFO] [1666752160.862127866] [lc_service_client]: Transition succeeded.
[INFO] [1666752160.863198507] [lc_service_client]: Success to get current state. Current state: inactive
[INFO] [1666752160.863342792] [lc_service_client]: Waiting 5 seconds...
[INFO] [1666752165.865516751] [lc_service_client]: Transition succeeded.
[INFO] [1666752165.866409810] [lc_service_client]: Success to get current state. Current state: unconfigured
[INFO] [1666752165.866521900] [lc_service_client]: Waiting 5 seconds...
[INFO] [1666752170.866729480] [lc_service_client]: Demo finished.
```

この後、1つ目のターミナルを見ると、サービス通信によって`lifecycle_talker`の状態が遷移していることがわかる。

```sh
$ ros2 run lifecycle_pubsub lifecycle_talker 
[INFO] [1666752056.733023340] [lifecycle_talker]: LifecycleTalker Constructor.
[INFO] [1666752150.856489201] [lifecycle_talker]: on_configure() called. Previous state: unconfigured
[INFO] [1666752151.857480000] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666752152.857425365] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666752153.857445424] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666752154.857462614] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666752155.857456989] [lifecycle_talker]: Lifecycle publisher is NOT activated.
[INFO] [1666752155.858798145] [lifecycle_talker]: on_activate() called. Previous state: inactive
[INFO] [1666752156.857471408] [lifecycle_talker]: Lifecycle publisher is activated.
[INFO] [1666752156.857612911] [lifecycle_talker]: publish: hello, world!
[INFO] [1666752157.857436322] [lifecycle_talker]: Lifecycle publisher is activated.
[INFO] [1666752157.857563215] [lifecycle_talker]: publish: hello, world!
[INFO] [1666752158.857453528] [lifecycle_talker]: Lifecycle publisher is activated.
[INFO] [1666752158.857580258] [lifecycle_talker]: publish: hello, world!
[INFO] [1666752159.857456205] [lifecycle_talker]: Lifecycle publisher is activated.
[INFO] [1666752159.857580554] [lifecycle_talker]: publish: hello, world!
[INFO] [1666752160.857505734] [lifecycle_talker]: Lifecycle publisher is activated.
[INFO] [1666752160.857645284] [lifecycle_talker]: publish: hello, world!
[INFO] [1666752160.861420573] [lifecycle_talker]: on_deactivate() called. Previous state: active
[INFO] [1666752165.864124583] [lifecycle_talker]: on_cleanup() called. Previous state: inactive
```

## 参考

- [ROS 2 Design：Managed nodes](http://design.ros2.org/articles/node_lifecycle.html)
- [ros2/demosリポジトリ：lifecycle](https://github.com/ros2/demos/tree/humble/lifecycle)
- [rclcpp_lifecycle package lifecycle_node.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp)
- [lifecycle_msgs package TransitionEvent.msg](https://github.com/ros2/rcl_interfaces/blob/humble/lifecycle_msgs/msg/TransitionEvent.msg)
