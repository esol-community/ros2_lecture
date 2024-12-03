# 実習ROS 2 QoSを設定する

## 環境

本記事は以下の環境を想定して記述されている。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

ROS 2ではPub&Sub通信とService通信に対して通信の品質（QoS: Quality of Service）を設定できる。  
QoSをカスタマイズすることで、例えば以下のような、求められる品質がそれぞれ異なる状況に対して、通信品質を使い分けられるようになる。

- 軽量なセンサデータを高速にやりとりする通信
- メッセージの到達確認を行うような同期通信
- 無線ネットワークのような損失の多い環境での通信

この記事では、QoSに関するROS 2の機能として、以下の内容を扱う。

- ROS 2におけるQoS設定について
  - QoSポリシー
  - QoSプロファイル
  - QoS互換性
- QoSに関するROS 2のCLIツール
- QoSをソースコード(C++)で設定する方法

**※補足: ROS 2とDDSの関係について**  
ROS 2のQoSの仕組みはROS 2独自のものではなく、DDS(Data Distribution Service)の仕様に由来している。
DDS(Data Distribution Service)はROS 2に依存しない通信規格であり、ノードの発見、通信内容のシリアライズ、データ転送の方法等を扱っている。  
DDS規格を実装しているベンダは複数あるが、ROS 2はこれらを抽象化するレイヤ（rmw: ROS Middleware Interface）を持っており、特定のDDS実装に依存しないようになっている。  
すなわち、ROS 2を用いてQoSを設定する場合、異なるDDS実装に対しても同じ設定方法が使える。  
なお、DDSで定義されているすべてのOoSポリシーがROS 2で利用できるわけではない。

## QoS(Quality of Service)

QoSは複数の要素から構成され、その組み合わせで決定される。  
それぞれの要素は「QoSポリシー」と呼ばれる。

### QoSポリシー

QoSポリシーは以下が存在する。  

- Historyポリシー：データサンプルを保存する数を指定する。以下の2つから選ぶ。

  |項目|説明|
  |---|---|
  |Keep last|直前のNサンプルを保存する(NはDepthポリシーで指定する)|
  |Keep all|全てを保存する（ミドルウェアの限界まで）|

- Depthポリシー：メッセージを受信するときのキューサイズを指定する。Historyポリシーが"Keep last"のときのみ有効である。

  |項目|説明|
  |---|---|
  |Queue size|サンプルを保存する数|

- Reliabilityポリシー: 通信の信頼性を指定する。以下の2つから選ぶ。

  |項目|説明|
  |---|---|
  |Best effort|ネットワークの状況により、サンプルが失われる場合がある|
  |Reliable|サンプルが確実に届くことを保証する|

- Durabilityポリシー: サンプルの持続性を指定する。以下の2つから選ぶ。

  |項目|説明|
  |---|---|
  |Transient local|サンプルは保持されており、Publishを始めた後に参加したSubscriberに対してもデータが届く|
  |Volatile|サンプルをPublishした後にSubscribeを始めたノードはそのサンプルを受信できない|

- Deadlineポリシー: 複数のメッセージ間の最大時間間隔を指定する。この期間を過ぎた場合、そのことを示すイベントが発生する。このイベントハンドラをROSユーザが記述することで、エラーを表示できる（[実装例：quality_service_demo deadline.cpp](https://github.com/ros2/demos/blob/humble/quality_of_service_demo/rclcpp/src/deadline.cpp)）。

  |項目|説明|
  |---|---|
  |Duration|メッセージ間の最大時間|

- Lifespanポリシー: メッセージが有効であることを示す期間を指定する。この期間を過ぎているメッセージは破棄され、受信側でデータを取得できない。

  |項目|説明|
  |---|---|
  |Duration|メッセージの有効期間|

- Livelinessポリシー: メッセージのPublisherが有効であることを確認する方法を指定する。以下の2つから選ぶ。

  |項目|説明|
  |---|---|
  |Automatic|いずれかのメッセージがPublishされると、全てのPublisherが有効であると判断する|
  |Manual by topic|トピックごとにPublisherの有効性を判断する|

- Lease Durationポリシー: LivelinessポリシーでPublisherが有効であるとみなされる時間を指定する。

  |項目|説明|
  |---|---|
  |Duration|Publisherが有効である時間|

### QoSプロファイル

開発者がそれぞれのQoSポリシーを設定するのは手間がかかるため、予め決められたQoSポリシーの組み合わせが「QoSプロファイル」として定義されている。  
QoSプロファイルには、以下のものがある。

- Profile Default  
  ROS 1の通信に近くなるような設定であり、Pub&Sub通信を想定している。  
  ReliabilityポリシーがReliable、キューサイズが10、DurabilityポリシーはVolatileで、他の設定はミドルウェアのデフォルトである。
- Service  
  上記と同様のQoSポリシーの組み合わせである。  
  サービス通信ではDurabilityポリシーがVolatileであることは重要である。  
  これにより再起動したサーバが過去のリクエストを受け取って処理しないようになる。
- Sensor Data  
  多少データが失われてもできるだけ早く通信できるようにする設定である。  
  キューサイズが小さく、ReliabilityポリシーはBest effortに設定されている。
- Parameters  
  Serviceプロファイルと基本的に同様だが、キューサイズが大きくなっている。
- System Default  
  全ての設定において、通信ミドルウェアのデフォルト設定を使用する。  
  ミドルウェアの実装により、異なる設定が使われる可能性がある。

具体的な定義は、[ros/rmwパッケージのqos_profiles.h](https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h)に定義されている。  
例として、Profile DefaultとSensor Dataの定義部分を引用して以下に示す。

```cpp
static const rmw_qos_profile_t rmw_qos_profile_default =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};
```

これらのQoSプロファイルはROS 2独自のものなので、コミュニティの動向によって中身のQoSポリシーが変わる可能性がある。

### QoS互換性

QoSは、PublisherとSubscriber、あるいはServiceサーバとServiceクライアントの両方に設定できる。  
そのため、トピック名やサービス名が一致していても、QoSの組み合わせが対応していないと通信は成立しない。  
例えばPub&Sub通信においては、PublisherのQoSポリシーは、SubscriberのQoSポリシーよりも同等か、それ以上に厳しい必要がある。  

具体例を挙げると、Reliabilityポリシーの組み合わせによる通信の成立の有無は以下のようになる。

|Publisher|Subscriber|通信の成立|
|---|---|---|
|Best effort|Best effort|○|
|Best effort|Reliable|×|
|Reliable|Best effort|○|
|Reliable|Reliable|○|

他のQoSポリシーに対する互換性は、[ROS 2公式ドキュメント](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html#qos-compatibilities)に記載がある。  
QoSを設定する際には、QoS互換性を考慮して通信が成立するよう注意する必要がある。

## CLIツール

ROS 2のQoSに関するCLIツールを説明する。  

`ros2 topic pub`コマンドでは、個別にQoSポリシーやQoSプロファイルを設定することが可能である。  
以下の例で、`system_default`プロファイルで指定したときのQoSを確認してみる。  
ターミナルで、以下を実行し、`sample_topic`という名前のトピックをPublishする。  
コマンドの詳細はヘルプ(`ros2 topic pub -h`)で確認できる。  

```sh
$ source /opt/ros/humble/setup.bash
$ ros2 topic pub /sample_topic std_msgs/msg/String "{data: Hello}" --qos-profile system_default
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Hello')

publishing #2: std_msgs.msg.String(data='Hello')

publishing #3: std_msgs.msg.String(data='Hello')

publishing #4: std_msgs.msg.String(data='Hello')

```

`ros2 topic info`コマンドにオプション`--verbose`をつけることで、トピックのQoSポリシーを確認できる。  
別のターミナルで、以下を実行すると、ReliabilityポリシーはReliable, HistoryポリシーはTransient localになっていることなどが確認できる。

```sh
$ ros2 topic info /sample_topic --verbose
Type: std_msgs/msg/String

Publisher count: 1

Node name: _ros2cli_408321
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: PUBLISHER
GID: 01.0f.53.cf.01.3b.4e.f1.01.00.00.00.00.00.05.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0

```

QoS互換性がないために通信ができない例を示す。  
1つ目のターミナルでReliabilityポリシーがBest effortのトピックをPublishし、2つ目のターミナルで同ポリシーがReliableのトピックをSubscribeする。  
すると、トピック名が一致しているのにも関わらずトピック通信ができていないことが確認できる。

```sh
# 1つ目のターミナル
$ ros2 topic pub /sample_topic std_msgs/msg/String "{data: Hello}" --qos-reliability best_effort
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Hello')

publishing #2: std_msgs.msg.String(data='Hello')

publishing #3: std_msgs.msg.String(data='Hello')

publishing #4: std_msgs.msg.String(data='Hello')
```

```sh
# 2つ目のターミナル
$ ros2 topic echo /sample_topic --qos-reliability reliable
# 何も出力されない
```

2つ目のターミナルで、Best effortを指定してSubscribeすると、これまで通り通信できるようになる。  

## ソースコード中でのQoS指定方法

ソースコード中では、通信のための変数（Publisher、Subscriber、Serviceサーバ、Serviceクライアント）を生成する際にQoSを指定できる。  
使用するのは、[rclcpp::QoSクラス](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/qos.hpp)と、QoSプロファイルの指定で用いた[rmw_qos_profile_t型](https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h)である。  

Publisherの場合の指定例は、[rclcpp::Node::create_publisher](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node.hpp#L162)の関数コメントに記載されている。以下に、リンク先で示されている例とそれに対するQoS設定を示す。

```cpp
// HistoryポリシーをKeep lastに、Depthサイズは10
pub = node->create_publisher<MsgT>("chatter", 10)

// 同上
pub = node->create_publisher<MsgT>("chatter", QoS(10));

// 同上
pub = node->create_publisher<MsgT>("chatter", QoS(KeepLast(10)));

// HistoryポリシーをKeep allにする
pub = node->create_publisher<MsgT>("chatter", QoS(KeepAll()));

// ReliabilityポリシーがBest effort, DurabilityポリシーがVolatile
pub = node->create_publisher<MsgT>("chatter", QoS(1).best_effort().volatile());

// Sensor dataプロファイルの設定で、HistoryポリシーをKeep last, Depthサイズを10にする
rclcpp::QoS custom_qos(KeepLast(10), rmw_qos_profile_sensor_data);
pub = node->create_publisher<MsgT>("chatter", custom_qos);
```

Subscriberの生成時にも、上記のように設定した`rclcpp::QoS`クラスを与えることでQoSを指定できる。（参考：[rclcpp::Node::create_subscription](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node.hpp#L219)）

Service通信のクライアントとサーバの生成時では、`rclcpp::Node::create_client()`関数の引数として`rmw_qos_profile_t`型を与えられるようになっている。  
デフォルトではサービス通信に適したQoSプロファイルが指定されている。（参考：[rclcpp::Node::create_client](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node.hpp#L252)、[rclcpp::Node::create_service](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node.hpp#L267)）

## 画像を用いたデモ

QoS設定の効果がより体感できるデモとして、[ROS 2公式ドキュメントに画像データの通信を題材にしたデモの手順](https://docs.ros.org/en/humble/Tutorials/Demos/Quality-of-Service.html)がある。  

このデモでは、意図的にネットワークに欠損が生じる状況を作り、webカメラのストリーミングの品質を確認している。  
ReliabilityポリシーがデフォルトのReliableのままの場合、フレームレートが低下することが確認できる。これは信頼性のある通信のためにメッセージの到着確認を行っているためである。  
一方、ReliabilityポリシーをBest effortに変更すると、フレームレートが向上する。このとき、一部の画像が欠損していることが、ターミナル出力から確認できる。

ROS 2公式ドキュメントから、デモの様子の画像を引用して以下に示す。

![qos_best_effort](https://raw.githubusercontent.com/ros2/demos/humble/image_tools/doc/qos-best-effort.png)

## Rviz2でのQoS指定

ROS 2上で用いる情報可視化ツールであるRviz2でも、QoSの設定を行うことが可能である。  
[近藤豊さんのHP上の記事](https://www.youtalk.jp/2019/12/13/qos-profile.html#rviz2%E3%81%A7%E3%81%AEqos%E3%83%97%E3%83%AD%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB%E8%A8%AD%E5%AE%9A)では、Rviz(ROS 1)とRviz2(ROS 2)の画面が比較して示されている。 
Rviz2では、QoSプロファイルを選択するドロップダウンメニューが追加されていることが確認できる。

## 参考

### QoSに関する参考リンク

- [ROS 2公式ドキュメント：About Quality of Service settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ROS 2公式ドキュメント：Using quality-of-service settings for lossy networks](https://docs.ros.org/en/humble/Tutorials/Demos/Quality-of-Service.html)
- [ROS 2 Design：ROS 2 Quality of Service policies](https://design.ros2.org/articles/qos.html)
- [ROS 2 Design：ROS QoS - Deadline, Liveliness, and Lifespan](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html)
- [rclcpp::QoS class](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/qos.hpp)
- [ros2/demosリポジトリ：quality_of_service_demo](https://github.com/ros2/demos/tree/humble/quality_of_service_demo)
- [ros/rmwリポジトリ：qos_profiles.h](https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h)
- [近藤豊さんHP：QoSプロファイルの実践と課題](https://www.youtalk.jp/2019/12/13/qos-profile.html)

### ROS 2とDDSの関連に関する参考リンク

- [ROS 2公式ドキュメント：About Different DDS/RTPS vendors](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html)
- [ROS 2公式ドキュメント：About Internal Interfaces](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html)
- [ROS 2 Design: ROS on DDS](https://design.ros2.org/articles/ros_on_dds.html)

### DDSに関する参考リンク

- [OMG Data Distribution Service version 1.4](https://www.omg.org/spec/DDS/1.4/PDF)
