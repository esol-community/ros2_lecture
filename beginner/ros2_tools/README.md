# 実習ROS 2  ROStools

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

ROS 2では様々なコマンドが用意されている。本記事ではそれらコマンドの種類やその機能について解説する。  
これらのコマンドはデバックや解析の際に役立つものであるため、これらの扱いを知ることは非常に重要である。

## ros2 コマンド群

### ros2 topic

`ros2 topic`コマンドではトピック周りの情報を確認することができる。  
以下、コマンドとその機能について記述する。  
なお、この章では実用例として[pub&sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)にて作成したノード[talker](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../pub_sub_comm/src/simple_talker.cpp)と[listener](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../pub_sub_comm/src/simple_listener.cpp)を用いる。  
特に使用ノードに対して言及がない場合、[talker](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../pub_sub_comm/src/simple_talker.cpp)を用いることとする。  

#### list

- 説明  
  現在機能しているトピックを表示する。  

- オプション  
  `-t`オプションを付けるとそのトピックの型も同時に表示する。

- 用例

  ```shell
  $ ros2 topic list
  /chatter
  /parameter_events
  /rosout
  ```

  - Ctrl＋Cでtalkerを終了させると/chatterも停止するため、この状態で`ros2 topic list`を実行すれば/chatterがなくなっているのを確認できる。

    ```shell
    $ ros2 topic list
    /parameter_events
    /rosout
    ```

  - `-t`オプションを付けた場合

    ```shell
    $ ros2 topic list -t
    /chatter [std_msgs/msg/String]
    /parameter_events [rcl_interfaces/msg/ParameterEvent]
    /rosout [rcl_interfaces/msg/Log]
    ```

#### echo

- 説明  
  現在トピックに配信されているメッセージを表示する。  
  このコマンドを用いれば、自身の作ったpublisherが正確に任意のトピックにpublishできているかを簡単に確認できる。
  - コマンド  
    `ros2 topic echo トピック名`  
    トピックの名前は"/"から始まることに注意する。

- 用例  

  ```shell
  $ ros2 topic echo /chatter
  data: hello, world!
  ---
  data: hello, world!
  ---
  data: hello, world!
  ---
  data: hello, world!
  ---
  (以下省略)
  ```

  ここでターミナルに出力される速度はpublisherの配信周期と同じである。

#### info

- 説明  
  指定したトピックの型とpublisher・subscriberの数を表示する。
  - コマンド  
    `ros2 topic info トピック名`
  
- 用例  
  以下は[talker](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../pub_sub_comm/src/simple_talker.cpp)と[listener](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../pub_sub_comm/src/simple_listener.cpp)両方を起動した状態での実行結果である。

  ```shell
  $ ros2 topic info /chatter
  Type: std_msgs/msg/String
  Publisher count: 1
  Subscription count: 1
  ```

  talkerとlistenerがそれぞれカウントされていることが確認できる。

#### pub

- 説明  
  コマンドラインから直接指定したトピックにpublishする。  
  このコマンドを使用すれば、自身が作成したsubscriberが正しく任意のトピックを受信できているか簡単に調べられる。
  - コマンド  
    `ros2 topic pub トピック名 トピックの型 "引数"`  
    引数はそのトピックの型が持っている要素を指定する。(型の構造は、`ros2 interface show`コマンドで確認できる。詳しくは[後述](#show))  
    ただし、引数はYAML構文で入力しなければならないことに注意する。  

- オプション  
  - --once  
    1回だけpublishする。
  - --rate  
    配信周期を設定する。ex)`--rate 1`なら1Hzでpublishし続ける。

- 用例  

  ```shell
  $ ros2 topic pub /chatter std_msgs/msg/String "{data: "hello"}"
  publisher: beginning loop
  publishing #1: std_msgs.msg.String(data='hello')

  publishing #2: std_msgs.msg.String(data='hello')

  publishing #3: std_msgs.msg.String(data='hello')

  (以下省略)
  ```

  別ターミナルでtalkerの代わりに[listener](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../pub_sub_comm/src/simple_listener.cpp)を起動させると、以下のように正常に受信していることが確認できる。

  ```shell
  $ ros2 run pub_sub_comm listener 
  [INFO] [1668410241.237764943] [simple_listener]: subscribe: hello
  [INFO] [1668410242.237581793] [simple_listener]: subscribe: hello
  [INFO] [1668410243.237441494] [simple_listener]: subscribe: hello
  (以下省略)
  ```

#### hz

- 説明  
  指定したトピックの配信周期を表示する。  
  - コマンド  
    `ros2 topic hz トピック名`

- 用例

  ```shell
  $ ros2 topic hz /chatter
  average rate: 9.999
        min: 0.100s max: 0.100s std dev: 0.00018s window: 12
  average rate: 9.999
        min: 0.100s max: 0.100s std dev: 0.00014s window: 23
  average rate: 10.000
        min: 0.100s max: 0.100s std dev: 0.00012s window: 34
  (以下省略)
  ```

  今回はtalker1つしか起動していないため、talkerの配信周期10hzがそのまま/chatterの配信周期になっていることが確認できる。  

### ros2 node

このコマンドではノード周りの情報を確認することができる。

#### list

- 説明  
  起動しているROSノードの一覧を表示する。  
  何のノードが立ち上がっているか確認したいときに使うとよい。

- 用例

  ```shell
  $ ros2 node list
  /simple_talker
  ```

#### info

- 説明  
  対象のノードと通信するSubscriber,Publisher,サービス及びアクションのリストを表示する。
  - コマンド  
    `ros2 node info ノード名`  
    ノード名にも"/"がいることに注意。  
    `ros2 node list`でノード名を調べてから行うと確実である。

- 用例

  ```shell
  $ ros2 node info /simple_talker 
  /simple_talker
    Subscribers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
    Publishers:
      /chatter: std_msgs/msg/String
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /rosout: rcl_interfaces/msg/Log
    Service Servers:
      /simple_talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /simple_talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /simple_talker/get_parameters: rcl_interfaces/srv/GetParameters
      /simple_talker/list_parameters: rcl_interfaces/srv/ListParameters
      /simple_talker/set_parameters: rcl_interfaces/srv/SetParameters
      /simple_talker/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    Service Clients:

    Action Servers:

    Action Clients:
  ```

### ros2 service

このコマンドではサービス周りの情報を確認することができる。  
なお、この章では[Service通信](https://qiita.com/s-kitajima/items/29607ff86a656e4d6099)で作成した[add_two_server](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../service_comm/src/service_server.cpp)と[add_two_client](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../service_comm/src/service_client.cpp)を使用する。

#### list

- 説明  
  現在機能しているサービスを調べることができる。

- オプション  
  `-t`オプションを付けるとそのサービスの型も同時に表示できる。

- 用例

    ```shell
    $ ros2 service list
    /add_two_ints
    /add_two_ints_server/describe_parameters
    /add_two_ints_server/get_parameter_types
    /add_two_ints_server/get_parameters
    /add_two_ints_server/list_parameters
    /add_two_ints_server/set_parameters
    /add_two_ints_server/set_parameters_atomically
    ```

    - `-t`オプションを付けた場合  

    ```shell
    $ ros2 service list -t
    /add_two_ints [example_interfaces/srv/AddTwoInts]
    /add_two_ints_server/describe_parameters [rcl_interfaces/srv/DescribeParameters]
    /add_two_ints_server/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]      
    /add_two_ints_server/get_parameters [rcl_interfaces/srv/GetParameters]
    /add_two_ints_server/list_parameters [rcl_interfaces/srv/ListParameters]
    /add_two_ints_server/set_parameters [rcl_interfaces/srv/SetParameters]
    /add_two_ints_server/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
    ```

#### type

- 説明  
  サービスの型を表示する。  
  - コマンド  
    `ros2 service type サービス名`  
    サービス名の先頭に"/"を付け忘れないよう注意する。
- 用例

  ```shell
  $ ros2 service type /add_two_ints
  example_interfaces/srv/AddTwoInts
  ```

#### find

- 説明  
  特定の型のサービスを一度に検索できる。  
  - コマンド  
    `ros2 service find 型名`
- 用例

  ```shell
  $ ros2 service find example_interfaces/srv/AddTwoInts
  /add_two_ints
  ```

#### call

- 説明  
  サービスを呼び出す。  
  このコマンドにより、server側のノードが正常に機能しているかを簡単に確かめることができる。  
  - コマンド  
    `ros2 service call サービスの名前 サービスの型名 引数`  
    引数はそのサービスの型が持っている要素を指定する。（型の構造は、`ros2 interface show`コマンドで確認できる。詳しくは[後述](#show)）  
    `ros2 topic pub`コマンドと同じく、引数はYAML構文で入力しなければならないので注意。

    ```shell
    $ ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 2}"
    requester: making request: example_interfaces.srv.AddTwoInts_Request(a=3, b=2)

    response:
    example_interfaces.srv.AddTwoInts_Response(sum=5)
    ```

    Server側が応答を返し、その応答を受けていることが確認できる。  
    Server側は以下の結果となる。

    ```shell
    $ ros2 run service_comm add_two_ints_server 
    [INFO] [1667892893.926295609] [add_two_ints_server]: Create AddTwoInts service server.
    [INFO] [1667893043.018153969] [add_two_ints_server]: Received request: a = 3, b = 2
    [INFO] [1667893043.018192050] [add_two_ints_server]: Sending back response: sum = 5

    ```

### ros2 interface

インターフェース周りの情報を確認できる。  
インターフェースとはメッセージやサービスのことである。  
詳しくは下記URLを参照。  
[公式：ROS 2インターフェースについて](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)

#### list

- 説明  
  インターフェースのリストを表示する。  
  ただし、すべてのインターフェースを表示するので数は多くなる。  
  そのため実行結果は割愛する。

#### show

- 説明  
  指定したインターフェースの構造を表示する。  
  - コマンド  
    `ros2 interface show 型名`  
    （型名は、メッセージなら`パッケージ名/msg/メッセージ名`となる。）

- 用例  
  以下はstd_msgs/msg/Stringを指定した時の結果である。

  ```shell
  $ ros2 interface show std_msgs/msg/String
  # This was originally provided as an example message.
  # It is deprecated as of Foxy
  # It is recommended to create your own semantically meaningful message.
  # However if you would like to continue using this please use the equivalent in example_msgs.

  string data
  ```

  これにより、std_msgs/msg/String型のメッセージはdataという要素を持っていることが確認できる。  

  - std_msgsパッケージで定義されているメッセージ型のフィールド名は、全てdataで統一されている。  
    フィールド名に意味がないため、より意味のあるフィールド名を持つメッセージ型の使用が推奨されている。  
    (参考：[std_msgsパッケージ README.md](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/README.md))

## ROS 2向けツール群

### rqt_graph

- 説明  
  ノードとトピックの送受の関係をGUIで図示する。  
  これによって、どのノードが何のトピックを介して通信しているか一目で確認できる。  
  - コマンド  
    `rqt_graph`

- 用例  
  以下は[talker](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../pub_sub_comm/src/simple_talker.cpp)と[listener](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/../pub_sub_comm/src/simple_listener.cpp)両方が起動している状態での結果である。

  ![rqt_graphの結果](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/./img/rqt_graph.png)

  なお、このコマンドが実行されたタイミングで起動しているノードを表示するため、先に表示させたいノードを起動させておく必要がある。  
  もし先に`rqt_graph`を起動させた場合はノードを起動させてから左上のrefreshボタンを押せば良い。そうすれば、その時点でリロードした結果を表示してくれる。

### plotjuggler 

- 説明  
  データ可視化ツールplotjugglerをrosbagに対応させたものである。  
  rosbagファイルに含まれているトピックデータを時系列でグラフ化する。  
  `sudo apt install ros-humble-plotjuggler-ros`としてインストールできる。  
  [参考：plotjuggler-ros-pluginsリポジトリ](https://github.com/PlotJuggler/plotjuggler-ros-plugins)
  - コマンド  
    `ros2 run plotjuggler plotjuggler`

- 用例  
  トピックを可視化した例を以下に示す。

  ![plotjuggler](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/./img/plotjuggler.png)

### rqt_bag

- 説明  
  Qtを利用したGUIフレームワークRQtによる可視化ツールである。  
  `sudo apt install ros-humble-rqt-bag`としてインストールできる。  
  - コマンド
    `ros2 run rqt_bag rqt_bag`

- 用例  
  rosbagファイルを読み込み、publishされているタイミングとその数値の確認、データのプロット等の機能がある。  
  rosbagデータを読み込み表示している例を以下に示す。

  ![rqt_bag](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_tools/./img/rqt_bag.png)

## 参考

- [ROS 2公式チュートリアル](https://docs.ros.org/en/humble/Tutorials.html)
  - [トピックを理解する](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
  - [ノードの理解](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
  - [turtlesimとrqtの使用](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
  - [サービスを理解する](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [ROS講座05 ROStools](https://qiita.com/srs/items/e6fb075ff57fa7ff8812)  
- [ROS 2インターフェースについて](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
