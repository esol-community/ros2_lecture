# 実習ROS 2 ROS 2 Launch 2：応用

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

この記事では、ROS 2で使われるlaunchシステムについて、以下の機能を解説し、実装する。  
launchシステムには他にも様々な機能があるが、本講座ではそれらの一例を紹介する。

- インクルード
- パラメータの指定
- 引数の指定
- 条件分岐によるノードの起動
- ノードのイベントハンドラ

具体的には、以下の3つをまとめて起動するlaunchファイルを作成する。

- [ROS 2 Launch 1：概要](https://qiita.com/s-kitajima/items/3b17d1c4a248299cc026)で作成したlaunchファイル
- パラメータを持つノード
- パラメータを持つノードが起動した2秒後に、ノードのパラメータを変更するイベント

## 前提

本記事は、[Pub&Sub通信](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/../pub_sub_comm/README.md)、及び[ROS 2 Launch 1：概要](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/../ros2_launch_1/README.md)で作成したワークスペースを再利用する。  
前提としているワーススペースと今回作成するパッケージの構成を以下に示す。

```sh
$ cd ~/ros2_lecture_ws/src/
$ tree           # ディレクトリ構成を表示するコマンド
.
├── pub_sub_comm
│   ├── CMakeLists.txt
│   ├── include
│   │   └── pub_sub_comm
│   ├── package.xml
│   └── src
│       ├── simple_listener.cpp
│       └── simple_talker.cpp
├── ros2_launch_1
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── publisher_nodes.launch.py
│   │   ├── publisher_nodes.launch.xml
│   │   ├── subscriber_nodes.launch.py
│   │   └── subscriber_nodes.launch.xml
│   ├── package.xml
│   └── README.md
└── ros2_launch_2       # この記事で作成するパッケージ
    ├── CMakeLists.txt
    ├── launch
    │   ├── launch_all.launch.py
    │   └── launch_event_handler.launch.py
    ├── package.xml
    ├── README.md
    └── src
        └── param_talker.cpp
```

## 作成するlaunchファイルの全体像

この記事では、以下の3つの新規ファイルを作成する。  

- [launch_all.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./launch/launch_all.launch.py)
- [launch_event_handler.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./launch/launch_timer_handler.lauch.py)
- [param_talker.cpp](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./src/param_talker.cpp)

これらのうち中心になるのは[launch_all.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./launch/launch_all.launch.py)である。このlaunchファイルから複数のプログラムを起動する。  
[launch_all.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./launch/launch_all.launch.py)から起動するプログラムを以下に示す。

1. `publisher_nodes.launch.py`
2. `subscriber_nodes.launch.py`
3. `launch_event_handler.launch.py`

1と2は、[ROS 2 Launch 1：概要](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/../ros2_launch_1/README.md)で作成したlaunchファイルである。

3のlaunchファイルでは、param_talkerノードを起動し、2秒後にパラメータを変更するアクションを実行する。

以降では、launchシステムの機能ごとに関連する部分のみの実装を示す。  
ファイルの全体像はリンク先を参照すること。

## 実装

### パッケージの作成

以下のコマンドでパッケージを作成する。  
このとき、依存関係にあるパッケージをオプションで指定する。(`--dependencies pub_sub_comm ros2_launch_1 rclcpp std_msgs`)。  
これにより、作成される[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./package.xml)に依存関係が記述される。

```sh
# パッケージを作成
$ cd ~/ros2_lecture_ws/src
$ ros2 pkg create ros2_launch_2 --build-type ament_cmake --dependencies pub_sub_comm ros2_launch_1 rclcpp std_msgs

# launchファイルを格納するディレクトリの作成
$ mkdir ros2_launch_2/launch
```

### launchファイルのインクルード

あるlaunchファイルから別のlaunchファイルをそのまま呼び出すことができる。  
これにより、機能ごとにlaunchファイルを分割して管理することができる。  

[launch_all.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./launch/launch_all.launch.py)では、[publisher_nodes.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/../ros2_launch_1/launch/publisher_nodes.launch.py)と[subscriber_nodes.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/../ros2_launch_1/launch/subscriber_nodes.launch.py)をインクルードしている。  
インクルードに関連する記述を抜粋して以下に示す。

```python
# 必要なモジュールのインポート
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

    # ...(省略)...

    # publisher_nodes.launch.pyのインクルード
    pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros2_launch_1'), 'launch'),
            '/publisher_nodes.launch.py'])
    )

    # subscriber_nodes.launch.pyのインクルード
    sub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros2_launch_1'), 'launch'),
            '/subscriber_nodes.launch.py'])
    )


    # ...(省略)...

    # IncludeLaunchDescription アクションを追加
    ld.add_action(pub_launch)
    ld.add_action(sub_launch)

    # ...(省略)...
```

### 条件分岐、引数

launchファイルで行うアクションに対して、条件分岐により、実行の有無を指示することができる。  
[launch_all.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./launch/launch_all.launch.py)では、launchファイル自体の引数を`DeclareLaunchArgument`というアクションによって作成している。  
引数`include_param_talker`によって、[launch_event_handler.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./launch/launch_event_handler.launch.py)をインクルードするかどうかを判定している。  

どのように引数を与えるか、については実行結果の項で説明する。  

条件分岐と引数に関わる記述を抜粋して以下に示す。

```python
# 必要なモジュールのインポート
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

    # ...(省略)...

    include_param_talker = LaunchConfiguration('include_param_talker')

    include_param_talker_arg = DeclareLaunchArgument(
        'include_param_talker',
        default_value="true",
        description='Whether to include param talker'
    )

    # IfConditionを用いた、launch_event_handler.launch.pyのインクルード
    param_talker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros2_launch_2'), 'launch'),
            '/launch_event_handler.launch.py']),
        condition=IfCondition(include_param_talker)
    )

    # ...(省略)...

    ld.add_action(include_param_talker_arg)
    ld.add_action(param_talker_launch)

    # ...(省略)...
```

### パラメータの設定

ROS 2ではノードごとにパラメータを与えることが可能であり、パラメータはlaunchファイルから与えられる。  

launchファイルでパラメータを与えるために、まずパラメータを持つノードを実装する。  
[param_talker.cpp](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./src/param_talker.cpp)のコンストラクタを以下に示す。`this->declare_parameter<std::string>("param_talker_parameter", "hello");`によって、ノードにパラメータが設定される。

```cpp
  // ...(省略)...

  // コンストラクタ
  ParamTalker()
  : Node("param_talker")   // ノード名をparam_talkerで初期化
  {
    // このノードが持つ、param_talker_parameterという名前のパラメータを宣言
    // デフォルト値は"hello"
    this->declare_parameter<std::string>("param_talker_parameter", "hello");

    // publisherの生成
    // 第一引数はトピック名、第二引数はバッファサイズ
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // タイマの生成
    // 100msごとに、timer_callback()関数が呼ばれるようにする
    timer_ = this->create_wall_timer(100ms, std::bind(&ParamTalker::timer_callback, this));
  }

  // ...(省略)...
```

launchファイルでは、`Node`のアクションを生成する際の引数`parameters`でパラメータを設定できる。
[launch_event_handler.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./launch/launch_event_handler.launch.py)から、パラメータの設定に関わる記述を抜粋して以下に示す。

```python
# 必要なモジュールのインポート
from launch_ros.actions import Node

    # ...(省略)...

    # パラメータを指定してノードを起動
    param_talker_node  = Node(
        package="ros2_launch_2",
        executable="param_talker",
        parameters=[
                # param_talker_parameterという名前のパラメータに
                # 値"earth"を設定
                {"param_talker_parameter": "earth"}
        ]
    )

    # ...(省略)...

    ld.add_action(param_talker_node)

    # ...(省略)...
```

### イベントハンドラの登録

`param_talker.cpp`で作成したノードを起動するlaunchファイルを作成する。  

ここで、このノードがパラメータを動的に取得することを確かめるため、2秒後にパラメータを変えることにする。   
変える先のパラメータはlaunchファイルの引数として与えることにする。  
これを実現するために、launchシステムで使える以下の複数のアクションを組み合わせる。  

- ノードを起動させるアクション：`Node`
- パラメータを変更するコマンドを表現できるアクション：`ExecuteProcess`
- 2秒後という時間を表現するアクション：`TimerAction`
- あるイベント（ここでは、ノードが起動したというイベント）を表現するアクション：`RegisterEventHandler`

また、イベントそのものとそれに対する処理を表現するために、`launch.event_handlers.OnProcessStart`を利用する。  
この型ではプロセスが開始したときに実行するアクションを指定できる。  

これらを用いて、「ノードが起動してから2秒後にパラメータ`param_talker_parameter`を変える」アクションを以下のように実装する。  
以下に[launch_event_handler.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./launch/launch_event_handler.launch.py)の一部を示す。

```python
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

    # ...(省略)...

    # ノードが起動するイベントを受けてパラメータを変更するアクションを定義する
    parameter_change_action = RegisterEventHandler(
        OnProcessStart(
            target_action=param_talker_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        change_parameter_action,
                        LogInfo(msg='Change the parameter of the node')
                    ],
                )
            ]
        )
    )

    # ...(省略)...
```

ここで、変数`param_talker_node`は`Node`アクションの変数である。  
`LogInfo`アクションは名前の通りログ出力を行うアクションであり、2秒後に`TimerAction`が実行されたことをターミナル上から確かめるために追加した。

変数`change_parameter_action`は以下のように定義している。  
ここでは、パラメータを変えるコマンド`ros2 param set <node_name> <parameter_name>`を、launchシステムでの型`ExecuteProcess`で保持している。

```python
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

    # ...(省略)...

    # パラメータを変更するアクションを作成する
    change_parameter_action = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set',
            ' /param_talker',
            ' param_talker_parameter ',
            param_talker_parameter
        ]],
        shell=True
    )

    # ...(省略)...
```

`param_talker`ノードは、以下のようにして起動している。  
起動時には、パラメータ`param_talker_parameter`の値を`earth`に設定している。

```python
from launch_ros.actions import Node

    # ...(省略)...

    # ROS 2パラメータを利用するノードを起動する
    # launchファイルから、起動時のパラメータを与えておく
    param_talker_node = Node(
        package="ros2_launch_2",
        executable="param_talker",
        name='param_talker',
        parameters=[
                {"param_talker_parameter": "earth"}
        ]
    )

    # ...(省略)...
```

また、このlaunchファイルでは、2秒後に変えるパラメータを引数`param_talker_parameter_arg`で与えられるようにしている。  
引数の作成方法は解説したため、ここでは省略する。

## 実行

### ビルド

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_2/./CMakeLists.txt)にこれまでの講座と同様にビルド設定を記述したあと、以下のコマンドでビルドを行う。  

`--packages-up-to`は、指定したパッケージとそのパッケージが依存するパッケージのみをビルドするオプションである。  
つまり、`ros2_launch_2`パッケージと、`pub_sub_comm`、`ros2_launch_1`パッケージがビルドされる。

```sh
$ cd ~/ros2_lecture_ws
$ colcon build --packages-up-to ros2_launch_2
```

### 結果1: 引数なしで起動

はじめに、`launch_all.launch.py`に引数を与えないで起動する。  
この場合、`IfCondition`で与えている引数がデフォルト値である`true`になる。  
従ってインクルードが実行され、インクルードされたlaunchファイルの中のノードが起動される。  

以下の実行結果を見ると、以下のことが分かる。

- 以下の3種類のノードが起動していること
  - /param_talker
  - /namespace_app1/talker_renamed1
  - /namespace_app2/talker_renamed2
- `launch_all.launch.py`が`2つのlaunchファイルをインクルードできていること
- /param_talkerに与えたパラメータが、launchファイル中で与えた`earth`になっていること

```sh
$ cd ~/ros2_lecture_ws
$ . install/setup.bash
$ ros2 launch ros2_launch_2 launch_all.launch.py 
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [talker-1]: process started with pid [25423]
[INFO] [talker-2]: process started with pid [25425]
[INFO] [listener-3]: process started with pid [25427]
[INFO] [listener-4]: process started with pid [25429]
[INFO] [param_talker-5]: process started with pid [25431]
[param_talker-5] [INFO] [1710328445.310763973] [param_talker]: publish: earth
[talker-2] [INFO] [1710328445.310862496] [namespace_app2.talker_renamed2]: publish: hello, world!
[talker-1] [INFO] [1710328445.310893386] [namespace_app1.talker_renamed1]: publish: hello, world!
[listener-3] [INFO] [1710328445.311624546] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-4] [INFO] [1710328445.311623727] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[param_talker-5] [INFO] [1710328445.410883855] [param_talker]: publish: earth
[talker-2] [INFO] [1710328445.410952373] [namespace_app2.talker_renamed2]: publish: hello, world!
[talker-1] [INFO] [1710328445.410968016] [namespace_app1.talker_renamed1]: publish: hello, world!
```

また、起動して2秒が経過すると、以下のように出力が変化する。  
以下の出力例からは、パラメータが`earth`から`Default parameter from launch file`に変化していることが分かる。

```sh
# ...(省略)...

[listener-3] [INFO] [1710328470.601871157] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-4] [INFO] [1710328470.602052897] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[talker-1] [INFO] [1710328470.701387748] [namespace_app1.talker_renamed1]: publish: hello, world!
[param_talker-5] [INFO] [1710328470.701583384] [param_talker]: publish: earth
[talker-2] [INFO] [1710328470.701599376] [namespace_app2.talker_renamed2]: publish: hello, world!
[listener-3] [INFO] [1710328470.701931104] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-4] [INFO] [1710328470.702077289] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[INFO] [launch.user]: Change the parameter of the node
[INFO] [param_talker param_talker_parameter "Default parameter from launch file"-6]: process started with pid [25571]
[talker-1] [INFO] [1710328470.801189385] [namespace_app1.talker_renamed1]: publish: hello, world!
[talker-2] [INFO] [1710328470.801499498] [namespace_app2.talker_renamed2]: publish: hello, world!
[param_talker-5] [INFO] [1710328470.801508816] [param_talker]: publish: earth

# ...(省略)...

[listener-3] [INFO] [1710328471.501668472] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-4] [INFO] [1710328471.502002819] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[talker-1] [INFO] [1710328471.601388843] [namespace_app1.talker_renamed1]: publish: hello, world!
[talker-2] [INFO] [1710328471.601596007] [namespace_app2.talker_renamed2]: publish: hello, world!
[param_talker-5] [INFO] [1710328471.601593916] [param_talker]: publish: Default parameter from launch file
[listener-3] [INFO] [1710328471.601923170] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-4] [INFO] [1710328471.602098005] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[INFO] [param_talker param_talker_parameter "Default parameter from launch file"-6]: process has finished cleanly [pid 25571]
[talker-1] [INFO] [1710328471.701390525] [namespace_app1.talker_renamed1]: publish: hello, world!
[talker-2] [INFO] [1710328471.701590261] [namespace_app2.talker_renamed2]: publish: hello, world!
[param_talker-5] [INFO] [1710328471.701596615] [param_talker]: publish: Default parameter from launch file
[listener-3] [INFO] [1710328471.701922758] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-4] [INFO] [1710328471.702090568] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[talker-1] [INFO] [1710328471.801356787] [namespace_app1.talker_renamed1]: publish: hello, world!
[talker-2] [INFO] [1710328471.801527530] [namespace_app2.talker_renamed2]: publish: hello, world!
[param_talker-5] [INFO] [1710328471.801568618] [param_talker]: publish: Default parameter from launch file
```

別のターミナルを開いて、以下のコマンドを実行すると、起動しているノードの一覧が取得できる。

```sh
$ ros2 node list
/namespace_app1/listener_renamed1
/namespace_app1/talker_renamed1
/namespace_app2/listener_renamed2
/namespace_app2/talker_renamed2
/param_talker
```

### 結果2: 引数でインクルードファイルを制御する

次に、launchファイルに`include_param_talker:=false`の引数を与えて起動してみる。  

```sh
$ cd ~/ros2_lecture_ws
$ . install/setup.bash
$ ros2 launch ros2_launch_2 launch_all.launch.py include_param_talker:=false
```

この場合、条件分岐の項で記述した`IfCondition`の条件がfalseになり、`launch_event_handler.launch.py`がインクルードされなくなる。  
その結果、起動されるノードは以下の4種類になる。

- /namespace_app1/listener_renamed1
- /namespace_app1/talker_renamed1
- /namespace_app2/listener_renamed2
- /namespace_app2/talker_renamed2

```sh
$ ros2 launch ros2_launch_2 launch_all.launch.py include_param_talker:=false
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [talker-1]: process started with pid [31414]
[INFO] [talker-2]: process started with pid [31416]
[INFO] [listener-3]: process started with pid [31418]
[INFO] [listener-4]: process started with pid [31420]
[talker-2] [INFO] [1710328795.935005623] [namespace_app2.talker_renamed2]: publish: hello, world!
[talker-1] [INFO] [1710328795.935306474] [namespace_app1.talker_renamed1]: publish: hello, world!
[listener-4] [INFO] [1710328795.935621625] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[listener-3] [INFO] [1710328795.935848857] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[talker-2] [INFO] [1710328796.035008844] [namespace_app2.talker_renamed2]: publish: hello, world!
[talker-1] [INFO] [1710328796.035310525] [namespace_app1.talker_renamed1]: publish: hello, world!
[listener-4] [INFO] [1710328796.035550772] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[listener-3] [INFO] [1710328796.035748968] [namespace_app1.listener_renamed1]: subscribe: hello, world!
```

別のターミナルを開いて、以下のコマンドを実行すると、確かに2つのノードしか起動していないことが確認できる。

```sh
$ ros2 node list
/namespace_app1/listener_renamed1
/namespace_app1/talker_renamed1
/namespace_app2/listener_renamed2
/namespace_app2/talker_renamed2
```

### 結果3: 引数でパラメータを制御する

最後に、launchファイルに`param_talker_parameter:=parameter_from_cli`の引数を与えて起動してみる。  

```sh
$ cd ~/ros2_lecture_ws
$ . install/setup.bash
$ ros2 launch ros2_launch_2 launch_all.launch.py param_talker_parameter:=parameter_from_cli
```

すると、2秒後に変化するパラメータが`parameter_from_cli`に変化することが分かる。

```sh
$ ros2 launch ros2_launch_2 launch_all.launch.py param_talker_parameter:=parameter_from_cli

# ...(省略)...

[param_talker-5] [INFO] [1710328905.462143697] [param_talker]: publish: parameter_from_cli
[listener-4] [INFO] [1710328905.462397793] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[talker-1] [INFO] [1710328905.462622614] [namespace_app1.talker_renamed1]: publish: hello, world!
[listener-3] [INFO] [1710328905.463076589] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[talker-2] [INFO] [1710328905.562023155] [namespace_app2.talker_renamed2]: publish: hello, world!
[param_talker-5] [INFO] [1710328905.562098567] [param_talker]: publish: parameter_from_cli
[listener-4] [INFO] [1710328905.562547602] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[talker-1] [INFO] [1710328905.562623962] [namespace_app1.talker_renamed1]: publish: hello, world!
[listener-3] [INFO] [1710328905.563086208] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[talker-2] [INFO] [1710328905.661947577] [namespace_app2.talker_renamed2]: publish: hello, world!
```

## 参考

- [ROS講座19 roslaunch2](https://qiita.com/srs/items/e7882078b8cf11dc51fb)
- [ROS 2公式チュートリアル：Managing large projects](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
