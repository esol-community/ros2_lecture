# 実習ROS 2 ROS 2 Launch 1：概要

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

このページでは、ROS 2で使われるlaunchシステムの目的とその機能について、概念的な説明をする。  
その後、2つのPublisherをまとめて起動するlaunchファイルと、2つのSubscriberをまとめて起動するlaunchファイルを記述し、実行結果を確認する。  
launchファイルは同じ内容のものをPythonとXMLの2種類で記述する。

## 本記事で解説していること

- launchファイルの目的
- [Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)のページで実装したPublisherとSubscriberを同時に起動するlaunchファイル（Python形式及びXML形式）
- launchファイルを記述する言語（Python, XML, YAML）による違いについて

本記事ではlaunchファイルの概念と基本的な作成方法を説明することに焦点を当てる。  
そのため、本記事で実装するlaunchファイルは単純なものとし、別の記事でlaunchシステムを活用した様々な機能を紹介する。

## 前提

本記事の内容は、[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)で作成したディレクトリ群があることを前提にしている。  
また、ワークスペースの名称は`ros2_lecture_ws`であると仮定して説明する。  
前提としているパッケージと今回作成するパッケージの構成を以下に示す。

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
├── ros2_launch_1       # この記事で作成するパッケージ
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── publisher_nodes.launch.py
│   │   ├── publisher_nodes.launch.xml
│   │   ├── subscriber_nodes.launch.py
│   │   └── subscriber_nodes.launch.xml
│   ├── package.xml
│   └── README.md
```

## launchファイル

### 説明

launchファイルは、複数のROSノードをまとめて起動し、それらの実行を制御するためのファイルである。  

ROS 2の場合、プログラムの基本単位となるノードは`ros2 run <パッケージ名> <実行ファイル名>`というコマンドで実行できる。  
この場合、ノード数だけターミナルを起動し、人が都度都度コマンドを実行する必要がある。  

launchファイルを用いると、1つのターミナルで複数のノードをまとめて起動できるようになる。  
また、launchシステムにはノードやそれらの間の通信を制御する様々な機能があり、柔軟で再利用可能なプログラムの構成を記述できる。  
launchファイルで実現できる機能の一例を以下に示す。

- 条件に応じて起動するノードを選択する
- 他のlaunchファイルをインクルードする
- launchファイルに対する引数を定義する
- ノード名やトピック名をremap(*注)する
- ノード名やトピック名にnamespace（名前空間）を追加する

(*注)：remapとは、ある名称を別の名称に置き換えることである。例えば、`/chatter1`という名前でソースコード中に実装されたトピックを`/chatter1_remapped`という名前としてノードを起動することがremapにあたる。

### パッケージの作成

launchファイルはパッケージの中に実装するため、まずはパッケージを作成する。

以下のように、`ros2 pkg create`コマンドでパッケージを作成する。  
本記事では`pub_sub_comm`に依存したパッケージを作成するため、オプションで依存パッケージを指定している。(`--dependencies pub_sub_comm`の部分)。  

```sh
# パッケージを作成
$ cd ~/ros2_lecture_ws/src
$ ros2 pkg create ros2_launch_1 --build-type ament_cmake --dependencies pub_sub_comm
```

### Pythonでの書き方

ROS 2ではPythonでlaunchファイルが記述されることが多いため、まずPythonを用いた実装を示す。  
`ros2_launch_1`のパッケージに`launch`というディレクトリを作成し、その中に`publisher_nodes.launch.py`というlaunchファイルを作成する。  

```bash
# ディレクトリを作成し、ファイルを新規作成
cd ~/ros2_lecture_ws/src/ros2_launch_1
mkdir launch
cd launch
touch publisher_nodes.launch.py
```

このlaunchファイルでは、同一の処理を行うPublisherノードを2つ起動する。  
実装したファイルの中身を以下に示す。launchファイルへのリンクは[こちら](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_1/../ros2_launch_1/launch/publisher_nodes.launch.py)。

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # launchの構成を示すLaunchDescription型の変数の定義
    ld = LaunchDescription()

    # publisher nodeを、"talker_renamed1"という名前で定義
    pub_node1 = Node(
        package='pub_sub_comm',
        executable='talker',
        name='talker_renamed1',
        namespace='namespace_app1',                 # namespace_app1というnamespaceを追加
        remappings=[('chatter', 'chatter_app1')]    # chatterトピックをchatter_app1トピックにremap
    )

    # publisher nodeを、"talker_renamed2"という名前で定義
    pub_node2 = Node(
        package='pub_sub_comm',
        executable='talker',
        name='talker_renamed2',
        namespace='namespace_app2',                 # namespace_app2というnamespaceを追加
        remappings=[('chatter', 'chatter_app2')]    # chatterトピックをchatter_app2トピックにremap
    )

    # LaunchDescriptionに、起動したいノードを追加する
    ld.add_action(pub_node1)
    ld.add_action(pub_node2)

    # launch構成を返すようにする
    return ld
```

Pythonで記述するlaunchファイルでは、`launch.LaunchDescription`型のオブジェクトを返す、`generate_launch_description()`という名前の関数を定義する。  

関数内では`launch.Action`型を継承したクラスである`launch_ros.Node`型の変数を`LaunchDescription`型の変数`ld`に追加している。  

ノードは`talker_renamed1`、`talker_renamed2`という名前にリネームしてから起動している。  
同じ名前のノードを複数起動することはできないが、ノード名をリネームすることで、同一の処理を行うノードを複数起動できるようになる。

また、1つ目のノードには`namespace_app1`、2つ目のノードには`namespace_app2`というnamespaceを追加している。  
これにより、トピック名やノード名を階層的に扱うことが可能になり、大規模なROS 2システムが扱いやすくなる。

さらにトピック名についても、2つのノードで異なる名前にリマップしている。  
これにより、ノードのソースコードで実装したトピック名を変更することが容易になり、ノードの再利用性が高まる。

同様にして、`subscriber_nodes.launch.py`を作成する。  
このlaunchファイルでは、`pub_sub_comm`パッケージのノード`listener`を、`listener_renamed1`、`listener_renamed2`という別個のノードとして起動する。  
本記事中では実装は省略するが、launchファイルへのリンクを[こちら](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_1/../ros2_launch_1/launch/subscriber_nodes.launch.py)に示す。

### XMLでの書き方

上記と同じ内容のlaunchファイルをXMLで実装する。  
`publisher_nodes.launch.py`を作成した場所に、`publisher_nodes.launch.xml`という名前のlaunchファイルを作成する。

```bash
# ファイルを新規作成
cd ~/ros2_lecture_ws/src/ros2_launch_1/launch
touch publisher_nodes.launch.xml
```

実装したファイルの中身を以下に示す。launchファイルへのリンクは[こちら](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_1/../ros2_launch_1/launch/publisher_nodes.launch.xml)。

```xml
<launch>
    <!-- publisher nodeを、"talker_renamed1"という名前で定義 -->
    <node pkg="pub_sub_comm" exec="talker" name="talker_renamed1" namespace="namespace_app1">
        <remap from="chatter" to="chatter_app1"/>
    </node>

    <!-- publisher nodeを、"talker_renamed2"という名前で定義 -->
    <node pkg="pub_sub_comm" exec="talker" name="talker_renamed2" namespace="namespace_app1">
        <remap from="chatter" to="chatter_app2"/>
    </node>
</launch>
```

XMLで記述する場合、要素と属性の組み合わせで情報を記述する。  

## 実行

### 設定ファイルの更新

パッケージ内の設定ファイルに、追加したlaunchファイルに関する記述を追加する。

[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_1/../ros2_launch_1/package.xml)には、下記の部分を追記する。  
このようにすることで、パッケージのビルド後に`ros2 launch`コマンドが通ることと、すべてのlaunchファイルのフォーマットが認識されることが保証される。  
追記する行とその周辺を以下に示す。

```diff
   <depend>pub_sub_comm</depend>
+
+  <exec_depend>ros2launch</exec_depend>

   <test_depend>ament_lint_auto</test_depend>
   <test_depend>ament_lint_common</test_depend>
```

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_launch_1/../ros2_launch_1/CMakeLists.txt)には、以下の部分を追記する。  
このようにすることで、launchファイルがワークスペースにインストールされる。  
追記する行とその周辺を以下に示す。

```diff
  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(pub_sub_comm REQUIRED)

+  
+ # install launch files
+ install(
+   DIRECTORY launch
+   DESTINATION share/${PROJECT_NAME}/
+ )
  
```

### ビルド

ワークスペースのトップディレクトリに移動し、`colcon build`コマンドを実行する(*注)。

```bash
cd ~/ros2_lecture_ws
colcon build --packages-select pub_sub_comm ros2_launch_1
. install/setup.bash
```

(*注)：launchファイルをワークスペースにインストールし、launchファイルを`ros2 launch`コマンドとして実行するためには、`colcon build`コマンドの実行が必要である。パッケージの中にコンパイルやリンクを必要とするファイルがない場合でも、コマンドの実行が必要であることに注意する。

### Pythonでの起動方法

`ros2 launch <package_name> <launch_file>`という構文で、launchファイルを実行できる。

```bash
# 構文：ros2 launch <package_name> <launch_file>
# 1つ目のターミナル：Publisherの同時起動
ros2 launch ros2_launch_1 publisher_nodes.launch.py

# 2つ目のターミナル：Subscriberの同時起動
ros2 launch ros2_launch_1 subscriber_nodes.launch.py
```

### XMLでの起動方法

Pythonのときと同様に、指定するファイル名をxmlにすることで、同様に実行できる。

```bash
# 構文：ros2 launch <package_name> <launch_file>

# 1つ目のターミナル：Publisherの同時起動
ros2 launch ros2_launch_1 publisher_nodes.launch.xml

# 2つ目のターミナル：Subscriberの同時起動
ros2 launch ros2_launch_1 subscriber_nodes.launch.xml
```

どちらの方法でも、1つのターミナルで同時にPublisherとSubscriberを起動することができる。  

ROS 2システム上で使用するlaunchファイルのフォーマットは統一されていなくてもよい。  
例えば、以下の状況において`A.launch`と`B.launch`を同時に起動できる。  

- あるlaunchファイル`A.launch`はPythonで書かれており、ノード`A1`と`A2`を起動する
- 別のlaunchファイル`B.launch`はXMLで書かれており、ノード`B1`と`B2`を起動する

また、ROS 2インタフェースが合っていれば、ノード`A1`と`B1`の通信ができる。  
つまり、ノードはどのlaunchファイルから起動されたかに関係なく通信できる。  

## 結果

ターミナルを開き、ワークスペースのセットアップスクリプトを実行してから、Publisherを起動する。  
すると、ログ出力から、異なる名前の2つのPublisherが同時に起動しており、同じ内容のトピックをPublishしていることがわかる。

```bash
$ ros2 launch ros2_launch_1 publisher_nodes.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [talker-1]: process started with pid [3601]
[INFO] [talker-2]: process started with pid [3603]
[talker-2] [INFO] [1710231655.950076094] [namespace_app2.talker_renamed2]: publish: hello, world!
[talker-1] [INFO] [1710231655.950079658] [namespace_app1.talker_renamed1]: publish: hello, world!
[talker-2] [INFO] [1710231656.050054310] [namespace_app2.talker_renamed2]: publish: hello, world!
[talker-1] [INFO] [1710231656.050082519] [namespace_app1.talker_renamed1]: publish: hello, world!
[talker-2] [INFO] [1710231656.149976967] [namespace_app2.talker_renamed2]: publish: hello, world!
[talker-1] [INFO] [1710231656.150041808] [namespace_app1.talker_renamed1]: publish: hello, world!
[talker-2] [INFO] [1710231656.250055714] [namespace_app2.talker_renamed2]: publish: hello, world!
[talker-1] [INFO] [1710231656.250088787] [namespace_app1.talker_renamed1]: publish: hello, world!
[talker-2] [INFO] [1710231656.350064832] [namespace_app2.talker_renamed2]: publish: hello, world!
[talker-1] [INFO] [1710231656.350067788] [namespace_app1.talker_renamed1]: publish: hello, world!
```

同様に、別のターミナルを起動し、以下のコマンドでSubscriberを起動する。  
Publisherと同様に、`listener_renamed1`と`listener_renamed2`のノードが起動しており、ともにメッセージを受け取っていることがわかる。

```bash
$ ros2 launch ros2_launch_1 subscriber_nodes.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [listener-1]: process started with pid [3626]
[INFO] [listener-2]: process started with pid [3628]
[listener-1] [INFO] [1710231660.649992085] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-2] [INFO] [1710231660.650022050] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[listener-1] [INFO] [1710231660.750524197] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-2] [INFO] [1710231660.750588888] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[listener-1] [INFO] [1710231660.850591883] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-2] [INFO] [1710231660.850660257] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[listener-1] [INFO] [1710231660.950597642] [namespace_app1.listener_renamed1]: subscribe: hello, world!
[listener-2] [INFO] [1710231660.950683223] [namespace_app2.listener_renamed2]: subscribe: hello, world!
[listener-1] [INFO] [1710231661.050594018] [namespace_app1.listener_renamed1]: subscribe: hello, world!
```

ノードとトピックの関係を図示するのに、`rqt_graph`コマンドを使用できる。  
3つ目のターミナルを開き、以下のコマンドを実行する。

```bash
$ rqt_graph
```

すると、以下のようなグラフが表示される。  

以下の図は、丸がノード、四角がトピックを示している。  
図より、2組のPublisherとSubscriberが、別々のnamespaceをつけられた状態で通信していることが分かる。  
例えば、ノード`/namespace_app1/taker_renamed1`は、トピック`/namespace_app1/chatter_app1`にメッセージをPublishしている。

![rqt_graph](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/ros2_launch_1/img/rqt_graph.png)

## launchファイルを記述する言語

現在のROS 2ではlaunchファイルの記述にPython, XML, YAMLの3種類の形式が使用できる。ROS 1ではXMLを用いることが標準であるが、ROS 2ではPythonを用いることが多い。  

その理由としては、以下の点が挙げられる。

- Pythonはスクリプト言語であるため、制御構造の扱いやすさ、言語のライブラリ、デバッグのしやすさなどの点で優れている。
- ROS 2のlaunchシステム自体がPythonで実装されているため、より低レベルの機能を利用できる。

上記の長所の裏返しとして、本記事の2つのlaunchファイルを比較して分かるように、Pythonで記述したlaunchファイルは複雑で冗長になりやすい面がある。

なお、ROS 1のlaunchファイルをROS 2のXML形式のlaunchファイルに移行する際には[公式サイトのガイド](https://docs.ros.org/en/humble/How-To-Guides/Launch-files-migration-guide.html)が参考になる。  

## 参考

- [ROS講座04 roslaunch1](https://qiita.com/srs/items/d7b0be3392a3a224b02f)
- [ROS 2公式チュートリアル：Launch](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2公式ドキュメント: Using Python, XML, and YAML for ROS 2 Launch Files](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html)
- [ros2/launch: Architecture of launch](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)
