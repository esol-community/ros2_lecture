# 実習ROS 2 Componentを使う

## 環境

本記事は以下の環境を想定して記述している。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

ROSソフトウェアの基本単位はノードであり、ノード間ではトピックなどのメッセージがやりとりされる。  
多数のノードやメッセージがあれば、その分コンピュータの処理負荷やネットワークの負荷は高くなる。  
ROS 2のComponentの仕組みを用いると、コンピュータの資源を効率的に使いながらノードを実装できる。  

そこで、この記事ではComponentの仕組みを説明し、[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)で実装したノードをComponentとして実行する。

## 通常のノードとの違い

[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)で実装したように、1つのノードに対して1つのmain関数を記述すると、1つのノードの処理が1つの実行可能ファイルになる。  
つまり、プロセスとノードが1対1で対応する。  

これに対して、Componentで実装されたノードは共有ライブラリとしてビルドされる。  
したがって、ノードごとにmain関数は存在しない。また、1つのプロセス内で複数のノードが動くため、プロセス内通信による効率的なメッセージ通信が実現できる。

一方で、Componentで実装すると複数のノードを1つのプロセスで動かせるため、1つのノードのクラッシュが他のノードに影響を及ぼす可能性が生じる。  
また、メモリ空間をノード間で共有することになるため、システムに応じてより慎重なセキュリティの考慮が求められる。例えば、適切に設計をしなかった場合、悪意のあるノードが動的に読み込まれ、プロセスを共有しているノードのメモリを読み込む可能性がある。

以降で示すように、通常のノードは実装を大きく変えることなくComponent化できる。  
そのため、双方の形式の長短を踏まえた上で、システム要求や開発フェーズに合わせて実行方式を使い分けることができる。

## 実装

まず、この記事で実装するプログラムのためのパッケージを作成する。  
任意のROS 2ワークスペースに移動したあと、以下のコマンドでパッケージ`component_pubsub`を作成する。  

```sh
ros2 pkg create component_pubsub --build-type ament_cmake
```

### Publisherの実装

まず、ROSトピックをPublishするノードを実装する。  
パッケージ内の`src`ディレクトリに、[talker_component.cpp](https://github.com/esol-community/ros2_lecture/tree/main/advanced/component_pubsub/./src/talker_component.cpp)ファイルを作成し、実装する（コード全体はリンク先を参照）。  

実装は、[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)とほぼ同様のものになっている。  

大きく異なるのは、以下の4点である。

1. クラスにnamespaceを設定していること
2. main関数がないこと
3. ファイルの最後で以下のようにノードをComponentとして登録していること
4. コンストラクタの引数に`rclcpp::NodeOptions`へのconst参照を追加していること

```cpp
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(component_pubsub::SimpleTalker)
```

`RCLCPP_COMPONENTS_REGISTER_NODE`がComponentとしてノードを実装するために重要である。  
これは`rclcpp_components/register_node_macro.hpp`で定義されているマクロであり、引数で渡されたクラスをコンポーネントとして登録する。  
これだけで、ノードの実装を変えることなくComponent化することができる。  

### Subscriberの実装

Publisherと同様に、[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)で実装したSubscriberのComponent版を実装する。  

パッケージ内の`src`ディレクトリに、[listener_component.cpp](https://github.com/esol-community/ros2_lecture/tree/main/advanced/component_pubsub/./src/listener_component.cpp)ファイルを作成し、実装する（コードはリンク先を参照）。  

### ビルド

ビルドのため、package.xmlとCMakeLists.txtに適切な情報を記述する。  

[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/advanced/component_pubsub/./package.xml)では、Componentを使用するときは`rclcpp_components`を依存関係に加える必要がある。  
[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/advanced/component_pubsub/./package.xml)の全体を、`rclcpp_components`に対してビルド依存と実行時依存を記述している部分を強調して以下に示す。  

```diff
 <?xml version="1.0"?>
 <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
 <package format="3">
   <name>component_pubsub</name>
   <version>0.1.0</version>
   <description>The simple example package using component</description>
   <maintainer email="maintainer@example.com">Maintainer Name</maintainer>
   <license>2-Clause BSD License</license>
 
   <buildtool_depend>ament_cmake</buildtool_depend>
   
   <build_depend>rclcpp</build_depend>
+  <build_depend>rclcpp_components</build_depend>
   <build_depend>std_msgs</build_depend>
 
   <exec_depend>rclcpp</exec_depend>
+  <exec_depend>rclcpp_components</exec_depend>
   <exec_depend>std_msgs</exec_depend>
 
   <test_depend>ament_lint_auto</test_depend>
   <test_depend>ament_lint_common</test_depend>
 
   <export>
     <build_type>ament_cmake</build_type>
   </export>
 </package>
```

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/advanced/component_pubsub/./CMakeLists.txt)では、これまでと異なる記述が必要になる。  

まず、ノードは実行可能ファイルではなく共有ライブラリとしてビルドするので、ターゲットの定義に`add_executable()`ではなく、`add_library()`を用いる。  
また、ノードをComponentとして登録するため、ROS 2独自のcmakeマクロ`rclcpp_components_register_node()`を用いる。  

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/advanced/component_pubsub/./CMakeLists.txt)のうち、PublisherのノードをComponentとしてビルドする設定の記述を以下に示す（ファイルの全体はリンク先を参照）。  
なお、ここでは`rclcpp_components_register_node()`というマクロで`EXECUTABLE talker_component_node`を指定している。  
これにより、ノードを実行可能ファイル`talker_component_node`としてもビルドできる。  

```cmake
# talker_componentを共有ライブラリとしてビルド
add_library(talker_component SHARED
  src/talker_component.cpp
)

# リンクの設定
ament_target_dependencies(talker_component
  rclcpp
  rclcpp_components
  std_msgs
)

# Componentの登録
# talker_component_nodeという名前の実行可能ファイルを定義
rclcpp_components_register_node(talker_component
  PLUGIN "component_pubsub::SimpleTalker"
  EXECUTABLE talker_component_node
)
```

Subscriberのノードに対しても上記と同様に記述する。

これらの定義のあと、Componentをインストールする設定を記述する。  
以下では、2つのターゲット`talker_component`と`listener_component`に対して、ターゲットの種類に応じたインストール先を指定している。  

```cmake
# install settings
install(
  TARGETS talker_component listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

package.xmlとCMakeLists.txtの設定が終わったら、ワークスペースのトップディレクトリに移動し、このパッケージをビルドする。 

```sh
colcon build --packages-up-to component_pubsub
```

## 実行結果

以下で説明するコマンドを実行するターミナルでは、予めこのパッケージをビルドしたワークスペースのセットアップスクリプトを実行しておく。

### component_containerで実行する場合

`component_container`は共有ライブラリとしてのノードを読み込むためのノードである。  
1つ目のターミナルで、以下を実行する。  

```sh
ros2 run rclcpp_components component_container
```

2つ目のターミナルで、起動したノードのノード名を確認すると、`/ComponentManager`という名前であることが分かる。  

```sh
$ ros2 node list
/ComponentManager
```

引き続き2つ目のターミナルで、Componentを読み込む。  
以下のコマンドを実行すると、`/ComponentManager`にノード`/simple_listener`が読み込まれる。

```sh
$ ros2 component load /ComponentManager component_pubsub component_pubsub::SimpleListener 
Loaded component 1 into '/ComponentManager' container node as '/simple_listener'
```

続けて、`/simple_talker`もComponentとして読み込ませる。  

```sh
$ ros2 component load /ComponentManager component_pubsub component_pubsub::SimpleTalker 
Loaded component 2 into '/ComponentManager' container node as '/simple_talker'
```

`/ComponentManager`が起動している1つ目のターミナルを見ると、以下のように2つのノードが読み込まれ、Pub&Sub通信ができていることが分かる。  

```sh
$ ros2 run rclcpp_components component_container
[INFO] [1665730772.446959928] [ComponentManager]: Load Library: /home/kitajima/workspace/ros2_lecture_ws/install/component_pubsub/lib/liblistener_component.so
[INFO] [1665730772.449506634] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleListener>
[INFO] [1665730772.449562872] [ComponentManager]: Instantiate class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleListener>
[INFO] [1665730984.723191140] [ComponentManager]: Load Library: /home/kitajima/workspace/ros2_lecture_ws/install/component_pubsub/lib/libtalker_component.so
[INFO] [1665730984.724594182] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleTalker>
[INFO] [1665730984.724667909] [ComponentManager]: Instantiate class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleTalker>
[INFO] [1665730984.831828683] [simple_talker]: publish: hello, world!
[INFO] [1665730984.832617508] [simple_listener]: subscribe: hello, world!
[INFO] [1665730984.931617134] [simple_talker]: publish: hello, world!
[INFO] [1665730984.932074372] [simple_listener]: subscribe: hello, world!
[INFO] [1665730985.031598369] [simple_talker]: publish: hello, world!
[INFO] [1665730985.031842945] [simple_listener]: subscribe: hello, world!
[INFO] [1665730985.131620699] [simple_talker]: publish: hello, world!
[INFO] [1665730985.132077638] [simple_listener]: subscribe: hello, world!
[INFO] [1665730985.231679357] [simple_talker]: publish: hello, world!
[INFO] [1665730985.232148024] [simple_listener]: subscribe: hello, world!
```

この状態で2つ目のターミナルから`ros2 component list`コマンドを実行すると、実行中のComponentとコンテナの状態を確認できる。  

```sh
$ ros2 component list
/ComponentManager
  1  /simple_listener
  2  /simple_talker
```

Componentをアンロードするには、`ros2 component unload`コマンドを用いる。  
以下では`/simple_listener`をアンロードしている。引数の数値`1`は上記で確認したIDである。  

```sh
$ ros2 component unload /ComponentManager 1
Unloaded component 1 from '/ComponentManager' container node
```

### ros2 runで実行する場合

この記事の実装では、CMakeLists.txtの設定で`talker_component_node`、`listener_component_node`という実行可能形式のノードも作成していた。  
これを用いると、`ros2 run`コマンドでノードを実行できる。  

この場合、1プロセスに対して1ノードが割り当てられ、Componentの恩恵を受けられない。  

```sh
# 1つ目のターミナルで実行
$ ros2 run component_pubsub talker_component_node 
[INFO] [1665731522.784825465] [simple_talker]: publish: hello, world!
[INFO] [1665731522.884692730] [simple_talker]: publish: hello, world!
[INFO] [1665731522.984707101] [simple_talker]: publish: hello, world!
[INFO] [1665731523.084693077] [simple_talker]: publish: hello, world!
[INFO] [1665731523.184753492] [simple_talker]: publish: hello, world!
[INFO] [1665731523.284696397] [simple_talker]: publish: hello, world!
[INFO] [1665731523.384724332] [simple_talker]: publish: hello, world!
```

```sh
# 2つ目のターミナルで実行
$ ros2 run component_pubsub listener_component_node 
[INFO] [1665731543.932085246] [simple_listener]: subscribe: hello, world!
[INFO] [1665731543.932478806] [simple_listener]: subscribe: hello, world!
[INFO] [1665731544.015646957] [simple_listener]: subscribe: hello, world!
[INFO] [1665731544.115659933] [simple_listener]: subscribe: hello, world!
[INFO] [1665731544.215645162] [simple_listener]: subscribe: hello, world!
[INFO] [1665731544.315532789] [simple_listener]: subscribe: hello, world!
[INFO] [1665731544.415672830] [simple_listener]: subscribe: hello, world!
```

## launchファイルで実行する場合

「component_containerで実行する場合」で示したようなノードの起動方法は、launchファイルを用いて行うこともできる。  

[launch_composable_nodes.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/advanced/component_pubsub/./launch/launch_composable_nodes.launch.py)に、Componentを使ったノードを起動するlaunchファイルの実装例を示している。  

このlaunchファイルでは、起動時にcomponent_container、talker、listenerが読み込まれる。ノードは`chatter_remapped`トピックを介して通信する。  
そして、2秒後にtalkerとlistenerを追加で読み込ませている。  

component_containerの起動は`launch_ros.actions.ComposableNodeContainer`を用いて行う。Componentはlaunchシステムでは`launch_ros.descriptions.ComposableNode`として表現されている。  

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # component containerのアクションを作成
    action_component_container = ComposableNodeContainer(
        name="component_pubsub_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        # componentで実装されたノードを指定
        composable_node_descriptions=[
            ComposableNode(
                name="simple_talker_component",
                package="component_pubsub",
                plugin="component_pubsub::SimpleTalker",
                remappings=[("/chatter", "/chatter_remapped")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),  # プロセス内通信を有効にする
            ComposableNode(
                name="simple_listener_component",
                package="component_pubsub",
                plugin="component_pubsub::SimpleListener",
                remappings=[("/chatter", "/chatter_remapped")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),  # プロセス内通信を有効にする
        ],
        output="screen",
    )
```

Componentの読み込みは`launch_ros.actions.LoadComposableNodes`を用いて行う。

```python
from launch.actions import TimerAction
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # ...(省略)...

    # 2秒後に、別の名前でtalkerとlistenerを起動
    action_delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            # Componentを読み込むActionの定義
            LoadComposableNodes(
                # component container の名前を指定して読み込む
                target_container="component_pubsub_container",
                composable_node_descriptions=[
                    # 起動させるComponentの定義
                    ComposableNode(
                        package="component_pubsub",
                        plugin="component_pubsub::SimpleTalker",
                        name="simple_talker_component",
                        namespace="delayed_nodes",
                        remappings=[("/chatter", "/chatter_remapped")],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                    ComposableNode(
                        name="simple_listener_component",
                        package="component_pubsub",
                        plugin="component_pubsub::SimpleListener",
                        namespace="delayed_nodes",
                        remappings=[("/chatter", "/chatter_remapped")],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
        ],
    )
```

launchファイルをインストール、実行すると、以下のように2組のtalkerとlistenerが起動し、通信が行われる。

```
$ ros2 launch src/tutorial/advanced/component_pubsub/launch/launch_composable_nodes.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [component_container-1]: process started with pid [17157]
[component_container-1] [INFO] [1731936757.864028096] [component_pubsub_container]: Load Library: /home/developer/workspace/ros2_lecture_ws/install/component_pubsub/lib/libtalker_component.so
[component_container-1] [INFO] [1731936757.865812091] [component_pubsub_container]: Found class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleTalker>
[component_container-1] [INFO] [1731936757.865854751] [component_pubsub_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleTalker>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/simple_talker_component' in container '/component_pubsub_container'
[component_container-1] [INFO] [1731936757.878440936] [component_pubsub_container]: Load Library: /home/developer/workspace/ros2_lecture_ws/install/component_pubsub/lib/liblistener_component.so
[component_container-1] [INFO] [1731936757.881292642] [component_pubsub_container]: Found class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleListener>
[component_container-1] [INFO] [1731936757.881329743] [component_pubsub_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleListener>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/simple_listener_component' in container '/component_pubsub_container'
[component_container-1] [INFO] [1731936757.974769392] [simple_talker_component]: publish: hello, world!
[component_container-1] [INFO] [1731936757.975875684] [simple_listener_component]: subscribe: hello, world!
[component_container-1] [INFO] [1731936758.074811214] [simple_talker_component]: publish: hello, world!

# ..(省略)...

[component_container-1] [INFO] [1731936759.619272705] [component_pubsub_container]: Found class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleTalker>
[component_container-1] [INFO] [1731936759.619359473] [component_pubsub_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleTalker>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/delayed_nodes/simple_talker_component' in container 'component_pubsub_container'
[component_container-1] [INFO] [1731936759.626360542] [component_pubsub_container]: Found class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleListener>
[component_container-1] [INFO] [1731936759.626391865] [component_pubsub_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<component_pubsub::SimpleListener>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/delayed_nodes/simple_listener_component' in container 'component_pubsub_container'
[component_container-1] [INFO] [1731936759.674863474] [simple_talker_component]: publish: hello, world!
[component_container-1] [INFO] [1731936759.675268268] [simple_listener_component]: subscribe: hello, world!
[component_container-1] [INFO] [1731936759.725756214] [delayed_nodes.simple_talker_component]: publish: hello, world!
[component_container-1] [INFO] [1731936759.726079414] [delayed_nodes.simple_listener_component]: subscribe: hello, world!
[component_container-1] [INFO] [1731936759.774902138] [simple_talker_component]: publish: hello, world!
[component_container-1] [INFO] [1731936759.775305990] [simple_listener_component]: subscribe: hello, world!
[component_container-1] [INFO] [1731936759.825882235] [delayed_nodes.simple_talker_component]: publish: hello, world!
[component_container-1] [INFO] [1731936759.826286283] [delayed_nodes.simple_listener_component]: subscribe: hello, world!
[component_container-1] [INFO] [1731936759.874835872] [simple_talker_component]: publish: hello, world!
```

## ROS 1のNodeletとの違い

ROS 1では、ノードの処理を効率化するためのNodeletという仕組みがあった。  
ROS 1のNodeletに対してROS 2のComponentは以下のような設計上の利点がある。  

- ノードの処理を行うクラスの実装を変える必要がない  
  [Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)と本記事では、ノード内部の処理が変わっていない。
- ノードの実行形態をビルド後に決められる  
  「実行結果」で示したように、1プロセスに1ノードでも、複数ノードでも実行できる。


## 参考

- [ROS講座84 Nodeletを使う](https://qiita.com/srs/items/2718feee04643b2c8afa)
- [ROS 2公式ドキュメント：About Composition](https://docs.ros.org/en/humble/Concepts/About-Composition.html)
- [ROS 2公式ドキュメント：Composing multiple nodes in a single process](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html)
- [ROS 2公式ドキュメント：Using ROS 2 launch to launch composable nodes](https://docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html)
- [ros2/demosパッケージ：composition](https://github.com/ros2/demos/tree/humble/composition)
- [ROSCon2019プレゼン資料：Composable Nodes in ROS2](https://roscon.ros.org/2019/talks/roscon2019_composablenodes.pdf)
