# 実習ROS 2 カスタムROSメッセージ

## 環境

本記事は以下の環境を想定した記事である。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

ROSではトピック通信にてメッセージを使用する。このページではデフォルトで用意されたメッセージではなく、オリジナルのメッセージを作成し、それを用いたトピック通信を可能にするノードの実装方法について解説する。  
このページの内容は、[ROS講座10 カスタムROSメッセージ](https://qiita.com/srs/items/7ac023c549e585caeed0)の内容を[ROS 2公式チュートリアル](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)をもとにROS 2対応させたものである。  

## 本記事で解説していること

- カスタムメッセージを定義・利用するパッケージの作成方法

## カスタムROSメッセージの定義

ここでは、`custom_message`という名前のパッケージを作成する。  
パッケージの作成は、以下のようにして行う。

```shell
cd ~/ros2_lecture_ws/src
ros2 pkg create --build-type ament_cmake custom_message  
```

### msgファイル作成

作成した`custom_message`ディレクトリ直下に`msg`ディレクトリを作成する。

```shell
mkdir custom_message/msg
```

msgディレクトリ直下にカスタムROSメッセージを定義するファイルを置く。  
今回ファイル名は`Custom.msg`とする。  
メッセージファイルに記述できるのは、型、変数名、及びコメントである。以下にメッセージファイルについての情報をまとめる。  

- 使用できる型
  - 基本型（int64やstringなど）
  - メッセージとして定義された別のメッセージ （"geometry_msgs/msg/PoseStamped" など）
- メッセージファイルの記述方法
  - "型名 変数名"
    - メッセージとして定義された別のメッセージを使用する場合、/msg/の記述を省略する。  
    つまり、"パッケージ名/メッセージ名 変数名"となる。  
  ex) geometry_msgs/PoseStamped pose
- コメントの記述方法
  - '#'に続くテキストはコメントとして扱われ、メッセージ定義として扱われない。

今回は以下のような型と変数を定義する。  
コメントの例も同時に示している。

```txt
# 行頭コメント：カスタムメッセージの定義
string word   # インラインコメント
int64 num
```

変数名の定義には以下のような制約がある。

- 小文字の英数字であり、単語の区切りにはアンダースコアを用いる(スネークケース)
- 先頭の文字には英字を用いる
- アンダースコアで終わってはならず、アンダースコアを二つ連続して付けてはならない

また、メッセージファイル名にも以下の制約がある。

- 先頭と単語の最初が大文字で始まる（アッパーキャメルケース）
- 英数字のみを用いる
- 拡張子は`.msg`を用いる

命名規則については、以下のURLが参考になる。  
[.msg / .srv / .actionファイルを使用したインタフェース定義](http://design.ros2.org/articles/legacy_interface_definition.html)

### package.xmlの設定

カスタムメッセージをビルド・実行できるよう、package.xmlに以下の行を追加する。  

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### CMakeLists.txtの設定

今回作成したmsgファイルが使えるよう、CMakeLists.txtに以下の行を追加する。  
`find_package(rosidl_default_generators REQUIRED)`でカスタムメッセージを使えるようになる。  
`rosidl_generate_interfaces(${PROJECT_NAME} "")`の""内には使用するカスタムメッセージを指定する。

```txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Custom.msg"
  )
```

### ビルド

自身のワークスペース直下でビルドを行う。

```shell
cd　~/ros2_lecture_ws
colcon build --packages-select custom_message
```

### カスタムROSメッセージの確認

カスタムROSメッセージが正常に定義できているか確認する。  
`ros2 interface show`コマンドでメッセージ型を確認できる。
ワークスペース内で環境のセットアップを行ってから、以下のコマンドを実行する。

```shell
cd ~/ros2_lecture_ws
. install/setup.bash
ros2 interface show custom_message/msg/Custom
```

下記のような表示が出ていれば、正しくメッセージを定義できている。

```txt
# 行頭コメント：カスタムメッセージの定義
string word   # インラインコメント
int64 num
```

## カスタムROSメッセージの使用

新しく`custom_msg_pubsub`パッケージを作成し、その中のノードで`custom_message`パッケージで定義した型を使用する。  
ROS 2ではこのように、メッセージを定義するパッケージと使用するパッケージは別にすることが推奨されている。

```shell
ros2 pkg create --build-type ament_cmake custom_msg_pubsub
```

### カスタムROSメッセージを使用するノード

ノードを作成する。このノードは、[Pub&Sub通信](https://github.com/esol-community/ros2_lecture/tree/main/beginner/custom_ros2_message/../pub_sub_comm/README.md)で作成した[simple_talker.cpp](https://github.com/esol-community/ros2_lecture/tree/main/beginner/custom_ros2_message/../pub_sub_comm/src/simple_talker.cpp)をベースにして、一部変更を加えたものである。

[custom_msg_talker.cpp](https://github.com/esol-community/ros2_lecture/tree/main/beginner/custom_ros2_message/./custom_msg_pubsub/src/custom_msg_talker.cpp)

カスタムメッセージをincludeする際は、"ROSパッケージ名/msg/msgファイル名.hpp"と記述する。  

### package.xmlの設定

このパッケージはカスタムメッセージを定義した`custom_message`に依存しているため、package.xmlに下記を追加する。

```xml
<depend>custom_message</depend>
```

### CMakeLists.txtの設定

こちらも通常と同じように、必要な情報を追記する。  
特徴としては、find_package()とament_target_dependencies()にメッセ―ジを定義したパッケージ名を記述している点である。

```cmake
find_package(rclcpp REQUIRED)
find_package(custom_message REQUIRED)

add_executable(custom_msg_talker src/custom_msg_talker.cpp)
ament_target_dependencies(custom_msg_talker rclcpp custom_message) 

install(TARGETS
  custom_msg_talker
  DESTINATION lib/${PROJECT_NAME})
```

### ビルド

自身のワークスペース内でビルドを行う。

```shell
cd ros2_lecture_ws
colcon build --packages-select custom_msg_pubsub
```

### 実行

`custom_msg_talker`ノードを起動するターミナルとトピックのデータを出力するターミナルの二つを用意する。  
実行前に自身のワークスペース内で`. install/setup.bash`を実行する必要がある。以下は、これを実行したことを前提としている。

- 1つ目のターミナル

```shell
ros2 run custom_msg_pubsub custom_msg_talker
```

- 2つ目のターミナル

```shell
ros2 topic echo /chatter 
```

### 結果

1つ目のターミナルでは、

```shell
$ ros2 run custom_msg_pubsub custom_msg_talker 
[INFO] [1666672244.756119460] [custom_msg_talker]: publish: string/Hello.world! , num/10
[INFO] [1666672245.756123913] [custom_msg_talker]: publish: string/Hello.world! , num/10
[INFO] [1666672246.756120039] [custom_msg_talker]: publish: string/Hello.world! , num/10

```

2つ目のターミナルでは、

```shell
$ ros2 topic echo /chatter 
word: Hello.world!
num: 10
---
word: Hello.world!
num: 10
---
word: Hello.world!
num: 10
---

```

という結果が得られる。これでカスタムメッセージがトピックに正しくpublishされていることが確認できる。

## 参考：型

rosのmsgファイルで使用できる基本型については、以下に詳細が記載されている。

[ROS 2インターフェイスについて：2.1.1 フィールド型](http://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html#field-types)

## 参考：Subscriberの実装

Subscriberの実装方法を記述する。基本的にはPublisherを実装した手順と同じように行えば実装できる。  
今回は`custom_msg_listener.cpp`というファイル名で作成している。

[custom_msg_listener.cpp](https://github.com/esol-community/ros2_lecture/tree/main/beginner/custom_ros2_message/./custom_msg_pubsub/src/custom_msg_listener.cpp)

次に、CMakeLists.txtに必要事項を追加する。

```diff
+ add_executable(custom_msg_listener src/custom_msg_listener.cpp)  
+ ament_target_dependencies(custom_msg_listener rclcpp custom_message)

install(TARGETS
   custom_msg_talker
+  custom_msg_listener
   DESTINATION lib/${PROJECT_NAME})
```

ビルドを行い、環境のセットアップを行った後にノードを実行すると、以下のような結果が得られる。

```shell
$ ros2 run custom_msg_pubsub custom_msg_listener 
[INFO] [1666674509.152357337] [custom_msg_listener]: subscribe: Hello.world!, 10
[INFO] [1666674510.152415992] [custom_msg_listener]: subscribe: Hello.world!, 10
[INFO] [1666674511.152278470] [custom_msg_listener]: subscribe: Hello.world!, 10

```

## 参考URL

- [ROS 2公式チュートリアル：Creating custom msg and srv files](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ROS 2公式ドキュメント：ROS 2インターフェイスについて](http://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
- [ROS講座10 カスタムROSメッセージ](https://qiita.com/srs/items/7ac023c549e585caeed0)
- [.msg / .srv / .actionファイルを使用したインタフェース定義](http://design.ros2.org/articles/legacy_interface_definition.html)
- [ROS 2独自メッセージの作成](https://qiita.com/NeK/items/26751b8ed12976ff977d)
