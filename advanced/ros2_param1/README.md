# 実習ROS 2 ros2 paramを使う1

## 環境

本記事は以下の環境を想定して記述している。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

ROS 2には処理の柔軟性を高めるために外部からパラメータを与える仕組みがある。  
パラメータは以下のように静的・動的に値を与えられる。これらのパラメータはROS 2のライブラリを介して取得し、処理に使用できる。

- 静的に与える方法
  - ソースコード中にパラメータの初期値を記述する
  - launchファイルで起動時の値を設定する
  - yamlファイルに記述する
- 動的に与える方法
  - コマンドライン引数でパラメータを直接指定する
  - コマンドライン引数でyamlファイルを指定する
  - ソースコード、launchファイル、コマンドラインから、パラメータを変更するサービス通信を呼ぶ

そこでこの記事では、様々な機能があるROS 2のパラメータの使用方法のうち、以下の内容を扱う。

- パラメータを扱うROS 2のAPIを用いたノードの実装（C++）
- パラメータを指定してノードを起動するlaunchファイルの実装（Python）
- コマンドラインツール`ros2 param`を用いて、動的にパラメータを扱う方法

ノードやlaunchファイルの実装では、実装方法によって静的にも動的にもパラメータを扱える。  
本記事では初期値を静的に与え、パラメータを取得して表示するノードを実装する。

## ROS 2におけるパラメータ

ROS 2ではノードごとにパラメータを持つ。  
ノード名に紐付けられたパラメータは以下の値を持つ。

- key：パラメータ名。string型である。
- value：パラメータの値。bool/int64/stringなどから任意の1つの型である。  
- descriptor：パラメータの説明、値の範囲、読み込み専用、型の動的変更などの情報を設定できる（デフォルトでは空の値）。  

valueとして持てる型は[rcl_interfaces/msg/ParameterType.msg](https://github.com/ros2/rcl_interfaces/blob/humble/rcl_interfaces/msg/ParameterType.msg)に、descriptorとして持てる値は[rcl_interfaces/msg/ParameterDescriptor.msg](https://github.com/ros2/rcl_interfaces/blob/humble/rcl_interfaces/msg/ParameterDescriptor.msg)に定義されている。  

## ros2 paramコマンド

`ros2 param`コマンドは、コマンドライン上でROS 2のパラメータに関する操作を行うコマンドである。  
以下のサブコマンドが実装されている。　　

- `list`  
  各ノードにどのパラメータがあるのかを一覧表示する。  
  ex) `ros2 param list`
- `get`  
  ノードが持つパラメータの値を取得し、表示する。  
  ex) `ros2 param get <ノード名> <パラメータ名>`
- `set`  
  ノードが持つパラメータの値を更新する。  
  ex) `ros2 param set <ノード名> <パラメータ名> <値>`
- `load`  
  指定したノードに対して、パラメータファイルからパラメータを読み込む。  
  ex) `ros2 param load <ノード名> <ファイル名>`
- `dump`  
  指定したノードのパラメータをファイルに書き出す。  
  ex) `ros2 param dump <ノード名>`
- `delete`  
  指定したノードが持つパラメータを削除する。  
  (値を空にするのではなく、パラメータの存在をなくす)  
  ex) `ros2 param delete <ノード名> <パラメータ名>`
- `describe`  
  パラメータのdescriptionを表示する。  
  ex) `ros2 param describe <ノード名> <パラメータ名>`

上記のうち幾つかのコマンドについては、実行した結果を以降の節で示す。

## 実装

まず、パッケージを作成する。  
任意のROS 2ワークスペースに移動したあと、以下のコマンドでパッケージ`ros2_param1`を作成する。  

```sh
ros2 pkg create ros2_param1 --build-type ament_cmake
```

### ノードの実装

パッケージ下の`src`ディレクトリに`param_talker.cpp`を作成し、ノードを実装する。  

ここでは、ParamTalkerというクラスを定義し、コンストラクタでパラメータの定義とタイマの登録を行う。
定義しているパラメータはstring型の`parameter1`(値"foo"で初期化)、int型の`parameter2`(値100で初期化)である。  
以下のコードのコメントにあるように、`parameter1`は読み書き可能だが、`parameter2`は読み込み専用とした。  
また、`parameter1`は動的型な変換を許可し、`parameter2`は許可しないように設定した。

タイマ処理は定期的に呼び出され、`parameter1`という名前がノードのパラメータとして宣言されているかをチェックする。  
宣言されている場合はそのパラメータの値とdescriptionを取得し、ターミナルへ情報を表示する。  
宣言されていない場合はその旨のメッセージを出力する。  

```cpp
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamTalker : public rclcpp::Node
{
public:
  ParamTalker()
  : Node("param_talker")
  {
    auto desc1 = rcl_interfaces::msg::ParameterDescriptor();
    // パラメータ1の説明を追加
    desc1.description = "sample parameter 1";
    // パラメータ1は書き込み可能とする
    desc1.read_only = false;
    // パラメータ1は動的型な変換を許可する
    desc1.dynamic_typing = true;

    auto desc2 = rcl_interfaces::msg::ParameterDescriptor();
    // パラメータ2の説明を追加
    desc2.description = "sample parameter 2";
    // パラメータ2は読み込み専用とする
    desc2.read_only = true;
    // パラメータ2は動的型な変換を許可しない
    desc2.dynamic_typing = false;

    // このノードが持つパラメータparameter1, parameter2を宣言
    // デフォルト値は"foo"
    this->declare_parameter<std::string>("parameter1", "foo", desc1);
    this->declare_parameter<int32_t>("parameter2", 100, desc2);

    // タイマの生成
    // 1000msごとに、timer_callback()関数が呼ばれるようにする
    timer_ = this->create_wall_timer(1000ms, std::bind(&ParamTalker::timer_callback, this));
  }

private:
  // タイマによって呼び出される関数
  void timer_callback()
  {
    const std::string param_name("parameter1");

    // パラメータを取得
    if (this->has_parameter(param_name)) {
      this->get_parameter(param_name, param_value_);
      auto descriptor = this->describe_parameter(param_name);

      // ターミナルへ情報表示
      RCLCPP_INFO(this->get_logger(), "%-12s: %s", descriptor.name.c_str(), param_value_.c_str());
      RCLCPP_INFO(this->get_logger(), "%-12s: %d", "type", descriptor.type);
      RCLCPP_INFO(this->get_logger(), "%-12s: %s", "description", descriptor.description.c_str());
    } else {
      // パラメータが存在しない場合、warningを表示
      RCLCPP_WARN(this->get_logger(), "No declared parameter: %s", param_name.c_str());
    }
  }

  // パラメータ文字列
  std::string param_value_;

  // 一定周期で処理を実行するタイマ
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamTalker>());
  rclcpp::shutdown();

  return 0;
}

```

### launchファイルの実装

パッケージ下の`launch`ディレクトリに`param_talker.launch.py`を作成し、launchファイルを実装する。  

ここでは、`parameter1`の値を"hello"、`parameter2`の値を200として、ノードを起動する処理を記述する。  
これらの値は上記のソースコードに書かれた初期値を上書きする。

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # ノードに与えるパラメータを定義
    node_params = [
        {"parameter1": "hello"},
        {"parameter2": 200}
    ]

    # パラメータを指定してノードを起動
    param_talker_node = Node(
        package="ros2_param1",
        executable="param_talker",
        parameters=node_params
    )

    ld.add_action(param_talker_node)

    return ld

```

## ビルド

[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/advanced/ros2_param1/./package.xml)と、[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/advanced/ros2_param1/./CMakeLists.txt)をこれまでの講座（例：[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)）と同様に編集し、パッケージ情報とビルド情報を適切に記載する。  

その後、ROS 2ワークスペースのトップディレクトリで、このパッケージをビルドする。

```sh
colcon build --packages-up-to ros2_param1
```

## 実行結果

### ノードの実行

まず、launchファイルからノードを実行する。  
`parameter1`の値、型([ParameterType](https://github.com/ros2/rcl_interfaces/blob/humble/rcl_interfaces/msg/ParameterType.msg)に対応する数値)、説明が出力されている。  
このとき、`parameter1`の値はC++ソースコード中で指定した`foo`ではなく、launchファイル中で与えた`hello`になっている。

```sh
$ ros2 launch ros2_param1 param_talker.launch.py 
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [param_talker-1]: process started with pid [166210]
[param_talker-1] [INFO] [1664246308.369849034] [param_talker]: parameter1  : hello
[param_talker-1] [INFO] [1664246308.370156830] [param_talker]: type        : 4
[param_talker-1] [INFO] [1664246308.370191636] [param_talker]: description : sample parameter 1
[param_talker-1] [INFO] [1664246309.369838897] [param_talker]: parameter1  : hello
[param_talker-1] [INFO] [1664246309.369973215] [param_talker]: type        : 4
[param_talker-1] [INFO] [1664246309.369991861] [param_talker]: description : sample parameter 1
[param_talker-1] [INFO] [1664246310.369819495] [param_talker]: parameter1  : hello
[param_talker-1] [INFO] [1664246310.369947331] [param_talker]: type        : 4
[param_talker-1] [INFO] [1664246310.369967733] [param_talker]: description : sample parameter 1
[param_talker-1] [INFO] [1664246311.369846976] [param_talker]: parameter1  : hello
[param_talker-1] [INFO] [1664246311.369970402] [param_talker]: type        : 4
[param_talker-1] [INFO] [1664246311.369989551] [param_talker]: description : sample parameter 1
[param_talker-1] [INFO] [1664246312.369884856] [param_talker]: parameter1  : hello
[param_talker-1] [INFO] [1664246312.370004976] [param_talker]: type        : 4
[param_talker-1] [INFO] [1664246312.370023551] [param_talker]: description : sample parameter 1
```

### ros2 paramコマンドの実行

この節では、前節のプログラムを実行させたまま別のターミナルを立ち上げ、`ros2 param`コマンドを実行したときの結果を確認する。  

`ros2 param list`コマンドでパラメータ一覧を取得すると、`parameter1`と`parameter2`が存在することがわかる。

```sh
$ ros2 param list
/param_talker:
  parameter1
  parameter2
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
```

`ros2 param get`コマンドを用いて、C++ソース中では取得していない`parameter2`を取得してみる。  
launchファイル中で与えた200が入っていることがわかる。

```sh
# 使い方：ros2 param get node_name parameter_name
$ ros2 param get /param_talker parameter2
Integer value is: 200
```

`ros2 param set`コマンドを用いると、コマンドライン上でパラメータを指定できる。

```sh
# 使い方：ros2 param get node_name parameter_name value
$ ros2 param set /param_talker parameter1 world
Set parameter successful
```

コマンド実行後、launchファイルで起動したノードの出力を見ると、`parameter1`の値が`hello`から`world`に変わっていることが確認できる。

```sh
[param_talker-1] [INFO] [1664246444.609839493] [param_talker]: description : sample parameter 1
[param_talker-1] [INFO] [1664246445.609640241] [param_talker]: parameter1  : world
[param_talker-1] [INFO] [1664246445.609781650] [param_talker]: type        : 4
[param_talker-1] [INFO] [1664246445.609818193] [param_talker]: description : sample parameter 1
[param_talker-1] [INFO] [1664246446.609730544] [param_talker]: parameter1  : world
[param_talker-1] [INFO] [1664246446.609867067] [param_talker]: type        : 4
```

なお、`parameter2`はソースコード中で読み取り専用として宣言したため、コマンドラインから値を設定することはできない。

```sh
$ ros2 param set /param_talker parameter2 0
Setting parameter failed: parameter 'parameter2' cannot be set because it is read-only
```

`ros2 param delete`コマンドを用いると、パラメータを削除できる。  
すなわち、ノードは`parameter1`という名前のパラメータを持たない状態になる。  
ただし、パラメータを削除するには宣言時にそのパラメータの型を動的に変更できるようにしておく必要がある。  

```sh
$ ros2 param delete /param_talker parameter1
Deleted parameter successfully
```

launchファイルで起動したノードの出力を見ると、上記のコマンドで`parameter1`がなくなったため、ソースコードで実装した通りにWarningが出力されている。

```sh
[param_talker-1] [WARN] [1664246521.611151486] [param_talker]: No declared parameter: parameter1
[param_talker-1] [WARN] [1664246522.611086956] [param_talker]: No declared parameter: parameter1
[param_talker-1] [WARN] [1664246523.611189387] [param_talker]: No declared parameter: parameter1
[param_talker-1] [WARN] [1664246524.611208181] [param_talker]: No declared parameter: parameter1
[param_talker-1] [WARN] [1664246525.611132905] [param_talker]: No declared parameter: parameter1
```

## 参考

- [ROS講座43 rosparamを使う](https://qiita.com/srs/items/b7a7abe313ad65b748f4)
- [ROS 2公式ドキュメント：Using parameters in a class(C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
- [ROS 2公式ドキュメント：Understanding parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [ROS 2公式ドキュメント：About parameters in ROS 2](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html)
- [ROS 2公式ドキュメント：Passing arguments to nodes via the command-line](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html)
- [rclcpp package node.hpp](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/node.hpp)
