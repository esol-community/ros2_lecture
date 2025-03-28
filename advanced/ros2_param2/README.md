# 実習ROS 2 ros2 paramを使う2

## 環境

本記事は以下の環境を想定して記述している。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

この記事では[ros2 paramを使う1](https://qiita.com/s-kitajima/items/995de6d937a9a938a495)に続いて、ROS 2のパラメータの処理に関する以下の内容を扱う。

- 現在のROS 2でサポートされているパラメータ型について
- yamlファイルからパラメータを渡すプログラムの実装
- リスト型のパラメータを使用する例

## パラメータの型について

ROS 2のパラメータで使用可能な型を以下に示す([参考](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html))。  

ROS 1では、辞書型や異なる複数の型をもつパラメータを扱うことができた。しかし、**現状のROS 2(Jazzyまで)では扱うことができない。**  
そのため、扱えるパラメータの制約が大きく、YAMLファイルで表現できる全ての形式のパラメータを扱えないことに注意が必要である。

- bool
- int64
- float64
- string
- byte[]
- bool[]
- int64[]
- float64[]
- string[]

※注  
ROS 2のパラメータ型と、C++, Pythonの型との対応は、[ROS 2公式ドキュメント](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html#field-types)に記載がある。

なお、[公式ドキュメント](https://docs.ros.org/en/humble/How-To-Guides/Parameters-YAML-files-migration-guide.html)に以下のような記述があるので、今後扱える型の種類が増える可能性がある。

> #### Feature parity
> Some features of ROS 1 parameters files do not exist in ROS 2:
>
> - Mixed types in a list is not supported yet ([related issue](https://github.com/ros2/rcl/issues/463))
>
> - deg and rad substitutions are not supported

## 実装

[ros2 paramを使う1](https://qiita.com/s-kitajima/items/995de6d937a9a938a495)と同様に、パッケージを作成する。  
任意のROS 2ワークスペースに移動したあと、以下のコマンドでパッケージ`ros2_param2`を作成する。  

```sh
ros2 pkg create ros2_param2 --build-type ament_cmake
```

### YAMLファイルでのパラメータの記述

パッケージ内に`config`ディレクトリを作成し、[params.yaml](https://github.com/esol-community/ros2_lecture/tree/main/advanced/ros2_param2/./config/params.yaml)ファイルを作成する。  
YAML形式で以下のようにパラメータを定義する。  
以下の例では、`param_talker_advanced`ノード用のパラメータを複数定義している。

```yaml
# config/params.yaml
param_talker_advanced:                        # ノード名をキーとして先頭に置く 
  ros__parameters:                            # 固定名のキー
    integer_param: 10                         # 以下、パラメータ
    string_param: "Hello world"
    boolean_params: [true, true, false, true]
    list_param:                               # キーを階層構造にすることも可能
      double_param : 3.14
```

先頭のキーはノード名となっている(`param_talker_advanced`)。そのため、launchファイルやオプションでノード名を変更した際にはYAMLファイルも変更が必要になる。

### ノードの実装

用意したYAMLファイルを読み込み、値を確認するためのノードを実装する。  
`src`ディレクトリ中に、[param_talker_advanced.cpp](https://github.com/esol-community/ros2_lecture/tree/main/advanced/ros2_param2/./src/param_talker_advanced.cpp)ファイルを作成し、ノードを実装する。  

ノードの実装の構造は[ros2 paramを使う1](https://qiita.com/s-kitajima/items/995de6d937a9a938a495)で説明したものと同様である。  
クラスのコンストラクタで各種のパラメータの宣言とタイマの設定を行い、タイマのコールバック関数でパラメータの値を取得、表示する。

```cpp
#include <chrono>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamTalkerAdvanced : public rclcpp::Node
{
public:
  ParamTalkerAdvanced()
  : Node("param_talker_advanced")
  {
    // パラメータの宣言
    this->declare_parameter<int64_t>("integer_param", 100);
    this->declare_parameter<std::string>("string_param", "foo");
    this->declare_parameter<std::vector<bool>>("boolean_params", std::vector<bool>{true});
    this->declare_parameter<double>("list_param.double_param", 0.0);

    // タイマの生成
    // 1000msごとに、timer_callback()関数が呼ばれるようにする
    timer_ = this->create_wall_timer(1000ms, std::bind(&ParamTalkerAdvanced::timer_callback, this));
  }

private:
  // タイマによって呼び出される関数
  void timer_callback()
  {
    // 整数型パラメータの取得と表示
    int64_t int_param;
    this->get_parameter("integer_param", int_param);
    RCLCPP_INFO(this->get_logger(), "integer_param: %ld", int_param);

    // 文字列型パラメータの取得と表示
    std::string str_param;
    this->get_parameter("string_param", str_param);
    RCLCPP_INFO(this->get_logger(), "string_param: %s", str_param.c_str());

    // 真偽型パラメータの取得と表示
    std::vector<bool> boolean_params;
    std::stringstream ss;
    std::string del = "";
    this->get_parameter("boolean_params", boolean_params);
    ss << "[";
    for (auto v : boolean_params) {
      ss << del << v;
      del = ", ";
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "boolean_params: %s", ss.str().c_str());

    // 実数型パラメータの取得と表示
    double double_param;
    this->get_parameter("list_param.double_param", double_param);
    RCLCPP_INFO(this->get_logger(), "list_param.double_param: %lf\n", double_param);
  }

  // 一定周期で処理を実行するタイマ
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamTalkerAdvanced>());
  rclcpp::shutdown();

  return 0;
}
```

### launchファイルの実装

パッケージ内の`launch`フォルダに[param_talker_advanced.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/advanced/ros2_param2/./launch/param_talker_advanced.launch.py)を作成し、launchファイルを実装する。  

launchファイルでは、パラメータが定義されたYAMLファイルのパスを取得し、それをノードに渡す処理を記述する。  
以下の実装では、ファイルへの絶対パスを表す文字列`config_path`を取得し、それをノードの`parameters`引数に渡している。  
また、`config_path`は以下のパスを`os.path.join()`を用いて結合して得ている。

- `get_package_share_directory()`によって得られたインストール先のパッケージの場所
- `config`ディレクトリ
- `params.yaml`ファイル

```python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # yamlファイルのパスを取得
    config_path = os.path.join(
        get_package_share_directory('ros2_param2'),
        'config',
        'params.yaml'
    )

    # パラメータを指定してノードを起動
    param_talker_node = Node(
        package='ros2_param2',
        executable='param_talker_advanced',
        parameters=[config_path]
    )

    ld.add_action(param_talker_node)

    return ld
```

### ビルド

[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/advanced/ros2_param2/./package.xml)をこれまでの記事（例：[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)）と同様に編集し、パッケージ情報を適切に記載する。  

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/advanced/ros2_param2/./CMakeLists.txt)もこれまでとほとんど同様に記述するが、`config`ディレクトリをインストールするように追記することが必要になる。  
これまでの記述と異なる部分を強調して以下に示す。

```diff
  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  
  add_executable(param_talker_advanced
    src/param_talker_advanced.cpp
  )
  
  ament_target_dependencies(param_talker_advanced
    rclcpp
  )
  
  install(TARGETS
    param_talker_advanced
    DESTINATION lib/${PROJECT_NAME}
  )
  
  install(
    DIRECTORY
    launch
+   config    # パラメータファイルがインストールされるように追加
    DESTINATION share/${PROJECT_NAME}/
  )
```

その後、ROS 2ワークスペースのトップディレクトリで、このパッケージをビルドする。

```sh
colcon build --packages-up-to ros2_param2
```

## 実行結果

ワークスペースのセットアップスクリプトを実行した後、launchファイルを実行する。  

実行例を以下に示す。ソースコード中で記述したパラメータではなく、YAMLファイルで与えたパラメータが表示されていることが分かる。  

```sh
$ ros2 launch ros2_param2 param_talker_advanced.launch.py 
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [param_talker_advanced-1]: process started with pid [66559]
[param_talker_advanced-1] [INFO] [1665106045.274766294] [param_talker_advanced]: integer_param: 10
[param_talker_advanced-1] [INFO] [1665106045.274852845] [param_talker_advanced]: string_param: Hello world
[param_talker_advanced-1] [INFO] [1665106045.274866644] [param_talker_advanced]: boolean_params: [1, 1, 0, 1]
[param_talker_advanced-1] [INFO] [1665106045.274870401] [param_talker_advanced]: list_param.double_param: 3.140000
[param_talker_advanced-1] 
[param_talker_advanced-1] [INFO] [1665106046.274758208] [param_talker_advanced]: integer_param: 10
[param_talker_advanced-1] [INFO] [1665106046.274799771] [param_talker_advanced]: string_param: Hello world
[param_talker_advanced-1] [INFO] [1665106046.274812689] [param_talker_advanced]: boolean_params: [1, 1, 0, 1]
[param_talker_advanced-1] [INFO] [1665106046.274816286] [param_talker_advanced]: list_param.double_param: 3.140000
[param_talker_advanced-1] 
[param_talker_advanced-1] [INFO] [1665106047.274758962] [param_talker_advanced]: integer_param: 10
[param_talker_advanced-1] [INFO] [1665106047.274784315] [param_talker_advanced]: string_param: Hello world
[param_talker_advanced-1] [INFO] [1665106047.274805575] [param_talker_advanced]: boolean_params: [1, 1, 0, 1]
[param_talker_advanced-1] [INFO] [1665106047.274808884] [param_talker_advanced]: list_param.double_param: 3.140000
[param_talker_advanced-1] 
[param_talker_advanced-1] [INFO] [1665106048.274807719] [param_talker_advanced]: integer_param: 10
[param_talker_advanced-1] [INFO] [1665106048.274853121] [param_talker_advanced]: string_param: Hello world
[param_talker_advanced-1] [INFO] [1665106048.274863529] [param_talker_advanced]: boolean_params: [1, 1, 0, 1]
[param_talker_advanced-1] [INFO] [1665106048.274866921] [param_talker_advanced]: list_param.double_param: 3.140000
[param_talker_advanced-1] 
```

また、`ros2 run`コマンドでノードを実行するときにも、パスを直接渡すことでパラメータファイルを与えることができる。  
例えば、`params.yaml`ファイルがホームディレクトリ直下にある場合(`~/params.yaml`)、以下のように実行できる。

```sh
$ ros2 run ros2_param2 param_talker_advanced --ros-args --params-file ~/params.yaml 
[INFO] [1665106154.940694528] [param_talker_advanced]: integer_param: 10
[INFO] [1665106154.940796499] [param_talker_advanced]: string_param: Hello world
[INFO] [1665106154.940811320] [param_talker_advanced]: boolean_params: [1, 1, 0, 1]
[INFO] [1665106154.940815548] [param_talker_advanced]: list_param.double_param: 3.140000

[INFO] [1665106155.940690462] [param_talker_advanced]: integer_param: 10
[INFO] [1665106155.940753139] [param_talker_advanced]: string_param: Hello world
[INFO] [1665106155.940766110] [param_talker_advanced]: boolean_params: [1, 1, 0, 1]
[INFO] [1665106155.940772962] [param_talker_advanced]: list_param.double_param: 3.140000

[INFO] [1665106156.940687680] [param_talker_advanced]: integer_param: 10
[INFO] [1665106156.940746077] [param_talker_advanced]: string_param: Hello world
[INFO] [1665106156.940755667] [param_talker_advanced]: boolean_params: [1, 1, 0, 1]
[INFO] [1665106156.940760453] [param_talker_advanced]: list_param.double_param: 3.140000

```

## 参考

- [ROS講座43 rosparamを使う2](https://qiita.com/srs/items/4a658522a7f5dea5b83f)
- [ROS 2公式ドキュメント：About parameters in ROS 2](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html)
- [ROS 2公式ドキュメント：Passing arguments to nodes via the command-line](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html)
- [ROS 2公式ドキュメント：Migrating YAML parameter files from ROS 1 to ROS 2](https://docs.ros.org/en/humble/How-To-Guides/Parameters-YAML-files-migration-guide.html#feature-parity)
