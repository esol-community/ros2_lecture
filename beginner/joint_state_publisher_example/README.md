# 実習ROS 2 joint_stateをpublishする

## 環境

本記事は以下の環境を想定して記述している。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

[実習ROS 2 URDFを記述する2](https://qiita.com/esol-h-matsumoto/items/c81e818224748dd6a73b)では、駆動するジョイントの角度をjoint_state_publisher_guiのウインドウから操作し、ロボットモデルを動かした。  

ただし、一般的にジョイント角度は外界の変化や制御指令に応じて変化させることが多い。その場合、プログラム内部でジョイント角度を変化させる仕組みが必要である。  

そこで、この記事では以下の内容を扱う。

- Rvizでロボットモデルの動きが表示される仕組み
- joint_stateを計算してpublishするノードの実装

## Rvizでロボットモデルの動きが表示される仕組み

[実習ROS 2 URDFを記述する2](https://qiita.com/esol-h-matsumoto/items/c81e818224748dd6a73b)でロボットモデルを表示させたときは、いくつかのノードが起動している。  
それぞれのノードと、入出力（トピックのPublish, Subscribeも含まれる）を列挙すると、以下のようになる。

- joint_state_publisher_guiノード
  - 役割：GUIウインドウを表示し、スライダーの操作量をjoint_stateトピックに変換してPublishする
  - 入力：ユーザのGUIスライダー操作
  - 出力：joint_statesトピック
- robot_state_publisherノード
  - 役割：urdfファイルの情報とjoint_stateトピックから、ロボットモデルとtfの情報をPublishする
  - 入力：joint_statesトピック
  - 出力：robot_description, tf, tf_staticトピック
- rviz2ノード
  - 役割：ROS 2トピックを可視化する
  - 入力：robot_descriptionトピック
  - 出力：GUIウインドウ上でのグラフィック表示

robot_descriptionトピックは`std_msgs/msg/String`型のトピックで、urdfファイルの内容が含まれている。  
joint_statesトピックは`sensor_msgs/msg/JointState`型([参考](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg))のトピックで、ジョイント名、ジョイント位置(角度)、速度、加速度を保持できる。

rqt_graph(参考：[実習ROS 2：ROStools](https://qiita.com/s-kubota/items/e9d69b44d6659d44e95c))でノードの関係を表示させると、以下のようになる(図中ではtfがtransform_listenerとして存在しており、またデバッグ用のノードが余分に描かれている)。

![rqt_graph_urdf_tutorial](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/joint_state_publisher_example/./.img/rqt_graph_urdf_tutorial.png)

以降では、joint_state_publisher_guiの代わりに、プログラム上で計算したジョイント角度をPublishするノードを実装する。  

## joint_stateを計算してpublishするノードの実装

### パッケージの作成

任意のROS 2ワークスペースに移動したあと、以下のコマンドでパッケージ`joint_state_publisher_example`を作成する。  

```sh
cd ~/ros2_lecture_ws/src
ros2 pkg create joint_state_publisher_example --build-type ament_cmake
```

### ソースコードの実装

[joint_state_publisher_example.cpp](https://github.com/esol-community/ros2_lecture/tree/main/beginner/joint_state_publisher_example/./src/joint_state_publisher_example.cpp)ファイルを作成し、ノードを実装する。  

このノードは、`simple_urdf4.urdf`の2つの可動ジョイントに対して、100msごとにjoint_statesトピックをPublishする。  
角度は三角関数を用いて周期的に変化するようにしている。  

```cpp
#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStatePublisherExample : public rclcpp::Node
{
public:
  JointStatePublisherExample()
  : Node("joint_state_publisher_example"), count_(0)
  {
    pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states",
      rclcpp::SystemDefaultsQoS());
    timer_ =
      this->create_wall_timer(100ms, std::bind(&JointStatePublisherExample::TimerCallback, this));
  }

private:
  void TimerCallback()
  {
    // JointState型のメッセージ
    sensor_msgs::msg::JointState joint_state;

    // タイムスタンプを格納
    const auto stamp = this->now();
    joint_state.header.stamp = stamp;

    // 各要素の配列のサイズを揃える
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.effort.resize(2);

    // joint名を格納
    joint_state.name[0] = "body1_joint";
    joint_state.name[1] = "body2_joint";

    // jointの値を格納
    joint_state.position[0] = 1.5 * std::sin(count_ / 10.0);
    joint_state.position[1] = 0.2 * (std::cos(count_ / 5.0) - 1);

    // joint_stateをpublish
    pub_joint_state_->publish(joint_state);

    // 状態をターミナルに出力
    RCLCPP_INFO(
      this->get_logger(), "%s : %4lf [rad], %s : %4lf [rad]", joint_state.name[0].c_str(),
      joint_state.position[0], joint_state.name[1].c_str(), joint_state.position[1]);

    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  uint32_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisherExample>());
  rclcpp::shutdown();
  return 0;
}

```

### launchファイルの実装

[joint_display.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/joint_state_publisher_example/./launch/joint_display.launch.py)ファイルを作成し、launchファイルを作成する。  
launchファイルについては、[実習ROS 2 ROS 2 Launch 1：概要](https://qiita.com/s-kitajima/items/3b17d1c4a248299cc026)や[実習ROS 2 ROS 2 Launch 2：応用](https://qiita.com/s-kitajima/items/ef113900656aa2ba4f59)で解説している。  

このlaunchファイルでは、以下の3つのノードを起動している。  

- robot_state_publisher
- joint_state_publisher_example_node(今回実装したノード)
- rviz2

また、以下の引数を定義し、デフォルト値を設定している。

- rviz2のコンフィグファイルを指定する引数

```python
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

def generate_launch_description():
    # install先からパッケージへのパスを取得
    ros2_lecture_urdf2_path = get_package_share_path("urdf2")
    urdf_tutorial_path = get_package_share_path('urdf_tutorial')

    # デフォルトのurdfファイルへのパス
    default_model_path = ros2_lecture_urdf2_path / 'urdf/simple_urdf5.urdf'
    # デフォルトのRvizの設定ファイルへのパス
    default_rviz_config_path = urdf_tutorial_path / 'rviz/urdf.rviz'

    # 引数の定義
    rviz_arg  = DeclareLaunchArgument(
      'rvizconfig', 
      default_value=str(default_rviz_config_path),
      description='Absolute path to rviz config file.',
   )

    # urdfファイルの中身を読む
    with open(default_model_path, 'r') as file:
        robot_description = file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_example',
        executable='joint_state_publisher_example_node',
        namespace='',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
```

### ビルド

[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/beginner/joint_state_publisher_example/./package.xml)に、パッケージの情報を適切に記述する。  
今回は、`sensor_msgs`、`rviz2`、 `robot_state_publisher`、`urdf_tutorial`パッケージへの依存関係の追加が必要になる。  
記述例を以下に示す。

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>joint_state_publisher_example</name>
  <version>0.1.0</version>
  <description>The example package to publish joint state</description>
  <maintainer email="maintainer@example.com">Maintainer Name</maintainer>
  <license>2-Clause BSD License</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rclcpp</build_depend>
  <build_depend>sensor_msgs</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>urdf_tutorial</exec_depend>


  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/beginner/joint_state_publisher_example/./CMakeLists.txt)にはビルド情報を記述する。
ビルド時に`sensor_msgs`への依存が必要になるほか、launchファイルをインストールするための設定を追記する。  
記述例を以下に示す。

```cmake
cmake_minimum_required(VERSION 3.5)
project(joint_state_publisher_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_state_publisher_example_node
  src/joint_state_publisher_example.cpp
)

ament_target_dependencies(joint_state_publisher_example_node
  rclcpp
  sensor_msgs
)

# install executable
install(TARGETS
  joint_state_publisher_example_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```

これらのファイルの説明は、[実習ROS 2 CMakeLists.txtとpackage.xmlの書き方](https://qiita.com/s-kitajima/items/3f4c7f2dd2d9e5e5a792)に記載されている。

ビルド設定を記述したら、ワークスペースのトップディレクトリで、パッケージを指定してビルドする。

```sh
cd ~/ros2_lecture_ws
colcon build --packages-up-to joint_state_publisher_example urdf2
```

## 実行結果

`. install/setup.bash`を実行する。続いてlaunchファイルを起動すると、複数のノードが起動し、自動的にジョイントが動くことが確認できる。

```sh
$ ros2 launch joint_state_publisher_example joint_display.launch.py 
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [151578]
[INFO] [joint_state_publisher_example_node-2]: process started with pid [151580]
[INFO] [rviz2-3]: process started with pid [151582]
[robot_state_publisher-1] Parsing robot urdf xml string.
[robot_state_publisher-1] Link body1_link had 1 children
[robot_state_publisher-1] Link body2_link had 1 children
[robot_state_publisher-1] Link body3_link had 0 children
[robot_state_publisher-1] [INFO] [1669337840.264192152] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1669337840.264240149] [robot_state_publisher]: got segment body1_link
[robot_state_publisher-1] [INFO] [1669337840.264243925] [robot_state_publisher]: got segment body2_link
[robot_state_publisher-1] [INFO] [1669337840.264246107] [robot_state_publisher]: got segment body3_link
[joint_state_publisher_example_node-2] [INFO] [1669337840.363153336] [joint_state_publisher_example]: body2_joint : 0.000000 [rad], body3_joint : 0.000000 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337840.463231517] [joint_state_publisher_example]: body2_joint : 0.149750 [rad], body3_joint : -0.003987 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337840.563132757] [joint_state_publisher_example]: body2_joint : 0.298004 [rad], body3_joint : -0.015788 [rad]
[rviz2-3] [INFO] [1669337840.647937981] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1669337840.648124650] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[joint_state_publisher_example_node-2] [INFO] [1669337840.663131659] [joint_state_publisher_example]: body2_joint : 0.443280 [rad], body3_joint : -0.034933 [rad]
[rviz2-3] [INFO] [1669337840.671580878] [rviz2]: Stereo is NOT SUPPORTED
[joint_state_publisher_example_node-2] [INFO] [1669337840.763162827] [joint_state_publisher_example]: body2_joint : 0.584128 [rad], body3_joint : -0.060659 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337840.863282720] [joint_state_publisher_example]: body2_joint : 0.719138 [rad], body3_joint : -0.091940 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337840.963315455] [joint_state_publisher_example]: body2_joint : 0.846964 [rad], body3_joint : -0.127528 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337841.063247247] [joint_state_publisher_example]: body2_joint : 0.966327 [rad], body3_joint : -0.166007 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337841.163362686] [joint_state_publisher_example]: body2_joint : 1.076034 [rad], body3_joint : -0.205840 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337841.263307845] [joint_state_publisher_example]: body2_joint : 1.174990 [rad], body3_joint : -0.245440 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337841.363320652] [joint_state_publisher_example]: body2_joint : 1.262206 [rad], body3_joint : -0.283229 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337841.463323866] [joint_state_publisher_example]: body2_joint : 1.336811 [rad], body3_joint : -0.317700 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337841.563226744] [joint_state_publisher_example]: body2_joint : 1.398059 [rad], body3_joint : -0.347479 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337841.663204973] [joint_state_publisher_example]: body2_joint : 1.445337 [rad], body3_joint : -0.371378 [rad]
[rviz2-3] Parsing robot urdf xml string.
[joint_state_publisher_example_node-2] [INFO] [1669337841.763164383] [joint_state_publisher_example]: body2_joint : 1.478175 [rad], body3_joint : -0.388444 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337841.863446470] [joint_state_publisher_example]: body2_joint : 1.496242 [rad], body3_joint : -0.397998 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337841.963300624] [joint_state_publisher_example]: body2_joint : 1.499360 [rad], body3_joint : -0.399659 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337842.063337811] [joint_state_publisher_example]: body2_joint : 1.487497 [rad], body3_joint : -0.393360 [rad]
[joint_state_publisher_example_node-2] [INFO] [1669337842.163231861] [joint_state_publisher_example]: body2_joint : 1.460771 [rad], body3_joint : -0.379352 [rad]
```

![result](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/joint_state_publisher_example/./.img/result.png)

## 参考

- [ROS講座16: joint_stateをpublishする](https://qiita.com/srs/items/3eb58fd32eee4d84a530)
- [実習ROS 2 URDFを記述する2](https://qiita.com/esol-h-matsumoto/items/c81e818224748dd6a73b)
- [ros/robot_state_publisherリポジトリ](https://github.com/ros/robot_state_publisher/tree/humble)
- [sensor_msgs/msg/JointState Message Reference](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg)
