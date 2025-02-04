# 実習ROS 2 xacroを使う2

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

このページでは、xacro(Xml mACRO)におけるマクロ機能の使い方を説明する。[実習ROS 2 xacroを使う1](https://qiita.com/esol-h-matsumoto/items/d8c0bf24840e5759f17f)で説明した定数や数式・条件式をマクロ機能と組み合わせることで、より簡単に、より分かりやすくxacroを記述できる。  
このページは、[ROS 2公式チュートリアル](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)を参考に、[ROS講座73 xacroを使う2](https://qiita.com/srs/items/9ac7f2f6e47732bb7535)の内容をROS 2対応させたものである。  

## 前準備

### 前提条件

このチュートリアルは、[実習ROS 2 Pub＆Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)を実行していることを前提に、Pub＆Sub通信のチュートリアルで作成したワークスペース`ros2_lecture_ws`を利用する。  
また、[実習ROS 2 xacroを使う1](https://qiita.com/esol-h-matsumoto/items/d8c0bf24840e5759f17f)の実施を前提とする。[実習ROS 2 xacroを使う1](https://qiita.com/esol-h-matsumoto/items/d8c0bf24840e5759f17f)で説明した、変数、数式、条件式の知識を前提に、より発展的な機能を解説する。

### ROS 2パッケージの作成

パッケージ`xacro2`およびxacroファイルを格納するディレクトリ`/xacro`、launchファイルを格納するディレクトリ`/launch`を作成する。

```bash
cd ~/ros2_lecture_ws/src
ros2 pkg create xacro2 --build-type ament_cmake --dependencies urdf_launch
mkdir ./xacro2/xacro
mkdir ./xacro2/launch
```

## モジュール化

### モジュール化の方法

1. モジュールの定義  
モジュールは以下の形式で定義する。モジュールには任意の数の引数を設定できる。

    ```text
    <xacro:macro name="*モジュール名*" params="*引数名1* *引数名2* ...">
      <モジュール内の処理>
    </xacro:macro>
    ```

    設定した引数は、モジュール内の処理の記述に利用できる。引数の値は以下のように記述すると利用できる。

    ```text
    <"${*引数名*}">
    ```

2. モジュールの利用  
定義したモジュールは以下のように書くと利用できる。

    ```text
    <xacro:*モジュール名* *引数1*="*引数1の値*" *引数2*="*引数2の値*" .../>
    ```

### モジュールを利用した例

モジュールの利用例として、xacroファイル`module.xacro`を作成する。このxacroファイルは、一辺が0.1mの大きさの立方体を、半径0.5mの円周上に30°間隔で4つ並べたものである。

```xml
<robot name="test_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  
  <!--モジュールの定義-->
  <xacro:macro name="box_macro" params="prefix parent radius">
    <joint name="${prefix}_joint" type="fixed">
      <origin rpy="0 0 0" xyz="${0.5*cos(radians(radius))} ${0.5*sin(radians(radius))} 0"/>
      <parent link="${parent}"/>
      <child  link="${prefix}_link"/>
    </joint>

    <link name="${prefix}_link">
      <visual>
        <geometry>
          <box size="0.1 0.1 0.1" />
        </geometry>
        <material name="red"/>
      </visual>
    </link>
  </xacro:macro>  

  <link name="base_link"/>

  <!--モジュールの利用-->
  <xacro:box_macro prefix="box0" parent="base_link" radius="0"/>
  <xacro:box_macro prefix="box1" parent="base_link" radius="30"/>
  <xacro:box_macro prefix="box2" parent="base_link" radius="60"/>
  <xacro:box_macro prefix="box3" parent="base_link" radius="90"/>
</robot>
```

[module.xacro](https://github.com/esol-community/ros2_lecture/tree/main/beginner/xacro2/./xacro/module.xacro)

1. モジュールの定義  
    このxacroでは、`prefix`、`parent`、`radius`という3つの引数を持つマクロ`box_macro`を定義している。  

    ```xml
    <!--モジュールの定義-->
    <xacro:macro name="box_macro" params="prefix parent radius">
      ...
    </xacro:macro>  
    ```

    モジュール内の処理では、受け取った引数を利用して、一辺が0.1mの立方体を1つ作成する。引数の利用箇所を以下に示す。  

    - 引数`prefix`を利用して、ジョイント名とリンク名を設定する。

      ```xml
        <joint name="${prefix}_joint" type="fixed">
          ...
          <child  link="${prefix}_link"/>
        </joint>

        <link name="${prefix}_link">
          ...
        </link>  
      ```

    - 引数`parent`を利用して、ジョイントの親リンクを設定する。

      ```xml
          <parent link="${parent}"/>
      ```

    - 引数`radius`を利用して、リンクの位置を設定する。

      ```xml
          <origin rpy="0 0 0" xyz="${0.5*cos(radians(radius))} ${0.5*sin(radians(radius))} 0"/>
      ```

2. モジュールの利用  
   以下の箇所でモジュール`box_macro`を利用し、リンク、ジョイントの接頭辞を`box0`～`box3`としたリンクを4つ作成している。それぞれ、親リンクは`base_link`、中心からの角度は0°～90°に設定した。  
   この例のように、同じ処理を繰り返すときにモジュールを利用することで、繰り返し処理が簡単に記述できる。

    ```xml
    <!--モジュールの利用-->
    <xacro:box_macro prefix="box0" parent="base_link" radius="0"/>
    <xacro:box_macro prefix="box1" parent="base_link" radius="30"/>
    <xacro:box_macro prefix="box2" parent="base_link" radius="60"/>
    <xacro:box_macro prefix="box3" parent="base_link" radius="90"/>
    ```

### RVizでの表示

作成した`module.xacro`をRViz上に表示すると以下のように表示される。立方体が4つ、円周上に並んでいることが分かる。

![module.xacroの表示結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/xacro2/./img/module.png)

## インクルード

### インクルードの使い方

以下のように記述することで、他のxacroファイルをインクルードできる。インクルードすると、インクルードしたxacroファイルの処理が実行される。  

```xml
<xacro:include filename="*インクルードするファイルパス*" />
```

### インクルードを利用した例

インクルードを利用したxacroファイル`include.xacro`を作成する。このxacroファイルは、先ほど作成した`module.xacro`をインクルードする。その後、`module.xacro`で作成したモジュールを利用して、新たなリンクを作成している。

```xml
<robot name="test_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <!--別のxacroファイルをインクルード-->
  <xacro:include filename="$(find xacro2)/xacro/module.xacro" />

  <!--インクルードしたxacroファイルのモジュールを利用-->
  <xacro:box_macro prefix="box4" parent="base_link" radius="120"/>
  <xacro:box_macro prefix="box5" parent="base_link" radius="150"/>
  <xacro:box_macro prefix="box6" parent="base_link" radius="180"/>
</robot>
```

[include.xacro](https://github.com/esol-community/ros2_lecture/tree/main/beginner/xacro2/./xacro/include.xacro)

以下の箇所で`module.xacro`をインクルードする。これにより、モジュール`box_macro`が定義され、`base_link`および`box0`～`box3`が作成される。

```xml
<!--別のxacroファイルをインクルード-->
<xacro:include filename="$(find xacro2)/xacro/module.xacro" />
```

以下の箇所では、インクルードしたモジュール`box_macro`を利用している。中心角が120°～180°の位置に、`box4`～`box6`という名前の立方体を3つ作成している。

```xml
<!--インクルードしたxacroファイルのモジュールを利用-->
<xacro:box_macro prefix="box4" parent="base_link" radius="120"/>
<xacro:box_macro prefix="box5" parent="base_link" radius="150"/>
<xacro:box_macro prefix="box6" parent="base_link" radius="180"/>
```

### RVizでの表示

作成した`include.xacro`をRViz上に表示すると以下のように表示される。`module.xacro`で作成した立方体4つに加えて、`include.xacro`で作成した立方体3つが作成されていることが分かる。

![include.xacroの表示結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/xacro2/./img/include.png)

## launchファイルで変数を渡す

これまでの例では、1つのxacroファイルからは決まった物体しか表示できなかった。xacroの変数機能を使うことで、launchファイルからxacroファイルに値を渡し、その値に基づいたモデルを作成できる。

### 変数の使い方

1. 変数の定義  
  xacroファイルに以下の形式で記述すると、変数を定義できる。定義した変数にはデフォルト値を設定できる。

    ```xml
    <xacro:arg name="*変数名*" default="*変数のデフォルト値*"/>
    ```

1. 変数の利用  
  変数を利用する際は以下の通りに記述する。

    ```xml
    $(arg *変数名*)
    ```

### 変数を使用した例

変数を利用したxacroファイル`launch_arg.xacro`を作成する。このxacroファイルは、変数`length`の値を使って座標中心と立方体の距離を設定する。

```xml
<robot name="test_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <!--デフォルト値0.0で変数lengthを定義-->
  <xacro:arg name="length" default="0.0"/>

  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>

  <link name="base_link"/>

  <joint name="body_joint" type="fixed">
    <!--変数lengthを利用-->
    <origin rpy="0 0 0" xyz="$(arg length) 0 0"/>
    <parent link="base_link"/>
    <child  link="body_link"/>
  </joint>
  <link name="body_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
      <material name="red"/>
    </visual>
  </link>
</robot>
```

[launch_arg.xacro](https://github.com/esol-community/ros2_lecture/tree/main/beginner/xacro2/./xacro/launch_arg.xacro)

1. 変数の定義  
以下の箇所で、デフォルト値0.0で変数`length`を定義している。

    ```xml
    <!--デフォルト値0.0で変数lengthを定義-->
    <xacro:arg name="length" default="0.0"/>
    ```

2. 変数の利用  
以下の箇所で変数を利用している。`body_link`のx軸上の位置を、変数の値に設定している。

    ```xml
    <!--変数lengthを利用-->
    <origin rpy="0 0 0" xyz="$(arg length) 0 0"/>
    ```

### launchファイルの作成

ディレクトリ`/xacro2/launch`にlaunchファイル`xacro_launch_with_length.launch.py`を作成する。このlaunchファイルは、xacroファイルの変数に値を設定して、モデルをRVizに表示する。  
このlaunchファイルを起動すると、robot_state_publisherとrviz2の2つのROSノードを起動する。robot_state_publisherはロボットの情報をrviz2に配信するノードである。

```python
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    default_model_path = str(get_package_share_path('xacro2')) + '/xacro/launch_arg.xacro'
    rviz_config_path = str(get_package_share_path('urdf_launch')) + '/config/urdf.rviz'

    # launchファイルのパラメータ設定
    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file')
    length_arg = DeclareLaunchArgument(name='length_value', default_value="1.0",
                                    description='length between center and robot')
    
    # robot_state_publisherのパラメータ設定
    # launchファイルのパラメータ"length_value"の値を、xacroのパラメータ"length"に設定
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model'), ' length:=', LaunchConfiguration('length_value')]),
        value_type=str
    )

    # robot_state_publisherとrviz2をLaunchDescriptionに追加
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    ld.add_action(model_arg)
    ld.add_action(length_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
```

[xacro_launch_with_length.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/xacro2/./launch/xacro_launch_with_length.launch.py)  

1. launchファイルのパラメータ  
作成したlaunchファイルは、パラメータ`model`と`length_value`を持つ。  
`model`は起動するモデルの絶対パスで、デフォルト値は`launch_arg.xacro`のパスとした。また、`length_value`はxacroファイルに渡す`length`の値で、デフォルト値は1.0に指定した。  
`ros2 launch`コマンドや別のlaunchファイルからこのlaunchファイルを起動するとき、これらのパラメータに値を設定できる。

    ```python
    # launchファイルのパラメータ設定
    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file')
    length_arg = DeclareLaunchArgument(name='length_value', default_value="1.0",
                                    description='length between center and robot')
    ```

2. xacroファイルの変数の設定  
以下の箇所で、robot_state_publisherに渡すパラメータを記述している。モデルのパスには、launchファイルのパラメータ`model`の値を設定した。また、xacroファイルの引数`length`には、launchファイルのパラメータ`length_value`の値を設定している。  
この設定により、launchファイルからxacroファイルの変数に値を渡すことができる。

    ```python
    # robot_state_publisherのパラメータ設定
    # launchファイルのパラメータ"length_value"の値を、xacroのパラメータ"length"に設定
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model'), ' length:=', LaunchConfiguration('length_value')]),
        value_type=str
    )
    ```

また、作成したlaunchファイルを起動するために、CMakeLists.txtとpackage.xmlに以下の行を追加する。

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/beginner/xacro2/./CMakeLists.txt)

```diff
 # install launch files
 install(
   DIRECTORY
+  launch
   xacro
   DESTINATION share/${PROJECT_NAME}/
 )
```

[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/beginner/xacro2/./package.xml)

```diff
+ <exec_depend>ros2launch</exec_depend>
```

### RVizで表示

作成した`launch_arg.xacro`をRVizで表示する。  
はじめに、これまでと同様に`urdf_launch`パッケージの`display.launch`を使用する。`display.launch`はxacroファイルの変数に値を渡さずにRVizで表示する。従って、xacroファイルの引数`length`にはデフォルト値が適用される。

```bash
ros2 launch urdf_launch display.launch.py urdf_package:=xacro2 urdf_package_path:=./xacro/launch_arg.xacro 
```

`display.launch`で`launch_arg.xacro`を表示すると以下のようになる。座標の中心からの距離に、デフォルト値（0.0m）が適用されていることが分かる。

![launch_arg.xacroの表示結果(デフォルト)](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/xacro2/./img/launch_arg%20default.png)

つぎに、作成したlaunchファイル`xacro_launch_with_length.launch.py`のデフォルト値を利用して、xacroファイルをRVizに表示する。ワークスペースでビルドしてオーバーレイを取得した後、以下のコマンドを実行する。

```bash
ros2 launch xacro2 xacro_launch_with_length.launch.py
```

launchすると、以下のウィンドウが表示される。座標の中心からの距離に、launchファイルのデフォルト値（1.0m）が適用されていることが分かる。

![launch_arg.xacroの表示結果(launchデフォルト)](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/xacro2/./img/launch_arg%20launch_default.png)

さいごに、launchファイルのパラメータ`length_value`に2を設定してlaunchする。以下のコマンドを実行する。

```bash
ros2 launch xacro2 xacro_launch_with_length.launch.py length_value:=2
```

launchすると以下のウィンドウが表示される。座標の中心からの距離に、launch時に設定した値（2.0m）が適用されていることが分かる。

![launch_arg.xacroの表示結果(length_value=2)](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/xacro2/./img/launch_arg%20set_length_value_2.png)

## 参考

- [ROS講座73 xacroを使う2](https://qiita.com/srs/items/9ac7f2f6e47732bb7535)
- [ROS 2公式チュートリアル：Using Xacro to clean up your code](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)
- [実習ROS 2 xacroを使う1](https://qiita.com/esol-h-matsumoto/items/d8c0bf24840e5759f17f)
