# 実習ROS 2 URDFを記述する2

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

[実習ROS 2 URDFを記述する1](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6)では、URDFの概要と基本的な記述方法を説明した。このページではURDFの書き方をより詳しく説明する。  
このページは、[ROS 2公式チュートリアル](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)を参考に、[ROS講座14 URDFを記述する2](https://qiita.com/srs/items/77f378230bf856a3625c)の内容をROS 2対応させたものである。  

## 前準備

### 前提条件

このチュートリアルは、[実習ROS 2 Pub＆Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)を実行していることを前提に、Pub＆Sub通信のチュートリアルで作成したワークスペース`ros2_lecture_ws`を利用する。  
また、[実習ROS 2 URDFを記述する1](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6)の実施を前提とする。

### ROS 2パッケージの作成

パッケージ`urdf2`を作成する。

```bash
cd ~/ros2_lecture_ws/src
ros2 pkg create urdf2 --build-type ament_cmake --dependencies urdf_launch
```

## 形状・色の記述

ディレクトリ`urdf`内に、`simple_urdf2.urdf`を作成する。

```xml
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.2"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <!--body1_linkという名前で球を作成-->
  <link name="body1_link">
    <visual>
      <geometry>
        <sphere radius="0.2"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0.0 1.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="body1_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="body1_link"/>
    <origin xyz="0.5 0 0.2" rpy="0 0 0"/>
  </joint>

  <!--body2_linkという名前で円柱を作成-->
  <link name="body2_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="body2_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="body2_link"/>
    <origin xyz="-0.5 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

[simple_urdf2.urdf](https://github.com/esol-community/ros2_lecture/tree/main/beginner/urdf2/./urdf/simple_urdf2.urdf)  

このURDFには、3つのリンクが存在する。親リンクは
`base_link`で、`body1_link`、`body2_link`はそれぞれ`base_link`の子リンクである。このように、リンクは複数の子リンクを持つことができる。  

URDFの可視化は、[実習ROS 2 URDFを記述する1](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6)の[パッケージのビルド](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6#パッケージのビルド)と[RVizでURDFの可視化](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6#rvizでurdfの可視化)と同様の手順で行う。上記リンク先の手順を、パッケージ名を"urdf2"に読み替えて実行する。  
RVizを起動すると、赤色の直方体、緑色の球と青色の円柱が表示される。

![simple_urdf2.urdfの表示結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/urdf2/./img/simple_urdf2.png)

### 形状の指定

直方体を指定する`box`の他に、以下の箇所で球`sphere`、円柱`cylinder`を指定して、球や円柱の形状を設定している。

- 球  
形状は`sphere`として、半径0.2[m]を設定

  ```xml
  <geometry>
    <sphere radius="0.2"/>
  </geometry>
  ```

- 円柱  
形状は`cylinder`として、長さ0.3[m]、半径0.1[m]を設定
  
  ```xml
  <geometry>
    <cylinder length="0.3" radius="0.1"/>
  </geometry>
  ```

### 色の指定

`simple_urdf1.urdf`と同様に、以下の箇所でrgbaで色を指定している。

- 緑  
rgbaに"0.0, 1.0, 0.0, 1.0"を設定

  ```xml
  <material name="green">
    <color rgba="0.0 1.0 0.0 1.0"/>
  </material>
  ```

- 青  
rgbaに"0.0, 0.0, 1.0, 1.0"を設定
  
  ```xml
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  ```

## 色の定義

`simple_urdf1.urdf`や`simple_urdf2.urdf`では、リンクごとにrgbaを使って色を指定した。同じ色のリンクが複数ある場合には、色を定義するとより簡単にURDFを作成できる。  
同じ色を持つ直方体を2つ並べたURDF`simple_urdf3.urdf`を例に、色の定義方法を説明する。

```xml
<robot name="test_robot">
  <!--"red"という名前でマテリアルを定義-->
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.2"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <!--"red"を使用して色を指定-->
      <material name="red"/>
      </visual>
  </link>
  
  <link name="body_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.2"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <!--"red"を使用して色を指定-->
      <material name="red"/>
    </visual>
  </link>

  <joint name="body_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="body_link"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
  </joint>
</robot>
```

[simple_urdf3.urdf](https://github.com/esol-community/ros2_lecture/tree/main/beginner/urdf2/./urdf/simple_urdf3.urdf)  

`simple_urdf3.urdf`は、2つの直方体`base_link`、`body_link`を持つURDFである。`simple_urdf3.urdf`をRVizで表示すると、以下のように赤の直方体が並んで表示される。

![simple_urdf3.urdfの表示結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/urdf2/./img/simple_urdf3.png)

### 色の定義と利用

以下の箇所で色の名前とrgbaを定義している。`<link>`タグや`<joint>`タグの内側で色を定義することで、全てのリンクで定義を利用できる。`<robot>`タグより外側では色を定義できない。

```xml
<!--"red"という名前でマテリアルを定義-->
<material name="red">
  <color rgba="1.0 0.0 0.0 1.0"/>
</material>
```

定義した色は以下のように利用できる。

```xml
<!--"red"を使用して色を指定-->
<material name="red"/>
```

## メッシュの利用

URDFでは、3Dのメッシュデータを利用した見た目の設定が可能である。  
`simple_urdf3.urdf`の直方体の一方をメッシュデータへ置き換えたURDF`simple_urdf4.urdf`を作成する。

```xml
<robot name="test_robot">
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <!-- メッシュでstlファイルを指定 -->
        <mesh filename="package://turtlebot3_description/meshes/bases/burger_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="red"/>
      </visual>
  </link>
  
  <link name="body_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.2"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="red"/>
    </visual>
  </link>

  <joint name="body_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="body_link"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
  </joint>
</robot>
```

[simple_urdf4.urdf](https://github.com/esol-community/ros2_lecture/tree/main/beginner/urdf2/./urdf/simple_urdf4.urdf)

`simple_urdf4.urdf`をRvizで表示すると、以下のように表示される。

![simple_urdf4.urdfの表示結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/urdf2/./img/simple_urdf4.png)

### meshタグ

`<geometory>`タグの中で`<mesh>`タグを使用することで、メッシュデータを設定する。ここでは、`turtlebot3_description`パッケージの、Turtlebot3のメッシュを指定した。  

使用できるメッシュデータの形式に制限は無いが、モデルを表示するアプリケーションによって対応状況が異なる。RViz2で表示する場合、STL形式(拡張子".stl")や、COLLADA形式(拡張子".dae")のファイルが使用できる。

`scale`では、3Dモデルのx, y, zの各軸の表示倍率を設定する。  
ROS 2では距離の単位にはメートルを使用する。一方で、3Dモデルはミリメートル単位でデータが作成されることが多い。`scale`で表示する大きさの倍率を指定し、単位系を揃える処理を行う。

## 駆動するジョイントの利用

ここまでで作成したURDFはジョイントに固定ジョイント`fixed`を利用していた。一方実際のロボットでは、アームやホイールなどの駆動部を持つものが一般的である。  
ここからは回転や直動する可動ジョイントを使用して、駆動が可能なURDF`simple_urdf5.urdf`を作成する。  

```xml
<robot name="test_robot">
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <material name="red"/>
    </visual>
  </link>
  
  <link name="body1_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <material name="red"/>
    </visual>
  </link>

  <!--回転ジョイント"revolute"を作成-->
  <joint name="body1_joint" type="revolute">
    <parent link="base_link"/>
    <child  link="body1_link"/>
    <origin xyz="0.1 0 0.5" rpy="0 0 0"/>
    <limit lower="-1.5" upper="1.5" effort="0" velocity="0"/>
    <axis xyz="1 0 0"/>
  </joint>

  <link name="body2_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <material name="red"/>
    </visual>
  </link>
  
  <!--直動ジョイント"prismatic"を作成-->
  <joint name="body2_joint" type="prismatic">
    <parent link="body1_link"/>
    <child  link="body2_link"/>
    <origin xyz="0.1 0 0.4" rpy="0 0 0"/>
    <limit lower="-0.4" upper="0" effort="0" velocity="0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

[simple_urdf5.urdf](https://github.com/esol-community/ros2_lecture/tree/main/beginner/urdf2/./urdf/simple_urdf5.urdf)

`simple_urdf5.urdf`は、3つの直方体`base_link`、`body1_link`、`body2_link`を持つ。それぞれのリンクをつなぐジョイントには、回転ジョイント、直動ジョイントを設定している。`simple_urdf5.urdf`をRvizで表示すると、以下のように表示される。

![simple_urdf5.urdfの表示結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/urdf2/./img/simple_urdf5.png)

画像に示すように、別ウィンドウにJoint State Publisherが表示される。このウィンドウ上でスライダーを操作すると、対応する可動ジョイントを動かすことができる。

### 回転ジョイント

回転ジョイントは以下のように、typeを"revolute"にすると利用できる。また"fixed"と比べると、動作範囲を設定する`<limit>`タグと、動作方向を設定する`<axis>`タグが追加されている。

```xml
<!--回転ジョイント"revolute"を作成-->
<joint name="body1_joint" type="revolute">
  <parent link="base_link"/>
  <child  link="body1_link"/>
  <origin xyz="0.1 0 0.5" rpy="0 0 0"/>
  <limit lower="-1.5" upper="1.5" effort="0" velocity="0"/>
  <axis xyz="1 0 0"/>
</joint>
```

### 直動ジョイント

直動ジョイントは以下のように、typeを"prismatic"にすると利用できる。回転ジョイントと同様に、`<limit>`タグと`<axis>`タグが追加されている。

```xml
<!--直動ジョイント"prismatic"を作成-->
<joint name="body2_joint" type="prismatic">
  <parent link="body1_link"/>
  <child  link="body2_link"/>
  <origin xyz="0.1 0 0.4" rpy="0 0 0"/>
  <limit lower="-0.4" upper="0" effort="0" velocity="0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### limitタグ

`<limit>`タグの項目は、"lower"、"upper"、"effort"、"velocity"の4つのパラメータを取る。これらの設定値はRVizによるURDFの表示やGazeboなどによるシミュレーションに使用される。

- lower・upper
  - 可動範囲の最小値と最大値
  - 値の意味はジョイントによって異なる
    - 回転ジョイント：回転角[rad]
    - 直動ジョイント：移動距離[m]
- effort
  - ジョイントが発揮できるトルクや力の最大値
  - 値の意味はジョイントによって異なる
    - 回転ジョイント：トルクの最大値[Nm]
    - 直動ジョイント：力の最大値[N]
- velocity
  - ジョイントの動く速さの最大値
  - 値の意味はジョイントによって異なる
    - 回転ジョイント：角速度[rad/s]
    - 直動ジョイント：速度[m/s]
  
### axisタグ

`<axis>`タグは"xyz"のパラメータをとり、ジョイントの可動方向を設定する。  
ジョイントの座標系を基準に、回転ジョイントは設定した軸回りに回転し、直動ジョイントは設定した軸方向に移動する。

### その他のジョイント

今回は固定ジョイント、回転ジョイント、直動ジョイントなどを使用した。URDFではそれ以外にも色々な種類のジョイントが利用できる。また`<limit>`や`<axis>`以外にも、ジョイントの設定に複数のタグが利用できる。ジョイントの種類や設定に用いるタグは、[ROS公式のジョイント解説ページ](http://wiki.ros.org/urdf/XML/joint)に詳しく書かれている。

## 参考

- [ROS講座14 URDFを記述する2](https://qiita.com/srs/items/77f378230bf856a3625c)
- [ROS 2公式チュートリアル：Building a visual robot model from scratch](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)
