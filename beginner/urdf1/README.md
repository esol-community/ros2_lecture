# 実習ROS 2 URDFを記述する1

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

この記事では、ロボットの構造を記述するための書式であるURDF（Unified Robot Description Format）の概要を説明する。ごく簡単なURDFファイルを作成したのち、作成したURDFをRVizで可視化する。  
この記事は、[ROS 2公式チュートリアル](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)を参考に、[ROS講座13 URDFを記述する1](https://qiita.com/srs/items/35bbaadd6c4be1e39bb9)の内容をROS 2対応させたものである。

## URDFとは

URDFはロボットの構造を記述するためのフォーマットである。URDFでは、ロボットの構造をリンクとジョイントの2種類の要素を使って表現する。リンクとはロボットの駆動しない1ブロックを指し、ジョイントはリンクとリンクをつなぐ関節部にあたる。例えば以下の図では、4つのリンクと3つのジョイントを持つロボットを表している。

<img src="https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/urdf1/img/urdf_image.png" width="360">

[ROS Wiki](http://wiki.ros.org/ja/urdf/Tutorials/Create%20your%20own%20urdf%20file)より引用

リンク、ジョイントはそれぞれ以下のような要素を持つ

- リンク
  - 見た目　  ：色や形など
  - 衝突判定  ：衝突判定を持つ範囲やその形状など
  - 物理特性  ：重さ、イナーシャ（慣性）など
- ジョイント
  - 種類　　  ：固定ジョイント、回転ジョイントなど
  - オプション：回転軸の方向、移動量の限界値など
  - 親子関係  ：親リンク、子リンク

URDFで記述されたロボットは、ジョイントに設定した親子関係に基づいて、木構造を持つ。上記の図の木構造はこのようになる。

```bash
リンク1
  ├──────────┐
リンク3       リンク2
  │   
リンク4
```

なお、木構造を持つURDFの特性上、閉ループ構造を持つロボット（パラレルリンクロボットなど）は記述できない。

## ROS 2パッケージの実装

urdf1というパッケージを作成し、作成したパッケージ内にsimple_urdf1.urdfというURDFファイルを作成する。URDFファイルは、<package_name>/urdf/以下に作成する。したがって、ディレクトリ構成は以下の形となる。  
※`include`ディレクトリおよび`src`ディレクトリはパッケージ作成時に自動生成される。この記事の範囲ではこれらのディレクトリを使用しないため、[GitHub上のサンプル](https://github.com/esol-community/ros2_lecture/tree/main/beginner/urdf1)には含まない。

```bash
urdf1
├── CMakeLists.txt
├── include
│   └── urdf1
├── package.xml
├── urdf　　　　　　　　　　　#新規作成
│   └── simple_urdf1.urdf
└── src
```

### ROS 2パッケージの作成

パッケージ`urdf1`を作成する。このチュートリアルは、[Pub＆Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)を実行していることを前提に、Pub＆Sub通信のチュートリアルで作成したワークスペース`ros2_lecture_ws`を利用する。
本記事の後半では、RVizを使用してURDFを可視化する。その際に`urdf_launch`パッケージを使用するため、`urdf_launch`パッケージへの依存関係をコマンドで指定する。

```bash
cd ~/ros2_lecture_ws/src
ros2 pkg create urdf1 --build-type ament_cmake --dependencies urdf_launch
```

### URDFファイルの作成

以下のコマンドでurdfディレクトリを作成する。

```bash
cd ./urdf1
mkdir urdf && cd urdf
```

作成したディレクトリ内にsimple_urdf1.urdfを作成し、以下の通り記載する。

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

  <link name="body_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0.0 1 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="body_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="body_link"/>
    <origin xyz="0 0 0.1"/>
  </joint>
</robot>
```

[simple_urdf1.urdf](https://github.com/esol-community/ros2_lecture/tree/main/beginner/urdf1/urdf/simple_urdf1.urdf)

### URDFファイルの書式の説明

simple_urdf1.urdfで使用したURDFの記法を説明する。より詳しく知りたい場合は、[ROS 2公式チュートリアル：Building a visual robot model from scratch](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)や、[ROS公式ドキュメント](http://wiki.ros.org/urdf/XML)などを参照すると良い。  

- `<robot>`タグ
  - *name=xxx*でロボットの名前を設定する
  - このタグ内にリンクやジョイントの情報を記述する
  - `<link>`タグ
    - *name=xxx*でリンクの名前を記述する
    - このタグ内にリンクの見た目や衝突判定、物理特性を記述する
    - `<visual>`タグ
      - このタグ内にリンクの見た目を記述する
      - `<geometry>`タグ
        - `<visual>`の形状と大きさを記述する
        - 形状は、箱型（box）や球（shpere）、円柱（cylinder）などが利用できる
        - 大きさの単位は[m]を使用する
        - 大きさの記述方法は形状により異なる
          - 箱型：xyzの順で辺の長さを指定する（以下は辺の長さが1mの箱型の例）  
          `<box size="1 1 1"/>`
          - 球：半径を指定する（以下は半径が1mの球の例）  
          `<sphere radius="1"/>`
          - 円柱：高さと半径を指定する（以下は高さ1m、半径1mの円柱の例）  
          `<cylinder length="1" radius="1"/>`
      - `<origin>`タグ
        - リンクの座標系に対する`<visual>`中心までの相対位置と相対角度を指定する  
        - 位置の単位は[m]で、xyzの順で指定する。
        - 回転角の単位は[rad]で、rpyの順で指定する
      - `<material>`タグ
        - *name=xxx*でmaterialの名前を設定する
        - このタグ内にリンクの色やテクスチャを記述する
        - `<color>`タグ
          - リンクの色を記述する
          - 色の指定には、rgbaを用いる
          - aはアルファ値（透明度）で、0のときに透明に、1のとき不透明になる
  - `<joint>`タグ
    - *name=xxx*でジョイントの名前を、*Type=xxx*でジョイントの種類を指定する
    - このタグ内にジョイントのオプションや親子関係を記述する
    - `<parent>`タグ
      - 親リンクを記述する
    - `<child>`タグ
      - 子リンクを記述する
    - `<origin>`タグ
      - 親リンクを基準とした子リンクの相対位置を指定する
  
### simple_urdf1.urdfの説明

```xml
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
```

このブロックでは、base_linkという名前のリンクを作成している。base_linkはロボットの位置を表す基準となるリンクである。  
`<geometry>`タグを使って、x軸方向に0.3、y軸方向に0.3、z軸方向に0.2の大きさの箱型（box）を指定している。また、`<origin>`タグでリンクの基準位置から箱型までの相対位置を指定している。相対位置の指定が`xyz="0 0 0"`のため、リンクの基準位置と箱型の中心が一致する。  
`<material>`タグではredという名前を付けて、リンクの色を赤色に設定している。  


```xml
  <link name="body_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0.0 1 0.0 1.0"/>
      </material>
    </visual>
  </link>
```

このブロックでは、body_linkという名前のリンクを作成している。  
`<geometry>`タグを使って、x軸方向に0.1、y軸方向に0.1、z軸方向に0.3の大きさの箱型を指定している。また、`<origin>`タグでリンクの基準位置から箱型までの相対位置を指定している。相対位置の指定が`xyz="0 0 0.15"`のため、z軸方向に0.15m移動した位置に箱型の中心が位置する。`<material>`タグではgreenという名前を付けて、リンクの色を緑色に設定している。  

```xml
  <joint name="body_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="body_link"/>
    <origin xyz="0 0 0.1"/>
  </joint>
```

このブロックでは、body_jointという名前のジョイントを記述している。ジョイントの種類には、駆動しないジョイントであるfixedを指定している。  
`<parent>`タグで親リンクをbase_linkに、`<child>`タグで子リンクをbody_linkに設定している。`<origin>`タグで、親リンクを基準とした、子リンクの相対位置を設定している。

## パッケージのビルド

CMakeLists.txtに以下の内容を追記する。

```txt
# install urdf files
install(
  DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)
```

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/beginner/urdf1/CMakeLists.txt)

以下のコマンドを実行し、作成したパッケージの依存関係を解決する。
URDFファイルのチェックおよび可視化に利用するパッケージがインストールされる。
urdf_launchパッケージが依存するパッケージ（urdfdomパッケージなど）がインストールされていない場合も、上記のコマンドでまとめてインストールされる。

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths ~/ros2_lecture_ws/src/urdf1
```

作成したパッケージをビルドする。

```bash
cd ~/ros2_lecture_ws
colcon build --packages-select urdf1
```

## 作成したURDFのチェックと可視化

### URDFファイルのチェック

作成したURDFが正しく記述できているか、"check_urdf"というツールで確認できる。check_urdfは以下のコマンドで使用する。

```bash
check_urdf ./src/urdf1/urdf/simple_urdf1.urdf
```

すると以下のような結果が表示され、URDFが正しく記述できていることを確認できる。

```bash
robot name is: test_robot
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  body_link
```

記述が正しくないときは、エラー情報が表示される。例えば`<robot name="test_robot">`と最終行の`</robot>`をコメントアウトすると、以下のように表示される。

```bash
Error:   Could not find the 'robot' element in the xml file
         at line 80 in /tmp/binarydeb/ros-humble-urdfdom-2.3.3/urdf_parser/src/model.cpp
ERROR: Model Parsing the xml failed
```

しかし、このツールでは検出できないミスも多い。例えば、"<origin**nn** xyz="0 0 0.1" rpy="0 0 0"/>"のような綴りミスは検出できない。このようにミスを含むURDFをRVizで表示すると、ミスのある行は無視され、`<origin>`にはデフォルトの値（xyz="0 0 0" rpy="0 0 0"）が反映されるため、意図した形状のURDFが表示されない。

### RVizでURDFの可視化

作成したURDFをRVizで可視化する。RVizの起動にはurdf_launchパッケージのdisplay.launchというlaunchファイルを使用する。なお、launchファイルの内容はこの記事では取り扱わない。  
ディレクトリ`ros2_lecture_ws/src/urdf1`から以下のコマンドを実行し、URDFを可視化する。このコマンドでは、launchファイルのパラメータに作成したURDFファイルのパッケージおよびパッケージ内の配置場所を指定し、launchファイルを起動している。

```bash
. install/setup.bash
ros2 launch urdf_launch display.launch.py urdf_package:=urdf1 urdf_package_path:=urdf/simple_urdf1.urdf 
```

なお、RVizの画面上に正常に表示されないときは以下の2点を確認する。

1. 左側の"Displays"欄の"Fixed Frame"に"base_link"が指定されていること
2. "Robot_model"にチェックが入っていること

![RViz_display](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/urdf1/img/RViz_display.png)

RViz上では2つの座標軸が表示されている。それぞれの座標軸は、URDFで定義した`base_link`と`body_link`の2つのリンクの座標軸を表す。ジョイントで指定した位置（`base_link`からz軸方向に0.1m移動した点）に、`body_link`が位置する。また、`base_link`の座標軸を中心とした赤色の箱型と、`body_link`からz軸方向に0.15mの点を中心とした緑色の箱型の2つが表示されている。

## 参考

- [ROS講座13 URDFを記述する1](https://qiita.com/srs/items/35bbaadd6c4be1e39bb9)
- [ROS 2公式チュートリアル：Building a visual robot model from scratch](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)
