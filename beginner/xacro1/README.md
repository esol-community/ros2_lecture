# 実習ROS 2 xacroを使う1

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

このページでは、xacro(Xml mACRO)を使ったURDFの作成方法を説明する。xacroを利用することで、複雑なURDFを効率的に作成できる。なお、URDFの概要や基本的な記述方法などは以下のページで説明しているので参考にしてほしい。

- [実習ROS 2 URDFを記述する1](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6)
- [実習ROS 2 URDFを記述する2](https://qiita.com/esol-h-matsumoto/items/c81e818224748dd6a73b)
  
このページは、[ROS 2公式チュートリアル](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)を参考に、[ROS講座68 xacroを使う1](https://qiita.com/srs/items/43528d00ee789171367f)の内容をROS 2対応させたものである。また、基本的にROS 1とROS 2でxacroの記述方法に違いはない。  

## xacroとは

xacroとは、URDFを効率的に作成できるマクロ言語である。  
URDFの記法では全てのリンクやジョイントを個別に記述する。形状やサイズが共通のリンク、特性が同じジョイントなどが複数ある場合でも、それぞれを記述していく必要があった。  
xacroが提供する機能を利用すると、繰り返しなどの処理をプログラマブルに記述し、効率的かつ分かりやすいURDFを作成できる。xacroが提供する機能には以下のものがある。

- 定数
- 数式・条件式
- マクロ

これらのうち、本ページでは定数、数式・条件式を利用する方法を説明する。

## 前準備

### 前提条件

このチュートリアルは、[実習ROS 2 Pub＆Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)を実行していることを前提に、Pub＆Sub通信のチュートリアルで作成したワークスペース`ros2_lecture_ws`を利用する。  
また、[実習ROS 2 URDFを記述する1](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6)と[実習ROS 2 URDFを記述する2](https://qiita.com/esol-h-matsumoto/items/c81e818224748dd6a73b)の実施を前提とする。

まだxacroのインストールが出来ていない場合は下記を参考にインストールする。  
```bash
sudo apt install ros-humble-xacro
source /opt/ros/humble/setup.bash
```  

### ROS 2パッケージの作成

パッケージ`xacro1`およびxacroファイルを格納するディレクトリ`/xacro`を作成する。

```bash
cd ~/ros2_lecture_ws/src
ros2 pkg create xacro1 --build-type ament_cmake --dependencies urdf_launch
mkdir ./xacro1/xacro
```

## xacroファイルの概要

xacroファイルの例として、定数を利用したxacroファイル`constants.xacro`を示す。

```xml
<!-- 名前空間の指定 -->
<robot name="constants_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- 定数の定義 -->
  <xacro:property name="side" value="0.5" />

  <link name="base_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <!-- 定数の利用 -->
        <box size="${side} ${side} ${side}" />
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <link name="body1_link">
    <visual>
      <origin rpy="0 0 0" xyz="1 1 1"/>
      <geometry>
        <!-- 定数の利用 -->
        <box size="${side} ${side} ${side}" />
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <link name="body2_link">
    <visual>
      <origin rpy="0 0 0" xyz="2 2 2"/>
      <geometry>
        <!-- 定数の利用 -->
        <box size="${side} ${side} ${side}" />
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
</robot>
```

[constants.xacro](https://github.com/esol-community/ros2_lecture/tree/main/beginner/xacro1/./xacro/constants.xacro)  

xacroファイルは作成したディレクトリ`/xacro`内に作成する。拡張子は.xacroである。URDFと同様にxmlのフォーマットで作成し、タグもURDFと同じものを使用する。これらのURDFの記述方法をベースに、定数や数式、マクロなどを使用した処理を追加する。  
また、xacroファイルでは`<robot>`タグの属性で名前空間を指定する。これにより有効なxacroファイルとして解釈される。

```xml
<robot name=*ロボット名* xmlns:xacro="http://ros.org/wiki/xacro">
```

## URDFへの変換

xacroからURDFへ変換する方法を説明する。  
変換前に先ほど示した`constants.xacro`を準備しておく。

```bash
cd ~/ros2_lecture_ws/src/xacro1/xacro
touch constants.xacro
<!-- vim等を使用しconstants.xacroを記述する -->
```

以下のコマンドを実行すると、作成したxacroファイルをURDFファイルに変換できる。

```bash
xacro ~/ros2_lecture_ws/src/xacro1/xacro/constants.xacro > ~/ros2_lecture_ws/src/xacro1/xacro/constants.urdf
```  

## 定数

### 定数の使い方

1. 定数の定義  
定数は以下のフォーマットで定義する。valueには数字のほか、文字列や論理値（true/false）といった変数型を使用できる。

    ```xml
    <xacro:property name="定数名" value="値">
    ```

2. 定数の利用  
定義した定数は以下のように書くと利用できる。

    ```xml
    ${定数名}
    ```

### 定数を利用した例

前述の定数を利用したxacroファイル`constants.xacro`を再掲し、定数を利用した部分について説明する。このxacroファイルでは、一辺が0.5mの大きさの立方体を3個作成している。

```xml
<!-- 名前空間の指定 -->
<robot name="constants_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- 定数の定義 -->
  <xacro:property name="side" value="0.5" />

  <link name="base_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <!-- 定数の利用 -->
        <box size="${side} ${side} ${side}" />
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <link name="body1_link">
    <visual>
      <origin rpy="0 0 0" xyz="1 1 1"/>
      <geometry>
        <!-- 定数の利用 -->
        <box size="${side} ${side} ${side}" />
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <link name="body2_link">
    <visual>
      <origin rpy="0 0 0" xyz="2 2 2"/>
      <geometry>
        <!-- 定数の利用 -->
        <box size="${side} ${side} ${side}" />
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
</robot>
```

1. 定数の定義  
   以下の通り記述して、値が0.5の定数`side`を定義している。

    ```xml
    <!-- 定数の定義 -->
    <xacro:property name="side" value="0.5" />
    ```

2. 定数の利用  
   以下の通り記述して、定義した定数`side`を使用している。

    ```xml
    <!-- 定数の利用 -->
    <box size="${side} ${side} ${side}" />
    ```

### URDFへの変換結果

前述の手順に則りURDFへの変換を行う。定義した定数が展開されてURDFに変換されていると分かる。

```xml
<geometry>
    <box size="0.5 0.5 0.5"/>
</geometry>
```

### RVizで表示

URDFの可視化は、[実習ROS 2 URDFを記述する1](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6)の[パッケージのビルド](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6#%E3%83%91%E3%83%83%E3%82%B1%E3%83%BC%E3%82%B8%E3%81%AE%E3%83%93%E3%83%AB%E3%83%89)と[RVizでURDFの可視化](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6#rviz%E3%81%A7urdf%E3%81%AE%E5%8F%AF%E8%A6%96%E5%8C%96)と同様の手順で行う。上記リンク先の手順を、パッケージ名を"xacro1"に読み替えて実行する。

なお、[実習ROS 2 URDFを記述する1](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6)では、`<パッケージ名>/urdf/`にURDFを格納した。一方今回は、`<パッケージ名>/xacro/`にxacroを格納するため、CMakeLists.txtに追記する内容は以下の通りとする。  

```txt
# install xacro files
install(
  DIRECTORY
  xacro
  DESTINATION share/${PROJECT_NAME}/
)
```

CMakeLists.txtに追記した後は、launch実行前にビルド及びsourceコマンドを実行する。

```bash
cd ~/ros2_lecture_ws
colcon build
source install/setup.bash
```

launch時のコマンドは以下となる。コマンドの引数で変換後のURDFファイルを指定する。

```bash
ros2 launch urdf_launch display.launch.py urdf_package:=xacro1 urdf_package_path:=xacro/constants.urdf 
```

RViz上には以下のような立方体が表示される。

![constants.xacroの表示結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/xacro1/./img/constants.png)

なお、今回使用しているlaunchファイル`display.launch.py`は、以下のコマンドのようにxacroファイルを直接パラメータとして与えることもできる。

```bash
ros2 launch urdf_launch display.launch.py urdf_package:=xacro1 urdf_package_path:=xacro/constants.xacro
```

このように直接xacroファイルを表示するとxacroの作成ミスなどに気づきにくい。Rvizでの表示がうまくいかないときはURDFに変換して中身を確認したり、[check_urdfでURDFのエラーを確認](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6#urdf%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB%E3%81%AE%E3%83%81%E3%82%A7%E3%83%83%E3%82%AF)すると良い。

## 数式

### 数式の使い方

以下のように記述すると、中カッコ内の記述は数式として扱われる。  
数式内では四則演算やxacroファイル内で定義した定数が利用できる。他にも三角関数のようなPythonのmathモジュールに含まれる数式などが利用できる。詳細はros/xacroリポジトリにある[wiki](https://github.com/ros/xacro/wiki)を確認するとよい。

```xml
${*数式*}
```

### 数式を利用した例

xacro1/xacro下に数式を利用したxacroファイル`math.xacro`を作成する。このxacroファイルは、一辺が0.1mの大きさの立方体を3つ、半径0.5mの円周上に30°間隔で並べたものである。

```xml
<robot name="math_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="side" value="0.1" />
  <!-- 定数radiusを定義 -->
  <xacro:property name="radius" value="0.5" />
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>

  <link name="base_link"/>

  <link name="body0_link">
    <visual>
      <geometry>
        <box size="${side} ${side} ${side}" />
      </geometry>
      <material name="red"/>
    </visual>
  </link>
  <joint name="body0_joint" type="fixed">
    <!-- 数式を利用して位置を指定 -->
    <origin rpy="0 0 0" xyz="${radius*cos(radians(0))} ${radius*sin(radians(0))} 0"/>
    <parent link="base_link"/>
    <child  link="body0_link"/>
  </joint>

  <link name="body1_link">
    <visual>
      <geometry>
        <box size="${side} ${side} ${side}" />
      </geometry>
      <material name="red"/>
    </visual>
  </link>
  <joint name="body1_joint" type="fixed">
    <!-- 数式を利用して位置を指定 -->
    <origin rpy="0 0 0" xyz="${radius*cos(radians(30))} ${radius*sin(radians(30))} 0"/>
    <parent link="base_link"/>
    <child  link="body1_link"/>
  </joint>

  <link name="body2_link">
    <visual>
      <geometry>
        <box size="${side} ${side} ${side}" />
      </geometry>
      <material name="red"/>
    </visual>
  </link>
  <joint name="body2_joint" type="fixed">
    <!-- 数式を利用して位置を指定 -->
    <origin rpy="0 0 0" xyz="${radius*cos(radians(60))} ${radius*sin(radians(60))} 0"/>
    <parent link="base_link"/>
    <child  link="body2_link"/>
  </joint>
</robot>
```

[math.xacro](https://github.com/esol-community/ros2_lecture/tree/main/beginner/xacro1/./xacro/math.xacro)

数式は以下の3か所で使用している。

- body0_joint

  ```xml
  <joint name="body0_joint" type="fixed">
    <!-- 数式を利用して位置を指定 -->
    <origin rpy="0 0 0" xyz="${radius*cos(radians(0))} ${radius*sin(radians(0))} 0"/>
    <parent link="base_link"/>
    <child  link="body0_link"/>
  </joint>
  ```

- body1_joint
  
  ```xml
  <joint name="body1_joint" type="fixed">
    <!-- 数式を利用して位置を指定 -->
    <origin rpy="0 0 0" xyz="${radius*cos(radians(30))} ${radius*sin(radians(30))} 0"/>
    <parent link="base_link"/>
    <child  link="body1_link"/>
  </joint>
  ```

- body2_joint

  ```xml
  <joint name="body2_joint" type="fixed">
    <!-- 数式を利用して位置を指定 -->
    <origin rpy="0 0 0" xyz="${radius*cos(radians(60))} ${radius*sin(radians(60))} 0"/>
    <parent link="base_link"/>
    <child  link="body2_link"/>
  </joint>
  ```

定義した定数`radius`と三角関数からなる数式で、base_linkに対するリンクの中心座標位置を指定している。  
Pythonのmathモジュールの関数`sin()`と`cos()`がラジアンを引数に取るため、角度はラジアンに変換して与えている。ラジアンへの変換はmathモジュールの関数`radians()`を使用している。

### RVizでの表示

作成した`math.xacro`をRViz上に表示すると以下のように表示される。再度ビルド、sourceコマンドを実行し、先ほどと同様にlaunchを実行する。立方体が3つ、円周上に並んでいることが分かる。

![math.xacroの表示結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/xacro1/./img/math.png)

## 条件式

### 条件式の使い方

条件式は以下の形式で記述する。URDFに展開される範囲を条件式で切り替えることができる。

```xml
<xacro:if value="${条件式}">
    *条件式が真のときURDFに展開される*
</xacro:if>
```

### 条件式を使用した例

条件式を利用したxacroファイル`condition.xacro`を作成する。このxacroファイルは、定数`color`の値によってリンクに設定する色を変化させている。

```xml
<robot name="condition_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="color" value="blue" />

  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
      <!-- 条件式を利用してリンクに色を設定 -->
      <xacro:if value="${color=='red'}">
        <material name="red"/>
      </xacro:if>
      <xacro:if value="${color=='blue'}">
        <material name="blue"/>
      </xacro:if>
    </visual>
  </link>
</robot>
```

[condition.xacro](https://github.com/esol-community/ros2_lecture/tree/main/beginner/xacro1/./xacro/condition.xacro)

以下の箇所で条件式が使われている。

```xml
<!-- 条件式を利用してリンクに色を設定 -->
<xacro:if value="${color=='red'}">
  <material name="red"/>
</xacro:if>
<xacro:if value="${color=='blue'}">
  <material name="blue"/>
</xacro:if>
```

`condition.xacro`の冒頭で定義した定数`color`の値に応じて、`<material>`タグとして展開される値が変化する。

### RVizで表示

作成した`condition.xacro`をRViz上に表示すると以下のように表示される。再度ビルド、sourceコマンドを実行し、先ほどと同様にlaunchを実行する。`color`の値には`blue`を設定しているので、立方体が青で表示されている。

![condition.xacroの表示結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/xacro1/./img/condition.png)

## 参考

- [ROS講座68 xacroを使う1](https://qiita.com/srs/items/43528d00ee789171367f)
- [ROS 2公式チュートリアル：Using Xacro to clean up your code](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)
