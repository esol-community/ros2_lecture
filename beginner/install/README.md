# 実習ROS 2 インストール

## 概要

この記事では、ROS 2のインストール方法について説明する。

## 対応関係

ROSにはバージョンがあり、そのバージョンに応じてサポートされているOSのバージョンが異なる。  
また、1つのバージョンのROSは複数のOSをサポートすることがあり、ROS 2ではサポートの品質はTierと呼ばれるカテゴリで分類されている。  
Ubuntu LTS(Long Term Support)に対応する、Tier1(最もサポートの品質が高いカテゴリ)のROSバージョンの組み合わせに絞ると、ROSとUbuntuのバージョンの関係は以下のようになる。  
表が示すように、使いたいROSのバージョンに対応したUbuntuを使用する必要がある。  

|ROSバージョン|Ubuntuバージョン|サポート期限|
|-|-|-|
|Jazzy|24.04|2029年5月|
|Humble|22.04|2027年5月|
|Foxy|20.04|2023年5月|

これらのうち、本記事ではROS 2 Humbleをインストールする手順を解説する。

なお、ROSのバージョンごとにサポートされているプラットフォームは、[REP2000](https://www.ros.org/reps/rep-2000.html)と呼ばれるROSコミュニティの標準で定められている。

## 事前準備

まだUbuntuのインストールが済んでいない場合はそちらをまず先に行う。  
基本的には、[Ubuntu公式サイト](https://releases.ubuntu.com/)から対象のバージョンを選択してインストールすればよい。  
今回はHumbleをインストールするため、Ubuntu22.04をインストールする。  
詳しいインストール方法については、[Ubuntu公式サイトのガイド](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)などを参照する形とし本記事では省略する。

## ROS 2のインストール方法

以下、ROS 2をインストールする際の手順を記載する。  
紹介する手順は[ROS 2公式ドキュメント：Installation](https://docs.ros.org/en/humble/Installation.html)を参考にしているため、こちらも合わせて確認すること。  
なお、今回はHumbleをインストールする手順を示す。他のバージョンをインストールする場合は、手順5でバージョン名を読み替えれば良い。

1. ロケールの設定

```shell
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

2. Ubuntuユニバースリポジトリを有効にする

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
```

3. ROS 2ダウンロードのための公開鍵を取得する

```shell
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

4. リポジトリをソースリストに追加する

```shell
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

5. ROS 2 Humbleのインストール

```shell
sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop python3-argcomplete
```

- `sudo apt install ros-humble-desktop`を実行するとRVizなどGUIツールも含めインストールすることが可能であるため、基本的にはこれを選べば良い。
  - GUIツールが不要な場合は、`sudo apt install ros-humble-ros-base`といったコマンドでもROS 2のインストールが可能である。
  - 詳しくは、以下のサイトを参照。  
    [ROS 2公式ドキュメント：Install ROS 2 packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#id4)
- `python3-argcomplete`でコマンドのtab補完が行えるようになる。

以上でインストールは完了である。  
正常にインストールされたか確認するため以下の作業を行う。

### ROS 2のパスを通す

ROS 2のコマンドを現在のシェルで使用できるようにするために、以下のコマンドを実行する。

```shell
source /opt/ros/humble/setup.bash
```

- これも他のバージョンをインストールしたなら`humble`の部分を読み替えれば良い。
- `source`の代わりに`.`を使用しても良い。

このコマンドは新しいターミナルを開いた際に毎度実行する必要がある。  
下記コマンドを実行しておくとターミナル起動時に自動で設定が反映されるようになるため、行っておくと良い。

```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

ただし、違うROS 2のバージョンをインストールして使用する場合は、~/.bashrcを編集して`humble`を対応するバージョン名に書き換える必要があることに注意。

### テストの実行

ターミナルを二つ開き、以下のコマンドをそれぞれのターミナルで実行する。  
上記の`source /opt/ros/humble/setup.bash`を事前に実行しておかないと以下のコマンドは機能しないので注意。

- 一つ目のターミナル
  ```shell
  ros2 run demo_nodes_cpp talker
  ```

- 二つ目のターミナル
  ```shell
  ros2 run demo_nodes_py listener
  ```

下記のような実行結果が出力されていれば、正常に動作している。

- 一つ目のターミナル

  ```shell
  $ ros2 run demo_nodes_cpp talker
  [INFO] [1670560867.243833739] [talker]: Publishing: 'Hello World: 1'
  [INFO] [1670560868.243814406] [talker]: Publishing: 'Hello World: 2'
  [INFO] [1670560869.243824838] [talker]: Publishing: 'Hello World: 3'
  (以下省略)
  ```

- 二つ目のターミナル

  ```shell
  $ ros2 run demo_nodes_py listener
  [INFO] [1670560867.251641902] [listener]: I heard: [Hello World: 1]
  [INFO] [1670560868.244443614] [listener]: I heard: [Hello World: 2]
  [INFO] [1670560869.244400955] [listener]: I heard: [Hello World: 3]
  (以下省略)
  ```

プログラムを停止したい場合は、そのプログラムが起動しているターミナル内でCtrl+Cを押す。

## 環境のセットアップ

[上記の手順](#ros-2のインストール方法)を行うことでROS 2のインストール自体は完了である。しかし、実際にROS 2を使用するにはいくつかの準備や設定が必要である。  
それらを以下に記す。

### ROS_DOMAIN_IDの設定

同一のネットワーク内で複数人がROS 2を使用すると、データが混線し自身の処理結果が意図しない形で他者に影響することがある。  
これを回避するためにROS_DOMAIN_IDを設定する。  
下記のコマンドで設定できる。

```shell
export ROS_DOMAIN_ID=<ID>
```

- \<ID>に設定できる数値は0~232である。しかし、安全に通信を行ううえでは、0から101までの数値を用いることが望ましい。この範囲内での任意の数値を入力する。  
このIDのルールについては、[ROS 2公式ドキュメント：The ROS_DOMAIN_ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)を参照すること。  

なお、以下のコマンドを実行しておくと毎度コマンドを実行せずに済む。

```shell
echo "export ROS_DOMAIN_ID=<ID>" >> ~/.bashrc
```

- ただし、使用するROS_DOMAIN_IDを変える場合はこちらも正しい数値に変更する必要がある。

これを設定すると、同じROS_DOMAIN_ID同士でしかデータの通信を行わないようにできる。  
なお、0を設定した場合はどのROS_DOMAIN_IDのデータともやり取りできる。

### ROS_LOCALHOST_ONLY

ROS_LOCALHOST_ONLYを設定すると、通信先をlocalhost、つまり自身のPCのみに限定できる。  
そのため、他のPCと通信を行わないのであればこちらも設定しておく。  
値は1にする。

```shell
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

### ワークスペースの作成

ROS 2を使用して作業を行う場合、まず始めにワークスペースを作成する必要がある。  
これによりROS 2のパッケージを使用できるようになる。  
以下のコマンドでワークスペースを作成できる。

```shell
mkdir -p ~/ros2_lecture_ws/src
cd ~/ros2_lecture_ws
colcon build
```

- ワークスペースを置くディレクトリ名は自由だが、今回は`ros2_lecture_ws`とした。
- `ros2_lecture_ws`の直下に`src`ディレクトリを作成している。  
  このディレクトリにはROS 2のパッケージを置く。
- ワークスペース下で`colcon build`を行うことでワークスペースの構築が完了する。
  - 正常にビルドができた場合、~/ros2_lecture_ws内に/build, /install, /logが生成される。
  - `colcon`がインストールされていない場合は下記コマンドでインストールする。
    ```shell
    sudo apt install python3-colcon-common-extensions
    ```

以降パッケージを作成してノードを実装した後は、このワークスペース内で`colcon build`を行う。

### パッケージの作成

ROSはパッケージという単位でプログラムを管理する。  
パッケージの作成は以下のようにして行う。

```shell
cd ~/ros2_lecture_ws/src
ros2 pkg create <package_name>
```

- パッケージ作成は必ずsrcディレクトリ以下で行う。
- <package_name>には任意のパッケージ名を入力する。

### パッケージのビルド

パッケージのビルド方法は二通りある。

- ワークスペース以下のすべてのパッケージを一度にビルドする。
  ```shell
  colcon build
  ```

- 対象のパッケージのみをビルドできる。
  ```shell
  colcon build --packages-select <package_name>
  ```

### オーバーレイ

以上でパッケージのビルドが完了するが、それだけでは実行ができない。  
ビルドしたパッケージが使用できるようにパスを通す。  

```shell
cd ~/ros2_lecture_ws
source install/local_setup.bash
```

#### 使用する`source`コマンドの違いまとめ

- `source /opt/ros/humble/setup.bash`  
  ROS 2にパスが通り、ROS 2のコマンドなどが使用可能になる。  
- `source ~/(ワークスペース名)/install/local_setup.bash`  
  ワークスペースにパスが通り、ワークスペース下でビルドしたパッケージが使用可能になる。

## 参考

- [ROS 2公式ドキュメント：Distributions](https://docs.ros.org/en/rolling/Releases.html)
- [ROS 2公式ドキュメント：Installation](https://docs.ros.org/en/humble/Installation.html)
- [ROS 2公式ドキュメント：The ROS_DOMAIN_ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
- [ROS 2公式チュートリアル](https://docs.ros.org/en/humble/Tutorials.html)
  - [環境の構成](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
  - [colconを使用したビルド](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
  - [ワークスペースの作成](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
  - [パッケージの作成](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS講座02 インストール](https://qiita.com/srs/items/e0e0a9dc3f94c2d3348e)
- [REP2000: ROS 2 Releases and Target Platforms](https://www.ros.org/reps/rep-2000.html)
