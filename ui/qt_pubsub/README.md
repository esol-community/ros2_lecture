# 実習ROS 2 QtでPub&Subする

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|
|Qt|Qt5.15.3|

## 概要

Qtで作成したGUIとROS 2のPub＆Sub通信を組み合わせて、GUI操作によりトピックをpublishするノードと、トピックをsubscribeしてGUIに表示するノードを作成する。実際に作成したノードを起動して、QtとROS 2の処理が組み合わされて動作することを確認する。  
このページは、[ROS講座94 Qtでpub・subする](https://qiita.com/srs/items/93ab82b96637f405e657)の内容をROS 2対応させたものである。  

## 前準備

### 前提条件

このチュートリアルの実施前に、次の環境を準備すること。

1. [実習ROS 2 Pub＆Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)のチュートリアルを実施し、ワークスペース`ros2_lecture_ws`を作成する。
2. [実習ROS 2 Qtを使う1 Qtの環境構築](https://qiita.com/esol-h-matsumoto/items/db11f5ddf78fac7f5eed#qt%E3%81%AE%E7%92%B0%E5%A2%83%E6%A7%8B%E7%AF%89)を実施する。

### ROS 2パッケージの作成

パッケージ`qt_pubsub`を作成する。

```bash
cd ~/ros2_lecture_ws/src
ros2 pkg create --build-type ament_cmake qt_pubsub
```

## ソースコードの作成

publisherとsubscriberの2つのROSノードを作成する。publisherのGUIにはテキストボックスとプッシュボタンを配置し、テキストボックスに入力した文字列をpublishする。subscriberのGUIにはテキストボックスを配置し、subscribeしたトピックの文字列を表示する。  

### publisherの作成

ディレクトリ`qt_pubsub/src/`内に、以下の5つのソースファイルを作成する。

- [qt_talker.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_talker.cpp)
- [qt_talker_GUI.hpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_talker_GUI.hpp)
- [qt_talker_GUI.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_talker_GUI.cpp)
- [qt_talker_rosNode.hpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_talker_rosNode.hpp)
- [qt_talker_rosNode.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_talker_rosNode.cpp)

1. ROSノード  
  クラス`QtTalker`ではROSノード`qt_talker`を定義し、文字列型のトピック`chatter`のpublisherを作成する。また、メンバ関数`publish_message()`は、引数で指定された文字列をトピックに格納し、トピック`chatter`をpublishする関数である。  
2. Qt  
   クラス`MainDialog`で1行のテキストエディタ、プッシュボタンを配置したダイアログを定義している。ボタンをクリックすると`publishString()`が実行されるように、ボタンクリックのシグナルと自作スロット`publishString()`を接続した。  
   `publishString()`ではテキストエディタから文字列を取得し、ROSノードの`publish_message()`へと文字列を渡している。
3. 実行  
   ROSとQtを組み合わせるにあたり、QtやROSの処理を実行する方法を工夫する必要がある。一般的に使用されるROSの`spin()`やQtの`exec()`は、ROSやQtの処理が終了するまでループ処理を行う。したがって、これらを利用するとQtかROSのどちらか一方しか実行できない。  
   そこで、ROSは`spin_some()`、Qtは`processEvents()`で処理を実行するとともに、whileを使ってループ構造を実装している。[spin_some()](https://github.com/ros2/rclcpp/blob/humble/rclcpp/src/rclcpp/executors.cpp#L18)は、関数が呼び出された時点で実行できるROSの処理を行う。[processEvents()](https://doc.qt.io/qt-5/qcoreapplication.html#processEvents)も同様に、関数呼び出し時点で実行できるQtの処理を行う関数である。  

    ```cpp
    // 実行
    while (rclcpp::ok())
    {
       rclcpp::spin_some(rosNode);
       app.processEvents();
    }
    ```

### subscriberの作成

ディレクトリ`qt_pubsub/src/`内に、以下の5つのソースファイルを作成する。

- [qt_listener.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_listener.cpp)
- [qt_listener_GUI.hpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_listener_GUI.hpp)
- [qt_listener_GUI.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_listener_GUI.cpp)
- [qt_listener_rosNode.hpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_listener_rosNode.hpp)
- [qt_listener_rosNode.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/src/qt_listener_rosNode.cpp)

1. ROSノード  
  クラス`QtListener`ではROSノード`qt_listener`を定義し、文字列型のトピック`chatter`のsubscriberを作成する。トピック購読時のコールバック関数に`chatter_callback()`を指定している。  
  `chatter_callback`は引数で受け取ったメッセージから文字列を取り出し、Qtの`stringCallback()`関数に渡す。  
2. Qt  
   クラス`MainDialog`で1行のテキストエディタを配置したダイアログを定義している。メンバ関数`stringCallback()`は、引数で指定された文字列をテキストエディタに出力する関数である。  
3. 実行  
   `qt_talker`での実装と同様に、`spin_some()`および`processEvents()`利用してROSとQtの処理を行う。

### ビルドの設定

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/CMakeLists.txt)および[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_pubsub/package.xml)にビルド設定を記述する。追加する行を以下に示す。

1. CMakeLists.txt  
他のパッケージへの依存を解決し、マクロ`Q_OBJECT`を利用するための設定、実行ファイルの作成に関して追記する。また、ビルド結果を`install/`配下に作成して、ROS 2コマンドで実行できるように設定している。

    ```diff
    find_package(ament_cmake REQUIRED)
    + find_package(rclcpp REQUIRED)
    + find_package(std_msgs REQUIRED)
    
    + # packages for Qt
    + find_package(Qt5Core REQUIRED)
    + find_package(Qt5Widgets REQUIRED)

    + set(CMAKE_AUTOMOC ON)
    
    + add_executable(qt_talker src/qt_talker.cpp src/qt_talker_GUI.cpp src/qt_talker_rosNode.cpp)
    + add_executable(qt_listener src/qt_listener.cpp src/qt_listener_GUI.cpp src/qt_listener_rosNode.cpp)
    
    + ament_target_dependencies(qt_talker rclcpp Qt5Core Qt5Widgets)
    + ament_target_dependencies(qt_listener rclcpp Qt5Core Qt5Widgets)

    + install(
    +   TARGETS
    +   qt_talker
    +   qt_listener
    +   DESTINATION lib/${PROJECT_NAME}
    +   )

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
    ```

2. package.xml  
他パッケージへの依存を記述する。
  
    ```diff
    + <depend>rclcpp</depend>
    + <depend>std_msgs</depend>
    
    + <build_depend>qtbase5-dev</build_depend>
    ```

## ROSノードの起動

作成したパッケージをビルドし、ノード`qt_talker`と`qt_listener`を実行する。

- パッケージのビルド

  ```bash
  cd ~/ros2_lecture_ws/
  colcon build
  ```

- qt_talkerの起動  
  1つ目の端末で以下を実行する。

  ```bash
  . install/setup.bash 
  ros2 run qt_pubsub qt_talker
  ```

- qt_listenerの起動  
  2つ目の端末で以下を実行する。

  ```bash
  . install/setup.bash 
  ros2 run qt_pubsub qt_listener
  ```

`qt_talker`を起動すると以下のようなウィンドウが表示される。1段目のエディタに文字を入力してボタンをクリックするとトピックが送信される。  

![qt_talkerの実行結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/ui/qt_pubsub/img/result_publisher.png)  

`qt_listener`を起動すると以下のようなウィンドウが表示される。トピックを購読すると、受信したメッセージが表示される。

![qt_listenerの実行結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/ui/qt_pubsub/img/result_subscriber.png)  

## 参考

- [ROS講座94 Qtでpub・subする](https://qiita.com/srs/items/93ab82b96637f405e657)
- [Qt日本語ブログ-Qtを始めよう!](https://www.qt.io/ja-jp/blog/tag/getting-started-with-qt)
