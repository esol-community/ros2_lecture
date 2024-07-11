# 実習ROS 2 ROS 2 Logger

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

プログラムを実装、実行させる上で、情報を出力する（printデバッグする）ことは手軽で有用な方法である。  
ROS(ROS 1, ROS 2)では、情報を出力するためにloggerの仕組みがあり、どのノードがいつ出力した情報であるかを簡単に表示できる。  
また、logger情報を統合的に表示する`rqt_console`というツールも存在する。  
したがって、ROSでは情報の出力にloggerの仕組みを用いることが通常のやり方になる。

そこで、本記事ではROS 2 Loggerについて、以下の内容を扱う。

- 各種のログレベルで情報を出力させるノードと、launchファイルの実装
- ログレベルを「Info」と「Debug」で動的に変更するノードの実装
- rqt_consoleによるログの表示のデモ
- より高度なログ出力制御方法の紹介
- ROS 1のLoggerとの違い（サービスでログレベルを変えることについて）

## ログの出力先

ROS 2のログシステムでは、以下の3つのログ出力先を扱える。

1. ターミナル
2. ストレージ上のログファイル
3. `/rosout`トピック

デフォルトでは、標準エラー出力と、`~/.ros`ディレクトリ配下のファイル、`/rosout`トピックにログを出力する。  
これらのログの出力先は、ノードごとに設定が可能である。

## ログレベルについて

ROS Loggerでは、出力するメッセージにログレベルを指定する。  
ログレベルはレベルが高い順に以下の種類がある。  

```
Fatal
Error
Warn
Info
Debug
```

どの情報をどのレベルで出力すべきという標準は存在しないが、[公式ドキュメント](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)には、ログレベルの目安が記載されている。  

ログの出力先には、設定されたログレベル以上のログが出力される。  
すなわち、ログレベルがInfoのときはInfo/Warn/Error/Fatalのログが、ログレベルがDebugのときはDebug/Info/Warn/Error/Fatalのログが出力される。  
デフォルトのログレベルはInfoとなっており、Debugレベルで出力したメッセージは出力されない。

## 実装

### パッケージの作成

以下のコマンドを実行し、`ros2_logger`という名前のパッケージを作成する。

```sh
# ワークスペースフォルダに移動
cd ~/ros2_lecture_ws/src

# パッケージを作成
ros2 pkg create ros2_logger --build-type ament_cmake --dependencies rclcpp 
```

### ログレベルを表示するプログラム

まず、各種のログレベルを表示するプログラムを実装する。  
パッケージ配下の`src`ディレクトリに、`logger_sample.cpp`というファイルを作成する。  

```sh
cd ~/ros2_lecture_ws/src/ros2_logger
touch src/logger_sample.cpp
```

実装するコードを以下に示す。  
プログラムの構造は、[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)と大きな変化はない。  

```cpp
#include <chrono>

#include "rclcpp/rclcpp.hpp"

class LoggerSample : public rclcpp::Node
{
public:
  LoggerSample()
  : Node("logger_sample"), count_(0)
  {
    // タイマの設定
    // 1000ms(1秒)ごとに、print_all_log_level()関数を実行する
    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&LoggerSample::print_all_log_level, this));
  }

private:
  // カウンタ
  uint32_t count_;

  // タイマの変数
  rclcpp::TimerBase::SharedPtr timer_;

  void print_all_log_level()
  {
    // それぞれのログレベルでメッセージを表示
    RCLCPP_FATAL(this->get_logger(), "print_all_log_level FATAL: %d", count_);
    RCLCPP_ERROR(this->get_logger(), "print_all_log_level ERROR: %d", count_);
    RCLCPP_WARN(this->get_logger(), "print_all_log_level WARN: %d", count_);
    RCLCPP_INFO(this->get_logger(), "print_all_log_level INFO: %d", count_);
    RCLCPP_DEBUG(this->get_logger(), "print_all_log_level DEBUG: %d", count_);

    count_++;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggerSample>());
  rclcpp::shutdown();
  return 0;
}

```

[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)と同様にしてビルドの設定([CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_logger/./CMakeLists.txt), [package.xml](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_logger/./package.xml))の設定を行い、ビルドを行う。  
ここでは、`logger_sample`という名前で実行可能ファイルをを作成する。  

(注意)：リンク先のCMakeLists.txtは、以下の項で実装するプログラムのビルド設定も含んだものになっている。　　

ビルドと、ワークスペースの設定は以下のコマンドで行う。

```sh
cd ~/ros2_lecture_ws
colcon build --packages-select ros2_logger
. install/setup.bash
```

プログラムを実行すると、Debug以外のログレベルのメッセージが出力されていることがわかる。

```sh
$ ros2 run ros2_logger logger_sample
[FATAL] [1662615938.519017788] [logger_sample]: print_all_log_level FATAL: 0
[ERROR] [1662615938.519294539] [logger_sample]: print_all_log_level ERROR: 0
[WARN] [1662615938.519320698] [logger_sample]: print_all_log_level WARN: 0
[INFO] [1662615938.519336865] [logger_sample]: print_all_log_level INFO: 0
[FATAL] [1662615939.518871860] [logger_sample]: print_all_log_level FATAL: 1
[ERROR] [1662615939.518931129] [logger_sample]: print_all_log_level ERROR: 1
[WARN] [1662615939.518939573] [logger_sample]: print_all_log_level WARN: 1
[INFO] [1662615939.518942735] [logger_sample]: print_all_log_level INFO: 1
[FATAL] [1662615940.518980823] [logger_sample]: print_all_log_level FATAL: 2
[ERROR] [1662615940.519107876] [logger_sample]: print_all_log_level ERROR: 2
[WARN] [1662615940.519129305] [logger_sample]: print_all_log_level WARN: 2
[INFO] [1662615940.519146145] [logger_sample]: print_all_log_level INFO: 2
[FATAL] [1662615941.518917530] [logger_sample]: print_all_log_level FATAL: 3
[ERROR] [1662615941.518981303] [logger_sample]: print_all_log_level ERROR: 3
[WARN] [1662615941.518986084] [logger_sample]: print_all_log_level WARN: 3
[INFO] [1662615941.518990018] [logger_sample]: print_all_log_level INFO: 3
[FATAL] [1662615942.518916550] [logger_sample]: print_all_log_level FATAL: 4
[ERROR] [1662615942.519037971] [logger_sample]: print_all_log_level ERROR: 4
[WARN] [1662615942.519058123] [logger_sample]: print_all_log_level WARN: 4
[INFO] [1662615942.519076469] [logger_sample]: print_all_log_level INFO: 4
```

Debugも含めて表示させるには、引数`--ros-args --log-level debug`を与えて実行する。

```sh
$ ros2 run ros2_logger logger_sample --ros-args --log-level debug
#####   ~~~~~~~~~~~ rclのログが大量に出るので省略 ~~~~~~~~~~~ #####
[DEBUG] [1662615969.235427232] [rcl]: Timeout calculated based on next scheduled timer: true
[DEBUG] [1662615969.235431511] [rcl]: Guard condition in wait set is ready
[DEBUG] [1662615969.235433653] [rcl]: Guard condition in wait set is ready
[DEBUG] [1662615969.235439233] [rcl]: Waiting with timeout: 0s + 999962583ns
[DEBUG] [1662615969.235441576] [rcl]: Timeout calculated based on next scheduled timer: true
[DEBUG] [1662615970.235553455] [rcl]: Timer in wait set is ready
[DEBUG] [1662615970.235661496] [rcl]: Calling timer
[FATAL] [1662615970.235698260] [logger_sample]: print_all_log_level FATAL: 0
[ERROR] [1662615970.235847486] [logger_sample]: print_all_log_level ERROR: 0
[WARN] [1662615970.235868729] [logger_sample]: print_all_log_level WARN: 0
[INFO] [1662615970.235885487] [logger_sample]: print_all_log_level INFO: 0
[DEBUG] [1662615970.235901980] [logger_sample]: print_all_log_level DEBUG: 0
[DEBUG] [1662615970.235944386] [rcl]: Waiting with timeout: 0s + 999460733ns
[DEBUG] [1662615970.235960036] [rcl]: Timeout calculated based on next scheduled timer: true
[DEBUG] [1662615970.235977184] [rcl]: Guard condition in wait set is ready
[DEBUG] [1662615970.235994696] [rcl]: Waiting with timeout: 0s + 999409711ns
[DEBUG] [1662615970.236006551] [rcl]: Timeout calculated based on next scheduled timer: true
[DEBUG] [1662615971.235600372] [rcl]: Timer in wait set is ready
[DEBUG] [1662615971.235689933] [rcl]: Calling timer
[FATAL] [1662615971.235709487] [logger_sample]: print_all_log_level FATAL: 1
[ERROR] [1662615971.235782768] [logger_sample]: print_all_log_level ERROR: 1
[WARN] [1662615971.235801615] [logger_sample]: print_all_log_level WARN: 1
[INFO] [1662615971.235829530] [logger_sample]: print_all_log_level INFO: 1
[DEBUG] [1662615971.235847057] [logger_sample]: print_all_log_level DEBUG: 1
```

次に、このノードを起動するlaunchファイルを実装する。  
パッケージの中に`launch`ディレクトリを作成し、その中に`ros2_logger.launch.py`というファイルを作成する。

```sh
mkdir ~/ros2_lecture_ws/src/ros2_logger/launch
touch ~/ros2_lecture_ws/src/ros2_logger/launch/ros2_logger.launch.py
```

このlaunchファイルは、引数`log_level`を持ち、それによって指定されたログレベルを起動するノードに対して指定する。  
サンプルコード[ros2_logger.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_logger/./launch/ros2_logger.launch.py)を以下に示す。  

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # ログレベルを表す設定を定義
    log_level = LaunchConfiguration("log_level")

    # ログレベルを表す引数を、デフォルトレベルinfoで定義
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    # 引数付きでノードを起動
    logger_sample_node = Node(
        package="ros2_logger",
        executable="logger_sample",
        name="logger_sample",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level],
    )

    ld.add_action(log_level_arg)
    ld.add_action(logger_sample_node)

    return ld
```

このlaunchファイルは例えば以下のコマンドで実行できる。  
実行結果はノード単体で起動したときと同様であるため、省略する。

```bash
$ ros2 launch ros2_logger ros2_logger.launch.py log_level:=debug
```

### ログレベルを変更するプログラム

一定時間ごとにログを出力しつつ、定期的にログレベルを切り替えるサンプルプログラムを実装する。  
このプログラムによって、ログレベルを動的に切り替えたときの出力の変化を確認する。  
作成したパッケージ配下の`src`ディレクトリに、`logger_switch_level.cpp`というファイルを作成する。  

```sh
cd ~/ros2_lecture_ws/src/ros2_logger
touch src/logger_switch_level.cpp
```

ファイルを作成したら、[リンク先](https://github.com/esol-community/ros2_lecture/tree/main/beginner/ros2_logger/./src/logger_switch_level.cpp)のように実装する。  

コード中でログレベルを切り替えている部分は以下の部分である。`next_log_level`には`RCUTILS_LOG_SEVERITY_INFO`等を設定する。

```cpp
    // ログレベルを設定
    auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), next_log_level);
```

上記の手順と同様にビルドを行い、実行する。  
以下の結果からは、countが0から4のときはDEBUGレベルの出力がないが、countが5になると出力されるようになることが分かる。

これは、ログの出力を1秒ごと、ログレベルの切り替えを5秒ごとに行うよう実装したためである。  

```sh
$ ros2 run ros2_logger logger_switch_level
[FATAL] [1662683095.148474711] [switch_logger_level]: print_all_log_level FATAL: 0
[ERROR] [1662683095.148790953] [switch_logger_level]: print_all_log_level ERROR: 0
[WARN] [1662683095.148820123] [switch_logger_level]: print_all_log_level WARN: 0
[INFO] [1662683095.148841598] [switch_logger_level]: print_all_log_level INFO: 0
[FATAL] [1662683096.148395470] [switch_logger_level]: print_all_log_level FATAL: 1
[ERROR] [1662683096.148535110] [switch_logger_level]: print_all_log_level ERROR: 1
[WARN] [1662683096.148560372] [switch_logger_level]: print_all_log_level WARN: 1
[INFO] [1662683096.148580671] [switch_logger_level]: print_all_log_level INFO: 1
[FATAL] [1662683097.148465962] [switch_logger_level]: print_all_log_level FATAL: 2
[ERROR] [1662683097.148624458] [switch_logger_level]: print_all_log_level ERROR: 2
[WARN] [1662683097.148647186] [switch_logger_level]: print_all_log_level WARN: 2
[INFO] [1662683097.148664582] [switch_logger_level]: print_all_log_level INFO: 2
[FATAL] [1662683098.148462374] [switch_logger_level]: print_all_log_level FATAL: 3
[ERROR] [1662683098.148601918] [switch_logger_level]: print_all_log_level ERROR: 3
[WARN] [1662683098.148626649] [switch_logger_level]: print_all_log_level WARN: 3
[INFO] [1662683098.148646353] [switch_logger_level]: print_all_log_level INFO: 3
[FATAL] [1662683099.148381183] [switch_logger_level]: print_all_log_level FATAL: 4
[ERROR] [1662683099.148517111] [switch_logger_level]: print_all_log_level ERROR: 4
[WARN] [1662683099.148539597] [switch_logger_level]: print_all_log_level WARN: 4
[INFO] [1662683099.148559388] [switch_logger_level]: print_all_log_level INFO: 4
[INFO] [1662683099.148588678] [switch_logger_level]: Setting log level to DEBUG

[FATAL] [1662683100.148547069] [switch_logger_level]: print_all_log_level FATAL: 5
[ERROR] [1662683100.148699902] [switch_logger_level]: print_all_log_level ERROR: 5
[WARN] [1662683100.148725278] [switch_logger_level]: print_all_log_level WARN: 5
[INFO] [1662683100.148746018] [switch_logger_level]: print_all_log_level INFO: 5
[DEBUG] [1662683100.148765319] [switch_logger_level]: print_all_log_level DEBUG: 5
[FATAL] [1662683101.148520034] [switch_logger_level]: print_all_log_level FATAL: 6
[ERROR] [1662683101.148670781] [switch_logger_level]: print_all_log_level ERROR: 6
[WARN] [1662683101.148697405] [switch_logger_level]: print_all_log_level WARN: 6
[INFO] [1662683101.148717487] [switch_logger_level]: print_all_log_level INFO: 6
[DEBUG] [1662683101.148736597] [switch_logger_level]: print_all_log_level DEBUG: 6
[FATAL] [1662683102.148495234] [switch_logger_level]: print_all_log_level FATAL: 7
[ERROR] [1662683102.148635177] [switch_logger_level]: print_all_log_level ERROR: 7
[WARN] [1662683102.148659308] [switch_logger_level]: print_all_log_level WARN: 7
[INFO] [1662683102.148677251] [switch_logger_level]: print_all_log_level INFO: 7
[DEBUG] [1662683102.148695903] [switch_logger_level]: print_all_log_level DEBUG: 7
```

### rqt_console

ROS Loggerで出力するログにフィルタをかけ、知りたい情報のみを表示するGUIツールとして、rqt_consoleがある。  
ここでは、この記事で実装したプログラムと、rqt_consoleを同時に実行したときの結果を確認する。

```sh
# ターミナル1で実行
$ ros2 run ros2_logger logger_sample
```

```sh
# ターミナル2で実行
$ ros2 run rqt_console rqt_console
```

rqt_consoleの外観を以下に示す。  
rqt_consoleでは、中段の「Exclude Messages」で表示しないログレベルを設定できる。  
また、下段の「Highlight Messages」で条件に一致したメッセージを強調表示できる。  

![rqt_console](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/beginner/ros2_logger/./img/rqt_console.png)

## より高度なログ出力制御

これまでに実装したものに加えて、ROS 2ではより高度なログ出力の制御が可能である。  
以下では、利用できる機能について簡単に紹介する。

これらの機能を使いこなすと、ログ出力の負荷を制御したり、必要なログのみを所望の場所に出力したりすることが可能になる。

### rclcppのログAPI

サンプルプログラムでは、`RCLCPP_DEBUG`, `RCLCPP_INFO`などのマクロを使用してログを出力した。  
RCLCPP（C++によるROS 2のクライアントライブラリ）では、他にも以下のようなログAPIが提供されている。

```cpp
// ログレベルを指定してログを出力
RCLCPP_INFO(get_logger(), "This message will be printed.");

// 最初の一度だけ出力
RCLCPP_INFO_ONCE(get_logger(), "This message will only be printed once.");

// 条件を満たしたときだけ出力
RCLCPP_INFO_EXPRESSION(get_logger(), count_ == 5, "This message will only be printed when count_ == 5.");

// 関数conditional_functionがtrueを返したときだけ出力
RCLCPP_INFO_FUNCTION(get_logger(), &conditional_function, "This message will only be printed when the function returns true.");

// 2回目以降のみ出力
RCLCPP_INFO_SKIPFIRST(get_logger(), "This message will be printed after the first message.");

// 2000msごとに出力
RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "This message will be printed at most once every 2000ms.");

// 2000msごとに出力（ただし、1回目は出力しない）
RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 2000, "This message will be printed after the first message and at most once every 2000ms.");

```

さらに、上記のそれぞれに対して、変数をストリームで渡せるバージョンも存在する。  
例えば、`RCLCPPP_INFO`では変数をprintf関数の書式で渡していたが、`RCLCPP_INFO_STREAM`では以下のようにストリームで渡せる。

```cpp
// RCLCPP_INFO(get_logger(), "This message will be printed with a number: %d", count_); と同じ
RCLCPP_INFO_STREAM(get_logger(), "This message will be printed with a number: " << count_);
```

### 環境変数によるログ設定

ログファイルの出力場所は、以下の順番で決められる。

1. 環境変数`ROS_LOG_DIR`が設定されている場合、そのディレクトリ
2. `ROS_LOG_DIR`が設定されていない場合、以下の通り
  1. 環境変数`ROS_HOME`が設定されている場合、`$ROS_HOME/log`ディレクトリ
  2. `ROS_HOME`が設定されていない場合、`~/.ros`ディレクトリ

### コマンドライン引数によるログ出力設定

ログはターミナル、ファイル、`/rosout`トピックに出力されるが、以下のようにして無効化できる。

```bash
# ターミナルにログを出力しない
$ ros2 run ros2_logger logger_sample --ros-args --disable-stdout-logs

# ログファイルにログを出力しない
$ ros2 run ros2_logger logger_sample --ros-args --disable-external-lib-logs

# /rosoutトピックにログを出力しない
$ ros2 run ros2_logger logger_sample --ros-args --disable-rosout-logs
```

## ROS 1との違い

ROS 1では、ノードに対してログレベルを変更するサービスを呼び出すことができた。  
そのため、ノードを使わずにCLIから動的にログレベルを変更できていた。  

ROS 2では、この機能はIron（Humbleの次のディストリビューション）からサポートされている。  
そのため、Humbleではノード外部からログレベルを変更できない。

## 参考

1. [ROS講座06 ROS Logger](https://qiita.com/srs/items/47e5fd8fe994431d92b7)

2. [ROS 2公式チュートリアル：About logging and logger configuration](https://docs.ros.org/en/humble/Concepts/About-Logging.html)

3. [ROS 2公式チュートリアル：Using rqt_console to view logs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)

4. [ROS 2公式デモ：Configuring loggers](https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html)
