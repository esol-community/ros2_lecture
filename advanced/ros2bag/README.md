# 実習ROS 2 rosbag2を使う

## 環境

本記事は以下の環境を想定して記述されている。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

`rosbag2`は、ROSトピックをファイルとして保存、再生するツールである。  
データを保存して再生することで同じ入力を繰り返し与えられるようになる。  
これにより、デバッグを容易にしたり、ソフトウェアの性能、品質の変化を繰り返し検証したりすることが可能になる。  
この記事では、`rosbag2`の基本的な使い方を解説する。  

## 前提

この記事では、実行結果を示すために[URDFを記述する2](https://qiita.com/esol-h-matsumoto/items/c81e818224748dd6a73b)で作成したファイルを利用する。  
そのため、`~/ros2_lecture_ws/`にROS 2のワークスペースがあり、その中に`urdf2`パッケージがあることを前提とする。

## インストール

`rosbag2`は基本的に`ros2 bag`コマンドで使用する。  
コマンドが認識されるように、必要なパッケージを以下のコマンドでインストールする。  
このコマンドは、`rosbag2`に関連するパッケージをまとめてインストールする。

```sh
sudo apt install ros-humble-rosbag2
```

`ros-humble-ros-base`パッケージを含む方法でROS 2 Humbleをインストールしていれば、既にインストールされているはずである。([参考](https://www.ros.org/reps/rep-2001.html#id24))

## rosbag2のCLIツール

`ros2 bag`コマンドの代表的なサブコマンドとして、以下の3つが挙げられる。  
以下では、順番に例を示しながら説明する。  

- `record` : トピックを記録しファイルに保存する
- `info` : ファイルの情報を表示する
- `play` : ファイルを再生する

### データの記録

よく用いられる使い方を実行例とともに示す。

- `ros2 bag record -a`

全てのトピックを保存する。  
ここでは、`simple_urdf5.urdf`のジョイント角度を変化させたときに流れるROSトピックを保存する例を示す。  

1つ目のターミナルで、ROSトピックを保存するコマンドを実行する。  
記録をやめるときは、`ctrl+C`でプログラムを終了させればよい。ただし、実際に終了させるのは2つ目のターミナルでジョイントを操作した後にする。

```sh
# 1つ目のターミナルで実行
$ source /opt/ros/humble/setup.bash
$ ros2 bag record -a
[INFO] [1712829858.209008073] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [1712829858.209803309] [rosbag2_storage]: Opened database 'rosbag2_2024_04_11-10_04_18/rosbag2_2024_04_11-10_04_18_0.db3' for READ_WRITE.
[INFO] [1712829858.210174977] [rosbag2_recorder]: Listening for topics...
[INFO] [1712829858.210181207] [rosbag2_recorder]: Event publisher thread: Starting
[INFO] [1712829858.211180195] [rosbag2_recorder]: Subscribed to topic '/tf'
[INFO] [1712829858.211784093] [rosbag2_recorder]: Subscribed to topic '/rosout'
[INFO] [1712829858.212109612] [rosbag2_recorder]: Subscribed to topic '/tf_static'
[INFO] [1712829858.212514715] [rosbag2_recorder]: Subscribed to topic '/robot_description'
[INFO] [1712829858.212934563] [rosbag2_recorder]: Subscribed to topic '/parameter_events'
[INFO] [1712829858.213463362] [rosbag2_recorder]: Subscribed to topic '/joint_states'
[INFO] [1712829858.213749679] [rosbag2_recorder]: Subscribed to topic '/events/write_split'
[INFO] [1712829858.213811301] [rosbag2_recorder]: Recording...
[INFO] [1712829861.828320826] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
[INFO] [1712829861.829472027] [rosbag2_recorder]: Event publisher thread: Exiting
[INFO] [1712829861.829592256] [rosbag2_recorder]: Recording stopped
[INFO] [1712829861.832200216] [rosbag2_recorder]: Recording stopped
```

ターミナルの出力からは、保存されるトピック名が確認できる。  
2つ目のターミナルで以下のコマンドを実行し、表示されたウインドウのスライダーからジョイント角度を変化させる。  

```sh
# 2つ目のターミナルで実行

# urdf2パッケージを実行するために必要な依存パッケージをインストールする。
$ cd ~/ros2_lecture_ws
$ rosdep install --from-paths src/urdf2 -y
$ source install/setup.bash

# simple_urdf5.urdfを表示する
$ ros2 launch urdf_launch display.launch.py urdf_package:=urdf2 urdf_package_path:=urdf/simple_urdf5.urdf 
```

![simple_urdf5](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/advanced/ros2bag/./.img/simple_urdf5.png)

ジョイントをいくらか動かしたら、2つのターミナルのプログラムを終了させる。  
実行を終了すると、新しくディレクトリが作成され、その中にいくつかのファイルが作成されていることが分かる。  
拡張子`.db3`で終わっているファイルにデータの中身が保存され、メタデータは`metadata.yaml`に保存される。(注：[デフォルトのrosbag2のデータフォーマットについて](#デフォルトのrosbag2のデータフォーマットについて))  

```sh
$ ls rosbag2_2024_04_11-09_53_38
metadata.yaml  rosbag2_2024_04_11-09_53_38_0.db3
```

- `ros2 bag record <トピック名...>`

特定のトピックのみを保存できる。トピックは複数指定できる。  

```sh
# /joint_states のみを保存する例
$ ros2 bag record /joint_states
[INFO] [1712829884.133863422] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [1712829884.134576549] [rosbag2_storage]: Opened database 'rosbag2_2024_04_11-10_04_44/rosbag2_2024_04_11-10_04_44_0.db3' for READ_WRITE.
[INFO] [1712829884.134876560] [rosbag2_recorder]: Listening for topics...
[INFO] [1712829884.134890311] [rosbag2_recorder]: Event publisher thread: Starting
[INFO] [1712829884.135692144] [rosbag2_recorder]: Subscribed to topic '/joint_states'
[INFO] [1712829884.135725894] [rosbag2_recorder]: Recording...
[INFO] [1712829884.135854573] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
[INFO] [1712829887.053425270] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
[INFO] [1712829887.054379654] [rosbag2_recorder]: Event publisher thread: Exiting
[INFO] [1712829887.054488154] [rosbag2_recorder]: Recording stopped
[INFO] [1712829887.055596658] [rosbag2_recorder]: Recording stopped
```

- `ros2 bag record -o <ディレクトリ名>`

出力ディレクトリ名を指定できる。

```sh
# rosbag_outputというディレクトリ名で保存する例
$ ros2 bag record -a -o rosbag_output
[INFO] [1712829921.680191718] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [1712829921.681924127] [rosbag2_storage]: Opened database 'rosbag_output/rosbag_output_0.db3' for READ_WRITE.
[INFO] [1712829921.682711612] [rosbag2_recorder]: Listening for topics...
[INFO] [1712829921.682730551] [rosbag2_recorder]: Event publisher thread: Starting
[INFO] [1712829921.685310849] [rosbag2_recorder]: Subscribed to topic '/tf'
[INFO] [1712829921.686693247] [rosbag2_recorder]: Subscribed to topic '/rosout'
[INFO] [1712829921.687176454] [rosbag2_recorder]: Subscribed to topic '/tf_static'
[INFO] [1712829921.687789462] [rosbag2_recorder]: Subscribed to topic '/robot_description'
[INFO] [1712829921.688541636] [rosbag2_recorder]: Subscribed to topic '/parameter_events'
[INFO] [1712829921.689306413] [rosbag2_recorder]: Subscribed to topic '/joint_states'
[INFO] [1712829921.689700160] [rosbag2_recorder]: Subscribed to topic '/events/write_split'
[INFO] [1712829921.689764745] [rosbag2_recorder]: Recording...
[INFO] [1712829925.407919031] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
[INFO] [1712829925.409472084] [rosbag2_recorder]: Event publisher thread: Exiting
[INFO] [1712829925.409651664] [rosbag2_recorder]: Recording stopped
[INFO] [1712829925.411853042] [rosbag2_recorder]: Recording stopped
```

その他にも様々なオプションがある。使い方は`ros2 bag record --help`を実行すると確認できる。

### データの情報表示

`ros2 bag info <保存ディレクトリ名>`を実行すると、保存したデータの情報を確認できる。

```sh
$ ros2 bag info rosbag2_2024_04_11-10_04_18

Files:             rosbag2_2024_04_11-10_04_18_0.db3
Bag size:          45.2 KiB
Storage id:        sqlite3
Duration:          3.586s
Start:             Apr 11 2024 10:04:18.213 (1712829858.213)
End:               Apr 11 2024 10:04:21.800 (1712829861.800)
Messages:          84
Topic information: Topic: /events/write_split | Type: rosbag2_interfaces/msg/WriteSplitEvent | Count: 0 | Serialization Format: cdr
                   Topic: /joint_states | Type: sensor_msgs/msg/JointState | Count: 36 | Serialization Format: cdr
                   Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 0 | Serialization Format: cdr
                   Topic: /robot_description | Type: std_msgs/msg/String | Count: 1 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 10 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 36 | Serialization Format: cdr

```

### データの再生

スライダーでジョイントを操作したときに記録したrosbagファイルを再生し、同じようにジョイントが動くことを確かめる。  

1つ目のターミナルでRvizを実行する。

```sh
ros2 run rviz2 rviz2
```

RvizのGUIウインドウを操作して、Fixed Frameを「base_link」とし、TF、RobotModelを表示させるようにする。  
また、左のパネルのRobotModelの項目を開き、以下の画像のように、「description_topic」を「robot_description」に指定する。

![rviz2_configuration](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/advanced/ros2bag/./.img/rviz2_configuration.png)

2つ目のターミナルで、先程記録したrosbagファイルのディレクトリを引数として、`ros2 bag play`コマンドを実行する。

```sh
$ ros2 bag play rosbag2_2024_04_11-10_04_18/
[INFO] [1712830034.099801306] [rosbag2_storage]: Opened database 'rosbag2_2024_04_11-10_04_18/rosbag2_2024_04_11-10_04_18_0.db3' for READ_ONLY.
[INFO] [1712830034.099846307] [rosbag2_player]: Set rate to 1
[INFO] [1712830034.106853886] [rosbag2_player]: Adding keyboard callbacks.
[INFO] [1712830034.106884151] [rosbag2_player]: Press SPACE for Pause/Resume
[INFO] [1712830034.106888756] [rosbag2_player]: Press CURSOR_RIGHT for Play Next Message
[INFO] [1712830034.106892600] [rosbag2_player]: Press CURSOR_UP for Increase Rate 10%
[INFO] [1712830034.106896418] [rosbag2_player]: Press CURSOR_DOWN for Decrease Rate 10%
[INFO] [1712830034.107178461] [rosbag2_storage]: Opened database 'rosbag2_2024_04_11-10_04_18/rosbag2_2024_04_11-10_04_18_0.db3' for READ_ONLY.
```

すると、Rviz上にロボットモデルが現れ、GUIウインドウで操作したときと全く同じようにロボットが動く。  
これは、データを記録したときと同じ情報がROSネットワークに流れ、Rvizがそれらを受け取って表示しているためである。  
具体的には、ロボットモデル、TF、joint_states等のトピックがrosbagファイルから再生されている。

![ros2bag_play](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/advanced/ros2bag/./.img/rosbag_play.png)

## launchファイル内で使う

[rosbag2リポジトリ](https://github.com/ros2/rosbag2/tree/humble#using-in-launch)では、launchファイル内で`ros2 bag`コマンドを使う例が紹介されている。  
launchファイル内で他のプログラムと合わせて記録や再生できるようになると便利な場合がある。  

以下に上記リンクで示されているlaunchファイルの実装([rosbag_record_all.launch.py](https://github.com/esol-community/ros2_lecture/tree/main/advanced/ros2bag/./rosbag_record_all.launch.py))を示す。  
なお`ros2 bag`コマンドに限らず、コマンド全般を以下のように実行することが可能である。

```python
import launch


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        )
    ])

```

## QoSオーバーライド

ROS 2ではトピックごとにQoSが定められている(参考：[実習ROS 2：QoSを設定する](https://qiita.com/s-kitajima/items/ebd40fadf2e53babf220))ため、rosbagファイルにもQoSの設定が保存される。  
rosbagファイルはデフォルトでは保存したときと同じQoSで再生されるが、QoSを上書きして再生することもできる。  

上書きするQoS設定はyamlファイルで行い、再生時に以下のようにパスを指定して実行する。

```sh
ros2 bag play <rosbagディレクトリへのパス> --qos-profile-overrides-path <yamlファイルへのパス>
```

設定ファイルの例を[qos_override.yaml](https://github.com/esol-community/ros2_lecture/tree/main/advanced/ros2bag/./qos_override.yaml)に示す。以下の例では、`joint_states`トピックのReliabilityポリシーとHistoryポリシーを設定している。  
設定できる項目の一覧は[ROS 2公式ドキュメント](https://docs.ros.org/en/humble/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html)に記載がある。  

```yaml
/joint_states:
  reliability: best_effort
  history: keep_all
```

## その他のツール

rosbagファイルに記録されているトピックの値をグラフとして可視化し、デバッグを容易にするツールとして、[rqt_bag](https://index.ros.org/r/rqt_bag/#humble)と、[plotjuggler](https://github.com/PlotJuggler/plotjuggler-ros-plugins)がある。  

2つのツールの具体的な説明は[実習ROS 2：ROStools](https://qiita.com/s-kubota/items/e9d69b44d6659d44e95c#ros-2向けツール群)に記載されている。

## デフォルトのrosbag2のデータフォーマットについて

ROS 2 Humbleでは、データはデータベースシステムの一種であるSQLiteのデータ構造で保存される。  
これに対して、ROS 2 Iron以降のディストリビューションでは、デフォルトのデータフォーマットがMCAPが使われている。この場合、データファイルの拡張子は`.mcap`となる。

mcapは、ロボット向けの利用を想定して設計されたデータコンテナフォーマットであり、以下のような特徴がある。

- マルチモーダルなセンサデータや、小さなデータ量で高頻度でPub/Subされるようなメッセージのロギングに適している
- 書き込み性能が優れており、SQLite形式よりも信頼性の高いデータ保存が可能
- Protocol Buffers, JSONなどといった、特定のシリアライゼーションフォーマットに依存しない

ROS 2 Iron以降のディストリビューションにおいても、SQLite形式でのrosbag2ツールの利用は可能である。  
ROS 2 Humbleにおいても、`ros-humble-rosbag2-storage-mcap`パッケージをインストールすることで、mcap形式でのデータの扱いが可能になる。

なお、MCAPのツールを用いて、SQLite形式のデータファイルとMCAP形式のデータファイルの相互変換ができる。

## 参考

- [ROS 2公式ドキュメント：Recording and playing back data](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [ROS 2公式ドキュメント：rosbag2: Overriding QoS Policies](https://docs.ros.org/en/humble/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html)
- [ros2/rosbag2リポジトリ: README.md](https://github.com/ros2/rosbag2/blob/rolling/README.md)
- [ros2/designリポジトリ：rosbags.md](https://github.com/ros2/design/blob/ros2bags/articles/rosbags.md)
- [plotjuggler-ros-plugins](https://github.com/PlotJuggler/plotjuggler-ros-plugins)
- [MCAPドキュメント](https://mcap.dev/)
- [MCAPドキュメント：SQLite形式とMCAP形式における書き込み性能の比較](https://mcap.dev/guides/benchmarks/rosbag2-storage-plugins)
- [ROS 2公式ドキュメント：デフォルトのrosbag2のファイル形式の変更について](https://docs.ros.org/en/humble/Releases/Release-Iron-Irwini.html#change-default-bag-file-type-to-mcap)
