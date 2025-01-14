# 実習ROS 2 ROS 2インタフェース

## 環境

本記事は以下の環境を想定した記事である。

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

ROS 2のデータのやり取りはROS 2独自のデータ型で行われる。  
本記事では、ROS 2のデータ通信インタフェースの概要を示し、そのうちメッセージについて使用方法を解説する。  

## ROS 2インタフェースの分類

ROS 2では、Pub&Sub通信、Service通信、Action通信の3つの通信方式がある。  
これらに対応して、データ通信インタフェースには以下の3種類がある。

- メッセージ：Pub&Sub通信で用いられ、拡張子`.msg`のファイルで定義される。基本となるインタフェースである。
- サービス：Service通信で用いられ、拡張子`.srv`のファイルで定義される。RequestとResponseという2つのメッセージから構成される。
- アクション：Action通信で用いられ、拡張子`.action`のファイルで定義される。Goal, Result, Feedbackという3つのメッセージから構成される。

## インタフェースの表記方法

全てのROS 2インタフェースはパッケージの中で定義されるため、ROS 2のインタフェースは以下のような階層構造で定義する。  
型名は大文字始まりで記述する。  

|     | ROS 2コマンド | C++ | Python |
| --- | ----- | --- | ------ |
| Pub&Sub通信 | `パッケージ名/msg/型名` | `パッケージ名::msg::型名` | `パッケージ名.msg.型名` |
| Service通信 | `パッケージ名/srv/型名` | `パッケージ名::srv::型名` | `パッケージ名.srv.型名` |
| Action通信  | `パッケージ名/action/型名` | `パッケージ名::action::型名` | `パッケージ名.action.型名` |

例えば、std_msgsというパッケージの中のInt32というインタフェースをC++のソースコード内で使用する場合、以下のように書く。

```
std_msgs::msg::Int32
```

なお、ROS 1ではこのインタフェースは以下のように表記される。  
ROS 2ではパッケージ名の後にメッセージやサービスなどの区別を書くが、ROS 1では区別しないため注意する。

```
std_msgs::Int32
```

なお、std_msgsパッケージには以下のリンク先のメッセージが定義されている。  
[std_msgsの型一覧](https://github.com/ros2/common_interfaces/tree/humble/std_msgs/msg)

## メッセージの定義

### 基本

メッセージは、ROS 2のビルトイン型（ROS 2がサポートしている基本型）もしくは別のパッケージで定義されたメッセージを含むものである。  
ROS 2のビルトイン型と、それらに対応するC++、Pythonのデータ型を以下に示す。

|型|C++|Python|
|-|-|-|
|byte|unit8_t|builtins.bytes*|
|float32|float|builtins.float*|
|float64|double|builtins.float*|
|int8|int8_t|builtins.int*|
|uint8|uint8_t|builtins.int*|
|int16|int16_t|builtins.int*|
|uint16|uint16_t|builtins.int*|
|int32|int32_t|builtins.int*|
|uint32|uint32_t|builtins.int*|
|int64|int64_t|builtins.int*|
|uint64|uint64_t|builtins.int*|
|char|char|builtins.str*|
|string|std::string|builtins.str|
|wstring|std::u16string|builtins.str|
|bool|bool|builtins.bool|

上記の表は、[ROS 2公式ドキュメント インターフェイス：フィールド型](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html#field-types)を参照して作成した。

例えば、`std_msgs`パッケージの`ColorRGBA`というメッセージは、以下のように定義されている。  
これは`ColorRGBA`が、C++において`float`で扱われる型で、`r`、`g`, `b`、`a`という要素を含んでいることを示す。

```msg
float32 r
float32 g
float32 b
float32 a
```

このメッセージの定義は、[std_msgs/msg/ColorRGBA.msg](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/ColorRGBA.msg)ファイルで見ることができる。

別のパッケージで定義されたメッセージを含む例として、`geometry_msgs`パッケージで定義された`Pose`というメッセージが挙げられる。  
このメッセージは3次元空間中のポーズを示すメッセージであり、以下に示すように、点を示す`Point`と回転を示す`Quaternion`というメッセージからなる。  
これらのメッセージも`geometry_msgs`パッケージで定義されている。

```msg
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
```

このメッセージの定義は、[geometry_msgs/msg/Pose.msg](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Pose.msg)ファイルで見ることができる。

### 配列

ROS 2のメッセージでは、以下の種類の配列を定義できる。

|項番|型|C++|Python|
|-|-|-|-|
|1|固定長配列|std::array<T, N>|builtins.list*|
|2|可変長配列（要素数の上限なし）|std::vector|builtins.list*|
|3|可変長配列（要素数の上限あり）|custom_class<T, N>|builtins.list*|
|4|文字列（文字数の上限あり）|std::string|builtins.str*|

表中の`custom_class<T, N>`は、ROS 2でこの型を実現するために独自の実装が行われていることを示す。

以下に、メッセージ定義の具体例を表の項番に対応させて示す。  

1. `int32`型の固定長配列（要素数=5）

  ```msg
  int32[5] data_name
  ```

2. `int32`型の可変長配列（要素数の上限なし）

  ```msg
  int32[] data_name
  ```

3. `int32`型の可変長配列（要素数の上限=5）

  ```msg
  int32[<=5] data_name
  ```

4. 文字列（文字数の上限=10）

  ```msg
  string<=10 data_name
  ```

### 文字列

ROS 2のビルドイン型で扱える文字列は、UTF-8であることが期待されている。  
そのため、日本語などのマルチバイト文字も人間が考える文字数で扱える。  
（[参考： ROS 2 Design Unicode Support](https://design.ros2.org/articles/wide_strings.html)）

たとえば、以下のように3文字を上限とするメッセージに、"あああ"といった3文字のデータを格納できる。

```msg
string<=3 data_name
```

また、文字列配列では以下のようにして複雑なメッセージを定義できる。

- 文字列配列（要素数の上限なし、文字列1要素の最大文字数=10）

  ```msg
  string<=10[] data_name
  ```

- 文字列配列（要素数の上限=5、文字列1要素の最大文字数=10）

  ```msg
  string<=10[<=5] data_name
  ```


### 定数

以下のようにしてメッセージに定数を定義できる。  
定数の要素名は全て大文字で定義する。

以下では、`CONSTANT_DATA`という要素名を定義し、その値を12345としている。

```msg
int32 CONSTANT_DATA=12345
```

文字列の場合は、以下のように定義する。

```msg
string CONSTANT_DATA="foo"
```

なお、定数で表現する型はROS 2のビルドイン型である必要であり、以下のように定数配列を定義することはできない。

```msg
# 注意：無効なメッセージ定義
int32[] CONSTANT_ARRAY=[1, 2, 3, 4, 5]
```

これは、ROS 2でメッセージ定義ファイルを解析するスクリプトの[実装仕様](https://github.com/ros2/rosidl/blob/humble/rosidl_adapter/rosidl_adapter/parser.py#L316)による。

## インタフェースの調べ方

`ros2 interface show "インタフェース名"`でそのインタフェースの構造を表示できる。  

例として、`std_msgs/msg/String`の情報を表示させた結果を以下に示す。  
結果から、`data`という要素を1つ持つメッセージであることがわかる。  
またコメントからは、より意味のある具体的なメッセージを定義することが推奨されていることがわかる。

```shell
$ ros2 interface show std_msgs/msg/String
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data
```

より実践的な型の例として、`sensor_msgs/msg/LaserScan`のインタフェース定義を示した結果を以下に示す。  
この型はロボティクスでよく使われる2D LiDAR（Light Detection and Ranging）というセンサ情報を格納するために使われるものである。

大きく分けて以下の3種類の情報が含まれており、汎用的に使える型であることがわかる。

- メッセージのタイムスタンプ等のメタデータの定義
- 最大/最小の角度や最大測定距離等のセンサの特性を示す値
- 最後に`float32`型の配列として距離とレーザ強度の計測値が格納されている。

```shell
$ ros2 interface show sensor_msgs/msg/LaserScan
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
```

## ソースコード上の使用例

ソースコード上でメッセージを利用してPub&Sub通信を行う実装は、[Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)の記事で解説している。  
この記事では、`std_msgs/msg/String`のメッセージをやりとりするノードを実装している。

## 備考

上記はすでに用意されているメッセージを使用しているが、自作した独自のメッセージを使用することもできる。  
独自のメッセージの作成方法は、[カスタムROSメッセージ](https://qiita.com/s-kubota/items/eeaa1914055415d5792b)の記事に記載している。

## 参考

- [ROS講座09 ROSメッセージ](https://qiita.com/srs/items/080f1ca2ec2b2c480d41)
- [ROS 2インターフェイスについて](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
- [ROS 2 Design: Unicode Support](https://design.ros2.org/articles/wide_strings.html)
- [std_msgs/msg/ColorRGBA.msgの定義](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/ColorRGBA.msg)
- [geometry_msgs/msg/Pose.msgの定義](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Pose.msg)
