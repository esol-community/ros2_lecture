# 実習ROS 2 CMakeLists.txtとpackage.xmlの書き方

## 環境

本記事は以下の環境を想定して記述している。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

ROS 2では、パッケージ情報を記述するファイルとして`package.xml`を、C++プログラムのビルド情報を記述するためのファイルとして`CMakeLists.txt`を用いる。  
この記事では、これら2つのファイルの役割と記述方法を説明する。具体的には、以下の事柄を扱う。  

- 背景
- package.xmlの書き方
  - package.xmlの最小構成
  - 依存関係の記述
  - その他の記述
  - action_tutorials_cppパッケージの例
- CMakeLists.txtの書き方
  - CMakeについて
  - CMakeの拡張：ament_cmake
  - CMakeLists.txtの最小構成
  - 実行ファイルを生成する最小構成
  - minimal_clientパッケージの例
- 参考

## 背景

ROS 2では、ソースコードや様々な設定ファイル等を「パッケージ」と呼ばれる単位で管理する。  
パッケージは再利用可能な形で作成されるのが望ましく、アプリケーションは複数のパッケージを組み合わせて実現される。これを実現するためには、パッケージ間の依存関係（パッケージが正しくビルドされ、動作するために必要となる他のパッケージの情報）を正しく記述する事が重要になる。  

また、ROS 2はUbuntu、Windows、macOSでサポートされており、プログラミング言語は主にC++とPythonが使われる。したがって、パッケージの情報は異なるプログラミング言語間で、また、異なる開発環境間で統一して使用できなくてはならない。  

このような背景からROS 2パッケージで必要とされるファイルが`CMakeLists.txt`と`package.xml`である。

## package.xmlの書き方

`package.xml`には、以下の情報を記述する役割がある。このファイルはパッケージのトップディレクトリに必ず存在しなくてはならない。

- ROSシステムとしてのパッケージ情報  
  依存関係、ビルドに必要な情報等
- パッケージのメタデータ  
  パッケージのバージョン情報、パッケージの説明、パッケージのメンテナー、ソフトウェアライセンス等

`package.xml`の記述方法には複数のバージョンがあるが、本記事では2022年9月時点で最新のバージョン3の記述方法について説明する。  
なお、`ros2 pkg create`コマンドで自動生成される`package.xml`のバージョンも3である。  

記述方法の仕様は[REP149](https://ros.org/reps/rep-0149.html)で決められている(REP: ROS Enhancement Proposal)ため、詳細な仕様についてはこのページを参照すると良い。  

バージョン2とバージョン3の違いは以下の3点であり、大きな変更はない。そのため、バージョン2で記述されたOSSのROS 2パッケージも存在する。
基本的に新しいプロジェクトを開始する場合は、最新の機能とサポートを利用できるバージョン3を推奨する。
既存のプロジェクトでバージョン2を利用している場合は互換性を考慮しバージョン2を選択することもある。  

- 依存関係のグループ化の機能の追加
- ABI(Application Binary Interface)のバージョン指定機能の追加
- 同一パッケージでROS 1とROS 2の両方に対応するための機能の追加

### package.xmlの最小構成

必ず記述しなければならない情報のみを書いた最低限の`package.xml`の例を以下に示す。

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>

<package format="3">
  <name>minimum_package</name>
  <version>0.0.1</version>
  <description>The minimum package</description>
  <maintainer email="maintainer@example.com">Maintainer Name</maintainer>
  <license>2-Clause BSD License</license>
</package>
```

はじめの2行では、このファイルがROS 2の`package.xml`バージョン3のファイルであることを宣言している。 
それ以降のタグの意味は次の通り。 

- `<package>`  
  トップレベルのタグ。以降のタグはこのタグの中に記述する。  
  `format`属性でバージョンを指定する。
- `<name>`  
  このパッケージの名称。後述する`CMakeLists.txt`のプロジェクト名と同一の名称にする。  
  パッケージ名はディレクトリの名前ではなく、このタグによって定義される。
- `<version>`  
  このパッケージのバージョン。`MAJOR.MINOR.PATCH`の順で数値で記載する。  
- `<description>`  
  このパッケージの簡単な説明。
- `<maintainer>`  
  このパッケージをメンテンナンスしている人のメールアドレスと名前。
- `<license>`  
  ソフトウェアライセンス。少なくとも1つ以上指定する必要がある。    
  必須ではないが、`file`属性を用いてパッケージ中のライセンス文へのパスを指定することができる。

### 依存関係の記述

ROSパッケージやシステムパッケージの依存を記述するタグは様々なものがあるが、必要なもののみを記述すればよい。  
よく用いられるタグは以下である。  

- `<buildtool_depend>`  
  ビルドの過程で使用するツールとして必要なものを記述する。  
  C++プログラムでは`ament_cmake`が、Pythonプログラムでは`ament_python`がよく用いられる。  
- `<build_depend>`  
  ビルドに必要なパッケージを記述する。
- `<exec_depend>`  
  実行時に必要なパッケージを記述する。
- `<test_depend>`  
  テスト時に必要なパッケージを記述する。
- `<depend>`  
  このタグで依存を書くと、`<build_depend>`、`<build_export_depend>`、`<exec_depend>`の3つ全てを書いたことになる。

また、以下のようなタグも記述することができる。

- `<build_export_depend>`  
  このパッケージをエクスポート(※)し、それを利用して別のパッケージをビルドするときに必要になるパッケージを記述する。  
- `<buildtool_export_depend>`  
  ビルドツールとして他のパッケージにエクスポート(※)する必要があるパッケージを記述する。 
- `<doc_depend>`  
  ドキュメントをビルドするときに必要なパッケージを記述する。
- `<conflict>`  
  このパッケージと競合する（同時にインストールできない）ものを記述する。
- `<replace>`  
  このパッケージを置き換えることができるものを記述する。
- `<group_depend>`  
  フォーマット バージョン3で指定可能。  
  いくつかのパッケージをまとめてグループとして扱ったとき、このパッケージが依存するグループを指定する。  
- `<member_of_group>`  
  フォーマット バージョン3で指定可能。  
  このパッケージが指定したグループに属することを示す。  
(※)エクスポートとは、 パッケージやデータ等を外部のパッケージで使えるように公開することを指す。  

### その他の記述

その他の様々な情報を記述するためのタグとして、`<export>`  がある。  
このタグの中に入れ子にして追加の情報を記述する。  
よく使われるものとして、C++のプログラムを実装するパッケージに対して指定する`<build_type>`  がある。  

### action_tutorials_cppパッケージの例

具体的な例として、ros2/demosリポジトリにある[action_tutorials_cppパッケージのpackage.xml](https://github.com/ros2/demos/blob/humble/action_tutorials/action_tutorials_cpp/package.xml)を以下に示す。

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>action_tutorials_cpp</name>
  <version>0.9.4</version>
  <description>C++ action tutorial cpp code</description>
  <maintainer email="mabel@openrobotics.org">Mabel Zhang</maintainer>
  <maintainer email="michael.jeronimo@openrobotics.org">Michael Jeronimo</maintainer>
  <license>Apache License 2.0</license>
  <author email="jacob@openrobotics.org">Jacob Perron</author>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>action_tutorials_interfaces</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>rclcpp_components</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## CMakeLists.txtの書き方

### CMakeについて

`CMakeLists.txt`(この名前は固定されている)は、[CMake](https://cmake.org)で用いる設定ファイルの名前である。
CMakeはROSとは独立して存在する、クロスプラットフォーム対応のオープンソースソフトウェアで、ソフトウェアのビルド、テスト、パッケージングためのツールである。  
なお、CMakeは幅広い言語に対応しており多くのプロジェクトで使用される。また、CMakeではクロスコンパイルが可能であり、ビルド環境と異なる実行環境でシステムを動かすことが可能となる。

CMakeの必要性は、具体的な例を挙げるとわかりやすい。  
作成したC++のプログラムをビルドする際、環境によってコンパイルに必要なオプションや設定ファイルが全く異なる。例として、Linux環境でgccを用いる場合と、Windows環境でVisual C++を用いる場合では必要な設定が異なってくる。CMakeを用いると、プラットフォームに依存しないCMake独自の構文でこれらの設定を定義し、同じCMakeの記述から各々の環境に対応したビルド設定ファイルを出力することができる。  

CMakeの構文については、[オンラインドキュメント](https://cmake.org/cmake/help/latest/)に説明されている。

### CMakeの拡張：ament_cmake

ROS 2ではCMakeをベースとして拡張したビルドシステム`ament_cmake`を使用する。`ament_cmake`は基本的にはCMakeの構文と同じ記述ができるが、ROS 2のために拡張されたマクロが定義されている。  
`CMakeLists.txt`はパッケージのトップディレクトリに配置する。  

`ament_cmake`における`CMakeLists.txt`の役割はCMakeと同様である。ROS 1ではPythonプログラムのパッケージにも`CMakeLists.txt`が必要であったが、ROS 2では不要になった。  

以降では、`ament_cmake`を用いる際の`CMakeLists.txt`について説明する。

### CMakeLists.txtの最小構成

最小構成の`CMakeLists.txt`を以下に示す。  

```cmake
cmake_minimum_required(VERSION 3.5)
project(minimum_package)

ament_package()
```

最初の2行はCMakeの構文、最後の`ament_package()`は`ament_cmake`の構文である。  

- `cmake_minimum_required()`  
  このファイルで要求するCMakeの最低バージョンを指定する。
- `project()`  
  プロジェクト名を指定する。
  ROSパッケージでは必ずパッケージ名と同一にしなくてはならない。
- `ament_package()`  
  この行までに書いてきた情報をamentシステムに登録する。
  CMakeプロジェクトに対して最後に一度だけ呼び出すようにする。

### 実行ファイルを生成する最小構成

実行ファイルを生成する場合の最小構成を以下に示す。  
以下の例では、`src/main.cpp`から`main`という実行ファイルを作成している。  

**注意**：以下の例は最小構成のため、`ros2 run`コマンドで実行することは出来ない（ワークスペースにインストールしていないため）。  

```cmake
cmake_minimum_required(VERSION 3.5)
project(minimum_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(main
  src/main.cpp
)

ament_target_dependencies(main
  rclcpp
)

ament_package()
```

新たに登場した構文について以下に示す。

- `find_package()`  
  このプロジェクトが利用するほかパッケージ（ここでのパッケージは、ROSパッケージ以外のライブラリ等も含む）を探す。
- `add_executable()`  
  実行可能ファイル`main`を宣言し、それに必要なソースファイルを与える。  
  このようなビルド対象のことをCMakeでは「ターゲット」と呼ぶ。  
- `ament_target_dependencies()`  
  `ament_cmake`独自の構文。  
  ターゲットが依存する（リンクさせる）ROS 2パッケージを追加する。  
  ROSではないライブラリをリンクさせたいときは、純粋なCMakeの構文である`target_link_libraries()`を用いる。

### minimal_clientパッケージの例

具体的な例として、ros2/demosリポジトリにある[minimal_clientパッケージのCMakeLists.txt](https://github.com/ros2/examples/blob/humble/rclcpp/services/minimal_client/CMakeLists.txt)を以下に示す。  

```cmake
cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_client)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(client_main main.cpp)
ament_target_dependencies(client_main rclcpp example_interfaces)

install(TARGETS client_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

最小構成の説明に含まれていない構文について、以下、抜き出して説明する。  

```cmake
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
```

C++14を使うための設定である。  
CMakeにおける変数`CMAKE_CXX_STANDARD`が設定されていないとき、この変数に`14`を設定している。  

```cmake
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

コンパイラがGNU C++コンパイラか、Clang系のコンパイラのときに、コンパイルオプションを設定する。  
`add_compile_options()`では、Warningに対する3つのオプションを指定している。  

```cmake
install(TARGETS client_main
  DESTINATION lib/${PROJECT_NAME})
```

ビルドしたターゲット`client_main`のインストール先を指定している。  
ビルドを実行したワークスペース下の`install/[パッケージ名]/lib/[パッケージ名]`下にターゲットが配置される。  
これにより、`ros2 run`等のコマンドでノードを実行できるようになる。  

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

テスト時の設定である。  
`package.xml`に記載されたリンタ（静的解析ツール）を実行する。  

## 参考

- [ROS講座97 CMakeList.txtとpackage.xmlの書き方](https://qiita.com/srs/items/30c81c3f26f1987b0afa#msg%E3%82%92%E4%BD%BF%E3%81%86%E5%81%B4)
- [ROS公式HP：REP149](https://ros.org/reps/rep-0149.html)
- [ROS 2公式ドキュメント：ament_cmake user documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
- [ROS 2公式ドキュメント：About the build system](https://docs.ros.org/en/humble/Concepts/About-Build-System.html)
- [ROS Japanユーザグループ講習会：ROS 2のワークスペース:colconとパッケージ](https://gbiggs.github.io/rosjp_ros2_intro/workspaces_and_colcon.html)
- [ROS Japanユーザグループ講習会：ROS 2のAPIの使い方](https://gbiggs.github.io/rosjp_ros2_intro/ros2_basics.html)
