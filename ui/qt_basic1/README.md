# 実習ROS 2 Qtを使う1

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|
|Qt|Qt5.15.3|

## 概要

このページでは、Qtを利用したGUI表示の設定に関して説明する。また、Qtのプログラムを作成し、ROSのビルドツールでQtのプログラムをビルドする。ビルドした実行ファイルを起動し、GUIが表示できることを確認する。  
このページは、[ROS講座69 Qtを使う1(ビルドの設定)](https://qiita.com/srs/items/fbb7c44111e560250873)および[ROS講座70 Qtを使う2(Layout、SIGNAL・SLOT)](https://qiita.com/srs/items/4c0e0bfd92c341f7da7b)の一部の内容をROS 2対応させたものである。  

## Qtとは

QtはGUI開発用のフレームワークで、LinuxやWindows等の様々なプラットフォームに対応したGUIを開発できる。  
ROS上で動作するGUIの開発にもQtが利用されている。たとえば、ROSのGUIツールであるrqtは「ROS Qt」のことであり、Qtを使って作成されている。ROS上でGUIを動作させると、GUIからトピックやサービスなどのROS機能を利用できたり、ROSの情報を可視化が可能になるなど、様々な利点がある。

Qtを利用したプログラムを再配布する際は、ライセンスに注意が必要である。  
Qtは、商用ライセンスとオープンソースライセンスのいずれかに準拠し利用できる。オープンソースライセンスでは、主要なモジュールはLGPLでライセンスされているが、一部モジュールはGPLでのライセンスとなっている。（[Qt Licensing](https://www.qt.io/qt-licensing)を参照）

## 前準備

### 前提条件

このチュートリアルは、[実習ROS 2 Pub＆Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)を実行していることを前提に、Pub＆Sub通信のチュートリアルで作成したワークスペース`ros2_lecture_ws`を利用する。  

### ROS 2パッケージの作成

パッケージ`qt_basic1`を作成する。

```bash
cd ~/ros2_lecture_ws/src
ros2 pkg create --build-type ament_cmake qt_basic1
```

### Qtの環境構築

Qtを利用するため、以下のコマンドで`qtbase5-dev` `qt5-qmake`をインストールする。  
※ [Qt公式のインストール手順](https://wiki.qt.io/Install_Qt_5_on_Ubuntu)には、`qt5-default`をインストールして環境を構築するとの説明がある。しかし、`qt5-default`はUbuntu16.04までのサポートのため、Ubuntu22.04では上記のパッケージを利用する。

```bash
sudo apt install qtbase5-dev qt5-qmake
```

QtにはQtCreatorと呼ばれる統合開発環境がある。QtCreatorは以下のような機能を持ち、効率的にGUIの開発ができる。

- ソースコードの編集
- GUIのデザイン作成
- GUIの処理をローコードで設定
- ソースコードのビルド
- プログラムの実行やデバッグ

しかし、本講座では簡単なQtの開発に焦点を当て、QtCreatorは利用しない。

## QWidgetを利用したウィンドウの表示

Qtのプログラムを動作させて、ウィンドウを表示する方法を説明する。  
処理は実行せず、ウィンドウのみを表示する最小のソースコードを作成する。

### ウィジェット

「ウィジェット」は、QtにおけるUI要素の基本単位である。ウィンドウそのものやウィンドウ内に表示されるテキスト、ボタン、スライダー、エディタなどは全てウィジェットと呼ばれる。  
ウィジェットには親子関係を設定する。親を持たない最上位のウィジェットはウィンドウになり、ウィンドウ名や最小化、最大化等のボタンが表示される。また、子ウィジェットは親ウィジェットの範囲内に配置され、文字列やボタン、スライダー、エディタ等の要素が該当する。以下の図は、この記事内で作成するUIを例に、ウィジェットの親子関係を示したものである。[参考：Qt日本語ブログ-Qt をはじめよう！ 第8回: QWidget の親子関係を学ぼう](https://www.qt.io/ja-jp/blog/2010/05/21/qwidget)

![ウィジェットの親子関係の例](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/ui/qt_basic1/./img/widget_parent_child.png)

ウィジェットは`QWidget`クラスもしくは、`QWidget`のサブクラスである。Qtを使ったソースコードではウィジェット単位でインスタンスを作成し、ウィジェットの親子関係や見た目、応答処理などの設定を記述する。  

簡単のため、以下では親ウィジェットを「ウィンドウ」、子ウィジェットを「オブジェクト」と記載する。

### 利用するQtの要素

ウィンドウの表示には、QApplicationとQWidgetという2つのクラスを利用する。  

1. QApplication  
  QApplicationは、Qtの設定や実行処理を管理するクラスである。  
  ウィンドウ数に関わらず、1つのQtアプリに対して1つのQApplicationクラスのインスタンスを作成する。このクラスの`exec()`関数を実行することで、ウィンドウの表示や応答処理といった各種の処理が行われる。  
  より詳細な説明は[Qt公式ドキュメント：QApplication Class](https://doc.qt.io/qt-5/qapplication.html)に記載されている。
2. QWidget  
  ウィンドウのGUI設定や処理の設定を行う基本になるクラスである。  
  ウィンドウやウィンドウ内に配置するオブジェクト等のGUI設定、ユーザーからの入力に対する応答処理の設定などを行う。  
  今回作成するウィンドウはオブジェクトや処理を持たないため、ウィンドウの表示のための設定のみ行う。関数`show()`を実行することで、QApplicationクラスの`exec()`を実行したときにウィンドウが表示される。  
  より詳細な説明は[Qt公式ドキュメント：QWidget Class](https://doc.qt.io/qt-5/qwidget.html)に記載されている。

### ソースファイルの作成

ディレクトリ`qt_basic1/src`内に、QApplicationとQWidgetを使ってウィンドウを表示するプログラム`qwidget.cpp`を作成する。  

```cpp
#include <QApplication>
#include <QWidget>

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  QWidget * window1 = new QWidget;
  window1->show();
  return app.exec();
}
```

[qwidget.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic1/./src/qwidget.cpp)

作成した`qwidget.cpp`の処理を説明する。  
インクルード処理では、QApplicationクラスおよびQWidgetクラスを利用するために両者をインクルードする。

>```cpp
>#include <QApplication>
>#include <QWidget>
>```

QApplicationクラスおよびQWidgetクラスのインスタンスを作成する。  
ここで、`window1`は`new`を利用してヒープ領域に作成している。Qtでは基本的に`new`を使用してウィンドウやオブジェクトを作成し、ヒープ領域にメモリを確保する。[参考：Qt日本語ブログ Qt をはじめよう！ 第12回: シグナルとスロットを作成しよう](https://www.qt.io/ja-jp/blog/2010/07/20/create-signals-and-slots)

> ```cpp
> QApplication app(argc, argv);
> QWidget * window1 = new QWidget;
> ```

作成したQWidgetクラスのインスタンスの表示を設定し、Qtの処理を実行する。

> ```cpp
> window1->show();
> return app.exec();
> ```

なお、ROS 2のノードとして動作するためには、以下のような処理を実装する必要がある。（[実習ROS 2 Pub＆Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)を参照）  

- `rclcpp::Node`クラスを継承したクラスの作成
- `rclcpp::init()`によるノードの初期化
- `rclcpp::spin()`や`rclcpp::spin_once()`によるノードの実行

このページで作成するプログラムでは上記の処理は実装しないため、ROS 2のビルドツールを使用してビルドするが、ROSノードとしては動作しない。  

### ビルドの設定

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic1/./CMakeLists.txt)および[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic1/./package.xml)にビルド設定を記述する。追加する行をdiff形式で以下に示す。

1. CMakeLists.txt  
qwidget.cppの`Qt5Core`および`Qt5Widgets`への依存を解決して、実行ファイルを作成する。また、ビルド結果を`qt_basic1/install/`に作成してROS 2コマンドで実行できるように設定している。

    ```diff
     # find dependencies
     find_package(ament_cmake REQUIRED)
    + # packages for Qt
    + find_package(Qt5Core REQUIRED)
    + find_package(Qt5Widgets REQUIRED)

     # uncomment the following section in order to fill in
     # further dependencies manually.
     # find_package(<dependency> REQUIRED)


    + add_executable(qwidget src/qwidget.cpp)

    + ament_target_dependencies(qwidget Qt5Core Qt5Widgets)

    + install(
    +   TARGETS
    +   qwidget
    +   DESTINATION lib/${PROJECT_NAME}
    +   )

     if(BUILD_TESTING)
       find_package(ament_lint_auto REQUIRED)
    ```

2. package.xml  
`qtbase5-dev`への依存を記述する。
  
    ```diff
     <buildtool_depend>ament_cmake</buildtool_depend>

    + <build_depend>qtbase5-dev</build_depend>

     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>
    ```

### GUIの起動

作成したパッケージをビルドし、`ros2 run`コマンドで`qwidget`を実行する。

```bash
cd ~/ros2_lecture_ws/
colcon build
. install/setup.bash 
ros2 run qt_basic1 qwidget
```

`qwidget`を起動すると、以下のように何も表示されないウィンドウが1つ表示される。

![qwidgetの実行結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/ui/qt_basic1/./img/result_qwidget.png)

## オブジェクトの表示

`qwidget`で表示したウィンドウには何も表示されていない。実際にGUIを利用するためには、ウィンドウにユーザーとの対話に必要なオブジェクトを配置する必要がある。  
ウィンドウ内にオブジェクトを配置する方法について説明する。オブジェクトはQtの機能の1つであるレイアウトを利用して配置する。

### オブジェクトの使い方

前述のとおり、GUIに使用するオブジェクトは`QWidget`クラスのサブクラスとして用意されている。例えば以下のようなオブジェクトがよく利用される。

- プッシュボタン：[QPushButtonクラス](https://doc.qt.io/qt-5/qpushbutton.html)
- 指定した文字列を表示：[QLabelクラス](https://doc.qt.io/qt-5/qlabel.html)
- 1行のテキストエディタ：[QLineEditクラス](https://doc.qt.io/qt-5/qlineedit.html)

オブジェクトを利用するには、オブジェクトに対応するクラスのインスタンスを作成し、レイアウトを利用してウィンドウに配置する。

### レイアウトの使い方

レイアウトはオブジェクトの配置やサイズなどを管理するクラスである。一般にはオブジェクトを配置する際は、大きさや座標といった様々なパラメータをオブジェクトごとに指定しなければならない。それと比較して、レイアウトを使うことでより簡単にオブジェクトを配置できる。  
レイアウトにはいくつかの種類がある。種類によってオブジェクトの配置や位置の指定方法が異なる。例えば、オブジェクトを水平方向に配置する`QHBoxLayout`や、垂直方向に配置する`QVBoxLayout`、グリッドに配置を指定する`QGridLayout`がある。  
[参考：Qt Documentation - Layout Management](https://doc.qt.io/qt-5/layout.html)

### オブジェクトを表示するソースコード

`qt_basic1/src/`以下に、レイアウトでオブジェクトを配置したウィンドウを表示するプログラム`layout.cpp`を作成する。  
このプログラムを実行すると、以下の3つのウィンドウが表示される。

- プッシュボタンが水平方向に配置されたウィンドウ
- プッシュボタンがグリッド状に配置されたウィンドウ
- 文字列とエディタが垂直方向に配置されたウィンドウ

```cpp
#include <QApplication>

// include layout
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

// include object
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);

  QWidget * window1 = new QWidget;
  QPushButton * button1A = new QPushButton("Button 1A");
  QPushButton * button1B = new QPushButton("Button 1B");
  QPushButton * button1C = new QPushButton("Button 1C");
  QHBoxLayout * layout1 = new QHBoxLayout;
  layout1->addWidget(button1A);
  layout1->addWidget(button1B);
  layout1->addWidget(button1C);
  window1->setLayout(layout1);
  window1->show();

  QWidget * window2 = new QWidget;
  QPushButton * button2A = new QPushButton("Button 2A");
  QPushButton * button2B = new QPushButton("Button 2B");
  QPushButton * button2C = new QPushButton("Button 2C");
  QGridLayout * layout2 = new QGridLayout;
  layout2->addWidget(button2A, 0, 0);
  layout2->addWidget(button2B, 0, 1);
  layout2->addWidget(button2C, 1, 0, 1, 2);
  window2->setLayout(layout2);
  window2->show();

  QWidget * window3 = new QWidget;
  QLabel * label = new QLabel("###Label###");
  QLineEdit * edit = new QLineEdit("###LineEdit###");
  QVBoxLayout * layout3 = new QVBoxLayout;
  layout3->addWidget(label);
  layout3->addWidget(edit);
  window3->setLayout(layout3);
  window3->show();

  return app.exec();
}
```

[layout.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic1/./src/layout.cpp)

1. オブジェクトの作成  
  以下では、ウィンドウ内に配置するオブジェクトのインスタンスを作成している。`qwidget.cpp`のQWidgetクラスと同様に、インスタンスは`new`を使用して作成する。

    > ```cpp
    > QPushButton * button1A = new QPushButton("Button 1A");
    > QPushButton * button1B = new QPushButton("Button 1B");
    > QPushButton * button1C = new QPushButton("Button 1C");
    > ```

    > ```cpp
    > QPushButton * button2A = new QPushButton("Button 2A");
    > QPushButton * button2B = new QPushButton("Button 2B");
    > QPushButton * button2C = new QPushButton("Button 2C");
    > ```

    > ```cpp
    > QLabel * label = new QLabel("###Label###");
    > QLineEdit * edit = new QLineEdit("###LineEdit###");
    > ```

2. オブジェクトの配置  
  以下ではレイアウトを作成、オブジェクトの設定を行うともに、レイアウトをウィンドウに適用している。  
  レイアウトの`addWidget()`関数でレイアウトにオブジェクトを設定する。レイアウトの種類によっては引数を与えてオブジェクトの配置を設定できる。また、作成したレイアウトはQWidgetの`setLayout()`関数でウィンドウに設定する。

    > ```cpp
    > QHBoxLayout * layout1  = new QHBoxLayout;
    > layout1->addWidget(button1A);
    > layout1->addWidget(button1B);
    > layout1->addWidget(button1C);
    > window1->setLayout(layout1);
    > ```

    > ```cpp
    > QGridLayout * layout2  = new QGridLayout;
    > layout2->addWidget(button2A,0,0);
    > layout2->addWidget(button2B,0,1);
    > layout2->addWidget(button2C,1,0,1,2);
    > window2->setLayout(layout2);
    > ```

    > ```cpp
    > QVBoxLayout * layout3  = new QVBoxLayout;
    > layout3->addWidget(label);
    > layout3->addWidget(edit);
    > window3->setLayout(layout3);
    > ```

3. ウィンドウの表示  
  最後に、これまでに作成したウィンドウを表示するように`show()`によって設定し、`exec()`でQtを実行する。

### ビルド設定

`qwidget`と同様にCMakeLists.txtにビルド設定を追記する。新たに依存するパッケージは無いため、package.xmlへの追記は不要である。

```diff
 add_executable(qwidget src/qwidget.cpp)
+ add_executable(layout src/layout.cpp)

 ament_target_dependencies(qwidget Qt5Core Qt5Widgets)
+ ament_target_dependencies(layout Qt5Core Qt5Widgets)

 install(
   TARGETS
   qwidget
+   layout
   DESTINATION lib/${PROJECT_NAME}
   )
```

### GUIの起動

作成したパッケージをビルドして、`ros2 run`コマンドで`layout`を実行する。

```bash
cd ~/ros2_lecture_ws/
colcon build
. install/setup.bash 
ros2 run qt_basic1 layout
```

`layout`を起動すると、以下のような3つのウィンドウが表示される。ウィジェットの親子関係が設定されて、レイアウトの設定どおりにオブジェクトが配置されていることが分かる。  

![layoutの実行結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/ui/qt_basic1/./img/result_layout.png)

なお、厳密にはウィンドウの子ウィジェットに各オブジェクトが設定されているわけではない。  
レイアウトとウィンドウの間にも親子関係があり、`setLayout()`関数ではレイアウトとウィンドウ間の親子関係を設定している（ウィンドウを親に、レイアウトを子に設定）。ウィンドウとオブジェクト間の親子関係は、レイアウトを介して間接的に設定されている。  
`QWidget`クラスには親子関係を直接設定する方法もある。しかし、単純なGUIではレイアウトを用いるほうが記述が容易である。また、実習ROS 2はROS 2の解説を目的としているため本講座では取り扱わない。

## メモリ管理について

ウィジェットが削除されるとき、子のウィジェットのメモリを自動的に解放する機能がある。したがって、オブジェクト等の作成時に確保したメモリを解放する必要はない。（[参考：Qt日本語ブログQtをはじめよう！第7回:Qtのオブジェクトモデルを理解しよう](https://www.qt.io/ja-jp/blog/2010/05/05/object-3)）   
一方で、以下のようにメモリが解放されない場合もあるため、メモリリークの発生には注意が必要である。

- 親を持たないウィジェットを作成したとき
- 親子関係にあるウィジェットが相互に参照しあう循環参照の状態になったとき

前者は適切に`delete`を行うか適切な親子関係を設定する必要がある。また、後者は相互に参照しあうポインターの片方を[QWeakPointer](https://doc.qt.io/qt-5/qweakpointer.html)にすると循環参照を回避できる。

## 参考

- [ROS講座69 Qtを使う1(ビルドの設定)](https://qiita.com/srs/items/fbb7c44111e560250873)
- [ROS講座70 Qtを使う2(Layout、SIGNAL・SLOT)](https://qiita.com/srs/items/4c0e0bfd92c341f7da7b)
- [Qt日本語ブログ-Qtを始めよう!](https://www.qt.io/ja-jp/blog/tag/getting-started-with-qt)
