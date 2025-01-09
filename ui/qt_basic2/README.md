# 実習ROS 2 Qtを使う2

## 環境

本記事は以下の環境を想定して記述している。  
|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|
|Qt|Qt5.15.3|

## 概要

[実習ROS 2 Qtを使う1](https://qiita.com/esol-h-matsumoto/items/db11f5ddf78fac7f5eed)では、Qtを利用したGUIの作成方法とROSのビルドツールを使ったビルド方法を説明した。しかし、配置したオブジェクトに対して必要な処理を実装していないため、ボタンを操作したりテキストエディタを編集しても何の変化も起きなかった。この記事では、「シグナル」と「スロット」と呼ばれる機能を使ってオブジェクトの処理を実装する方法を説明する。  
このページは、[ROS講座70 Qtを使う2(Layout、SIGNAL・SLOT)](https://qiita.com/srs/items/4c0e0bfd92c341f7da7b)の一部と[ROS講座71 Qtを使う3(classの作成)](https://qiita.com/srs/items/97b06f7b44a9d24718a2)の内容をROS 2対応させたものである。  

## 前準備

### 前提条件

このチュートリアルの実施前に、次の環境を準備すること。

1. [実習ROS 2 Pub＆Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)のチュートリアルを実施し、ワークスペース`ros2_lecture_ws`を作成する。
2. [実習ROS 2 Qtを使う1 Qtの環境構築](https://qiita.com/esol-h-matsumoto/items/db11f5ddf78fac7f5eed#qt%E3%81%AE%E7%92%B0%E5%A2%83%E6%A7%8B%E7%AF%89)を実施する。

### ROS 2パッケージの作成

パッケージ`qt_basic2`を作成する。

```bash
cd ~/ros2_lecture_ws/src
ros2 pkg create --build-type ament_cmake qt_basic2
```

## シグナル・スロット

Qtではシグナルやスロットと呼ばれる機能を使って応答処理を実装する。シグナル・スロットの基本的な使い方について説明する。  

### シグナル・スロットとは

1. シグナル  
  ウィジェットへの入力や内部状態の変化といったイベントが発生したときに、そのイベントを別のウィジェットに発信する仕組みである。シグナルの例を以下に示す。

     - プッシュボタンが押下される：ウィジェット`QPushButton`のシグナル`clicked`
     - エディタの文字が編集される：ウィジェット`QLineEdit`のシグナル`textChanged`
     - スライダーの値が変更される：ウィジェット`QSlider`のシグナル`valueChanged`

2. スロット  
  シグナル発生時の応答処理を記述したC++の関数であり、シグナルに対して紐づけて利用する。GUIのイベントが発生してシグナルが送られたとき、そのシグナルに紐づけられたスロットの処理が実行される。POSIXのシグナルハンドラに相当するものである。

各ウィジェットの持つシグナルやスロットは、[Qt Documentation - Qt Widgets C++ Classes](https://doc.qt.io/qt-5/qtwidgets-module.html)から該当のウィジェットを選択して、"Public Slots"や"Signals"の項目から確認できる。

### シグナル・スロットを接続する

シグナルとスロットを紐づけ、シグナル発生時のスロットを設定することを「接続する」と呼ぶ。シグナルとスロットを接続することで、オブジェクトの操作などのイベント発生時の応答処理が実装できる。  
シグナルとスロットの接続は`connect()`関数を利用する。`connect()`関数の記載形式を以下に示す。ここではオブジェクト1の持つシグナルと、オブジェクト2の持つスロットを接続する場合を例に記している。

```txt
connect(*オブジェクト1*, *シグナル名*, *オブジェクト2*, *スロット名*)
```

なお、一つのシグナルを複数のスロットに、あるいは複数のシグナルを一つのスロットに接続することもできる。

![シグナルとスロットのイメージ](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/ui/qt_basic2/./img/signal_slot_img.png)  
[Qt Documentation - Signals & Slots](https://doc.qt.io/qt-6/signalsandslots.html#signals-and-slots)

### シグナルとスロットの引数

シグナルおよびスロットはそれぞれ引数を持つ。引数を利用するとシグナルとスロットの間でデータの受け渡しができる。接続するスロットとシグナルは、以下の2つの条件をともに満たす必要がある。

- 引数のデータ型が同じ
- 引数の数が同じか、シグナルよりスロットの引数が少ない

シグナルよりスロットの引数が少ない場合、シグナルの余分な引数は無視される。また、引数の条件を満たさないスロットとシグナルを接続した場合は、以下のようなエラーが出てビルドに失敗する。

```bash
error: static assertion failed: Signal and slot arguments are not compatible.
```

## シグナル・スロットを利用したGUIを作成

シグナルとスロットの仕組みを利用したGUIを作成する。  
GUIには1行のテキストエディタ、文字列を表示するラベルとプッシュボタンを配置する。シグナルとスロットの仕組みを利用して、以下の2つの応答処理を実装する。

- テキストエディタの文字を編集 → 編集した文字をラベルに表示
- プッシュボタン押下 → GUIを閉じる。

### ソースファイルの作成

ディレクトリ`qt_basic2/src/`内に、QApplicationとQWidgetを使ってウィンドウを表示するプログラム`signal_slot.cpp`を作成する。

```cpp
#include <QApplication>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  QWidget* window = new QWidget;  
  QVBoxLayout* layout  = new QVBoxLayout;

  QLineEdit*   edit    = new QLineEdit("");
  QLabel*      label   = new QLabel("");
  QPushButton* button  = new QPushButton("Quit");

  layout->addWidget(edit);
  layout->addWidget(label);
  layout->addWidget(button);
  
  window->setLayout(layout);

  QObject::connect(edit, &QLineEdit::textChanged, label, &QLabel::setText);
  QObject::connect(button, &QPushButton::clicked, &app, &QApplication::quit);

  window->show();

  return app.exec();
}
```

[signal_slot.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic2/./src/signal_slot.cpp)

`signal_slot.cpp`におけるシグナルとスロットの接続設定を説明する。  

以下の行ではテキストエディタの編集を通知するシグナル[textChanged](https://doc.qt.io/qt-5/qlineedit.html#textChanged)と、ラベルの表示テキストを設定するスロット[setText](https://doc.qt.io/qt-5/qlabel.html#text-prop)[^1]を接続している。それぞれQString型(Qtで利用される文字列型)の引数を持っているため、テキストエディタに入力した文字列をラベルが受け取って表示することができる。

> ```cpp
> QObject::connect(edit, &QLineEdit::textChanged, label, &QLabel::setText);
>```

以下の行ではプッシュボタンの押下を通知するシグナル[clicked](https://doc.qt.io/qt-5/qabstractbutton.html#click)[^2]と、ウィンドウを閉じてQtの処理を停止するQApplicationクラスのスロット[quit](https://doc.qt.io/qt-5/qcoreapplication.html#quit)[^2]を接続している。  
このようにシグナルとスロットを利用することで、ウィンドウなどオブジェクト以外のウィジェットとも通信ができる。

> ```cpp
> QObject::connect(button, &QPushButton::clicked, &app, &QApplication::quit);
>```

[^1]:QLabelのメンバ関数`text()`と同じ実装になっているので、スロット`setText`の説明は`text()`にリンクされる。
[^2]: 継承したクラスの関数のため、別関数のページへリンクしている。それぞれ`clicked`はボタンの抽象化クラス`QAbstractButton`、`quit`はQtのメイン処理を実行するループを実装したクラス`QCoreApplication`の関数を利用している。

### ビルドの設定

[CMakeLists.txt](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic2/./CMakeLists.txt)および[package.xml](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic2/./package.xml)にビルド設定を記述する。追加する行を以下に示す。

1. CMakeLists.txt  
signal_slot.cppの`Qt5Core`および`Qt5Widgets`への依存を解決して、実行ファイルを作成する。また、ビルド結果を`qt_basic2/install/`に作成してROS 2コマンドで実行できるように設定している。

    ```diff
    find_package(ament_cmake REQUIRED)
    + # packages for Qt
    + find_package(Qt5Core REQUIRED)
    + find_package(Qt5Widgets REQUIRED)

    + add_executable(signal_slot src/signal_slot.cpp)

    + ament_target_dependencies(signal_slot Qt5Core Qt5Widgets)

    + install(
    +   TARGETS
    +   signal_slot
    +   DESTINATION lib/${PROJECT_NAME}
    +   )

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
    ```

2. package.xml  
`qtbase5-dev`への依存を記述する。
  
    ```diff
    + <build_depend>qtbase5-dev</build_depend>
    ```

### GUIの起動

作成したパッケージをビルドし、`ros2 run`コマンドで`signal_slot`を実行する。

```bash
cd ~/ros2_lecture_ws/
colcon build
. install/setup.bash 
ros2 run qt_basic2 signal_slot
```

`signal_slot`を起動すると以下のようなウィンドウが1つ表示される。1段目のエディタに入力した文字が2段目に表示される。また、"Quit"のボタンをクリックするとウィンドウが閉じてQtの処理を終了する。シグナルとスロットが接続され、GUIの応答処理が実装できたことが分かる。

![signal_slotの実行結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/ui/qt_basic2/./img/result_signal_slot.png)

## スロットの作成

シグナルやスロットは既定のものを利用することに加えて自作も可能である。自分でスロットを作成すると、任意の応答処理が利用できるためGUIの複雑な処理が実装できる。スロットを自作する方法に関して以下で説明する。  
なお、シグナルの作成については記事内で取り扱わない。スロットと似た要領で作成することが可能である。

### QDialogクラス

スロットやシグナルの作成など、より詳細なGUI設定を行う場合は`QDialog`クラスを利用する。  

1. QWidgetクラスとの関係  
  QWidgetクラスとQDialogクラスの間には親子関係が設定できる。  
  親となるQWidgetクラスへのポインタをQDialogクラスのコンストラクタに渡すと、QWidgetクラスとQDialogクラスの間の親子関係を設定できる。親子関係を設定した場合、子のQDialogクラスはウィンドウのオブジェクトとしてふるまう。一方で、親を持たないQDialogクラスはウィンドウになる。
2. QDialogクラスの継承  
  QDialogクラスでウィンドウの設定を行うには、QDialogクラスを継承してサブクラスを作成する。サブクラスで追加する関数でGUI表示の設定やシグナルとスロットの接続などのGUI設定を行うことで、親のQWidgetクラスのウィンドウを設定する。

### スロットの自作

スロットの自作は`QDialog`クラスのサブクラスで行う。  
スロットとして利用したい処理はクラスのメンバ関数として作成する。クラス宣言を以下のように記述すると、その関数はスロットとして利用できる。

```txt
class <クラス名> : <継承の設定>
{
  Q_OBJECT

private:
  ...

public slots:
  <スロットにしたい関数の宣言>;
  ...

protected slots:
  <スロットにしたい関数の宣言>;
  ...

private slots:
  <スロットにしたい関数の宣言>;
  ...
}
```

1. マクロ  
`Q_OBJECT`はウィジェットとして動作するために必要な処理を記載したマクロである。これを記載しなければ、`public slots`などのスロットの宣言が利用できない。
2. スロットのアクセス権  
スロットには`public`、`protected`、`private`といったアクセス権を設定する。（シグナルにはアクセス権の設定はない。）  
アクセス権の意味は通常のメンバ宣言と同様である。すなわち、`public slots`はクラス外から利用でき、`private slots`はクラス内でのみ利用できる。基本的には同じクラス内で利用する場合がほとんどなため、`private slots`で十分なことが多い。`public slots`や`protected slots`を利用するパターンとしては、「ウィンドウを二つ表示したうえで、一方のウィンドウのシグナルともう一方のウィンドウのスロットを接続する」といった例があげられる。

## 作成したスロットを使ったGUIの作成

スロットを作成するとともに、そのスロットをシグナルに接続して利用する。  
GUIには文字列を表示するラベル、1行のテキストエディタ、プッシュボタンを1つ配置する。表示されているボタンをクリックすると、テキストエディタに入力されている文字列がラベルに表示される。このとき、ボタン押下のシグナル`QPushButton::clicked`に自作したスロットを接続して処理を実現する。

## ソースコード

`qt_basic2/src/`以下に`my_slot.cpp`および`my_dialog.hpp`、`my_dialog.cpp`を作成する。`my_slot.cpp`にはメイン関数を、`my_dialog.hpp`と`my_dialog.cpp`には、QDialogのサブクラスを記載する。  

- [my_slot.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic2/./src/my_slot.cpp)

  ```cpp
  #include <QApplication>
  #include <QDialog>

  #include "my_dialog.hpp"

  int main(int argc, char ** argv)
  {
    QApplication app(argc, argv);
    QWidget * window = new QWidget;
    MainDialog * dialog = new MainDialog(window);
    dialog->show();
    return app.exec();
  }
  ```

- [my_dialog.hpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic2/./src/my_dialog.hpp)

  ```cpp
  #include <QDialog>
  #include <QLabel>
  #include <QPushButton>
  #include <QLineEdit>

  class MainDialog : public QDialog
  {
    Q_OBJECT

  public:
    MainDialog(QWidget * parent);

  private slots:
    void setLabelText();

  private:
    QLabel * label;
    QLineEdit * lineEdit;
    QPushButton * setButton;
  };
  ```

- [my_dialog.cpp](https://github.com/esol-community/ros2_lecture/tree/main/ui/qt_basic2/./src/my_dialog.cpp)

  ```cpp
  #include <QDialog>
  #include <QLabel>
  #include <QPushButton>
  #include <QLineEdit>
  #include <QVBoxLayout>

  #include "my_dialog.hpp"

  MainDialog::MainDialog(QWidget * parent)
  : QDialog(parent)
  {
    label = new QLabel("empty");
    setButton = new QPushButton("Set");
    lineEdit = new QLineEdit;

    connect(setButton, &QPushButton::clicked, this, &MainDialog::setLabelText);

    QVBoxLayout * layout = new QVBoxLayout;
    layout->addWidget(label);
    layout->addWidget(lineEdit);
    layout->addWidget(setButton);
    setLayout(layout);
  }

  void MainDialog::setLabelText()
  {
    QString text = lineEdit->text();
    label->setText(text);
  }
  ```

1. QDialogのサブクラスの作成  
  `QDialog`を継承したクラス`MainDialog`を作成している。  
  マクロ`Q_OBJECT`を記述するとともに、`private slots`を使って引数と返り値を持たない関数`setLabelText()`をスロットに設定している。また、コンストラクタは`QWidget`型へのポインタを受け取る。  
  (my_dialog.hpp)

    ```cpp
    ...
    class MainDialog : public QDialog
    {
      Q_OBJECT

    public:
      MainDialog(QWidget * parent);

    private slots:
      void setLabelText();
    ...
    }
    ```

2. 親子関係の設定  
  コンストラクタが受け取った`QWidget`型のポインタは、ベースクラス`QDialog`のコンストラクタへ渡される。これにより、`QWidget`のインスタンスが親、MainDialogクラスのインスタンスを子とした親子関係を設定する。  
  (my_dialog.cpp)

    ```cpp
    ...
    MainDialog::MainDialog(QWidget * parent)
    : QDialog(parent)
    {
      ...
    }
    ```

3. ウィジェットの設定  
  コンストラクタではレイアウトの設定や、シグナル・スロットの接続を行っている。
  文字列を表示するラベル`QLabel`とプッシュボタン`QPushButton`、１行のテキストエディタ`QLineEdit`のインスタンスを作成する。また、ボタン押下のシグナル`clicked`と自作したスロット`setLabelText`を接続している。最後に、それらのオブジェクトを`QVBoxLayout`および`setLayout()`を使って、それらのオブジェクトをウィンドウに配置している。  
  (my_dialog.cpp)

    ```txt
    ...
      label = new QLabel("empty");
      setButton = new QPushButton("Set");
      lineEdit = new QLineEdit;

      connect(setButton, &QPushButton::clicked, this, &MainDialog::setLabelText);

      QVBoxLayout * layout = new QVBoxLayout;
      layout->addWidget(label);
      layout->addWidget(lineEdit);
      layout->addWidget(setButton);
      setLayout(layout);
    }
    ...
    ```

    自作したスロット`setLabelText`では、テキストエディタに入力されたテキストをラベルにセットして表示する。  
    (my_dialog.cpp)
  
    ```cpp
    ...
    void MainDialog::setLabelText()
    {
      QString text = lineEdit->text();
      label->setText(text);
    }
    ```

4. ウィンドウの表示  
  最後にウィンドウを表示してQtの処理を実行する。QDialogを使うときはQDialogクラスの`show()`関数を利用する。  
  (my_slot.cpp)

    ```txt
    ...
      dialog->show();
      return app.exec();
    }
    ```

### ビルド設定

CMakeLists.txtにビルド設定を追記する。新たに作成したオブジェクトの実行ファイルを作成する設定を追加する。また、マクロ`Q_OBJECT`を利用するために`set(CMAKE_AUTOMOC ON)`を追記する。

```diff
+ set(CMAKE_AUTOMOC ON)

add_executable(signal_slot src/signal_slot.cpp)
+ add_executable(my_slot src/my_slot.cpp src/my_dialog.cpp)

ament_target_dependencies(signal_slot Qt5Core Qt5Widgets)
+ ament_target_dependencies(my_slot Qt5Core Qt5Widgets)

install(
  TARGETS
  signal_slot
+   my_slot
  DESTINATION lib/${PROJECT_NAME}
  )
```

新たに依存するパッケージは無いため、package.xmlへの追記は不要である。

### GUIの起動

作成したパッケージをビルドして、`ros2 run`コマンドで`my_slot`を実行する。

```bash
cd ~/ros2_lecture_ws/
colcon build
. install/setup.bash 
ros2 run qt_basic2 my_slot
```

`my_slot`を起動すると以下のウィンドウが表示される。2段目のエディタに文字を入力して3段目のボタンをクリックすると、入力した文字が1段目のラベルに表示される。作成したスロットがシグナルと接続され、意図通りに動作していることが分かる。  

![my_slotの実行結果](https://raw.githubusercontent.com/esol-community/ros2_lecture/main/ui/qt_basic2/./img/result_my_slot.png)

## 参考

- [ROS講座70 Qtを使う2(Layout、SIGNAL・SLOT)](https://qiita.com/srs/items/4c0e0bfd92c341f7da7b)
- [ROS講座71 Qtを使う3(classの作成)](https://qiita.com/srs/items/97b06f7b44a9d24718a2)
- [Qt日本語ブログ-Qtを始めよう!](https://www.qt.io/ja-jp/blog/tag/getting-started-with-qt)
