# 実習ROS 2 ROS 2パッケージのディレクトリ構成

## 環境

本記事は以下の環境を想定して記述している。  

|項目|値|
|---|---|
|OS|Ubuntu 22.04|
|ROS|ROS 2 Humble|

## 概要

ROS 2のパッケージには様々なファイル、ディレクトリがある。  
この記事では、ROS 2ワークスペースの全体像と、パッケージに必要なファイル、慣例的に使われるディレクトリ名などを整理する。  
また、ROS 1との違いについてもまとめる。

## ROS 2のワークスペース構成

ROS 2のワークスペース構成について説明する。  
初めに全体像を以下図に示す。
localとはローカルPCの任意の場所、ws1・ws2はそれぞれワークスペースを表す。  
hello_packageとworld_packageはそれぞれパッケージを表す。各ファイル、ディレクトリについての詳細は以降で説明する。  

```sh
local/
├── ws1           # ワークスペース1
    ├── build     # ビルドで生成される中間ファイル等が置かれる場所
    ├── install   # ビルド生成物がインストールされる場所
    ├── log       # ビルド時のログが置かれる場所
    ├── src       # 開発者がパッケージを実装する場所
        ├── hello_world               # パッケージでないただのディレクトリ。無くても動作に問題はない。
            ├── hello_package         # helloパッケージ
                ├── CMakeLists.txt    # ビルド設定ファイル
                ├── package.xml       # パッケージ情報ファイル
                ├── hello.launch      # launchファイル
                ├── hello.cpp         # ソースコード
                └── ...               # その他関連ファイル
            └── world_package         # worldパッケージ
                └── ...               # worldパッケージの関連ファイル
├── ws2           # ワークスペース2
    ├── ...       # 関連するファイル群
├── ...           # ワークスペース3以降
```

ROS 2では、ROS 1同様、関連する全てのファイルはワークスペース内に保存される。そのため、開発者はまずワークスペースを作成する必要がある。ワークスペースはローカルPCの任意の場所に作成でき、用途に応じて複数作成することも可能である。さらに、複数のワークスペースを組み合わせることができるため、柔軟な開発が可能になる。  
また自分で作成したワークスペースだけでなく、ROS 2パッケージをビルドしてできたワークスペースもある。  


## ROS 2のワークスペース内部構成

ワークスペースの内部構成を説明する。  

ワークスペースでビルドを行うと、ワークスペース直下に`build`、`install`、`log`ディレクトリが作られる。  
開発者はワークスペース直下に`src`ディレクトリを作成し、その中でパッケージ（ソースコード等を含む）を作成して開発を行う。また`src`ディレクトリ内には、C++用のパッケージとPython用のパッケージを作成する事が出来る。そのため、1つのプロジェクト内で機能毎に異なるプログラミング言語のパッケージがあっても、1つのワークスペースで一括管理する事が出来る。  

パッケージを作成する際は、`ros2 pkg create`コマンドを使用する。C++用のパッケージとPython用のパッケージでは、自動生成されるファイルやパッケージ構成が異なる。

以下に、ワークスペースのディレクトリ構成例を示す。この例ではワークスペース`test_ws`に、C++パッケージ`sample_cpp_package`、Pythonパッケージ`sample_python_package`、`hello`ディレクトリを配置している。さらに、`hello`ディレクトリにはC++パッケージ`hello_package`を配置している。  

```sh
test_ws/
├── build       # ビルドで生成される中間ファイル等が置かれる場所
├── install     # ビルド生成物がインストールされる場所
├── log         # ビルド時のログが置かれる場所
└── src         # 開発者がパッケージを実装する場所
    ├── hello                       # パッケージではないただのディレクトリ
    │   └── hello_package           # C++パッケージhello_package
    │       └──...
    ├── sample_cpp_package          # C++パッケージsample_cpp_package
    │   └── ...
    └── sample_python_package       # Pythonパッケージsample_python_package
        └── ...
```

## ROS 2パッケージ内のディレクトリ構成

多くのディレクトリ名は任意で、慣例的なものも多い。  
ここでは、使用頻度が高いと思われるものを説明する。

### 必須ファイル

- `package.xml`  
  ROSパッケージであることを示すファイルであり、パッケージ作成時に自動生成される。  
  詳細は[こちらの記事](https://qiita.com/s-kitajima/items/3f4c7f2dd2d9e5e5a792)を参照

### C++パッケージでよく使うもの

いずれのファイル、ディレクトリも、パッケージ作成時に自動生成される。

- `CMakeLists.txt`  
  ビルドの設定を記述するファイルである。  
  詳細は[こちらの記事](https://qiita.com/s-kitajima/items/3f4c7f2dd2d9e5e5a792)を参照
- `src/`  
  ソースコードを置く場所である。  
  直下にソースコードを置いても、さらに深いディレクトリに置いてもよい。  
  いずれにせよ、CMakeLists.txtでソースコードの場所を正しく記述する必要がある。
- `include/`  
  ヘッダファイルを置く場所である。  
  他のパッケージからインクルードした際にわかりやすくなるよう、この下にパッケージ名のディレクトリを作ってヘッダファイルを置くことが多い。  

### Pythonパッケージでよく使うもの

いずれのファイル、ディレクトリも、パッケージ作成時に自動生成される。  

- `setup.py`  
  ROSとは独立した、Pythonが持つパッケージングのために使われるファイルである。  
  適切に記述することで、ROS 2パッケージを純粋なPythonパッケージとして扱うことができる。  
  記述する情報は、`package.xml`に似たものが多い。  
  なお、`setup.py`の代わりに`setup.cfg`や`pyproject.toml`といったファイルを用いてパッケージ情報を記述する場合がある。([参考](https://setuptools.pypa.io/en/latest/setuptools.html))
- `<package_name>/`配下  
  ここに開発者が実装するPythonスクリプトを置く。  
  自動生成される`__init__.py`は、このディレクトリがPythonのパッケージとして扱われることを示すためのファイルである。

### 起動・初期設定

- `launch/`  
  ROSシステムの起動構成を記述するlaunchファイルを置く場所である。  
  (参考：[実習ROS 2 ROS 2 Launch 1：概要](https://qiita.com/s-kitajima/items/3b17d1c4a248299cc026))  
  (参考：[実習ROS 2 ROS 2 Launch 2：応用](https://qiita.com/s-kitajima/items/ef113900656aa2ba4f59))

### カスタムメッセージ定義

独自のメッセージ型を定義してそれを利用する場合は、再利用性を高めるためにメッセージを定義するパッケージと利用するパッケージを分けることが多い。  
メッセージを定義しているパッケージの具体例として、[ROS 2公式リポジトリのexample_interfacesパッケージ](https://github.com/ros2/example_interfaces/tree/humble)がある。  
(参考：[実習ROS 2 カスタムROSメッセージ](https://qiita.com/s-kubota/items/eeaa1914055415d5792b))

- `msg/`  
  トピック通信で用いるメッセージ定義ファイルを置く場所である。
- `srv/`  
  サービス通信で用いるサービス定義ファイルを置く場所である。
- `action/`  
  アクション通信で用いるアクション定義ファイルを置く場所である。

### モデル定義

ロボットモデルの定義のために使うファイルを置く場所や、シミュレーションのために用いるファイルを置く場所などがある。  

- `urdf/`  
  ロボットモデルを定義するurdfファイルを置く場所である。  
  (参考：[実習ROS 2 URDFを記述する1](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6))
- `xacro/`  
  マクロを用いてロボットモデルを記述するxacroファイルを置く場所である。
- `worlds/`  
  Gazebo上でシミュレーションを行う世界を定義するworldファイルを置く場所である。
- `models/`  
  Gazebo上でのモデルを表現するsdf等のモデルファイルを置く場所である。

### 設定・スクリプト

ROSで用いる様々なパラメータやスクリプトファイルを置く場所がある。  

- `config/`  
  パラメータを記述するyamlファイル等を置く場所である。
- `rviz/`  
  Rvizの表示設定ファイルを置く場所である。
- `scripts/`  
  シェルスクリプト等のスクリプトファイルを置く場所である。  

## ROS 1との違い

ROS 1とROS 2でディレクトリ構成で大きく異なるのは以下の2点である。  
いずれも、ビルドシステム、ビルドツールがROS 1とROS 2で異なることによる。

- ワークスペースのトップにあるディレクトリが異なる。
  - ROS 1の場合
  
  ```sh
  ros1_workspace    # ワークスペース名
   ├──build
   ├──devel
   └──src
  ```

  - ROS 2の場合

  ```sh
  ros2_workspace    # ワークスペース名
   ├──build
   ├──install
   ├──log
   └──src
  ```

- Pythonパッケージで必要なファイルが異なる。具体的な説明は上記の説明を参照。

## 参考

- [ROS講座64 ROSパッケージのディレクトリ構成](https://qiita.com/srs/items/7a8d7708d1c2d6d2ba78)
- [ROS 2公式ドキュメント：Creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [ROS 2公式ドキュメント：Creating a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [setuptools公式ドキュメント：Building and Distributing Packages with Setuptools](https://setuptools.pypa.io/en/latest/setuptools.html)