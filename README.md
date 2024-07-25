# 実習ROS 2

ROS 2のハンズオン教材「実習ROS 2」の目次のページです。  
テーマごとに解説ページへのリンクを記載しています。  
リンクのないページについては、今後順次公開予定です。

## 導入

- [コンセプト](https://qiita.com/s-kitajima/items/09a9e3885bf83b1f8c05)  
  ROS（ROS 1, ROS 2共通）と、ROS 2のコンセプトについて説明します。

- インストール  
  ROS 2のインストール方法について説明します。

### 入門

#### 基礎編

- [Pub&Sub通信](https://qiita.com/s-kitajima/items/5a4d7f06413120010e6b)  
  ROSのデータ通信の基本となるPublish/Subscribe通信を行うノードをC++で作成、実行します。

- [Service通信](https://qiita.com/s-kitajima/items/29607ff86a656e4d6099)  
  クライアント/サーバ型の通信方法であるService通信を行うノードをC++で作成、実行します。

- [ROS 2 Launch 1：概要](https://qiita.com/s-kitajima/items/3b17d1c4a248299cc026)  
  複数のノードをまとめて起動する方法であるlaunchというシステムについて説明し、簡単な実行例を示します。

- [ROS 2 Launch 2：応用](https://qiita.com/s-kitajima/items/ef113900656aa2ba4f59)  
  launchシステムで使用できる応用機能の一部を説明します。

- [ROS 2 tools](https://qiita.com/s-kubota/items/e9d69b44d6659d44e95c)  
  デバッグ等に使用できるCLI上でのコマンド、GUIツールについてまとめています。

- [ROS 2 Logger](https://qiita.com/s-kitajima/items/42bf046f4a3de95147d8)  
  ROS 2で情報をターミナルに出力するログシステムについて説明します。

- [ROS 2インタフェース](https://qiita.com/s-kubota/items/b17bce1c5f2cfe8a93b9)  
  ROS 2のデータ通信インタフェースについて説明し、使用方法を示します。

- [カスタムROSメッセージ](https://qiita.com/s-kubota/items/eeaa1914055415d5792b)  
  自作のROS 2メッセージ型を定義し、使用する方法について説明します。

#### 可視化編

- [urdfを記述する1](https://qiita.com/esol-h-matsumoto/items/13e5f278244fd6b576c6)  
  ロボットを記述するフォーマットであるURDFの概念と、基本的な書式を説明します。

- URDFを記述する2  
  URDFで様々な色や形のlink、可動jointを記述します。

- xacroを使う1  
  URDFを効率的に記述する方法であるxacroを利用して、定数、数式・条件式を記述します。

- xacroを使う2  
  xacroのマクロ機能のうち、モジュール、インクルード、変数を利用したxacroを記述します。

- tf2 broadcast, listen  
  tf2の概要とbroadcast, listenの方法を説明します。

- joint_stateをpublishする  
  joint_stateをROS 2ノードからpublishし、Rviz上のロボットを動かす方法を解説します。

### 上級

#### 発展編

- [Action通信](https://qiita.com/s-kitajima/items/6f9f544010d49fdbe1d5)  
  クライアント/サーバ型で、実行中の情報をフィードバックして受け取れる通信方式であるActionを解説、実装します。

- ros2 paramを使う1  
  ROS 2ノードに与えるパラメータの概念、CLIツール、基本的な与え方について説明します。

- ros2 paramを使う2  
  ROS 2ノードで扱えるパラメータの型の種類や、YAMLファイルからパラメータを与える方法について説明します。

- rosbag2を使う  
  ROS 2でトピックを保存、再生する手段であるrosbag2について解説します。

#### ROS 2追加要素編

- Componentを使う  
  ノードを共有ライブラリとしてビルド、実行するComponentという概念を説明し、これを用いたノードを実装します。

- Lifecycleノード  
  ノードの状態遷移を表現できるLifecycleという仕組みを説明し、これを用いたノードを実装します。

- QoSを設定する  
  ROS 2の通信品質を決定しているQoSという概念と、それらを設定する方法を説明します。

#### 開発環境編

- CMakeLists.txtとpackage.xmlの書き方  
  2つのファイルの役割や、書式について説明します。

- ROS 2パッケージのディレクトリ構成  
  ROS 2のワークスペース及びパッケージのディレクトリ構成について説明します。

#### UI編

- Qtを使う1  
  QtでGUIを作成し、作成したGUIをROS上で動かして画面に表示する方法を説明します。

- Qtを使う2  
  QtでGUIの応答処理を実装する方法を説明します。

- QtでPub&Subする  
  QtとROS 2を組み合わせて、GUIからトピックを扱う方法を説明します。
