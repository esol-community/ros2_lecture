# 実習ROS 2 コンセプト

## 概要

ロボットソフトウェアを開発するには、非常に多くの分野の知識を必要とする。  
ROS(Robot Operating System)は開発者がロボットアプリケーションを効率良く構築し、ロボットソフトウェア開発を全世界で推進していくためのオープンソースプラットフォームである。  

現在ROSは、はじめに開発されたROS（ROS 2と対比してROS 1とも呼ばれる）と、次世代バージョンのROS 2がある。  
ROS 1とROS 2はアーキテクチャが大きく異なるが、共通するコンセプトも存在する。

そこで、この記事では、ROSの導入として以下の内容を扱う。

- ROS(ROS 1, ROS 2共通)のコンセプト
- ROS 1に対するROS 2のコンセプト
- 開発者視点でのROS 2の利点

## ROS(Robot Operating System)のコンセプト

ROSは、Operating Systemの名が入っているが、WindowsやLinux等のOSと同カテゴリのものではなく、それらの上で動作するSDK（Software Development Kit）に近い。  

ROSを説明するのに使われる図式として、以下の式がある([参考：The ROS Ecosystem](https://www.ros.org/blog/ecosystem/))。

> ROS = Plumbing + Tools + Capabilities + Community

- **Plumbing(通信)**  
  ソフトウェアをノードと呼ばれる単位に分割して、それらの間の通信の仕組みを提供する。  
  これによりプログラムの再利用性を高めることができる。  
  **ROSの要素の中でも中核にあるものである。**
- **Tools(ツール群)**  
  ロボットアプリケーションの開発に必要な、可視化、ロギング、データの保存と再生のツールを提供する。  
  これによりデバッグが容易になり、開発効率を高めることができる。
- **Capabilities(機能群)**  
  ロボットアプリケーションに必要になる個々の機能、例えば、センサドライバや自律移動、マニピュレーション、マッピングなどの機能をライブラリとして提供する。
- **Community(エコシステム)**  
  ROSはオープンソースのプロジェクトであり、様々な立場の人や団体が貢献することで、開発が進められている。  
  オンライン上で活動しているWebサイトとしては、[ROS Answers](https://answers.ros.org/questions/)や[ROS Discourse](https://discourse.ros.org/)が挙げられる。  
  またオフラインでは、[ROSCon](https://roscon.ros.org/)や[ROSConJP](https://roscon.jp/)といった開発者会議が開催されている。

ROSのコンセプトについては書籍やインターネット上でも多くの情報が入手できる。  
[本ページの末尾](#rosの全体像に関する参考資料)に参考資料を挙げている。

## ROS 2のコンセプト

ROS 1が広く利用されるにしたがって、当初ROS 1が想定していなかった以下のようなユースケースが出てきた。  

- 複数ロボットへの対応
- 計算資源に制約のある環境（組み込み）での活用
- 不安定なネットワーク環境での通信
- 製品化を想定した機能（セキュリティ、安全性など）
- リアルタイム性

ROSの中核にあるのはPublish/Subscribe型のメッセージ通信であるが、ROS 1ではこれらを独自実装の通信ミドルウェアで実現していた。独自実装の通信システムのままこれらのユースケースに対応するのは難しい面があった。

そこで、ROS 1が対応できないユースケースを想定してROSを新しく構築しなおしたものがROS 2である。ROS 2の通信ミドルウェアはDDS(Data Distribution Service)規格に準拠したものであり、基本的にROS 1と互換性がない。  

ROS 1とROS 2の技術的な違いは多々存在する（[参考：Changes between ROS 1 and ROS 2](https://design.ros2.org/articles/changes)）が、ROSユーザとして開発するときに意識するべき基本的な違いとして以下が挙げられる。

- **マルチプラットフォームのサポート**  
  ROS 1はUbuntuのみに対応しているが、ROS 2はWindowsやmacOSも想定して開発されている。（[参考：REP2000](https://www.ros.org/reps/rep-2000.html)）
- **言語標準の変更**  
  ROS 1はC++03, Python2系をサポートしているが、ROS 2はC++14以降、Python3系をサポートしている。
- **マスタノード(roscore)の廃止**  
  DDSを採用したことで、ノード（ROSプログラムの実行単位）間の検出等を担うマスタノードが不要になった。これにより、中央集権型ではない分散型の通信システムが実現できる。

## 開発者視点でのROSのメリット

上記以外で、開発者視点でROS(ROS 1, ROS 2共通)を採用するメリットとして以下が挙げられる。

- **インストールが容易**  
  公式ドキュメントに具体的なインストール手順が公開されており、開発初心者であっても容易にインストールできる。
- **ライブラリも一緒にバージョン管理されている**  
  ROSのディストリビューションに応じてROS対応のライブラリも管理されているので、個別にバージョンを気にする必要がない。
- **ビルドシステムも一緒に提供されている**  
  ROSパッケージや外部ライブラリの依存関係を解決し、ビルドするビルドシステムもROSと一緒に提供されている。
- **ユーザが多く、情報が手に入りやすい**  
  コミュニティベース、オープンソースで開発が進められており、ユーザも増え続けているため、インターネット上で多くの情報が手に入る。（[参考：ROSユーザ数推移](https://metrics.ros.org/index.html)）

## 参考

- [ROS講座01 コンセプト](https://qiita.com/srs/items/d7c7202236d2abe562c8)
- [ROS公式サイト：The ROS Ecosystem](https://www.ros.org/blog/ecosystem/)
- [ROS 2 Design: Why ROS 2?](https://design.ros2.org/articles/why_ros2.html)
- [ROS 2 Design: Changes between ROS 1 and ROS 2](https://design.ros2.org/articles/changes)

### ROSの全体像に関する参考資料

- [（スライド）次世代ロボットフレームワークROS2の紹介](https://swest.toppers.jp/SWEST20/program/pdfs/s2a_public.pdf)  
- [（論文）Robot Operating System 2: Design, architecture, and uses in the wild](https://www.science.org/stoken/author-tokens/ST-483/full)  
- [（書籍）ROSロボットプログラミングバイブル](https://www.ohmsha.co.jp/book/9784274221965/)  
- [（書籍）ROS2ではじめよう 次世代ロボットプログラミング](https://gihyo.jp/book/2019/978-4-297-10742-0)  
