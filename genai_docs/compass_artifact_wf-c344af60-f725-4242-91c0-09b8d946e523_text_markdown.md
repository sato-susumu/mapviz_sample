# ROS2版mapvizの完全ガイド：2025年最新版

**ROS2版mapvizは成熟した実用レベルの2D可視化ツールであり、2025年9月時点でバージョン2.5.10がリリースされている。** Southwest Research Institute（SwRI）によって積極的にメンテナンスされており、Humble、Iron、Jazzy、Rolling、Kiltedなど全ての主要ROS2ディストリビューションに対応する。屋外ロボティクス、自動運転車、GPS可視化に特化した設計で、タイルマップ統合とプラグインベースのアーキテクチャを特徴とする。

本ツールの最大の強みは大規模な2D地図データの可視化性能にあり、RVizが3D可視化に特化しているのに対し、mapvizは2Dデータの高速表示と衛星画像統合に最適化されている。ROS1からの移植はほぼ完了しているが、一部のナビゲーション機能（move_baseプラグイン等）には制限が残る。それでも、GPSデータ可視化、センサーデータ表示、経路計画の視覚化において卓越した性能を発揮し、現在も月次ペースでアップデートが継続されている。

## 基本情報と開発状況

mapvizの中核的な役割は、ロボティクスアプリケーション向けの高度にカスタマイズ可能なモジュラー型2D可視化環境を提供することにある。RVizと同様のプラグインアーキテクチャを採用しているが、**2Dデータに特化**している点が決定的に異なる。屋外ロボティクス、自動運転車両、GPS/GNSS可視化、衛星画像統合、2Dセンサーデータ表示といったユースケースで真価を発揮する。

ROS2への移植作業は既に完了しており、2025年に入ってからも極めて活発な開発が続いている。**最新バージョンは2.5.10（2025年9月4日リリース）**で、2025年中に10以上のバージョンがリリースされた。9月のリリースではGLUTライブラリの参照エラーが修正され、8月には空白文字のトリミング動作が改善された。7月にはライブラリの誤命名修正と新APIへの再帰的ミューテックスの切り替えが行われ、6月にはOpenCVを依存関係に追加し、5月には**Boostライブラリとdeprecatedなamentマクロの削除を含む大規模なCMakeエクスポート修正**が実施された。

対応するROS2ディストリビューションは**Humble Hawksbill（LTS）、Iron Irwini、Jazzy Jalisco（最新LTS）、Kilted、Rolling Ridley**の全てで、バージョン2.5.10が同時に利用可能となっている。古いディストリビューションであるFoxy FitzroyとGalactic Geocheloneもサポートされているが、Foxyはサポート範囲が限定的である。注目すべきは、将来リリース予定のKiltedディストリビューションへの対応が既に完了している点で、これは長期的な開発コミットメントを示している。

公式GitHubリポジトリは**https://github.com/swri-robotics/mapviz**に位置し、ROS2開発は`ros2-devel`ブランチで行われている。リポジトリには325以上のスターと158のフォークがあり、活発なコミュニティの存在を示している。リポジトリは複数のパッケージから構成されており、メインの`mapviz`可視化アプリケーション、`mapviz_interfaces`インターフェース定義、`mapviz_plugins`プラグイン集、`multires_image`多解像度画像サポート、`tile_map`タイルベース地図統合（OpenStreetMapとGoogle Mapsサポート）が含まれる。

メンテナンスはSouthwest Research Institute（SWRI）のロボティクス部門が担当しており、オリジナル開発者はMarc AlbanとP. J. Reedである。2025年の主要コントリビューターにはScott K Logan、DangitBen（Ben Andrew）、David Anthony、David V. Lu!!、Robert Brothersが名を連ねる。コミュニティからのフィードバックによれば、**メンテナーの応答は非常に速く、GitHubイシューへの返信が10分以内**というケースも報告されている。

ライセンスは**BSD-3-Clause**で、商用利用、改変、配布、私的使用が認められる寛容なオープンソースライセンスである。これはROSエコシステムで広く使用されており、商用アプリケーションとの互換性も高い。公式ドキュメントサイト（https://swri-robotics.github.io/mapviz/）も維持されており、ROS Index（https://index.ros.org/p/mapviz/）では「推奨」使用レベルとして分類され、ステータスは「開発済み（fully functional, production-ready）」と記載されている。

## インストール方法と依存関係

mapvizのインストールには**バイナリパッケージインストール**と**ソースからのビルド**の2つの方法がある。通常のユーザーや本番環境ではバイナリインストールが推奨され、最新機能が必要な開発者や特定の問題を解決したい場合はソースビルドが適している。

### バイナリパッケージによるインストール

最も簡単な方法は、Ubuntu/Debianのaptパッケージマネージャーを使用することである。以下のコマンドで、mapviz本体と主要プラグインをすべてインストールできる：

```bash
sudo apt-get install ros-$ROS_DISTRO-mapviz \
    ros-$ROS_DISTRO-mapviz-plugins \
    ros-$ROS_DISTRO-tile-map \
    ros-$ROS_DISTRO-multires-image
```

`$ROS_DISTRO`は使用中のROS2ディストリビューション名に置き換える。`humble`はROS2 Humble Hawksbill（Ubuntu 22.04）、`jazzy`はROS2 Jazzy Jalisco、`rolling`はROS2 Rolling Ridleyを指す。Foxy Fitzroy（Ubuntu 20.04）の場合、パッケージの入手可能性が限定的であるため、ソースからのビルドが推奨される。

### ソースからのビルド手順

ソースからのビルドは、最新の修正や機能を利用したい場合に適している。まず、colconワークスペースをセットアップする：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

リポジトリのクローンには、**vcstoolを使用する方法**と**gitを直接使用する方法**がある。vcstoolを使用する場合は、まずreposファイルを作成する：

```bash
cat > mapviz.repos <<EOF
repositories:
  mapviz:
    type: git
    url: https://github.com/swri-robotics/mapviz.git
    version: ros2-devel
EOF

vcs import < mapviz.repos
```

gitを直接使用する場合は、以下のようにシンプルにクローンする：

```bash
git clone -b ros2-devel https://github.com/swri-robotics/mapviz.git
```

次に、rosdepを使用して依存関係をインストールする。これにより、必要なすべてのパッケージが自動的に検出され、インストールされる：

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

ビルドにはcolconを使用する。標準的なビルドコマンドは以下の通りだが、**開発中は`--symlink-install`オプション**を付けることで、再ビルド時間を短縮できる：

```bash
colcon build --symlink-install
```

特定のmapvizパッケージのみをビルドする場合は：

```bash
colcon build --symlink-install --packages-select mapviz_interfaces mapviz mapviz_plugins tile_map multires_image
```

**Raspberry Piなどメモリ制約のあるシステム**では、並列ビルドを無効化する必要がある：

```bash
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential
```

ビルド完了後、ワークスペースをソースする：

```bash
source ~/ros2_ws/install/setup.bash
```

毎回ソースする手間を省くため、.bashrcに追加することが推奨される：

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 依存関係の詳細

mapvizは多数の依存関係を必要とする。**ビルドツール依存関係**として、`ament_cmake`（ROS2ビルドシステム）、`pkg-config`、`qt5-qmake`が必要である。

**システム依存関係**（Debian/Ubuntuパッケージ）には、Qt5関連ライブラリ（`libqt5-core`、`libqt5-opengl`、`libqt5-opengl-dev`）、OpenGL関連ライブラリ（`glut`/`freeglut3`、`libglew-dev`）、`libopencv-dev`（OpenCVコンピュータビジョンライブラリ）、X11ライブラリ（`libxi-dev`、`libxmu-dev`）、`yaml-cpp`（YAML解析）が含まれる。

**ROS2パッケージ依存関係**は広範囲に及ぶ。コア機能として`rclcpp`（ROS2 C++クライアントライブラリ）、`pluginlib`（動的プラグインローディング）、メッセージタイプとして`geometry_msgs`、`image_transport`、`mapviz_interfaces`、変換処理として`tf2`、`tf2_geometry_msgs`、`tf2_ros`、SWRI独自ユーティリティとして**`swri_math_util`と`swri_transform_util`**（変換処理に不可欠）、GUIフレームワークとして`rqt_gui_cpp`と`rqt_gui`が必要である。

ビルド専用依存関係（ランタイムでは不要）には、`libqt5-opengl-dev`、`ros_environment`、`pkg-config`、`qt5-qmake`がある。開発依存関係である`libglew-dev`、`libxi-dev`、`libxmu-dev`、`libopencv-dev`はコンパイル時に必要となる。

### インストール時の既知の問題と解決策

**Qtバージョン競合（主にMac OS）**: システムのQtインストールとconda/mambaのQtインストールの間で競合が発生し、「non-virtual member function marked 'override' hides virtual member function」というエラーが出ることがある。これはRoboStackインストールでの既知の問題であり、**本番環境ではUbuntu/Debianの使用が推奨**される。Mac上では、環境内で一貫したQtバージョンを確保し、システムQtとconda/mamba Qtを混在させないことが重要である。

**SWRI依存関係の欠落**: 特定のUbuntuバージョンやaptリポジトリにない場合、「No definition of [swri_yaml_util] for OS version」というエラーが発生する。解決策は、追加のSWRI依存関係を手動でクローンすることである：

```bash
cd ~/ros2_ws/src
git clone -b ros2-devel https://github.com/swri-robotics/marti_common.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

**tile_mapプラグインのネットワークエラー203**: タイルマッププロバイダーからのレート制限により「NETWORK ERROR: 203」が表示されることがある。解決策として、パンニング速度を落とす、オフラインキャッシング用のDocker版MapProxyを使用する、適切なレート制限を持つカスタムタイルソースを設定する、といった方法がある。一部のタイルソース（MapQuest等）は既に廃止されているため、利用可能なソースを選択する必要がある。

**変換エラー**: 「No transformer from 'wgs84' to 'map'」や「No transform between wgs84 and map」というエラーは、変換ブロードキャスターの欠落または誤ったフレーム設定が原因である。解決するには、**`initialize_origin.py`ノードが実行中であることを確認**し、静的変換パブリッシャーが必要な変換をブロードキャストしていることを検証し、GPSフィックスメッセージが発行されているかチェックする必要がある。

**プラグインロード失敗**: 最近の修正（Issue #836）で追加のエラーハンドリングが導入されたが、プラグイン依存関係の欠落や誤ったプラグインパスが原因でロードエラーが発生することがある。全てのプラグインパッケージ（`mapviz_plugins`、`tile_map`、`multires_image`）がビルドされていることを確認し、plugin.xmlファイルが正しくインストールされているか検証し、ビルド後にワークスペースをソースする必要がある。

**CMakeエクスポート問題**: Issue #842でCMakeエクスポートが修正され、deprecatedなamentマクロが削除された。この問題が発生した場合は、ros2-develブランチの最新バージョンを使用し、ワークスペース全体を再ビルドする：`colcon build --cmake-clean-cache`。

**GLUTライブラリ参照エラー**: Issue #862で誤ったGLUT_LIBRARY参照が修正された。2024年以降のコミットを含む最新バージョンに更新することで解決する。

**Raspberry Piでのメモリ問題**: ビルドプロセスがフリーズまたはクラッシュする場合、メモリ制約を考慮した並列ビルドの無効化が必要である：`MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential`。

### プラットフォーム互換性

公式にサポートされているのは、**Ubuntu 22.04（Jammy）**でROS2 Humble、Jazzy、Kilted、Rollingを使用する環境、および**Ubuntu 20.04（Focal）**でROS2 Foxyを使用する環境（限定的サポート）である。Debianも適切なROS2インストールがあれば動作するはずである。

テスト済みプラットフォームには、Ubuntu 22.04 + ROS2 Humble（主要ターゲット）、Ubuntu 20.04 + ROS2 Foxy（限定的）、Raspberry Pi 4 with Ubuntu 22.04（メモリ制約ビルドが必要）がある。**Ubuntu 18.04（Bionic）はROS2に公式サポートされず**、Ubuntu 17.10（Artful）はSWRIパッケージ定義が欠落している。Mac OSはRoboStackとのQt競合があり本番環境には推奨されず、Windowsは公式サポート外である。

重要な注意点として、新規のUbuntu 22.04にインストールする際は、ROS2パッケージをインストールする前に`systemd`と`udev`を更新することで、システム重要パッケージの削除問題を回避できる：

```bash
sudo apt update && sudo apt upgrade
# その後ROS2をインストール
```

インストール検証には、以下のコマンドを使用する：

```bash
ros2 pkg list | grep mapviz
```

これにより、`mapviz`、`mapviz_interfaces`、`mapviz_plugins`、`tile_map`、`multires_image`が表示されるはずである。簡単な起動テストは：

```bash
ros2 run mapviz mapviz
```

## 機能と特徴

mapvizの**主要機能**は、2D可視化ツールとしての専門性に集約される。屋外ロボット向けのトップダウン視点に特化しており、RVizが3D重視であるのとは対照的である。プラグインベースアーキテクチャは高い拡張性を提供し、RVizと同様のコンセプトだが2Dデータに最適化されている。

**タイルマップ統合**は特筆すべき機能で、OpenStreetMap、Bing Maps（APIキー必要）、Google Maps衛星ビュー（プロキシ経由）、Stamen Designマップ、カスタムWMTSソースなど、複数の外部マップソースをサポートする。多解像度画像サポートにより、カスタムマップや背景画像を表示でき、変換管理機能は座標フレーム処理のための組み込みトランスフォームマネージャーを提供する。

設定の永続化機能により、.mvcファイルを通じて表示設定の保存と読み込みが可能である。GUI機能には、インタラクティブな座標ピッキング、測定ツール、ポイントアンドクリック発行、カスタマイズ可能な表示順序（描画レイヤー）、イメージトランスポート選択GUIが含まれる。

### ROS1版との主な違い

アーキテクチャ面では、**通信レイヤーがカスタムTCP/UDPプロトコルからDDSミドルウェアに変更**された。ROS2のQoSポリシーが全面的にサポートされ、プラグインサブスクリプションでQoS設定が可能になった。ビルドシステムはcatkinからament_cmake/colconに移行し、起動システムはXML roslaunchからROS2 Pythonランチファイルに変更された。

**ROS2版で欠落または制限されている機能**が存在することは認識しておくべきである。最も重要なのは、**Move Baseプラグインが完全に移植されていない**点で、Issue #728によればナビゲーション機能（move_baseやplan_route）が欠落または制限されている。アクションサーバー統合については、ROS1の一部のインタラクティブナビゲーション機能が利用できない。コマンド速度パブリッシングについても、ナビゲーション用の直接的なcmd_velパブリッシングがROS2版ではまだ実装されていない。

一方、**ROS2版での改善**も多数ある。プラグインロードにおける改善されたエラーハンドリング（Issue #836）、テクスチャードマーカープラグインサブスクリプションの修正、トピックリスト表示の改善、全プラグインにわたるQoSハンドリングの更新、空白文字トリミング動作の強化などが実施された。

### 利用可能なプラグイン

mapvizは**30以上のプラグイン**を提供し、幅広いデータタイプの可視化に対応している。

**データ可視化プラグイン**として、Attitude Indicator（IMU、Odometry、TFメッセージからの姿勢可視化）、Coordinate Picker（クリックした座標を指定フレームで出力しクリップボードにコピー）、Disparity（sensor_msgs/DisparityImageを'jet'カラーマップで表示）、Float（std_msgs/Float32/64等を固定位置に表示）、GPS（gps_common/GPSFixメッセージデータ可視化）、Grid（2Dグリッドオーバーレイ表示）、Image（sensor_msgs/Imageをディスプレイにオーバーレイ）、LaserScan（sensor_msgs/LaserScanデータ可視化）、Marker（visualization_msgs/MarkerまたはMarkerArray表示、RVizマーカーと類似）、NavSat（sensor_msgs/NavSatFixデータ可視化、local_xy_origin初期化が必要）、Occupancy Grid（nav_msgs/OccupancyGridマップをカスタマイズ可能なカラースキームで表示）、Odometry（nav_msgs/Odometryメッセージ軌跡可視化）、Path（nav_msgs/Pathメッセージデータ表示）、PointCloud2（sensor_msgs/PointCloud2データ可視化）、Pose（geometry_msgs/Poseメッセージ表示）、Pose Array（geometry_msgs/PoseArrayメッセージ可視化）、String（std_msgs/Stringメッセージを固定位置に表示）、Textured Marker（marti_visualization_msgs/TexturedMarkerおよびTexturedMarkerArray表示、クアッド上の密な画像データ用）、TF Frame（TF変換を矢印またはポイントとして可視化）がある。

**インタラクティブプラグイン**には、Draw Polygon（カスタムポリゴンの描画とパブリッシュ）、Measuring（マップ上のポイント間距離測定）、Point Click Publisher（マップクリック時にgeometry_msgs/PointStampedをパブリッシュ）、Point Drawing（インタラクティブポイント描画ツール）が含まれる。

**背景/マッププラグイン**として、Tile Map（各種ソースからのタイルベースマップ背景表示）、Multi-Res Image（多解像度カスタムマップ画像表示）、Robot Image（カスタムロボットアイコン画像表示）が提供される。

**ナビゲーションプラグイン（ROS2では制限あり）**には、Move Base（move_baseアクションサーバーへのインターフェース、ROS2では制限付き）、Plan Route（ルート計画プラグイン、ROS2では制限付き）、Route（ルート情報表示、marti_nav_msgs）がある。

**カスタムMARTIプラグイン**として、Object（marti_nav_msgs/ObstaclesとTrackedObjects表示）、Marti Nav Path（marti_nav_msgs/Path表示）、Marti Nav Plan（marti_nav_msgs/Plan表示）が利用可能である。

### サポートされるデータタイプ

**センサーメッセージ（sensor_msgs）**：DisparityImage、Image、LaserScan、NavSatFix、PointCloud2

**ナビゲーションメッセージ（nav_msgs）**：OccupancyGrid、Odometry、Path

**ジオメトリメッセージ（geometry_msgs）**：Point、PointStamped、Pose、PoseArray、PoseWithCovarianceStamped

**可視化メッセージ（visualization_msgs）**：Marker、MarkerArray

**標準メッセージ（std_msgs）**：String、Float32、Float64

**GPSメッセージ**：gps_common/GPSFix（レガシーサポート）

**カスタムMARTIメッセージ（marti_*_msgs）**：marti_visualization_msgs（TexturedMarker、TexturedMarkerArray）、marti_common_msgs（Float32Stamped、Float64Stamped）、marti_sensor_msgs（Velocity）、marti_nav_msgs（Path、Plan、Obstacles、TrackedObjects）

さらに、**TF2変換のフルサポート**により、全ての座標フレーム変換が可能である。

## 使用方法と設定

mapvizの**基本的な起動方法**は非常にシンプルである。最もシンプルな起動コマンドは：

```bash
ros2 run mapviz mapviz
```

設定ファイルを指定して起動する場合は：

```bash
ros2 run mapviz mapviz --ros-args -p config:=/path/to/config.mvc
```

ランチファイルを使用する場合は：

```bash
ros2 launch mapviz mapviz.launch.py
```

**GPSベースの可視化には、ローカルXY原点の初期化が必要**である。これは別のノードで実行する：

```bash
ros2 run mapviz initialize_origin.py
```

### 設定ファイルの作成方法

設定ファイルは**.mvc（Mapviz Config）拡張子を持つYAML形式**である。設定ファイルには、ウィンドウ設定、座標フレーム設定、表示プラグインの構成が含まれる。

典型的な設定ファイル構造は以下の通りである：

```yaml
capture_directory: "~"
fixed_frame: map
target_frame: <none>
fix_orientation: false
rotate_90: true
enable_antialiasing: true
show_displays: true
show_status_bar: true
show_capture_tools: true
window_width: 1848
window_height: 1016
view_scale: 0.09229598
offset_x: 0
offset_y: 0
use_latest_transforms: true
background: "#a0a0a4"
image_transport: raw

displays:
  - type: mapviz_plugins/tile_map
    name: Map
    config:
      visible: true
      collapsed: true
      custom_sources:
        - base_url: http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
          max_zoom: 19
          name: GMaps
          type: wmts
        - base_url: https://tile.openstreetmap.org/{level}/{x}/{y}.png
          max_zoom: 19
          name: OSM
          type: wmts
      bing_api_key: ""
      source: GMaps
      
  - type: mapviz_plugins/navsat
    name: GPS Location
    config:
      visible: true
      collapsed: true
      topic: /gps/fix
      color: "#fce94f"
      draw_style: points
      position_tolerance: 0.5
      buffer_size: 0
      show_covariance: true
      show_all_covariances: false
```

**重要な設定注意点**として、**プラグインの順序が重要**である。プラグインはリストされた順序で描画されるため、背景マップを最初に、データオーバーレイを最後に配置すべきである。フレーム設定では、fixed_frameは通常'map'であり、target_frameは特定のTFフレームに従うように設定できる。カスタムWMTSソースでは、カスタムURLを持つ複数のタイルマップソースを追加可能である。

### 一般的な使用例

**例1：GPS原点を使用したカスタムランチファイル**

実用的なランチファイルの例を以下に示す：

```python
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('your_package_name')
    
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'mapviz_config',
            default_value=PathJoinSubstitution([current_pkg, 'mapviz', 'mapviz.mvc']),
            description='Full path to the Mapviz config file to use',
        ),
        
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            output={'both': 'log'},
            parameters=[{
                'config': LaunchConfiguration('mapviz_config'),
                'autosave': False
            }],
        ),
        
        Node(
            package='mapviz',
            executable='initialize_origin.py',
            name='initialize_origin',
            parameters=[
                {'local_xy_frame': 'map'},
                {'local_xy_navsatfix_topic': 'gps/fix/origin'},
                {'local_xy_origin': 'auto'},
                {'local_xy_origins': """[
                    {'name': 'pitt', 
                     'latitude': 40.438889608527084, 
                     'longitude': -79.95833630855975, 
                     'altitude': 273.1324935602024, 
                     'heading': 0.0}
                ]"""},
            ],
        ),
    ])
```

このランチファイルは、mapviz本体と原点初期化ノードを同時に起動し、GPSベースの可視化に必要な設定を自動的に適用する。

**例2：robot_localization統合のためのGPS datumスクリプト**

robot_localizationパッケージと統合する場合、最初の有効なGPSフィックスを再パブリッシュし、datumを設定するスクリプトが有用である：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from robot_localization.srv import SetDatum
from sensor_msgs.msg import NavSatFix, NavSatStatus

class GpsDatum(Node):
    """最初の有効なGPSフィックスを再パブリッシュし、robot_localizationでdatumを設定"""
    
    def __init__(self):
        super().__init__('gps_datum')
        self.gps_datm_msg_ = None
        
        self.sub_gps_ = self.create_subscription(
            NavSatFix, 'gps/fix', self.sub_gps_cb, qos_profile_sensor_data
        )
        
        self.pub_gps_datum_ = self.create_publisher(
            NavSatFix, 'gps/fix/origin',
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.rl_datum_client = self.create_client(SetDatum, 'datum')
        
    def sub_gps_cb(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        self.gps_datm_msg_ = msg
        self.get_logger().info('Successfully set origin')
        
    def timer_callback(self):
        if self.gps_datm_msg_ is not None:
            self.pub_gps_datum_.publish(self.gps_datm_msg_)
```

**例3：Google Maps衛星画像統合**

Google Maps衛星画像を使用するには、MapProxyのDocker版を使用する：

```bash
mkdir ~/mapproxy
docker run -p 8080:8080 -v ~/mapproxy:/mapproxy -d -t --restart=always danielsnider/mapproxy

# Mapviz GUIでカスタムWMTSソースを追加：
# URL: http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
# Max Zoom: 19
```

重要なチュートリアルポイントとして、**GPS可視化のためにmapvizを起動する前に必ず原点を初期化**すること、プラグインの描画順序が重要（tile_mapを最初に、次にデータオーバーレイ）であること、変換フレームが適切にブロードキャストされている必要がある（wgs84 → map → base_link）ことを覚えておく必要がある。

## 既知のバグと制限事項

### 主要な制限事項

**ナビゲーションプラグインの問題（Issue #728）**が最も重要な制限である。`move_base`プラグインがROS2で完全に機能しておらず、`plan_route`プラグインも制限されているか移植されていない。ナビゲーション用の直接的なcmd_velパブリッシング機能がなく、ユーザーはROS1版と同様のナビゲーション機能を要求している。

**変換の問題**も一般的である。「No transformer from 'wgs84' to 'map'」というエラー（Issues #73、#87）が頻繁に報告される。これには、local_xy_originの適切な初期化が必要であり、TFブロードキャストのタイミング問題がマップ表示を妨げることがあり、複数ノード起動時の変換同期問題が発生する可能性がある。

**ランチファイルの問題（Issue #712）**として、初期のROS2 Foxyバージョンではランチファイルに問題があり、`local_xy_origins`パラメータのフォーマット問題や、一部の設定でのQoS deprecation警告が報告されている。

### 修正済みの問題

最近修正された問題には、**Issue #836（テクスチャードマーカープラグインサブスクリプションとプラグインロードエラーハンドリングの修正）**、**Issue #842（CMakeエクスポートの修正）**、**Issue #862（GLUTライブラリ参照エラーの修正）**、**Issue #859（空白文字トリミング動作の改善）**、**Issue #833（トピックリスト表示問題の修正）**、**Issue #827（適切なQoSを使用するプラグインサブスクライバーの更新）**がある。

### 現在の回避策

**ナビゲーション用**には、Mapviz move_baseプラグインの代わりにNav2スタックを別途使用し、ナビゲーションタスクにはNav2ウェイポイントフォローイング統合を使用し、別のナビゲーション可視化ツールの使用を検討する。

**変換問題用**には、Mapvizの前にinitialize_origin.pyが実行中であることを確認し、robot_localizationノードを最初に起動し、適切なランチファイル順序を使用し、TFツリーが適切にパブリッシュされていることを検証する（`ros2 run tf2_tools view_frames`）。

**マップ表示問題用**には、aptパッケージの代わりに最新のソースビルドを使用し（時には新機能あり）、マップタイルがアクセス可能であることを確認し（オンラインソースのネットワーク接続）、Bing mapsを使用する場合はBing APIキーをチェックし、カスタムソースのWMTS URL形式を検証する。

既知の互換性注意点として、Waylandディスプレイの問題（警告：「Ignoring XDG_SESSION_TYPE=wayland on Gnome」）があり、一部の機能は最新の修正のためソースからのビルドが必要な場合があり、イメージトランスポート選択には手動設定が必要である。

## コミュニティサポートと開発情報

### 公式リソース

**ドキュメント**として、公式ウェブサイト（https://swri-robotics.github.io/mapviz/）、プラグインドキュメント（https://swri-robotics.github.io/mapviz/plugins/）、ROS2 APIドキュメント（https://docs.ros.org/en/rolling/p/mapviz/）が利用可能である。

**ソースコード**は、GitHubリポジトリ（https://github.com/swri-robotics/mapviz）、ROS2ブランチ（ros2-devel）、イシュートラッカー（https://github.com/swri-robotics/mapviz/issues）、リリースリポジトリ（https://github.com/ros2-gbp/mapviz-release）で公開されている。

**ROS Index**では、パッケージ概要（https://index.ros.org/p/mapviz/）、リポジトリ概要（https://index.ros.org/r/mapviz/）、プラグインパッケージ（https://index.ros.org/p/mapviz_plugins/）が提供されている。

### コミュニティフォーラム

**ROS Discourse**では、一般的なディスカッションとアナウンスが行われ、コミュニティメンバーは応答性の高いメンテナーを報告している。注目すべきスレッドには「Easy offline Google Maps layer for MapViz」がある。

**ROS Answers（Robotics Stack Exchange）**は、特定の問題に対する技術的なQ\u0026Aに使用され、タグ：mapvizで検索できる。

**GitHubイシュー**では、アクティブなイシュートラッキング（158フォーク、積極的にメンテナンス中）が行われ、ユーザーから迅速な応答時間が報告されており、機能リクエストとバグレポートが受け付けられている。

### チュートリアルと例

**主要チュートリアル**として、Robotics Knowledgebase（https://roboticsknowledgebase.com/wiki/tools/mapviz/）がROS2セットアップ、Google Maps統合、カスタムランチファイルをカバーしている。

**Navigation2チュートリアル**では、GPSナビゲーションチュートリアルにMapvizの使用法が含まれ、nav2_gps_waypoint_follower_demoに例がある。

**サードパーティリソース**として、GNSS可視化に関するMedium記事、YouTubeプレゼンテーション（ROSCon 2015）、コミュニティブログ投稿とチュートリアルが利用可能である。

### メンテナー情報

**組織**：Southwest Research Institute（SwRI）  
**主要開発者**：Marc Alban、P.J. Reed  
**メンテナーメール**：swri-robotics@swri.org  
**応答時間**：コミュニティは数分から数時間以内の応答を報告  
**ライセンス**：BSD 3-Clause

### 関連ツール

**補完的ツール**として、3D可視化のためのRVizと並行して動作する。**代替2D可視化ツール**には、Vizanti（Webベース）、Foxglove（Webベース）がある。**統合**では、robot_localization、Nav2、GPSドライバーと連携する。

### サポート推奨事項

既知の問題についてはまずGitHubイシューを確認し、一般的な質問にはROS Discourseを使用し、ログを含む詳細なバグレポートをGitHubに提出し、最新の修正のためにソースからビルドし、ディスカッションのためにROSコミュニティフォーラムに参加することが推奨される。

## 結論

ROS2版mapvizは、2025年9月の最新バージョン2.5.10に至るまで、成熟した実用レベルの2D可視化ツールとして確立されている。Southwest Research Instituteによる積極的なメンテナンスと、全主要ROS2ディストリビューションへの対応により、**屋外ロボティクス、自動運転車両、GPS/GNSS可視化アプリケーションにおいて最良の選択肢**となっている。

ROS1からの移植はほぼ完了しており、コアとなる可視化機能は完全に動作する。30以上のプラグインを通じて、センサーデータ、ナビゲーション情報、GPS位置、占有グリッド、ポイントクラウド、画像など、ほぼ全ての一般的なROS2メッセージタイプを可視化できる。タイルマップ統合により、OpenStreetMap、Google Maps、Bingマップなどの実世界地図上にロボットデータを重ねて表示できる点は、屋外アプリケーションにおいて圧倒的な利点である。

制限事項として、一部のインタラクティブナビゲーション機能（move_baseプラグイン等）がROS2で完全に移植されていない点は認識しておく必要がある。しかし、Nav2スタックと組み合わせることで、ナビゲーション可視化のニーズは十分に満たせる。変換設定には注意が必要だが、適切な初期化とランチファイル構成により、安定した動作が得られる。

2025年も継続的なアップデートが行われており、GLUT参照エラー、CMakeエクスポート、空白文字トリミングなど、細かな改善が着実に進められている。Boost依存関係の削除やdeprecatedなamentマクロの除去など、コードベースの現代化も進んでいる。**応答性の高いメンテナー、活発なコミュニティ、寛容なBSD-3-Clauseライセンス**により、学術研究から商用製品まで、幅広い用途での採用が可能である。

ROS2プロジェクトで2D可視化、特にGPS統合や屋外ロボティクスを必要とする場合、mapvizは**本番環境での使用に十分な成熟度と信頼性**を備えた、推奨される選択肢である。