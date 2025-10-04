# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

ROS 2 Humble用のMapviz（GPS地図可視化ツール）サンプルプロジェクトです。Mapvizのデフォルト設定の問題を解決し、使いやすくするためのラッパーlaunchファイルと診断ツールを提供します。

**動作環境:**
- OS: Ubuntu 22.04
- ROS: ROS 2 Humble
- Mapviz: 2.5.10
- Python: 3.10

## 主要コマンド

### Mapviz起動（GPS座標を原点とする）
GPS座標を先に配信する必要があります:
```bash
# ターミナル1: GPS座標を配信（大阪駅）
bash publish_osaka.sh

# ターミナル2: Mapvizを起動
ros2 launch mapviz_auto.launch.py
```

### Mapviz起動（固定座標を原点とする）
GPS配信不要、大阪駅の固定座標を使用:
```bash
ros2 launch mapviz_fixed_origin.launch.py
```

### システム診断ツールの実行
```bash
python3 mapviz_doctor.py
```

### その他のGPS配信スクリプト
```bash
bash publish_tokyo.sh      # 東京駅
bash publish_newyork.sh    # ニューヨーク
bash publish_osaka.sh      # 大阪駅
```

### 全ROSノードの停止
```bash
bash stop_all.sh
```

### デバッグ用コマンド
```bash
# トピック確認
ros2 topic echo /local_xy_origin --once
ros2 topic echo /fix --once
ros2 topic hz /fix

# TF変換確認
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map gps_link

# ノード確認
ros2 node list
ros2 topic list
```

## アーキテクチャ

### 主要コンポーネント

**Launchファイル:**
- `mapviz_auto.launch.py`: `/fix`トピックから受信した初回GPS座標を原点に自動設定
- `mapviz_fixed_origin.launch.py`: 事前定義された大阪駅の座標を原点として使用

両launchファイルの共通処理:
1. キャッシュ設定（`~/.mapviz_config`）を削除してクリーンスタート
2. `autosave: False`を設定して不要な設定永続化を防止
3. TFチェーンを作成: `map → odom → base_link → gps_link`
4. `initialize_origin.py`ノードを起動（`swri_transform_util`パッケージから）
5. カスタム設定で`mapviz`ノードを起動

**診断ツール:**
- `mapviz_doctor.py`: トピック、TF変換、GPS座標、local_xy_originの健全性をチェック
  - 各チェック項目の状態を色分け表示（OK/WARNING/ERROR）
  - 想定構成: トピック `/fix`, `/local_xy_origin`、TFチェーン `map → odom → base_link → gps_link`

**設定ファイル:**
- `mapviz.mvc`: カスタム地図タイルソース（国土地理院地図、OpenStreetMap）を含むMapviz設定

**GPS配信スクリプト:**
- `publish_osaka.sh`: 大阪駅のGPS座標を10Hzで配信
- `publish_tokyo.sh`: 東京駅のGPS座標を10Hzで配信
- `publish_newyork.sh`: ニューヨークのGPS座標を10Hzで配信
- すべて`ros2 topic pub`コマンドで`/fix`トピックに`sensor_msgs/msg/NavSatFix`を配信

### 重要なトピックとフレーム

**必須トピック（Mapviz表示に必須）:**
- `/tf`: TF変換（publisherが必要）
- `/tf_static`: 静的TF変換（publisherが必要）
- `/local_xy_origin` (geometry_msgs/PoseStamped): GPS原点座標 - **Mapvizの地図タイル表示に必須**
  - **重要**: `position.x`は経度、`position.y`は緯度（通常のREP 103とは異なる）
  - QoS: `TRANSIENT_LOCAL`（後から起動するノードでも受信可能）

**オプショナルトピック（GPS可視化用）:**
- `/fix` (sensor_msgs/NavSatFix): GPS座標（10Hz推奨）

**TFチェーン:**
```
map → odom → base_link → gps_link
```

すべてのTF変換は静的変換（static_transform_publisher）で単位行列（原点、回転なし）を使用。

### Mapviz特有の注意点

1. **AutoSave動作**: デフォルトのAutoSaveは`~/.mapviz_config`を作成し、launchファイルの設定を上書きします。両launchファイルはこれを無効化し、起動時にキャッシュを削除します。

2. **local_xy_originフォーマット**: x, y, z値は経度、緯度、高度を表します（通常のデカルト座標ではない）。
   - 例（大阪駅）: x=135.495951（経度）, y=34.702485（緯度）, z=5.0（高度）
   - これはREP 103の標準とは異なる特殊な仕様です

3. **Stamen Maps利用不可**: デフォルトのStamenタイル地図は利用できなくなりました（2025年10月時点）。このプロジェクトでは国土地理院（GSI）とOpenStreetMapのタイルを使用します。

4. **initialize_origin.py動作**: `swri_transform_util`パッケージのこのノードが`/local_xy_origin`を配信します。
   - `auto`モード: 初回GPS fixを待って原点を設定
   - `fixed`モード: 事前定義された座標を原点として設定

### 地図タイルソース（mapviz.mvcに設定）

- OpenStreetMap: https://tile.openstreetmap.org/{level}/{x}/{y}.png
- GSI_Standard (国土地理院 標準地図): https://cyberjapandata.gsi.go.jp/xyz/std/{level}/{x}/{y}.png
- GSI_Pale (国土地理院 淡色地図): https://cyberjapandata.gsi.go.jp/xyz/pale/{level}/{x}/{y}.png
- GSI_English (国土地理院 英語地図): https://cyberjapandata.gsi.go.jp/xyz/english/{level}/{x}/{y}.png
- GSI_Photo (国土地理院 写真): https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{level}/{x}/{y}.jpg

デフォルト設定: GSI_Photo

## 開発ノート

- ドキュメントは日本語中心（README.md、genai_docs/内）
- ビルドシステムなし（CMakeLists.txt/package.jsonなし） - 純粋なPython launchファイルとスクリプト
- シェルスクリプトは`ros2 topic pub`でGPS座標を配信
- 診断ツール（`mapviz_doctor.py`）は特定のトピック名とTFチェーンを前提 - 構成が異なる場合は警告を表示

## トラブルシューティング

### 地図タイルが表示されない
- `/local_xy_origin`が配信されているか確認: `ros2 topic echo /local_xy_origin --once`
- `position.x`（経度）と`position.y`（緯度）が正しいか確認
- `mapviz_doctor.py`で診断実行

### GPS座標は表示されるが地図が表示されない
- `/local_xy_origin`の座標が経度・緯度の順で正しく設定されているか確認
- UTM座標を誤って設定していないか確認

### TF変換エラー
- `ros2 run tf2_tools view_frames`でTFツリーを確認
- 必要なチェーン: `map → odom → base_link → gps_link`
- launchファイルがstatic_transform_publisherを正しく起動しているか確認

## 関連ドキュメント

- `README.md`: 使い方、起動方法、診断ツールの使用方法
- `genai_docs/MAPVIZ_TILE_DISPLAY_GUIDE.md`: Mapvizの地図タイル表示の詳細ガイド、座標変換の仕組み
- `genai_docs/initialize_origin_guide.md`: initialize_origin.pyノードの使用方法（存在する場合）
- `genai_docs/mapviz_ros2_guide.md`: ROS 2でのMapviz使用ガイド（存在する場合）
