# mapvizのサンプル

## Mapvizを使う場合のポイント
- AutoSaveがデフォルトで有効です。設定内容が `~/.mapviz_config` に自動保存される。
- 起動時に `~/.mapviz_config` が存在すると、その設定が優先的に読み込まれる。
- 2025年10月現在、Mapvizのプリセット地図タイルStamen Mapは利用できない。変更が必要。
- Mapviz起動には最低限/local_xy_originトピックが必要。このトピックのx,y,zが特殊で、緯度・経度・高度をそのまま設定する必要がある。
  - 例：大阪駅の場合、x=135.495951, y=34.702485, z=5.0
  - 別途別パッケージのinitialize_origin.pyを使うことで解決できが、挙動が分かりづらい。

## このサンプルで改善できること
- AutoSaveを無効にして起動
- 起動時に `~/.mapviz_config`を自動削除
- GPS座標の配信サンプル、初回GPS座標を原点として起動するlaunchファイルを用意 (mapviz_auto.launch.py)
- 固定座標を原点として起動するlaunchファイルを用意 (mapviz_fixed_origin.launch.py)
- 国土地理院の地図タイル(標準、淡色、写真、英語)、OpenStreetMapの地図タイルの設定を用意
- うまく動作しない場合の診断ツールを用意 (mapviz_doctor.py)

## 動作確認環境

| 項目 | バージョン |
|------|-----------|
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Mapviz | 2.5.10 |
| Python | 3.10 |

## Mapvizのインストール方法

```bash
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-mapviz \
    ros-${ROS_DISTRO}-mapviz-plugins \
    ros-${ROS_DISTRO}-tile-map \
    ros-${ROS_DISTRO}-multires-image \
    ros-${ROS_DISTRO}-swri-transform-util
```


## 起動方法

### 方法1: GPS座標を受け取る場合

#### 1. GPS座標の発行

表示したい場所のスクリプトを起動します：

**日本の中心(大阪駅)の場合:**
```bash
bash publish_osaka.sh
```

**東京駅の場合:**
```bash
bash publish_tokyo.sh
```


#### 2. Mapvizの起動

別のターミナルで、またはバックグラウンドで起動します：

```bash
ros2 launch mapviz_auto.launch.py
```
※ AutoSaveを無効にしています。GUIを変更した場合は、手動で保存してください。
※ 起動時にローカルの設定ファイル(`~/.mapviz_config`)を削除しています。

### 方法2: 固定原点で起動する場合

GPS座標を受け取らずに、大阪駅を原点とする固定座標系で起動：

```bash
ros2 launch mapviz_fixed_origin.launch.py
```

※ この方法ではGPSパブリッシャー（publish_*.sh）を起動する必要がありません。
※ AutoSaveを無効にしています。GUIを変更した場合は、手動で保存してください。
※ 起動時にローカルの設定ファイル(`~/.mapviz_config`)を削除しています。

## システム診断ツール

Mapvizシステムが正常に動作しているかを確認するための診断ツール `mapviz_doctor.py` が用意しました。

### 使用方法

Mapvizシステムを起動した状態で、以下のコマンドを実行します：

```bash
python3 mapviz_doctor.py
```

### 正常時の出力例

すべてのチェックに合格した場合の出力例：

```
🔍 Mapviz システム診断ツール
============================================================
【前提条件】
このツールは以下の設定を前提としています:
  ・トピック名:
    - GPS座標: /fix (sensor_msgs/NavSatFix)
  ・TF変換チェーン:
    - map → odom → base_link → gps_link
※ 上記設定と異なる場合、診断結果が正しく表示されません。
============================================================

============================================================
  1. トピックリストの確認
============================================================
✅ OK  ROS2トピックリスト取得
     7個のトピックを検出:
     /diagnostics
     /fix
     /local_xy_origin
     /parameter_events
     ... (他 3個を省略)
✅ OK  Mapviz必須トピックpublisher確認
     必須トピックpublisher確認:
     /tf (TF変換用): ✓ Publisher 1件
     /tf_static (静的TF変換用): ✓ Publisher 3件
✅ OK  GPS関連トピックpublisher確認
     GPS関連トピックpublisher確認:
     /fix (GPS座標表示用): ✓ Publisher 1件
     /local_xy_origin (GPS原点設定用): ✓ Publisher 1件

============================================================
  2. TF変換の確認 [GPS表示用・オプション]
============================================================
✅ OK  TF変換 (map → gps_link)
     TFフレーム確認:
     ✓ mapフレーム: 存在
     ✓ odomフレーム: 存在
     ✓ base_linkフレーム: 存在
     ✓ gps_linkフレーム: 存在
     ✓ TF変換 map → gps_link (最終): 成功 (x=0.00, y=0.00, z=0.00)

     段階的なTF変換:
       ✓ map → odom: 成功 (x=0.00, y=0.00, z=0.00)
       ✓ odom → base_link: 成功 (x=0.00, y=0.00, z=0.00)
       ✓ base_link → gps_link: 成功 (x=0.00, y=0.00, z=0.00)

============================================================
  3. GPS座標データの確認 [トピック: /fix] [GPS表示用・オプション]
============================================================
✅ OK  GPS fix受信
     frame_id: gps_link, 緯度: 34.702485, 経度: 135.495951, 高度: 5.0m

============================================================
  4. Local XY Origin設定の確認 [トピック: /local_xy_origin]
============================================================
✅ OK  Origin設定
     frame_id: map, 位置: (135.50, 34.70, 5.00)

============================================================
  診断完了
============================================================
✅ すべてのチェックに合格しました!
```
