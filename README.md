# mapvizのサンプル

ROS 2 Humbleでmapvizを使用して、世界各地のGPS位置と地図タイルを表示するデモです。

## 概要

mapvizのサンプルです。

## 動作確認環境
- Ubuntu 22.04
- ROS 2 Humble
- Mapviz

## 座標情報
高度は適当です。

| 場所 | 緯度 | 経度 | 高度 |
|------|------|------|------|
| 大阪駅 | 34.702485 | 135.495951 | 5.0m |
| 東京駅 | 35.681236 | 139.767125 | 5.0m |

## 起動方法

### 方法1: GPS座標を受け取る場合

#### 1. GPS座標の発行

表示したい場所のスクリプトを起動します：

**大阪駅の場合:**
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

### 方法2: 固定原点で起動する場合

GPS座標を受け取らずに、大阪駅を原点とする固定座標系で起動：

```bash
ros2 launch mapviz_fixed_origin.launch.py
```

この方法ではGPSパブリッシャー（publish_*.sh）を起動する必要がありません。

## 動作確認方法

### GPS座標の確認

GPS座標が正しく発行されているか確認：

```bash
ros2 topic echo /fix --once
```

期待される出力例（大阪の場合）：
```
latitude: 34.702485
longitude: 135.495951
altitude: 5.0
```

### Originの確認

initialize_originが正しく座標を設定したか確認：

```bash
ros2 topic echo /local_xy_origin --once
```

### TFツリーの確認

TFツリーが正しく構築されているか確認：

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

期待されるTFツリー：
```
map → odom → base_link → gps_link
```

### トピックリストの確認

必要なトピックが発行されているか確認：

```bash
ros2 topic list | grep -E "gps|local_xy"
```

期待される出力：
```
/fix
/local_xy_origin
```

## 表示内容

Mapvizウィンドウには以下が表示されます：

1. **地図タイル**: OpenStreetMapまたはGSI（国土地理院）の地図
2. **赤いGPSマーカー**: 現在のGPS位置
3. **スケール**: 広域表示（view_scale: 0.05）

## 起動成功の確認方法（ログチェック）

### GPS Publisher起動時のログ

`bash publish_*.sh`実行後、以下のログが出ればGPS発行成功：

```
Waiting for at least 1 matching subscription(s)...
publisher: beginning loop
publishing #1: sensor_msgs.msg.NavSatFix(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='gps_link'), status=sensor_msgs.msg.NavSatStatus(status=0, service=1), latitude=34.702485, longitude=135.495951, altitude=5.0, ...)
```

✅ **確認ポイント**: `latitude`、`longitude`、`altitude`が期待する座標になっている

### Mapviz起動時のログ（正常パターン）

`ros2 launch mapviz_auto.launch.py`実行後、以下のログが順番に出れば起動成功：

#### 1. Initialize Origin成功
```
[initialize_origin.py-1] [INFO] [xxxxx]: Origin: auto
[initialize_origin.py-1] [INFO] [xxxxx]: Frame: map
[initialize_origin.py-1] [INFO] [xxxxx]: Got NavSat message.
[initialize_origin.py-1] [INFO] [xxxxx]: Origin from 'navsat' source set to 34.702485, 135.495951, 5.0
[initialize_origin.py-1] [INFO] [xxxxx]: Successfully set origin; unsubscribing.
```

✅ **確認ポイント**:
- `Origin from 'navsat' source set to` の後の座標が、起動したGPS publisherの座標と一致
- `Successfully set origin; unsubscribing.` が出力されている

#### 2. LocalXyWgs84Util初期化成功
```
[ros2-5] [INFO] [xxxxx]: LocalXyWgs84Util initializing origin to lat: 34.702485, lon: 135.495951, alt: 5.000000
```

✅ **確認ポイント**: 緯度経度が期待する座標と一致

#### 3. Mapviz設定読み込み成功
```
[ros2-5] [INFO] [xxxxx]: Loading configuration from /home/taro/.mapviz_config
[ros2-5] [INFO] [xxxxx]: fixed frame selected: map
[ros2-5] [INFO] [xxxxx]: Target frame selected: base_link
```

✅ **確認ポイント**:
- `fixed frame selected: map`
- `Target frame selected: base_link`

#### 4. プラグイン起動成功
```
[ros2-5] [INFO] [xxxxx]: creating: mapviz_plugins/tile_map
[ros2-5] [INFO] [xxxxx]: creating: mapviz_plugins/navsat
```

✅ **確認ポイント**: `tile_map`と`navsat`プラグインが作成されている

#### 5. GPS受信成功
```
[ros2-5] [INFO] [xxxxx]: Subscribing to /fix
[ros2-5] [INFO] [xxxxx]: OK
```

✅ **確認ポイント**: 最終的に`OK`が表示される

### よくある警告（無視してOK）

以下の警告は起動直後に一時的に表示されますが、数秒後に解消されるため無視してOK：

```
[WARN] [xxxxx]: [transform_manager]: No transformer from 'wgs84' to 'map'.
[ERROR] [xxxxx]: "map" passed to lookupTransform argument target_frame does not exist.
[WARN] [xxxxx]: Wgs84Transformer not initialized
```

これらは、initialize_originがまだoriginを設定していない、またはTFが完全に準備できていないときに出ます。

### エラーログ（要対応）

以下のログが**継続的に**出る場合は問題あり：

#### ❌ Origin設定失敗
```
[ERROR] [xxxxx]: Unhandled std::exception in Qt event loop: bad file: /home/taro/.mapviz_config
```

**対処法**:
```bash
cp /home/taro/temp2/mapviz.mvc ~/.mapviz_config
```

#### ❌ GPS座標が受信できない
```
[WARN] [xxxxx]: No messages received.
```
が継続的に出る場合

**対処法**: GPS publisherが起動しているか確認
```bash
ps aux | grep publish
```

#### ❌ 間違った座標が設定された
```
[initialize_origin.py-1] [INFO] [xxxxx]: Origin from 'navsat' source set to 35.681236, 139.767125, 5.0
```
東京の座標なのに大阪を起動したかった場合

**対処法**: 古いプロセスが残っているため、完全停止して再起動
```bash
bash stop_all.sh
sleep 3
bash publish_osaka.sh
ros2 launch mapviz_auto.launch.py
```

### 完全起動の確認コマンド

すべて正常に起動したか一度に確認：

```bash
# GPS座標が1Hzで発行されているか
ros2 topic hz /fix
# 期待: average rate: 1.000

# Originが設定されているか
ros2 topic echo /local_xy_origin --once
# 期待: position.x（経度）とposition.y（緯度）が表示される

# Mapvizが"OK"状態か
# ターミナルのログで [INFO] OK が出ていることを確認
```

すべて成功していれば、mapvizウィンドウに地図タイルと赤いGPSマーカーが表示されます。

## トラブルシューティング

### 地図が表示されない

1. GPS座標が正しく発行されているか確認
   ```bash
   ros2 topic hz /fix
   ```

2. initialize_originがoriginを設定したか確認
   ```bash
   ros2 topic echo /local_xy_origin --once
   ```

3. すべてのプロセスを停止して再起動
   ```bash
   bash stop_all.sh
   # 3秒待つ
   bash publish_osaka.sh  # または他の場所
   # 別のターミナルで
   ros2 launch mapviz_auto.launch.py
   ```

### 古い座標が表示される

これは古いバックグラウンドプロセスが残っている可能性があります：

```bash
bash stop_all.sh
sleep 3
# プロセスが完全に停止したか確認
ps aux | grep -E "ros2|publish" | grep -v grep
# 何も表示されなければOK、表示されたら手動でkill
```

### ターミナルが落ちる

stop_all.shを使用してください。`killall bash`は使用しないでください。

## 技術詳細

### システム構成

1. **GPS Publisher (publish_*.sh)**
   - `ros2 topic pub`コマンドでGPS座標を1Hzで発行
   - トピック: `/fix` (sensor_msgs/NavSatFix)

2. **Initialize Origin (swri_transform_util)**
   - 最初のGPS座標から原点を自動設定
   - トピック: `/local_xy_origin` (geometry_msgs/PoseStamped)
   - モード: `auto` (自動初期化)

3. **Static TF Publishers**
   - `map → odom`: 地図フレーム
   - `odom → base_link`: ロボット基準フレーム
   - `base_link → gps_link`: GPSセンサフレーム

4. **Mapviz**
   - 地図タイル表示プラグイン
   - GPS位置表示プラグイン
   - 設定ファイル: `~/.mapviz_config` (mapviz.mvcからコピー)

### カスタマイズ

#### 新しい場所を追加

新しいGPS座標を追加するには、スクリプトをコピーして座標を変更します：

```bash
cp publish_osaka.sh publish_custom.sh
# エディタで座標を変更
# LAT=新しい緯度
# LON=新しい経度
# ALT=新しい高度
```

#### 地図の表示スケールを変更

`mapviz.mvc`と`~/.mapviz_config`の`view_scale`を変更：
- `0.05`: 広域表示（現在の設定）
- `0.5`: 中域表示
- `5.0`: 詳細表示

## 参考情報

- [Mapviz公式ドキュメント](https://github.com/swri-robotics/mapviz)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [OpenStreetMap](https://www.openstreetmap.org/)
- [国土地理院 地理院タイル](https://maps.gsi.go.jp/development/ichiran.html)
