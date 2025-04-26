
# gnss_preprocessing_node

## 1. ノード概要

### 役割・目的
- ROS2環境でGNSSデータをリアルタイムに受信し、地理座標系（緯度・経度・高度）をローカル座標系（ENU座標系）に変換する。
- TF（Transform）を使用して、座標系間（map→odom、base_link→gps）の静的な関係を定義・公開する。
- GNSSセンサー情報を加工し、ロボットの自己位置推定や経路計画の基礎データとして利用可能にする。

---

## 2. 入出力設計

### 入力（Subscribe）

<!--
| 入力元ノード名 | トピック名 | 型 | 説明 |
|---|---|---|---|
| GNSSセンサーノード | `/ublox_gps_node/fix` | `sensor_msgs::msg::NavSatFix` | GNSSから取得した緯度・経度・高度情報 |
-->


| トピック名 | 型 | 説明 |
|---|---|---|
|  `/ublox_gps_node/fix` | `sensor_msgs::msg::NavSatFix` | GNSSから取得した緯度・経度・高度情報 |

### 出力（Publish）
<!---
| 出力先ノード名 | トピック名 | 型 | 説明 |
|---|---|---|---|
| 自己位置推定等 | `/gnss_pose` | `geometry_msgs::msg::PoseStamped` | ENU座標系へ変換された現在位置 |
| 経路生成・可視化等 | `/gnss_path` | `nav_msgs::msg::Path` | GNSSベースの移動経路 |
--->

| トピック名 | 型 | 説明 |
|---|---|---|
|  `/gnss_pose` | `geometry_msgs::msg::PoseStamped` | ENU座標系へ変換された現在位置 |
|  `/gnss_path` | `nav_msgs::msg::Path` | GNSSベースの移動経路 |


### 座標変換（TF）

| 親フレーム | 子フレーム | 説明 |
|---|---|---|
| `map` | `odom` | 固定された地図座標系からロボットが移動する座標系への静的変換 |
| `base_link` | `gps` | ロボット本体座標系からGNSSセンサー搭載位置への静的変換 |

---

## 3. 処理フロー


---

## 4. ノード起動設定（launchファイル解説）

```python
Node(
    package='gnss_preprocessing',
    executable='gnss_preprocessing',
    name='gnss_preprocessing',
    output='screen'
)
```

- `package`：ROS2パッケージ `gnss_preprocessing`
- `executable`：実行するバイナリファイル (`gnss_preprocessing`)
- `name`：ノード名（`gnss_preprocessing`としてROS2に登録）
- `output`：ノードのログをターミナル画面に出力（デバッグ容易化）

### 静的TF設定

```python
# map → odom
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_to_odom',
    arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
)

# base_link → gps
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='baselink_to_navsat',
    arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'gps']
)
```

- `tf2_ros`のstatic_transform_publisherを用いて、座標系間の静的な関係を定義する。

---

## 5. クラス設計（PlantUMLによる詳細クラス図）


![alt text](image.png)
実装クラス構造に基づいた詳細なクラス設計を以下に示します。

```plantuml
@startuml

class GnssPreprocessingCore {
  - double lat0
  - double lon0
  - double hig0
  - double pi
  - double a
  - double ONE_F
  - double E2
  
  - Publisher<PoseStamped> gnss_pose_pub
  - Publisher<Path> gnss_path_pub
  - Subscription<NavSatFix> gnss_sub
  - TransformBroadcaster odom_to_baselink_broadcaster
  - Path gnss_path

  + GnssPreprocessingCore(double lat, double lon, double hig)
  + ~GnssPreprocessingCore()

  - void gnssCallback(NavSatFix::SharedPtr gnss_msg)
  - Vector3d blh2ecef(double lat_deg, double lon_deg, double h)
  - Vector3d ecef2enu(Vector3d dest, Vector3d origin)
  - double deg2rad(double deg)
  - double rad2deg(double rad)
}

class SaveGnssPath {
  - double pre_x
  - double pre_y
  - double dist_thread
  - ofstream output_file
  - Subscription<PoseStamped> gnss_sub

  + SaveGnssPath()
  + ~SaveGnssPath()
  
  - void gnss_callback(PoseStamped::SharedPtr gnss_msg)
}

GnssPreprocessingCore --> PoseStamped : publishes
GnssPreprocessingCore --> Path : publishes
GnssPreprocessingCore <-- NavSatFix : subscribes
SaveGnssPath <-- PoseStamped : subscribes

@enduml
```

---

## 6. 起動時の処理概要


1. ROS2ノードとして初期化される（初期座標は引数またはハードコードで指定）。
2. Subscriberで`/ublox_gps_node/fix`を受信待機。
3. GNSSデータ受信毎に以下を処理:
   - BLH→ECEF→ENU座標変換を実施。
   - 変換した位置情報を`PoseStamped`としてPublish。
   - 軌跡情報として`Path`へ記録・Publish。

---
