#!/usr/bin/env python3
"""
Mapvizシステムの動作確認スクリプト
READMEの「動作確認方法」セクションを自動化
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import subprocess
import sys
import time
from typing import Optional, List, Tuple
from enum import Enum


class CheckLevel(Enum):
    """チェック結果のレベル"""
    OK = "OK"
    WARNING = "WARNING"
    ERROR = "ERROR"


class Color:
    """ターミナルカラーコード"""
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


class MapvizDoctor(Node):
    """Mapvizシステムの健全性チェッカー"""

    def __init__(self):
        super().__init__('mapviz_doctor')
        self.gps_fix: Optional[NavSatFix] = None
        self.local_xy_origin: Optional[PoseStamped] = None
        self.fix_sub = None
        self.origin_sub = None

    def create_subscribers(self):
        """サブスクライバーを作成（トピックチェック後に呼ぶ）"""
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

        # 通常のQoS設定でGPS fixをサブスクライブ
        self.fix_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            10
        )

        # Transient Local QoS設定でlocal_xy_originをサブスクライブ
        # (latchedトピック用: 過去のメッセージも受信可能)
        latched_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.origin_sub = self.create_subscription(
            PoseStamped,
            '/local_xy_origin',
            self.origin_callback,
            latched_qos
        )

    def fix_callback(self, msg: NavSatFix):
        """GPS fixメッセージのコールバック"""
        self.gps_fix = msg

    def origin_callback(self, msg: PoseStamped):
        """Local XY originメッセージのコールバック"""
        self.local_xy_origin = msg


def print_header(title: str):
    """セクションヘッダーを表示"""
    print(f"\n{Color.CYAN}{Color.BOLD}{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}{Color.RESET}")


def print_result(check_name: str, level: CheckLevel, message: str = ""):
    """チェック結果を表示"""
    if level == CheckLevel.OK:
        status = f"{Color.GREEN}✅ OK{Color.RESET}"
    elif level == CheckLevel.WARNING:
        status = f"{Color.YELLOW}⚠️  WARNING{Color.RESET}"
    else:  # ERROR
        status = f"{Color.RED}❌ ERROR{Color.RESET}"

    print(f"{status}  {check_name}")
    if message:
        color = Color.GREEN if level == CheckLevel.OK else (Color.YELLOW if level == CheckLevel.WARNING else Color.RED)
        print(f"     {color}{message}{Color.RESET}")


def check_topic_list(node: Node) -> Tuple[CheckLevel, List[str], str]:
    """トピックリストの確認"""
    try:
        # rclpy APIを使ってトピックリストを取得
        topic_names_and_types = node.get_topic_names_and_types()
        topics = [name for name, _ in topic_names_and_types]

        if not topics or len(topics) == 0:
            return CheckLevel.ERROR, [], "トピックが1つも見つかりません (ROS2が起動していない可能性)"

        # トピックリストを改行区切りで表示（4個以上は省略）
        if len(topics) <= 4:
            topic_list = "\n     ".join(topics)
            return CheckLevel.OK, topics, f"{len(topics)}個のトピックを検出:\n     {topic_list}"
        else:
            topic_list = "\n     ".join(topics[:4])
            omitted = len(topics) - 4
            return CheckLevel.OK, topics, f"{len(topics)}個のトピックを検出:\n     {topic_list}\n     ... (他 {omitted}個を省略)"
    except Exception as e:
        return CheckLevel.ERROR, [], f"トピックリスト取得エラー: {str(e)}"


def check_required_publishers(node: Node) -> Tuple[CheckLevel, str]:
    """必要なトピック(/tf, /tf_static)のpublisher確認 (Mapviz必須)"""
    # Mapvizに必須のトピック
    required_critical = {
        '/tf': 'TF変換用',
        '/tf_static': '静的TF変換用'
    }

    results = []
    missing = []
    for topic, desc in required_critical.items():
        pub_count = node.count_publishers(topic)
        if pub_count > 0:
            results.append(f"{topic} ({desc}): ✓ Publisher {pub_count}件")
        else:
            results.append(f"{topic} ({desc}): ✗ Publisher 0件")
            missing.append(topic)

    details = "\n     ".join(results)

    if missing:
        return CheckLevel.ERROR, f"必須トピックpublisher確認:\n     {details}"
    else:
        return CheckLevel.OK, f"必須トピックpublisher確認:\n     {details}"


def check_optional_publishers(node: Node) -> Tuple[CheckLevel, str]:
    """オプショナルなトピック(/fix, /local_xy_origin)のpublisher確認"""
    optional = {
        '/fix': 'GPS座標表示用',
        '/local_xy_origin': 'GPS原点設定用'
    }

    results = []
    missing_count = 0
    for topic, desc in optional.items():
        pub_count = node.count_publishers(topic)
        if pub_count > 0:
            results.append(f"{topic} ({desc}): ✓ Publisher {pub_count}件")
        else:
            results.append(f"{topic} ({desc}): ✗ Publisher 0件")
            missing_count += 1

    details = "\n     ".join(results)

    if missing_count > 0:
        return CheckLevel.WARNING, f"GPS関連トピックpublisher確認:\n     {details}"
    else:
        return CheckLevel.OK, f"GPS関連トピックpublisher確認:\n     {details}"


def get_all_frames(tf_buffer: 'Buffer') -> List[str]:
    """TFバッファから利用可能なすべてのフレームIDを取得"""
    try:
        all_frames = tf_buffer.all_frames_as_string()
        # all_frames_as_stringの出力から個別のフレームIDを抽出
        # 出力形式: "Frame frame_id exists with parent parent_id."
        import re
        frames = re.findall(r'Frame (\S+) exists', all_frames)
        return list(set(frames))  # 重複削除
    except:
        return []


def check_transform_chain(tf_buffer: 'Buffer', source: str, target: str) -> Tuple[bool, str, List[str]]:
    """TF変換チェーンを確認し、中間フレームを返す"""
    from rclpy.duration import Duration

    try:
        transform = tf_buffer.lookup_transform(
            target,
            source,
            rclpy.time.Time(),
            timeout=Duration(seconds=2.0)
        )
        trans = transform.transform.translation

        # 変換チェーンを取得
        chain = []
        all_frames = get_all_frames(tf_buffer)

        # TF2には直接チェーンを取得するAPIがないため、
        # 変換が成功したことのみ報告
        return True, f"x={trans.x:.2f}, y={trans.y:.2f}, z={trans.z:.2f}", chain
    except Exception as e:
        return False, str(e), []


def check_tf_tree(node: Node) -> Tuple[CheckLevel, str]:
    """TFツリーの確認 (GPS表示に必要) - 段階的にチェック
    想定TFチェーン: map → odom → base_link → gps_link
    """
    from tf2_ros import Buffer, TransformListener
    import rclpy
    from rclpy.duration import Duration

    try:
        # TF2バッファとリスナーを作成
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)

        # 少し待機してTFが蓄積されるのを待つ
        for i in range(10):  # 最大1秒待機
            rclpy.spin_once(node, timeout_sec=0.1)

        results = []
        overall_status = CheckLevel.OK

        # 1. 段階的な変換をチェックしてフレーム存在を確認
        # map → odom
        success1, info1, _ = check_transform_chain(tf_buffer, 'odom', 'map')

        # odom → base_link
        success2, info2, _ = check_transform_chain(tf_buffer, 'base_link', 'odom')

        # base_link → gps_link
        success3, info3, _ = check_transform_chain(tf_buffer, 'gps_link', 'base_link')

        # map → gps_link (最終)
        success_final, info_final, _ = check_transform_chain(tf_buffer, 'gps_link', 'map')

        # フレーム存在確認（変換成功から推定）
        map_exists = success1 or success_final
        odom_exists = success1 or success2
        base_link_exists = success2 or success3
        gps_link_exists = success3 or success_final

        # フレーム存在表示
        results.append(f"{'✓' if map_exists else '✗'} mapフレーム: {'存在' if map_exists else '不在'}")
        results.append(f"{'✓' if odom_exists else '✗'} odomフレーム: {'存在' if odom_exists else '不在'}")
        results.append(f"{'✓' if base_link_exists else '✗'} base_linkフレーム: {'存在' if base_link_exists else '不在'}")
        results.append(f"{'✓' if gps_link_exists else '✗'} gps_linkフレーム: {'存在' if gps_link_exists else '不在'}")

        # 不在のフレームがあればエラー
        if not (map_exists and odom_exists and base_link_exists and gps_link_exists):
            overall_status = CheckLevel.ERROR

        # 最終変換結果
        if success_final:
            results.append(f"✓ TF変換 map → gps_link (最終): 成功 ({info_final})")
        else:
            results.append(f"✗ TF変換 map → gps_link (最終): 失敗")
            if overall_status == CheckLevel.OK:
                overall_status = CheckLevel.WARNING

        # 段階的な変換
        results.append("")  # 空行
        results.append("段階的なTF変換:")

        if success1:
            results.append(f"  ✓ map → odom: 成功 ({info1})")
        else:
            results.append(f"  ✗ map → odom: 失敗")
            if overall_status == CheckLevel.OK:
                overall_status = CheckLevel.WARNING

        if success2:
            results.append(f"  ✓ odom → base_link: 成功 ({info2})")
        else:
            results.append(f"  ✗ odom → base_link: 失敗")
            if overall_status == CheckLevel.OK:
                overall_status = CheckLevel.WARNING

        if success3:
            results.append(f"  ✓ base_link → gps_link: 成功 ({info3})")
        else:
            results.append(f"  ✗ base_link → gps_link: 失敗")
            if overall_status == CheckLevel.OK:
                overall_status = CheckLevel.WARNING

        # 結果をまとめる
        detail_message = "\n     ".join(results)

        if overall_status == CheckLevel.ERROR:
            return CheckLevel.ERROR, f"TFフレーム確認:\n     {detail_message}"
        elif overall_status == CheckLevel.WARNING:
            return CheckLevel.WARNING, f"TFフレーム確認:\n     {detail_message}"
        else:
            return CheckLevel.OK, f"TFフレーム確認:\n     {detail_message}"

    except Exception as e:
        return CheckLevel.WARNING, f"TF変換チェックエラー: {str(e)[:100]} (GPS表示には必要)"


def check_gps_data(node: MapvizDoctor) -> Tuple[CheckLevel, str]:
    """GPS座標データの確認 (GPS表示には必須だがMapvizには必須ではない)"""
    if node.gps_fix is None:
        return CheckLevel.WARNING, "GPS fixメッセージが受信できていません (GPS表示には必要)"

    msg = node.gps_fix
    details = (
        f"frame_id: {msg.header.frame_id}, "
        f"緯度: {msg.latitude:.6f}, "
        f"経度: {msg.longitude:.6f}, "
        f"高度: {msg.altitude:.1f}m"
    )

    # 座標が妥当な範囲か確認（緯度-90~90, 経度-180~180）
    if -90 <= msg.latitude <= 90 and -180 <= msg.longitude <= 180:
        return CheckLevel.OK, details
    else:
        return CheckLevel.WARNING, f"座標値が異常: {details}"


def check_origin_data(node: MapvizDoctor) -> Tuple[CheckLevel, str]:
    """Local XY originデータの確認 (mapviz GPS表示に必須)"""
    if node.local_xy_origin is None:
        return CheckLevel.ERROR, "local_xy_originメッセージが受信できていません (mapviz GPS表示に必須)"

    origin = node.local_xy_origin
    details = (
        f"frame_id: {origin.header.frame_id}, "
        f"位置: ({origin.pose.position.x:.2f}, "
        f"{origin.pose.position.y:.2f}, "
        f"{origin.pose.position.z:.2f})"
    )
    return CheckLevel.OK, details


def main():
    """メイン処理"""
    print(f"\n{Color.BOLD}{Color.BLUE}🔍 Mapviz システム診断ツール{Color.RESET}")
    print("=" * 60)

    # 前提条件の表示
    print(f"\n{Color.CYAN}{Color.BOLD}【前提条件】{Color.RESET}")
    print("このツールは以下の設定を前提としています:")
    print(f"  {Color.YELLOW}・トピック名:{Color.RESET}")
    print("    - GPS座標: /fix (sensor_msgs/NavSatFix)")
    print(f"  {Color.YELLOW}・TF変換チェーン:{Color.RESET}")
    print("    - map → odom → base_link → gps_link")
    print(f"{Color.CYAN}※ 上記設定と異なる場合、診断結果が正しく表示されません。{Color.RESET}")
    print("=" * 60)

    # ROS2初期化
    rclpy.init()
    node = MapvizDoctor()

    error_count = 0
    warning_count = 0

    try:
        # ノード初期化後、少し待機してトピックリストを取得
        for i in range(5):  # 0.5秒待機
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)

        # 1. トピックリストの確認（サブスクライバー作成前）
        print_header("1. トピックリストの確認")
        level, topics, message = check_topic_list(node)
        print_result("ROS2トピックリスト取得", level, message)
        if level == CheckLevel.ERROR:
            error_count += 1

        if level == CheckLevel.OK:
            level, message = check_required_publishers(node)
            print_result("Mapviz必須トピックpublisher確認", level, message)
            if level == CheckLevel.ERROR:
                error_count += 1

            level, message = check_optional_publishers(node)
            print_result("GPS関連トピックpublisher確認", level, message)
            if level == CheckLevel.WARNING:
                warning_count += 1

        # 2. TFツリーの確認 (GPS表示に必要)
        print_header("2. TF変換の確認 [GPS表示用・オプション]")
        level, message = check_tf_tree(node)
        print_result("TF変換 (map → gps_link)", level, message)
        if level == CheckLevel.WARNING:
            warning_count += 1
        elif level == CheckLevel.ERROR:
            error_count += 1

        # 3. サブスクライバーを作成してメッセージ受信
        node.create_subscribers()

        # メッセージ受信のために短時間スピン
        for i in range(6):  # 0.5秒 x 6 = 3秒
            rclpy.spin_once(node, timeout_sec=0.5)
            time.sleep(0.5)

        # 追加で少し待機（QoS negotiation用）
        time.sleep(0.5)
        rclpy.spin_once(node, timeout_sec=0.5)

        # 3. GPS座標の確認 (オプション)
        print_header("3. GPS座標データの確認 [トピック: /fix] [GPS表示用・オプション]")
        level, message = check_gps_data(node)
        print_result("GPS fix受信", level, message)
        if level == CheckLevel.WARNING:
            warning_count += 1

        # 4. Origin設定の確認 (mapviz GPS表示に必須)
        print_header("4. Local XY Origin設定の確認 [トピック: /local_xy_origin]")
        level, message = check_origin_data(node)
        print_result("Origin設定", level, message)
        if level == CheckLevel.ERROR:
            error_count += 1
        elif level == CheckLevel.WARNING:
            warning_count += 1

        # 最終結果
        print_header("診断完了")

        if error_count == 0 and warning_count == 0:
            print(f"{Color.GREEN}{Color.BOLD}✅ すべてのチェックに合格しました！{Color.RESET}")
        elif error_count == 0:
            print(f"{Color.YELLOW}{Color.BOLD}⚠️  {warning_count}件の警告があります (GPS表示機能が制限される可能性){Color.RESET}")
        else:
            print(f"{Color.RED}{Color.BOLD}❌ {error_count}件のエラーがあります (Mapvizが正常に動作しない可能性){Color.RESET}")
            if warning_count > 0:
                print(f"{Color.YELLOW}   + {warning_count}件の警告{Color.RESET}")

        print(f"\n{Color.CYAN}詳細な確認方法:{Color.RESET}")
        print("  GPS座標データ確認: ros2 topic echo /fix --once")
        print("  GPS座標周期確認: ros2 topic hz /fix")
        print("  Origin設定確認: ros2 topic echo /local_xy_origin --once")
        print("  TF変換確認: ros2 run tf2_ros tf2_echo map gps_link")
        print("  TFツリー可視化: ros2 run tf2_tools view_frames  (frames.pdfが生成されます)")
        print("  Initialize Origin単独起動: ros2 run swri_transform_util initialize_origin.py")
        print("  Mapviz単独起動: ros2 run mapviz mapviz")
        print()

    except KeyboardInterrupt:
        print(f"\n\n{Color.YELLOW}中断されました{Color.RESET}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
