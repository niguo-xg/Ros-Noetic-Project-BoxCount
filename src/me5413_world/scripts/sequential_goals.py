#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Int32, String, Bool
import tf
import time
import json
import math

# 全局变量，用于保存来自订阅者的数据
bridge_detected = False
bridge_location_str = None # String "x,y,angle"
bridge_location_coords = None # Tuple (x, y, angle) after parsing
target_box_pose = None

# --- Parameters ---
BRIDGE_CROSSING_OFFSET = 1.5 # 过桥后目标点相对桥中心的偏移距离 (米) - 需要调整
SCAN_ROTATIONS = 4 # 扫描时的旋转次数
SCAN_PAUSE_TIME = 2.0 # 每次旋转后的暂停扫描时间 (秒)
FIND_BRIDGE_TIMEOUT = 15.0 # 等待桥梁检测的超时时间 (秒)
UNLOCK_WAIT_TIME = 10.0 # 发送解锁命令后的等待时间 (秒)
NAVIGATION_TIMEOUT_DEFAULT = rospy.Duration(60.0) # 默认导航超时
NAVIGATION_TIMEOUT_BRIDGE = rospy.Duration(90.0) # 过桥导航超时
NAVIGATION_TIMEOUT_ROTATION = rospy.Duration(30.0) # 扫描旋转超时


# --- Waypoint Definitions (根据你的地图和坐标系仔细调整!) ---

# 区域 1: 初始导航路径点 (从起点到区域2入口)
# 例如: 从 (0,0) -> (5, 0) -> (10, -2) 到达区域2入口附近
# !! 这些点需要根据你的实际地图精确设置 !!
initial_nav_points = [

    (21.3, -22, 0) # 假设这个点是区域2的入口附近
]

# 区域 2: 箱子扫描点 (在桥前区域内)
# !! 这些点应位于地图上蓝色虚线框出的桥前区域 !!
pre_bridge_scan_points = [
    (18.9, -21.5),   # 示例扫描点1
    (18.9, -3.29),   # 示例扫描点2
    (16.3, -3.29),
    (15.9, -21.6),
    (12.9, -21.6),
    (12.3, -3.29)
]

# 区域 4: 目标箱子搜索/扫描点 (在桥后区域内)
# !! 这些点应位于地图上蓝色虚线框出的桥后区域 !!
post_bridge_scan_points = [
    (24.0, -8.0),   # 示例扫描/搜索点1
    (24.0, -11.0)   # 示例扫描/搜索点2
]

# -------------------------------------------------------------

# 回调函数 (与之前版本相同)
def bridge_detected_callback(msg): global bridge_detected; bridge_detected = msg.data
def bridge_location_callback(msg):
    global bridge_location_str, bridge_location_coords
    bridge_location_str = msg.data
    try:
        parts = bridge_location_str.split(',')
        if len(parts) == 3: x, y, angle = float(parts[0]), float(parts[1]), float(parts[2]); bridge_location_coords = (x, y, angle)
        else: rospy.logwarn(f"Invalid bridge location format: {bridge_location_str}"); bridge_location_coords = None
    except Exception as e: rospy.logerr(f"Error parsing bridge location: {e}"); bridge_location_coords = None

def target_pose_callback(msg):
    global target_box_pose
    target_box_pose = msg

# --- TF Listener for getting current pose ---
# (最好在主流程或类初始化时创建)
tf_listener = None

def init_tf_listener():
    """Initializes the TF listener."""
    global tf_listener
    if tf_listener is None:
        try:
            tf_listener = tf.TransformListener()
            # Give TF buffer time to fill
            rospy.sleep(2.0)
            rospy.loginfo("TF Listener initialized.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize TF Listener: {e}")
            tf_listener = None # Ensure it's None if failed


def get_current_pose():
    """Gets the current robot pose (x, y, yaw) in the map frame using TF."""
    if tf_listener is None:
        rospy.logerr("TF Listener not initialized, cannot get current pose.")
        return None
    try:
        # Define target and source frames
        target_frame = "map"
        source_frame = "base_link" # Or your robot's base frame

        # Wait for the transform to become available
        tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))

        # Lookup the transform
        (trans, rot_quat) = tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))

        # Convert quaternion to Euler angles to get yaw
        euler = tf.transformations.euler_from_quaternion(rot_quat)
        yaw = euler[2] # z-axis rotation

        rospy.logdebug(f"Current pose in map: ({trans[0]:.2f}, {trans[1]:.2f}), Yaw: {yaw:.2f} rad")
        return (trans[0], trans[1], yaw)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
        rospy.logerr(f"TF Error getting current pose from {source_frame} to {target_frame}: {e}")
        return None
    except Exception as e:
        rospy.logerr(f"Unexpected error getting current pose: {e}")
        return None



# Helper function to create MoveBaseGoal (与之前版本相同)
def create_goal(x, y, yaw=0.0):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(*q)
    return goal

# Action Client Helper (与之前版本相同)
def send_goal_and_wait(client, goal, timeout=NAVIGATION_TIMEOUT_DEFAULT):
    rospy.loginfo(f"Sending goal: Pos({goal.target_pose.pose.position.x:.2f}, {goal.target_pose.pose.position.y:.2f}), Yaw({tf.transformations.euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])[2]:.2f} rad)")
    client.send_goal(goal)
    if client.wait_for_result(timeout):
        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED: rospy.loginfo("Goal reached successfully.\n"); return True
        else: rospy.logwarn(f"Goal failed: {client.get_goal_status_text()} (State: {state})\n"); return False
    else: rospy.logwarn("Goal timed out.\n"); client.cancel_goal(); return False


# --- 修改后的 scan_for_boxes 函数 ---
def scan_for_boxes(client, position):
    """
    Attempts to navigate to a position. If successful, scans by rotating at that position.
    If navigation fails, scans by rotating at the robot's current location.
    """
    target_x, target_y = position
    rospy.loginfo(f"--- Preparing to scan near ({target_x:.2f}, {target_y:.2f}) ---")

    # 1. 尝试移动到目标扫描点
    goal_move = create_goal(target_x, target_y, 0.0)
    rospy.loginfo(f"Attempting to move to target scan point ({target_x:.2f}, {target_y:.2f})...")
    reached_target_point = send_goal_and_wait(client, goal_move)

    # 2. 确定实际执行扫描的位置
    scan_exec_x, scan_exec_y = target_x, target_y # 默认在目标点执行
    scan_location_source = "target waypoint"

    if not reached_target_point:
        rospy.logwarn(f"Failed to reach exact target scan point ({target_x:.2f}, {target_y:.2f}).")
        rospy.loginfo("Attempting to get current location to scan from there...")
        current_pose = get_current_pose()

        if current_pose is not None:
            scan_exec_x, scan_exec_y, _ = current_pose # 使用当前位置执行扫描
            scan_location_source = "current robot location"
            rospy.loginfo(f"Will perform scan rotations at current location: ({scan_exec_x:.2f}, {scan_exec_y:.2f})")
        else:
            # 如果连当前位置都获取失败，无法执行旋转扫描
            rospy.logerr("Could not get current robot pose. Aborting scan sequence for this point.")
            # 停止扫描指令（以防万一之前已经激活）
            scan_pub = rospy.Publisher('/start_box_scan', Bool, queue_size=1, latch=True)
            scan_msg = Bool(data=False); scan_pub.publish(scan_msg)
            return False # 表示扫描未执行

    # 3. 激活箱子扫描 (无论在哪里执行)
    scan_pub = rospy.Publisher('/start_box_scan', Bool, queue_size=1, latch=True)
    scan_msg = Bool(data=True)
    scan_pub.publish(scan_msg)
    rospy.loginfo(f"Box scanning activated (at {scan_location_source}).")
    rospy.sleep(0.5)

    # 4. 在实际执行位置旋转扫描
    rospy.loginfo(f"Performing {SCAN_ROTATIONS} rotations at ({scan_exec_x:.2f}, {scan_exec_y:.2f})...")
    initial_orientation = 0.0 # 或者可以尝试获取当前朝向: current_pose[2] if current_pose else 0.0
    for i in range(SCAN_ROTATIONS):
        if rospy.is_shutdown(): break # 检查 ROS 是否关闭
        yaw = initial_orientation + (i + 1) * (2 * math.pi / SCAN_ROTATIONS)
        # 使用实际执行扫描的位置创建旋转目标
        goal_rotate = create_goal(scan_exec_x, scan_exec_y, yaw)
        rospy.loginfo(f"Rotating to yaw {yaw:.2f} rad...")
        if not send_goal_and_wait(client, goal_rotate, timeout=NAVIGATION_TIMEOUT_ROTATION):
             rospy.logwarn("Rotation failed or timed out, continuing scan.")
        rospy.loginfo(f"Pausing for {SCAN_PAUSE_TIME}s to scan...")
        rospy.sleep(SCAN_PAUSE_TIME)

    # 5. 停止箱子扫描
    scan_msg.data = False
    scan_pub.publish(scan_msg)
    rospy.loginfo("Box scanning deactivated.")
    rospy.sleep(0.5)

    rospy.loginfo(f"--- Finished scanning sequence near ({target_x:.2f}, {target_y:.2f}) ---")
    # 返回 True 表示扫描序列已尝试执行（可能在目标点或当前点）
    # 如果连当前位置都获取失败导致无法旋转，上面已经 return False 了
    return True


# 寻找桥梁 (与之前版本相同)
def find_bridge(client):
    rospy.loginfo("--- Finding bridge ---")
    global bridge_detected, bridge_location_coords
    bridge_detected = False; bridge_location_coords = None
    find_bridge_pub = rospy.Publisher('/find_bridge', Bool, queue_size=1, latch=True)
    find_msg = Bool(data=True); find_bridge_pub.publish(find_msg)
    rospy.loginfo("Command sent to find bridge.")
    start_time = rospy.Time.now(); rate = rospy.Rate(2)
    while not bridge_detected and (rospy.Time.now() - start_time) < rospy.Duration(FIND_BRIDGE_TIMEOUT):
        rospy.loginfo("Waiting for bridge detection...")
        rate.sleep()
        if rospy.is_shutdown(): return False
    if bridge_detected and bridge_location_coords:
        rospy.loginfo(f"Bridge found at {bridge_location_coords[:2]} angle {bridge_location_coords[2]:.2f} rad")
        return True
    else: rospy.logwarn("Bridge not found within timeout."); return False


# 穿越桥梁 (与之前版本相同，使用动态目标点)
def cross_bridge(client):
    rospy.loginfo("--- Crossing bridge ---")
    if not bridge_detected or not bridge_location_coords:
        rospy.logerr("Cannot cross bridge: Bridge not detected or location unknown.")
        return False
    bridge_x, bridge_y, bridge_angle = bridge_location_coords
    unlock_pub = rospy.Publisher('/unlock_command', Bool, queue_size=1, latch=True)
    unlock_msg = Bool(data=True); unlock_pub.publish(unlock_msg)
    rospy.loginfo("Unlock command sent. Waiting...")
    rospy.sleep(UNLOCK_WAIT_TIME)
    rospy.loginfo("Wait finished. Calculating crossing goal...")
    target_x = bridge_x + BRIDGE_CROSSING_OFFSET * math.cos(bridge_angle)
    target_y = bridge_y + BRIDGE_CROSSING_OFFSET * math.sin(bridge_angle)
    target_yaw = bridge_angle
    rospy.loginfo(f"Bridge crossing goal: ({target_x:.2f}, {target_y:.2f}), Yaw: {target_yaw:.2f} rad")
    goal = create_goal(target_x, target_y, target_yaw)
    if send_goal_and_wait(client, goal, timeout=NAVIGATION_TIMEOUT_BRIDGE):
        rospy.loginfo("Successfully navigated across the bridge area.")
        return True
    else: rospy.logerr("Failed to navigate across the bridge."); return False


# --- 主任务流程 ---
def sequential_goals():
    rospy.init_node('sequential_goal_publisher')

    # !!! 初始化 TF Listener !!!
    init_tf_listener()
    if tf_listener is None:
        rospy.logfatal("TF Listener failed to initialize. Cannot proceed reliably.")
        return

    # --- 订阅与客户端设置 ---
    # 删除不再需要的订阅
    #rospy.Subscriber('/box_count', Int32, box_count_callback)
    #rospy.Subscriber('/box_types', String, box_numbers_callback)
    #rospy.Subscriber('/box_positions', String, box_positions_callback)
    #rospy.Subscriber('/least_common_number', String, least_common_number_callback)
    rospy.Subscriber('/bridge_detected', Bool, bridge_detected_callback)
    rospy.Subscriber('/bridge_location', String, bridge_location_callback)
    rospy.Subscriber('/target_box_pose', PoseStamped, target_pose_callback)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(30.0)):
         rospy.logfatal("Move Base action server not available. Exiting.")
         return
    rospy.loginfo("Move_base action server connected.")

    # --- 任务执行 ---
    rospy.loginfo("\n\n=== Starting Mission Pipeline ===\n")
    mission_successful = True # 跟踪任务是否成功

    # == Phase 1: Navigate Initial Area (Area 1) ==
    rospy.loginfo("\n\n--- Phase 1: Navigating Initial Area ---\n")
    phase_success = True
    mission_successful = True  # 确保 mission_successful 变量有初始值

    for point in initial_nav_points:
        if rospy.is_shutdown():
            mission_successful = False
            break

        rospy.loginfo(f"Moving to navigation point: {point}")
        goal = create_goal(point[0], point[1])  # 默认朝向

        retries = 10  # 允许重试 10 次
        success = False
        for _ in range(retries):
            if send_goal_and_wait(client, goal, NAVIGATION_TIMEOUT_DEFAULT):
                rospy.loginfo(f"Successfully reached {point}")
                success = True
                break
            rospy.logwarn(f"Attempt failed for {point}, retrying...")

        if not success:
            rospy.logerr(
                f"\n\nFailed to reach initial navigation point {point} after {retries} attempts. Aborting Phase 1.\n")
            phase_success = False
            break

    rospy.loginfo("\n\n--- Phase 1 Completed ---\n")
    if not phase_success:
        mission_successful = False

    # == Phase 2: Scan Pre-Bridge Area (Area 2) ==
    if mission_successful:
        rospy.loginfo("\n\n--- Phase 2: Scanning Pre-Bridge Area (Area 2) ---\n")
        phase_success = True
        for point in pre_bridge_scan_points:
            if rospy.is_shutdown(): mission_successful = False; break
            # scan_for_boxes 返回 True 表示扫描序列完成，不代表导航一定成功
            # 我们需要确保至少在一个点成功扫描
            scan_for_boxes(client, point) # 即使失败也尝试下一个点
            # 这里可以根据需要添加更复杂的成功判断逻辑
        rospy.loginfo("\n\n--- Phase 2 Completed ---\n")
        # 假设扫描本身总是"完成"，即使中间有导航失败

    # == Phase 3: Find Bridge ==
    if mission_successful:
        rospy.loginfo("\n\n--- Phase 3: Finding Bridge ---\n")
        bridge_is_found = find_bridge(client)
        rospy.loginfo(f"\n\n--- Phase 3 Completed (Found: {bridge_is_found}) ---\n")
        if not bridge_is_found:
             rospy.logerr("\n\nBridge not found. Cannot proceed.\n")
             mission_successful = False

    # == Phase 4: Cross Bridge ==
    if mission_successful:
        rospy.loginfo("\n\n--- Phase 4: Crossing Bridge ---\n")
        bridge_crossed = cross_bridge(client)
        rospy.loginfo(f"\n\n--- Phase 4 Completed (Crossed: {bridge_crossed}) ---\n")
        if not bridge_crossed:
            rospy.logerr("\n\nFailed to cross bridge. Cannot proceed.\n")
            mission_successful = False

    # == Phase 5: Scan Post-Bridge Area (Area 4) ==
    # 即使目标是找箱子，也先扫描一下区域4，更新全局信息
    if mission_successful:
        rospy.loginfo("\n\n--- Phase 5: Scanning Post-Bridge Area (Area 4) ---\n")
        phase_success = True
        for point in post_bridge_scan_points:
             if rospy.is_shutdown(): mission_successful = False; break
             scan_for_boxes(client, point)
        rospy.loginfo("\n\n--- Phase 5 Completed ---\n")

    # == Phase 6: Navigate to Target Box ==
    if mission_successful:
        rospy.loginfo("\n\n--- Phase 6: Navigating to Target Box ---\n")
        phase_success = True
        start_time = rospy.Time.now()
        while target_box_pose is None and (rospy.Time.now() - start_time) < rospy.Duration(60):
            rospy.loginfo_throttle(5, "Waiting for target box pose...")
            rospy.sleep(1)
        if target_box_pose is not None:
            goal = MoveBaseGoal()
            goal.target_pose = target_box_pose
            if send_goal_and_wait(client, goal, NAVIGATION_TIMEOUT_DEFAULT):
                rospy.loginfo("Successfully navigated to target box.")
            else:
                rospy.logerr("Failed to navigate to target box.")
                mission_successful = False
        else:
            rospy.logerr("Timeout waiting for target box pose.")
            mission_successful = False

    # --- Mission End ---
    if mission_successful:
        rospy.loginfo("\n\n=== Mission Pipeline Completed Successfully (or partially) ===\n")
    else:
        rospy.logerr("\n\n=== Mission Pipeline Failed ===\n")


if __name__ == '__main__':
    try:
        sequential_goals()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.\n")
    except Exception as e:
        rospy.logfatal(f"Unhandled exception in sequential_goals: {e}", exc_info=True) # 添加堆栈跟踪