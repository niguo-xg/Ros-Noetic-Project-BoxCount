#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point, PoseArray, Pose, PointStamped, PoseStamped # Added PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import tf # Added TF
import time

class BridgeCrosser:
    """动态桥梁检测与穿越模块 (with Laser Confirmation)"""

    def __init__(self):
        rospy.init_node('bridge_crosser', anonymous=True)

        # --- Parameters ---
        # River boundaries (can be parameters)
        self.river_x_min = rospy.get_param("~river_x_min", 19.0)
        self.river_x_max = rospy.get_param("~river_x_max", 22.0)
        self.river_y_min = rospy.get_param("~river_y_min", -15.0)
        self.river_y_max = rospy.get_param("~river_y_max", -5.0)
        # Bridge detection params
        self.map_bridge_threshold = rospy.get_param("~map_bridge_threshold", 40) # Occupancy value below this might be bridge
        self.min_bridge_width = rospy.get_param("~min_bridge_width", 0.5)
        self.max_bridge_width = rospy.get_param("~max_bridge_width", 3.0)
        # Laser confirmation params
        self.laser_confirm_enabled = rospy.get_param("~laser_confirm_enabled", True)
        self.laser_bridge_width_expected = rospy.get_param("~laser_bridge_width_expected", 1.5) # Approx width to check in laser scan (meters)
        self.laser_dist_tolerance = rospy.get_param("~laser_dist_tolerance", 0.5) # Max allowed deviation from expected dist (meters)
        self.laser_max_gap_fraction = rospy.get_param("~laser_max_gap_fraction", 0.3) # Max fraction of invalid points allowed on bridge path
        # TF Frames
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.laser_frame = rospy.get_param("~laser_frame", "front_laser") # Adjust if needed

        # Bridge state
        self.bridge_detected = False
        self.bridge_location = None  # Stores (x, y, angle) in map frame
        self.river_boundaries = None

        # Publishers
        self.bridge_detected_pub = rospy.Publisher('/bridge_detected', Bool, queue_size=1, latch=True) # Latch last msg
        self.bridge_location_pub = rospy.Publisher('/bridge_location', String, queue_size=1, latch=True) # Latch last msg
        self.potential_bridges_pub = rospy.Publisher('/potential_bridges', PoseArray, queue_size=10) # Debugging
        self.unlock_pub = rospy.Publisher('/unlock_blockade', Bool, queue_size=1, latch=True) # Latch last msg

        # Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.find_bridge_sub = rospy.Subscriber('/find_bridge', Bool, self.find_bridge_callback)
        self.unlock_command_sub = rospy.Subscriber('/unlock_command', Bool, self.unlock_command_callback)

        # TF Listener
        try:
            self.tf_listener = tf.TransformListener()
            rospy.loginfo("TF Listener initialized for Bridge Crosser.")
        except Exception as e:
             rospy.logerr(f"Failed to initialize TF Listener in Bridge Crosser: {e}")
             self.tf_listener = None

        # Map and Scan Data
        self.map_data = None
        self.map_info = None
        self.current_scan = None
        self.potential_bridges_map = [] # Bridges found from map analysis

        rospy.loginfo("Dynamic Bridge Crosser initialized.")
        rospy.loginfo(f"Laser confirmation enabled: {self.laser_confirm_enabled}")

    def map_callback(self, msg):
        """处理地图数据"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width)) # Reshape for easier indexing
        self.map_info = msg.info
        self.detect_river_boundaries() # Set boundaries based on params for now

    def scan_callback(self, msg):
        """处理激光雷达数据"""
        self.current_scan = msg
        # Laser confirmation is triggered after map analysis if enabled

    def find_bridge_callback(self, msg):
        """接收查找桥梁的命令"""
        if msg.data:
            rospy.loginfo("Received find_bridge command.")
            if self.map_data is not None and self.map_info is not None:
                self.detect_and_confirm_bridge()
            else:
                rospy.logwarn("Cannot detect bridge: Map data not available yet.")
        # else: Optional: handle stopping the search?

    def unlock_command_callback(self, msg):
        """接收解锁命令"""
        if msg.data and self.bridge_detected:
            self.unlock_blockade()

    def detect_river_boundaries(self):
        """Set river boundaries (currently uses parameters)"""
        self.river_boundaries = {
            "x_min": self.river_x_min, "x_max": self.river_x_max,
            "y_min": self.river_y_min, "y_max": self.river_y_max
        }
        # rospy.logdebug(f"River boundaries set to: {self.river_boundaries}") # Debug level

    def world_to_map(self, wx, wy):
        """将世界坐标转换为地图像素坐标 (row, col)"""
        if self.map_info is None: return None, None
        ox, oy = self.map_info.origin.position.x, self.map_info.origin.position.y
        res = self.map_info.resolution
        col = int((wx - ox) / res)
        row = int((wy - oy) / res)
        if 0 <= row < self.map_info.height and 0 <= col < self.map_info.width:
            return row, col
        return None, None

    def map_to_world(self, row, col):
        """将地图像素坐标转换为世界坐标"""
        if self.map_info is None: return None, None
        ox, oy = self.map_info.origin.position.x, self.map_info.origin.position.y
        res = self.map_info.resolution
        wx = ox + col * res + res / 2.0 # Center of cell
        wy = oy + row * res + res / 2.0 # Center of cell
        return wx, wy

    def get_map_value(self, row, col):
        """获取地图上特定像素的值"""
        if self.map_data is None or not (0 <= row < self.map_info.height and 0 <= col < self.map_info.width):
            return -1 # Unknown or out of bounds
        return self.map_data[row, col]

    def detect_and_confirm_bridge(self):
        """主检测流程：先用地图找，再用激光确认（如果启用）"""
        rospy.loginfo("Starting bridge detection process...")
        self.bridge_detected = False # Reset detection state

        # 1. Detect potential bridges from map
        self.potential_bridges_map = self.detect_bridge_candidates_from_map()
        self.publish_potential_bridges(self.potential_bridges_map) # Publish map candidates

        if not self.potential_bridges_map:
            rospy.logwarn("No potential bridge candidates found from map analysis.")
            self.publish_bridge_state() # Publish not detected
            return

        # 2. Filter candidates with Laser Scan (if enabled)
        confirmed_bridges = self.potential_bridges_map
        if self.laser_confirm_enabled:
            rospy.loginfo("Performing laser confirmation of map candidates...")
            if self.current_scan is None:
                 rospy.logwarn("Laser confirmation enabled, but no current scan data available.")
            elif self.tf_listener is None:
                 rospy.logwarn("Laser confirmation enabled, but TF listener not available.")
            else:
                confirmed_bridges = self.filter_bridges_with_laser(self.potential_bridges_map, self.current_scan)
                self.publish_potential_bridges(confirmed_bridges, is_confirmed=True) # Publish confirmed ones (optional viz)
                if not confirmed_bridges:
                     rospy.logwarn("Laser confirmation filtered out all map candidates.")

        # 3. Confirm the best bridge from the remaining candidates
        if confirmed_bridges:
            if self.select_best_bridge(confirmed_bridges):
                 rospy.loginfo(f"Bridge detection successful. Confirmed at {self.bridge_location}")
            else:
                 rospy.logwarn("Failed to select a best bridge from confirmed candidates.")
        else:
            rospy.logwarn("No confirmed bridge candidates remaining after filtering.")

        # 4. Publish final detection state
        self.publish_bridge_state()


    def detect_bridge_candidates_from_map(self):
        """从地图数据分析检测潜在桥梁的位置"""
        if self.map_data is None or self.map_info is None or self.river_boundaries is None:
            rospy.logwarn("Missing data for map-based bridge detection")
            return []

        rospy.loginfo("Analyzing map to detect potential bridge candidates...")
        potential_bridges = []
        steps = 30 # Increase steps for finer scan

        # Iterate through Y coordinates in the river area
        y_points = np.linspace(self.river_boundaries["y_min"], self.river_boundaries["y_max"], steps)

        for y in y_points:
            bridge_segments_on_row = []
            in_potential_bridge = False
            segment_start_col = None

            # Scan across X for this Y
            x_scan_world = np.linspace(self.river_boundaries["x_min"] - 1.0, # Scan slightly wider area
                                       self.river_boundaries["x_max"] + 1.0,
                                       steps * 3)

            for x in x_scan_world:
                row, col = self.world_to_map(x, y)
                if row is None: # Out of map bounds
                    if in_potential_bridge: # End segment if we go out of bounds
                        is_bridge_like = False # Treat out of bounds as not bridge
                    else: continue
                else:
                    map_value = self.get_map_value(row, col)
                    # Check if cell is traversable (might be bridge, or just land near river)
                    is_bridge_like = (0 <= map_value < self.map_bridge_threshold)

                if is_bridge_like and not in_potential_bridge:
                    # Start of a potential bridge segment
                    in_potential_bridge = True
                    segment_start_col = col
                    segment_start_x = x
                elif not is_bridge_like and in_potential_bridge:
                    # End of a potential bridge segment
                    in_potential_bridge = False
                    segment_end_col = col - 1 # Previous column was the end
                    segment_end_x = x - self.map_info.resolution # Approx end x

                    # Check if segment width is reasonable
                    width_m = segment_end_x - segment_start_x
                    # Check if segment center is within river X bounds
                    center_x = (segment_start_x + segment_end_x) / 2.0

                    if (self.min_bridge_width < width_m < self.max_bridge_width and
                        self.river_boundaries["x_min"] <= center_x <= self.river_boundaries["x_max"]):
                        # Store center X and Y, assume 0 angle for now (horizontal bridge)
                        # More advanced: check connectivity across river, estimate angle
                        bridge_info = (center_x, y, 0.0) # (x, y, angle_rad)
                        potential_bridges.append(bridge_info)
                        rospy.logdebug(f"Map candidate found: Center=({center_x:.2f}, {y:.2f}), Width={width_m:.2f}m")

        rospy.loginfo(f"Found {len(potential_bridges)} potential candidates from map.")
        return potential_bridges


    def filter_bridges_with_laser(self, candidates, scan_msg):
        """使用激光雷达数据过滤地图候选桥梁"""
        if not candidates or scan_msg is None or self.tf_listener is None:
            return []

        rospy.loginfo("Filtering map candidates using laser scan...")
        confirmed_by_laser = []
        scan_time = scan_msg.header.stamp

        try:
            # Get transform from map to laser frame at the time of the scan
            self.tf_listener.waitForTransform(self.laser_frame, self.map_frame, scan_time, rospy.Duration(0.5))
            (trans, rot_quat) = self.tf_listener.lookupTransform(self.laser_frame, self.map_frame, scan_time)
            transform_matrix = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot_quat)
            )
            # Inverse transform: laser -> map (needed later? maybe not)
            # self.tf_listener.waitForTransform(self.map_frame, self.laser_frame, scan_time, rospy.Duration(0.5))
            # (trans_l2m, rot_l2m) = self.tf_listener.lookupTransform(self.map_frame, self.laser_frame, scan_time)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Error in laser filter setup: {e}")
            return [] # Cannot proceed without transform

        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        ranges = scan_msg.ranges
        num_ranges = len(ranges)

        for bridge_cand_x, bridge_cand_y, bridge_cand_angle in candidates:
            # --- Simulate bridge path in laser frame ---
            # Define bridge centerline points in map frame (simple horizontal line for now)
            half_width_check = self.laser_bridge_width_expected / 2.0
            p1_map = np.array([bridge_cand_x - half_width_check, bridge_cand_y, 0, 1])
            p2_map = np.array([bridge_cand_x + half_width_check, bridge_cand_y, 0, 1])

            # Transform points to laser frame
            p1_laser = np.dot(transform_matrix, p1_map)
            p2_laser = np.dot(transform_matrix, p2_map)

            # Calculate expected angle range and distance in laser frame
            angle1_laser = math.atan2(p1_laser[1], p1_laser[0])
            angle2_laser = math.atan2(p2_laser[1], p2_laser[0])
            dist1_laser = math.sqrt(p1_laser[0]**2 + p1_laser[1]**2)
            dist2_laser = math.sqrt(p2_laser[0]**2 + p2_laser[1]**2)

            # Determine angular range to check in the scan
            start_angle = min(angle1_laser, angle2_laser)
            end_angle = max(angle1_laser, angle2_laser)
            expected_dist_avg = (dist1_laser + dist2_laser) / 2.0

            # Convert angles to indices
            start_index = max(0, int((start_angle - angle_min) / angle_inc))
            end_index = min(num_ranges - 1, int((end_angle - angle_min) / angle_inc))

            if start_index >= end_index:
                rospy.logdebug("Bridge candidate angle range too small or invalid in laser frame.")
                continue # Skip this candidate

            # --- Analyze laser points in the calculated range ---
            valid_points = 0
            total_points_in_range = 0
            consistent_dist_points = 0
            obstacle_points = 0

            for i in range(start_index, end_index + 1):
                r = ranges[i]
                total_points_in_range += 1
                # Check if range is valid (not inf, nan, or out of sensor range)
                if scan_msg.range_min < r < scan_msg.range_max:
                    valid_points += 1
                    # Check if distance is close to expected bridge distance
                    if abs(r - expected_dist_avg) <= self.laser_dist_tolerance:
                        consistent_dist_points += 1
                    # Check if distance is significantly *shorter* (potential obstacle on bridge)
                    elif r < expected_dist_avg - self.laser_dist_tolerance * 1.5: # More strict for obstacles
                        obstacle_points += 1
                # else: point is invalid (inf, nan, out of range) - treated as gap

            # --- Evaluate candidate based on analysis ---
            if total_points_in_range == 0: continue

            gap_fraction = 1.0 - (float(valid_points) / total_points_in_range)
            consistency_fraction = float(consistent_dist_points) / valid_points if valid_points > 0 else 0

            rospy.logdebug(f"Candidate ({bridge_cand_x:.1f},{bridge_cand_y:.1f}): "
                          f"Indices [{start_index}-{end_index}], "
                          f"Exp Dist {expected_dist_avg:.2f}m, "
                          f"Gap Frac {gap_fraction:.2f}, "
                          f"Consist Frac {consistency_fraction:.2f}, "
                          f"Obstacles {obstacle_points}")

            # Decision logic (tune these thresholds)
            passed_laser_check = False
            if (gap_fraction <= self.laser_max_gap_fraction and # Not too many gaps
                consistency_fraction >= 0.7 and                # Most valid points have consistent distance
                obstacle_points <= 1):                           # Allow very few potential obstacles
                passed_laser_check = True
                rospy.loginfo(f"Candidate at ({bridge_cand_x:.2f}, {bridge_cand_y:.2f}) PASSED laser confirmation.")
                confirmed_by_laser.append((bridge_cand_x, bridge_cand_y, bridge_cand_angle))
            else:
                 rospy.loginfo(f"Candidate at ({bridge_cand_x:.2f}, {bridge_cand_y:.2f}) FAILED laser confirmation.")


        rospy.loginfo(f"Laser confirmation finished. {len(confirmed_by_laser)} candidates remain.")
        return confirmed_by_laser


    def select_best_bridge(self, candidates):
        """确认最佳的桥梁候选 (简单策略: 选Y坐标中间的)"""
        if not candidates:
            self.bridge_location = None
            self.bridge_detected = False
            return False

        # Sort by Y coordinate
        candidates.sort(key=lambda b: b[1])
        middle_index = len(candidates) // 2
        self.bridge_location = candidates[middle_index] # Store (x, y, angle)
        self.bridge_detected = True
        rospy.loginfo(f"Best bridge selected: {self.bridge_location}")
        return True


    def publish_bridge_state(self):
        """发布最终的桥梁检测状态和位置"""
        detected_msg = Bool(data=self.bridge_detected)
        self.bridge_detected_pub.publish(detected_msg)

        location_msg = String()
        if self.bridge_detected and self.bridge_location:
            location_msg.data = f"{self.bridge_location[0]},{self.bridge_location[1]},{self.bridge_location[2]}"
        else:
            location_msg.data = "" # Publish empty string if not detected
        self.bridge_location_pub.publish(location_msg)
        rospy.logdebug(f"Published bridge state: Detected={self.bridge_detected}, Location='{location_msg.data}'")


    def publish_potential_bridges(self, bridges_list, is_confirmed=False):
        """发布潜在/确认的桥梁位置，用于可视化"""
        if not bridges_list: return

        pose_array = PoseArray()
        pose_array.header.frame_id = self.map_frame
        pose_array.header.stamp = rospy.Time.now()

        for bx, by, bangle in bridges_list:
            pose = Pose()
            pose.position.x = bx
            pose.position.y = by
            q = tf.transformations.quaternion_from_euler(0, 0, bangle)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            pose_array.poses.append(pose)

        # Optionally publish to a different topic if confirmed? For now, use the same.
        self.potential_bridges_pub.publish(pose_array)
        rospy.logdebug(f"Published {len(bridges_list)} {'confirmed' if is_confirmed else 'potential'} bridge poses.")


    def unlock_blockade(self):
        """发布解锁障碍的信号"""
        if not self.bridge_detected:
            rospy.logwarn("Bridge not detected yet, cannot unlock blockade")
            return False
        rospy.loginfo("Sending blockade unlock signal")
        msg = Bool(data=True)
        self.unlock_pub.publish(msg)
        # The waiting happens in sequential_goals.py
        return True

    def run(self):
        """主循环 (可以简化，主要逻辑在回调和find命令触发)"""
        rate = rospy.Rate(1) # Low rate, just keeps node alive
        while not rospy.is_shutdown():
            # Periodic republishing of state might be useful if latching fails
            # self.publish_bridge_state()
            rate.sleep()


if __name__ == '__main__':
    try:
        crosser = BridgeCrosser()
        crosser.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as main_e:
         rospy.logfatal(f"Unhandled exception in Bridge Crosser main: {main_e}", exc_info=True)