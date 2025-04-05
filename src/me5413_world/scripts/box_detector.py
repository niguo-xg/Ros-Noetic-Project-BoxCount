#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Int32, String, Bool
from geometry_msgs.msg import Point, PointStamped, PoseStamped  # Added PoseStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import json
import tf  # Added TF

# 尝试导入OCR库，如果不可用则发出警告
try:
    import easyocr
    EASYOCR_AVAILABLE = True
except ImportError:
    rospy.logwarn("easyocr not available - OCR features will be limited")
    EASYOCR_AVAILABLE = False
# 尝试导入OCR库，如果不可用则发出警告
try:
    import pytesseract
    TESSERACT_AVAILABLE = True
except ImportError:
    rospy.logwarn("pytesseract not available - OCR features will be limited")
    TESSERACT_AVAILABLE = False



class BoxDetector:
    """箱子检测类 - 识别带有数字的箱子"""
    def __init__(self):
        rospy.init_node('box_detector', anonymous=True)

        self.bridge = CvBridge()
        # --- Parameters ---
        # Get parameters or use defaults
        self.simulation_mode = rospy.get_param('~simulation_mode', False)
        # Camera parameters (adjust based on actual camera)
        self.camera_fov_horizontal_degrees = rospy.get_param('~camera_fov_h', 60.0)
        self.camera_fov_horizontal_rad = math.radians(self.camera_fov_horizontal_degrees)
        # TF Frame IDs
        self.map_frame_id = rospy.get_param('~map_frame', 'map')
        self.robot_base_frame_id = rospy.get_param('~robot_base_frame', 'base_link')
        # Assume laser is mounted relative to base_link, check your robot's TF tree
        self.laser_frame_id = rospy.get_param('~laser_frame', 'front_laser') # Adjust if necessary
        # Box detection parameters
        self.min_box_dist = rospy.get_param('~min_box_separation', 2) # Increased slightly

        # New Parameter: Known Box Size (meters) - Important
        self.known_box_size = rospy.get_param('~known_box_size', 0.8) # meters, default to 0.8m
        # Target Box
        self.target_number = rospy.get_param('~target_number', '1') # Default to box '1'

        self.image_sub = rospy.Subscriber('/front/image_raw', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/front/scan', LaserScan, self.laser_callback)

        # 发布者
        self.box_count_pub = rospy.Publisher('/box_count', Int32, queue_size=10)
        self.box_types_pub = rospy.Publisher('/box_types', String, queue_size=10)
        self.least_common_number_pub = rospy.Publisher('/least_common_number', String, queue_size=10)
        self.box_positions_pub = rospy.Publisher('/box_positions', String, queue_size=10) # Publishes dict {number: [x, y]}
        self.target_pose_pub = rospy.Publisher('/target_box_pose', PoseStamped, queue_size=1) # Target box pose

        # 订阅扫描命令和机器人位置 (using TF now, pose subscriber might be redundant but kept for now)
        self.scan_sub = rospy.Subscriber('/start_box_scan', Bool, self.scan_callback)
        # We'll use TF listener instead of relying solely on /robot_pose
        self.tf_listener = tf.TransformListener()

        self.detected_boxes = []  # 存储已检测到的箱子位置 (global coords) [(x,y), (x,y), ...]
        self.detected_numbers_list = [] # Store number and position of detected numbers
        self.box_numbers = {}  # 存储箱子上的数字及其出现次数 { '1': 2, '3': 1 }
        self.box_positions = {}  # 存储每个数字箱子的位置 { '1': [x1, y1], '2': [x2, y2]}
        self.current_image = None
        self.current_scan = None
        # robot_position from /robot_pose is less used now, TF is primary
        # self.robot_position = (0, 0)
        self.scanning_active = False
        # EasyOCR reader
        if EASYOCR_AVAILABLE:
            self.ocr_reader = easyocr.Reader(['en'], gpu=True)
        else:
            self.ocr_reader = None
        #Store the value of the target
        self.target_box_pose = None
        # Keep track of previous poses to help smooth
        self.previous_poses = {}  # Store previous poses for each number
        self.alpha = 0.2  # Weight for the moving average

        if self.simulation_mode:
            self.simulate_box_data()

        rospy.loginfo("Box detector initialized.")
        rospy.loginfo(f"Simulation mode: {self.simulation_mode}")
        rospy.loginfo(f"Target number: {self.target_number}")
        rospy.loginfo(f"Known Box Size: {self.known_box_size}m")
        rospy.loginfo(f"Map frame: {self.map_frame_id}, Base frame: {self.robot_base_frame_id}, Laser frame: {self.laser_frame_id}")


    def simulate_box_data(self):
        """为测试生成模拟数据"""
        simulated_numbers = ['1', '2', '3', '1', '2', '4', '5', '2', '3']
        for num in simulated_numbers:
            self.box_numbers[num] = self.box_numbers.get(num, 0) + 1

        # Generate some simulated positions - Adjust coordinates based on your map
        self.box_positions = {
            '1': [15.5, -8.3],
            '2': [17.4, -9.7],
            '3': [16.8, -12.5],
            '4': [24.2, -7.9],
            '5': [23.1, -11.1]
        }
        self.detected_boxes = [tuple(pos) for pos in self.box_positions.values()]

        self.update_box_count()
        self.find_least_common_number()
        rospy.loginfo("Simulated box data generated")

    def image_callback(self, data):
        """处理相机图像"""
        try:
            # Store the image and its timestamp
            self.current_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.last_image_time = data.header.stamp

            if self.scanning_active:
                # Only process if we also have recent laser data
                if self.current_scan and abs((self.last_image_time - self.current_scan.header.stamp).to_sec()) < 0.1:
                     self.process_current_data()
                # else:
                #     rospy.logdebug("Waiting for synchronized laser scan...")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
        except Exception as e:
             rospy.logerr(f"Error in image_callback: {e}")


    def laser_callback(self, data):
        """处理激光雷达数据"""
        self.current_scan = data
        # Processing is triggered by image_callback when both are available

    # robot_pose_callback is less critical now with TF, could be removed if TF is reliable
    # def robot_pose_callback(self, msg):
    #     """Update robot position (less used now)"""
    #     # self.robot_position = (0, 0)
    #     pass

    def scan_callback(self, msg):
        """接收扫描命令"""
        if msg.data and not self.scanning_active:
            rospy.loginfo("Starting box scan with number recognition")
            # Clear previous scan results? Optional, depends on desired behavior
            # self.detected_boxes = []
            # self.box_numbers = {}
            # self.box_positions = {}
            self.scanning_active = True
        elif not msg.data and self.scanning_active:
            rospy.loginfo("Stopping box scan")
            self.scanning_active = False

    def detect_numbers_from_image(self):
        """从图像中检测数字"""
        if self.current_image is None:
            rospy.logwarn("No current image available for detection.")
            return []

        # OCR using EasyOCR
        number_candidates = []  # List of (center_x, number, img_width)

        if EASYOCR_AVAILABLE and self.ocr_reader is not None:
            try:
                # 获取最小置信度阈值
                min_confidence = rospy.get_param('~min_confidence', 0.8)  # 默认值为 0.8

                results = self.ocr_reader.readtext(self.current_image, allowlist="0123456789") # Limit to digits 0-9

                # 添加日志输出，显示检测到的所有结果
                rospy.logdebug("EasyOCR raw results:")
                for detection in results:
                    text = detection[1]
                    confidence = detection[2]
                    rospy.logdebug(f"  - Text: '{text}', Confidence: {confidence:.2f}")

                for detection in results:
                    text = detection[1] # The text from the detection
                    confidence = detection[2]
                    if len(text) == 1 and text.isdigit() and confidence >= min_confidence: # Only accept single digit results
                        # Valid single digit detected
                        center_x = (detection[0][0][0] + detection[0][2][0]) / 2.0 # Center X pixel
                        rospy.loginfo(f"EasyOCR recognized: {text} at pixel x={center_x:.1f} with confidence {confidence:.2f}")
                        number_candidates.append((center_x, text, self.current_image.shape[1])) # Store center_x, number, image_width
                    else:
                        rospy.loginfo(f"EasyOCR raw output: '{text}', invalid or confidence too low.")

            except Exception as e:
                rospy.logerr(f"Error during EasyOCR: {e}")

        else:
             rospy.logwarn_throttle(10, "EasyOCR not available, or OCR Reader not initialized.")
             return []
        return number_candidates

    def estimate_box_center_position(self, image_center_x, image_width):
        """
        Estimate the box's global center position, knowing the detected digit location.
        It first estimates the digit's global position, then projects along a local normal vector by half the box size.
        Returns (x, y) in the map frame or None if estimation fails.
        """
        rospy.loginfo("estimate_box_center_position: Starting...")

        digit_position = self.estimate_box_global_position(image_center_x, image_width)  # Use existing function
        if digit_position is None:
            rospy.logwarn("estimate_box_center_position: digit_position is None, returning None.")
            return None

        try:
            # 1. Get a second point near the digit by perturbing image_center_x slightly
            delta_x = 2  # Pixel offset (adjust this value as needed)
            image_center_x_neighbor = image_center_x + delta_x
            if image_center_x_neighbor > image_width:
                image_center_x_neighbor = image_width # Limit on max bound

            neighbor_position = self.estimate_box_global_position(image_center_x_neighbor, image_width) # Use existing function

            if neighbor_position is None:
                rospy.logwarn("estimate_box_center_position: neighbor_position is None, can't estimate normal. Returning None.")
                return None

            # 2. Calculate a local normal vector
            dx = neighbor_position[0] - digit_position[0]
            dy = neighbor_position[1] - digit_position[1]

            # Normalize vector (if dx and dy are near zero, this will be NaN)
            magnitude = math.sqrt(dx**2 + dy**2)
            if magnitude < 0.001:  # Avoid division by near-zero
                rospy.logwarn("estimate_box_center_position: Magnitude too small, can't estimate normal. Returning None.")
                return None

            normal_x = -dy / magnitude  # Normal vector (pointing outwards from the surface)
            normal_y = dx / magnitude

            # 3. Project to the *center* of the box
            box_center_x = digit_position[0] + (self.known_box_size / 2.0) * normal_x
            box_center_y = digit_position[1] + (self.known_box_size / 2.0) * normal_y
            box_center = (box_center_x, box_center_y)
            rospy.loginfo(f"Estimated box CENTER position from digit: ({box_center[0]:.2f}, {box_center[1]:.2f}) m")

            return box_center

        except Exception as e:
            rospy.logerr(f"Error calculating box center offset: {e}")
            return None

    def estimate_box_global_position(self, image_center_x, image_width):
        """
        Estimate the box's global position using laser scan data corresponding
        to the box's horizontal position in the image and TF.
        Returns (x, y) in the map frame or None if estimation fails.
        This now estimates position of the *digit itself*
        """
        if self.current_scan is None or image_width == 0:
            rospy.logwarn("Missing laser scan or zero image width for position estimation.")
            return None

        try:
            # --- 1. Calculate angle of the box center in the laser scan ---
            # Horizontal angle corresponding to the pixel center x
            # Assumes camera center pixel (image_width / 2) corresponds to 0 angle
            pixel_offset = image_center_x - image_width / 2.0
            angle_offset_rad = (pixel_offset / image_width) * self.camera_fov_horizontal_rad

            # Target angle relative to the laser's forward direction (usually angle 0)
            target_angle_rad = -angle_offset_rad # Negative sign depends on camera vs laser frame conventions

            # Find the closest angle in the laser scan data
            angle_min = self.current_scan.angle_min
            angle_inc = self.current_scan.angle_increment
            num_ranges = len(self.current_scan.ranges)

            target_index = int((target_angle_rad - angle_min) / angle_inc)

            # Clamp index to valid range
            target_index = max(0, min(num_ranges - 1, target_index))

            # --- 2. Get distance from laser scan ---
            distance = self.current_scan.ranges[target_index]
            range_min = self.current_scan.range_min
            range_max = self.current_scan.range_max

            if not (range_min < distance < range_max) or math.isinf(distance) or math.isnan(distance):
                rospy.logwarn(f"Invalid laser distance ({distance:.2f}m) at index {target_index} for angle {math.degrees(target_angle_rad):.1f} deg.")
                return None

            # --- 3. Calculate position in laser frame ---
            # Coordinates relative to the laser sensor frame
            x_laser = distance * math.cos(angle_min + target_index * angle_inc)
            y_laser = distance * math.sin(angle_min + target_index * angle_inc)

            # --- 4. Transform to map frame using TF ---
            # Create a PointStamped message in the laser's frame
            point_laser = PointStamped()
            point_laser.header.stamp = self.current_scan.header.stamp # Use laser timestamp
            point_laser.header.frame_id = self.laser_frame_id
            point_laser.point.x = x_laser
            point_laser.point.y = y_laser
            point_laser.point.z = 0 # Assume box base is on the same Z plane

            # Wait for transform and transform the point
            self.tf_listener.waitForTransform(self.map_frame_id,
                                              self.laser_frame_id,
                                              point_laser.header.stamp, # Use timestamp from point
                                              rospy.Duration(0.5)) # Timeout

            point_map = self.tf_listener.transformPoint(self.map_frame_id, point_laser)

            rospy.logdebug(f"Estimated digit global position: ({point_map.point.x:.2f}, {point_map.point.y:.2f}) "
                          f"from laser dist {distance:.2f}m at angle {math.degrees(target_angle_rad):.1f}deg")

            return (point_map.point.x, point_map.point.y)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Error transforming point from {self.laser_frame_id} to {self.map_frame_id}: {e}")
            return None
        except IndexError:
             rospy.logerr(f"Laser scan index out of bounds.")
             return None
        except Exception as e:
            rospy.logerr(f"Error estimating box global position: {e}")
            return None


    def update_box_count(self):
        """更新箱子计数并发布"""
        # Count unique boxes based on stored positions
        box_count = len(self.detected_boxes)
        count_msg = Int32(data=box_count)
        self.box_count_pub.publish(count_msg)

        # Publish box numbers/types (count of each number)
        types_msg = String(data=json.dumps(self.box_numbers))
        self.box_types_pub.publish(types_msg)

        # Publish box positions (dictionary: {'number': [x, y]})
        positions_msg = String(data=json.dumps(self.box_positions))
        self.box_positions_pub.publish(positions_msg)

        rospy.loginfo(f"Published - Box count: {box_count}, Numbers: {self.box_numbers}, Positions: {self.detected_numbers_list}")
        # Note: Returning box_count isn't used elsewhere, so removed return


    def find_least_common_number(self):
        """找出出现次数最少的数字并发布"""
        if not self.box_numbers:
            rospy.loginfo("No box numbers recorded yet.")
            # Publish empty or None? Let's publish empty string.
            self.least_common_number_pub.publish(String(data=""))
            return None

        # Find the minimum count
        min_count = min(self.box_numbers.values())
        # Find all numbers with that minimum count
        least_common_numbers = [num for num, count in self.box_numbers.items() if count == min_count]

        # If tie, maybe pick the smallest number? Or just the first one.
        least_common_num = sorted(least_common_numbers)[0] # Pick smallest number in case of tie

        least_common_msg = String(data=str(least_common_num))
        self.least_common_number_pub.publish(least_common_msg)

        rospy.loginfo(f"Least common box number: {least_common_num} (Count: {min_count})")
        return least_common_num


    def is_new_box(self, position, box_number):
        """判断是否是新箱子 (based on global position)"""
        rospy.loginfo(f"is_new_box: Checking if box {box_number} at position {position} is new...") # 函数开始
        if position is None:
            rospy.logwarn("is_new_box: Position is None, returning False.")
            return False

        rospy.logdebug("is_new_box: Iterating through existing boxes...") # 准备开始循环
        for existing_pos, existing_num in self.detected_numbers_list:
            rospy.loginfo(f"is_new_box: Comparing with existing box {existing_num} at {existing_pos}")
            dist = math.sqrt((position[0] - existing_pos[0]) ** 2 + (position[1] - existing_pos[1]) ** 2)
            rospy.logdebug(f"is_new_box: Distance to existing box: {dist:.2f} m")
            if dist < self.min_box_dist:
                rospy.logwarn(f"is_new_box: Box is too close to existing box, returning False.")
                return False
        rospy.loginfo("is_new_box: No close existing box found, returning True.") # 循环结束， 没找到
        # No close box found, it's new
        return True

    # estimate_global_position is replaced by estimate_box_global_position

    def process_current_data(self):
        """处理当前的传感器数据检测箱子和数字"""
        if self.current_image is None or self.current_scan is None:
            rospy.logdebug("Waiting for current image and scan data...")
            return

        boxes_added_or_updated = False

        # --- 1. Detect numbers from Image (gets number and pixel location) ---
        number_candidates = self.detect_numbers_from_image() # List: [(center_x, number, img_width)]

        for center_x, box_number, img_width in number_candidates:

            # --- 2. Estimate Global Position ---
            box_center_position = self.estimate_box_center_position(center_x, img_width)

            if box_center_position:
                rospy.loginfo(f"process_current_data: Estimated box center position: {box_center_position}") # Log:估计了位置
                # --- 3. Check if it's a new box ---
                is_new = self.is_new_box(box_center_position, box_number)
                rospy.loginfo(f"process_current_data: is_new is: {is_new}") # Log: is_new的判断结果

                if is_new:
                    rospy.loginfo(f"process_current_data: New box for number {box_number} at {box_center_position[0]:.2f}, {box_center_position[1]:.2f}")
                    # Only add the new box number if it hasn't been seen
                    self.detected_numbers_list.append((box_center_position, box_number))
                    self.detected_boxes.append(box_center_position)
                    self.box_numbers[box_number] = self.box_numbers.get(box_number, 0) + 1
                    rospy.loginfo(f"process_current_data: Box positions after adding: {self.box_positions}") # Log: 添加到box_positions 之后的结果
                    boxes_added_or_updated = True
                else:
                    rospy.loginfo("process_current_data: It is not a new box")
                    boxes_added_or_updated = False

                #---4.Publish Pose--------------------------------------------------
                smoothed_position = self.update_moving_average(box_number, box_center_position)
                self.publish_target_pose(smoothed_position)

        # --- 4. Update and Publish if changes occurred ---
        if boxes_added_or_updated:
            rospy.loginfo("process_current_data: Box data updated, publishing...")
            self.update_box_count()
            self.find_least_common_number() # Recalculate least common


    def update_moving_average(self, box_number, new_position):
        """Update the moving average of the box position."""
        if box_number not in self.previous_poses:
            # If it's the first time seeing this number, just set the position
            self.previous_poses[box_number] = new_position
            rospy.loginfo(f"Initialize pose for number: {box_number} with {new_position}")
            return new_position  # Return directly to avoid calculations on initial data

        # Calculate moving average
        old_position = self.previous_poses[box_number]
        new_x = self.alpha * new_position[0] + (1 - self.alpha) * old_position[0]
        new_y = self.alpha * new_position[1] + (1 - self.alpha) * old_position[1]
        smoothed_position = (new_x, new_y)
        self.previous_poses[box_number] = smoothed_position  # Update the tracked position
        rospy.loginfo(f"Update pose for number: {box_number} to {smoothed_position}")

        return smoothed_position

    def publish_target_pose(self, position):
        """Publish the target pose."""
        if position is None:
            rospy.logwarn("No target box pose to publish.")
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.map_frame_id
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = 0  # Assuming boxes are on the ground plane
        pose_msg.pose.orientation.w = 1.0  # Default orientation (no rotation)

        self.target_pose_pub.publish(pose_msg)
        rospy.loginfo(f"Published target pose: {position}")

    def run(self):
        """主循环"""
        rate = rospy.Rate(5)  # Process data at a reasonable rate (e.g., 5Hz)

        while not rospy.is_shutdown():
            # Processing is now mainly done in callbacks when data arrives and scanning is active
            # This loop can be used for periodic checks or background tasks if needed.
            # We can periodically republish state even if no new boxes detected:
            # self.update_box_count()
            # self.find_least_common_number()
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = BoxDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as main_e:
         rospy.logfatal(f"Unhandled exception in main: {main_e}")