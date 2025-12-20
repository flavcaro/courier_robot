#!/usr/bin/env python3
"""
AprilTag Localizer Node

This node detects AprilTags in camera images and publishes pose corrections
to help the robot localize itself in the environment.

AprilTag positions (known landmarks):
- Coordinate system: cell (x, y) maps to world position (x+0.5, y+0.5)
- Tag ID 0: Start position at cell (0,0) -> center (0.5, 0.5)
- Tag ID 1: Goal position at cell (2,4) -> south edge
- etc.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
import cv2
import math
from cv_bridge import CvBridge

# Try to import pupil_apriltags, fall back to cv2.aruco
try:
    from pupil_apriltags import Detector
    USE_PUPIL_APRILTAGS = True
except ImportError:
    USE_PUPIL_APRILTAGS = False


class AprilTagLocalizer(Node):
    """
    Detects AprilTags and provides localization corrections.
    """
    
    def __init__(self):
        super().__init__('apriltag_localizer')
        
        # Known AprilTag positions in WORLD frame (x, y, z, yaw)
        # Tags are flat against walls/obstacles, facing INTO the room
        # 12 wall tags + 16 obstacle tags = 28 total
        self.tag_positions = {
            # South wall (y = 0.01-0.02) - tags face NORTH (+Y)
            0: {'x': 0.5, 'y': 0.02, 'z': 0.15, 'yaw': 1.5708},
            1: {'x': 2.5, 'y': 0.02, 'z': 0.15, 'yaw': 1.5708},
            2: {'x': 4.5, 'y': 0.02, 'z': 0.15, 'yaw': 1.5708},
            # North wall (y = 4.98-4.99) - tags face SOUTH (-Y)
            3: {'x': 0.5, 'y': 4.98, 'z': 0.15, 'yaw': -1.5708},
            4: {'x': 2.5, 'y': 4.98, 'z': 0.15, 'yaw': -1.5708},
            5: {'x': 4.5, 'y': 4.98, 'z': 0.15, 'yaw': -1.5708},
            # West wall (x = 0.01-0.02) - tags face EAST (+X)
            6: {'x': 0.02, 'y': 0.5, 'z': 0.15, 'yaw': 0.0},
            7: {'x': 0.02, 'y': 2.5, 'z': 0.15, 'yaw': 0.0},
            8: {'x': 0.02, 'y': 4.5, 'z': 0.15, 'yaw': 0.0},
            # East wall (x = 4.98-4.99) - tags face WEST (-X)
            9: {'x': 4.98, 'y': 0.5, 'z': 0.15, 'yaw': 3.14159},
            10: {'x': 4.98, 'y': 2.5, 'z': 0.15, 'yaw': 3.14159},
            11: {'x': 4.98, 'y': 4.5, 'z': 0.15, 'yaw': 3.14159},
            # Obstacle (1,1) at world (1.5, 1.5) - 4 sides
            12: {'x': 1.5, 'y': 1.05, 'z': 0.25, 'yaw': 1.5708},   # South, faces N
            13: {'x': 1.5, 'y': 1.95, 'z': 0.25, 'yaw': -1.5708},  # North, faces S
            14: {'x': 1.05, 'y': 1.5, 'z': 0.25, 'yaw': 0.0},      # West, faces E
            15: {'x': 1.95, 'y': 1.5, 'z': 0.25, 'yaw': 3.14159},  # East, faces W
            # Obstacle (1,2) at world (2.5, 1.5) - 4 sides
            16: {'x': 2.5, 'y': 1.05, 'z': 0.25, 'yaw': 1.5708},
            17: {'x': 2.5, 'y': 1.95, 'z': 0.25, 'yaw': -1.5708},
            18: {'x': 2.05, 'y': 1.5, 'z': 0.25, 'yaw': 0.0},
            19: {'x': 2.95, 'y': 1.5, 'z': 0.25, 'yaw': 3.14159},
            # Obstacle (3,1) at world (1.5, 3.5) - 4 sides
            20: {'x': 1.5, 'y': 3.05, 'z': 0.25, 'yaw': 1.5708},
            21: {'x': 1.5, 'y': 3.95, 'z': 0.25, 'yaw': -1.5708},
            22: {'x': 1.05, 'y': 3.5, 'z': 0.25, 'yaw': 0.0},
            23: {'x': 1.95, 'y': 3.5, 'z': 0.25, 'yaw': 3.14159},
            # Obstacle (3,3) at world (3.5, 3.5) - 4 sides
            24: {'x': 3.5, 'y': 3.05, 'z': 0.25, 'yaw': 1.5708},
            25: {'x': 3.5, 'y': 3.95, 'z': 0.25, 'yaw': -1.5708},
            26: {'x': 3.05, 'y': 3.5, 'z': 0.25, 'yaw': 0.0},
            27: {'x': 3.95, 'y': 3.5, 'z': 0.25, 'yaw': 3.14159},
        }
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.fx = 277.0  # Default focal length
        self.fy = 277.0
        self.cx = 160.0  # Default principal point
        self.cy = 120.0
        
        # Tag size in meters (updated to match world_spawner)
        self.tag_size = 0.15
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # AprilTag detector setup
        if USE_PUPIL_APRILTAGS:
            self.detector = Detector(
                families='tag36h11',
                nthreads=1,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
            )
            self.get_logger().info('Using pupil_apriltags detector')
        else:
            # Use OpenCV ArUco as fallback (similar to AprilTag)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.get_logger().info('Using OpenCV ArUco detector (AprilTag 36h11)')
        
        # Current odometry
        self.current_odom = None
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/apriltag_pose', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/apriltag_debug', 10)
        
        # Detection counter for logging
        self.detection_count = 0
        # Throttle logging to avoid flooding the console
        self._last_detection_log = 0.0
        self._detection_log_interval = 1.0  # seconds
        # Throttle debug image publication
        self._last_debug_img_time = 0.0
        self._debug_img_interval = 0.5  # seconds
        
        self.get_logger().info('AprilTag Localizer started')
        self.get_logger().info(f'Known tags: {list(self.tag_positions.keys())}')
        
    def camera_info_callback(self, msg):
        """Update camera intrinsics from camera info message."""
        if self.camera_matrix is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ], dtype=np.float32)
            self.dist_coeffs = np.array(msg.d, dtype=np.float32) if len(msg.d) > 0 else np.zeros(5)
            self.get_logger().info(f'Camera calibration received: fx={self.fx:.1f}, fy={self.fy:.1f}')
    
    def odom_callback(self, msg):
        """Store current odometry."""
        self.current_odom = msg
        
    def image_callback(self, msg):
        """Process camera image to detect AprilTags."""
        if self.camera_matrix is None:
            self.get_logger().warn('No camera_info yet — skipping image processing')
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Convert to grayscale for detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTags
        detections = self.detect_tags(gray)
        
        if detections:
            self.process_detections(detections, cv_image, msg.header.stamp)
            
        # Publish debug image
        self.publish_debug_image(cv_image, detections, msg.header.stamp)
    
    def detect_tags(self, gray_image):
        """Detect AprilTags in grayscale image."""
        detections = []
        
        if USE_PUPIL_APRILTAGS:
            # Use pupil_apriltags
            results = self.detector.detect(
                gray_image,
                estimate_tag_pose=True,
                camera_params=[self.fx, self.fy, self.cx, self.cy],
                tag_size=self.tag_size
            )
            for r in results:
                detections.append({
                    'id': r.tag_id,
                    'corners': r.corners,
                    'center': r.center,
                    'pose_R': r.pose_R if hasattr(r, 'pose_R') else None,
                    'pose_t': r.pose_t if hasattr(r, 'pose_t') else None,
                })
        else:
            # Use OpenCV ArUco
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray_image)
            
            if ids is not None:
                for i, tag_id in enumerate(ids.flatten()):
                    corner = corners[i][0]
                    center = np.mean(corner, axis=0)
                    
                    # Estimate pose using solvePnP
                    pose_R, pose_t = self.estimate_pose_opencv(corner)
                    
                    detections.append({
                        'id': int(tag_id),
                        'corners': corner,
                        'center': center,
                        'pose_R': pose_R,
                        'pose_t': pose_t,
                    })
        
        return detections
    
    def estimate_pose_opencv(self, corners):
        """Estimate tag pose using OpenCV solvePnP."""
        if self.camera_matrix is None:
            # Use default camera matrix
            self.camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ], dtype=np.float32)
            self.dist_coeffs = np.zeros(5, dtype=np.float32)
        
        # 3D points of tag corners (in tag frame)
        half_size = self.tag_size / 2
        object_points = np.array([
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0]
        ], dtype=np.float32)
        
        # 2D points in image
        image_points = corners.astype(np.float32)
        
        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points,
            self.camera_matrix, self.dist_coeffs
        )
        
        if success:
            R, _ = cv2.Rodrigues(rvec)
            return R, tvec
        return None, None
    
    def process_detections(self, detections, cv_image, stamp):
        """Process detected tags and compute robot pose."""
        for detection in detections:
            tag_id = detection['id']
            
            if tag_id not in self.tag_positions:
                self.get_logger().warn(f'Unknown tag ID: {tag_id}')
                continue
            
            pose_t = detection['pose_t']
            pose_R = detection['pose_R']
            
            if pose_t is None or pose_R is None:
                continue
            
            # Get tag's known world position
            tag_world = self.tag_positions[tag_id]
            
            # Distance to tag
            distance = np.linalg.norm(pose_t)
            
            # Compute robot position based on tag detection
            # The tag gives us camera-to-tag transform
            # We need to compute world-to-robot transform
            
            # Simplified: use tag position and estimated distance
            # More accurate would require full 6DOF pose estimation
            
            robot_pose = self.compute_robot_pose(tag_world, pose_t, pose_R)
            
            if robot_pose is not None:
                # Throttled info-level logging to keep console readable
                now = self.get_clock().now().nanoseconds / 1e9
                self.detection_count += 1
                if (now - self._last_detection_log) >= self._detection_log_interval or (self.detection_count % 10 == 0):
                    self.get_logger().info(
                        f'Detected Tag {tag_id} dist={distance:.2f}m -> '
                        f'robot_est=({robot_pose[0]:.2f},{robot_pose[1]:.2f},{math.degrees(robot_pose[2]):.1f}deg)'
                    )
                    self._last_detection_log = now
                self.publish_pose(robot_pose, tag_id, distance, stamp)
    
    def compute_robot_pose(self, tag_world, pose_t, pose_R):
        """
        Compute robot world pose from tag detection.
        
        Args:
            tag_world: Dict with tag's world position (x, y, z, yaw)
            pose_t: Translation from camera to tag
            pose_R: Rotation from camera to tag
        
        Returns:
            Tuple (x, y, yaw) of robot in world frame
        """
        # Camera-to-tag translation
        tx, ty, tz = pose_t.flatten()
        
        # Distance from camera to tag
        distance = np.sqrt(tx**2 + ty**2 + tz**2)
        
        # Angle to tag in camera frame (horizontal)
        angle_to_tag = np.arctan2(tx, tz)
        
        # Tag's world position and orientation
        tag_x = tag_world['x']
        tag_y = tag_world['y']
        tag_yaw = tag_world['yaw']  # Direction tag is facing
        
        # Robot is behind the tag (from tag's perspective)
        # Compute robot position relative to tag
        
        # Simplified estimation:
        # - Robot is 'distance' away from tag
        # - Robot is looking at the tag
        
        # The tag faces a certain direction (tag_yaw)
        # Robot sees the tag, so robot is roughly opposite to tag_yaw
        
        # Robot orientation: facing the tag means robot_yaw ≈ tag_yaw + π
        robot_yaw = tag_yaw + np.pi - angle_to_tag
        
        # Robot position: distance away from tag in the direction robot is facing
        robot_x = tag_x - distance * np.cos(robot_yaw)
        robot_y = tag_y - distance * np.sin(robot_yaw)
        
        # Normalize yaw
        while robot_yaw > np.pi:
            robot_yaw -= 2 * np.pi
        while robot_yaw < -np.pi:
            robot_yaw += 2 * np.pi
        
        return (robot_x, robot_y, robot_yaw)
    
    def publish_pose(self, robot_pose, tag_id, distance, stamp):
        """Publish robot pose estimate."""
        x, y, yaw = robot_pose
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        # This pose is an estimate in the world/map frame
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose_msg.pose.pose.orientation.z = np.sin(yaw / 2)
        pose_msg.pose.pose.orientation.w = np.cos(yaw / 2)
        
        # Covariance based on distance (further = less certain)
        base_cov = 0.1 * distance
        pose_msg.pose.covariance[0] = base_cov  # x
        pose_msg.pose.covariance[7] = base_cov  # y
        pose_msg.pose.covariance[35] = 0.1  # yaw
        
        self.pose_pub.publish(pose_msg)
        # Also broadcast a map->odom correction transform so the rest of the
        # system (AMCL/Nav2) can use this as a pose correction.
        try:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = 0.0
            qz = float(np.sin(yaw / 2.0))
            qw = float(np.cos(yaw / 2.0))
            # Set quaternion (x,y) = 0 because we only rotate around Z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)
            self.get_logger().debug(f'Broadcasted map->odom from tag {tag_id}: ({x:.2f},{y:.2f},{math.degrees(yaw):.1f}deg)')
        except Exception as e:
            self.get_logger().error(f'Failed to broadcast TF map->odom: {e}')
    
    def publish_debug_image(self, cv_image, detections, stamp):
        """Publish debug image with detected tags highlighted."""
        # Throttle debug image publishing to avoid flooding topic and UI
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_debug_img_time) < self._debug_img_interval:
            return
        debug_img = cv_image.copy()
        
        for detection in detections:
            corners = detection['corners'].astype(np.int32)
            center = tuple(detection['center'].astype(np.int32))
            tag_id = detection['id']
            
            # Draw tag outline
            cv2.polylines(debug_img, [corners], True, (0, 255, 0), 2)
            
            # Draw center
            cv2.circle(debug_img, center, 5, (0, 0, 255), -1)
            
            # Draw tag ID
            cv2.putText(debug_img, f'ID: {tag_id}', 
                       (center[0] - 20, center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Draw distance if available
            if detection['pose_t'] is not None:
                distance = np.linalg.norm(detection['pose_t'])
                cv2.putText(debug_img, f'{distance:.2f}m',
                           (center[0] - 20, center[1] + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # Convert back to ROS message
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header.stamp = stamp
            debug_msg.header.frame_id = 'camera_link'
            self.debug_image_pub.publish(debug_msg)
            self._last_debug_img_time = now
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = AprilTagLocalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Defensive cleanup: destroy node if still present and shutdown rclpy.
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            # rclpy.shutdown() can raise if shutdown was already called elsewhere
            rclpy.shutdown()
        except Exception:
            # Ignore shutdown errors (double shutdown may occur during tests/launch)
            pass


if __name__ == '__main__':
    main()
