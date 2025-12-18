#!/usr/bin/env python3
"""
AprilTag Localizer Node

This node detects AprilTags in camera images and publishes pose corrections
to help the robot localize itself in the environment.

AprilTag positions (known landmarks):
- Tag ID 0: Start position at cell (0,0) -> (0.5, 0.5)
- Tag ID 1: Goal position at cell (4,0) -> (0.5, 4.5)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
import cv2
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
        
        # Known AprilTag positions in world frame (x, y, z, yaw)
        # These correspond to where the tags are placed in the simulation
        self.tag_positions = {
            0: {'x': 0.5, 'y': 0.2, 'z': 0.3, 'yaw': 1.5708},   # Start - facing east (+X)
            1: {'x': 0.5, 'y': 4.5, 'z': 0.3, 'yaw': 0.0},      # Goal - facing north (+Y)
        }
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.fx = 277.0  # Default focal length
        self.fy = 277.0
        self.cx = 160.0  # Default principal point
        self.cy = 120.0
        
        # Tag size in meters
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
                self.publish_pose(robot_pose, tag_id, distance, stamp)
                self.detection_count += 1
                
                if self.detection_count % 10 == 0:
                    self.get_logger().info(
                        f'Tag {tag_id} detected at distance {distance:.2f}m, '
                        f'robot at ({robot_pose[0]:.2f}, {robot_pose[1]:.2f})'
                    )
    
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
        pose_msg.header.frame_id = 'odom'
        
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
    
    def publish_debug_image(self, cv_image, detections, stamp):
        """Publish debug image with detected tags highlighted."""
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
