import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
import tf2_ros

class PoseDetectionNode(Node):
    def __init__(self):
        super().__init__('pose_detection_node')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)
        
        self.image_publisher = self.create_publisher(Image, '/camera/image_annotated', 10)
        self.pose_publisher = self.create_publisher(PoseArray, '/pose_landmarks', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        base_options = python.BaseOptions(model_asset_path='pose_landmarker.task')
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)
        
        self.latest_depth_image = None
    
    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
    def image_callback(self, msg):
        self.get_logger().info('Received image')
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv_image)
        
        detection_result = self.detector.detect(mp_image)
        annotated_image = self.draw_landmarks(cv_image, detection_result)
        
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='rgb8')
        self.image_publisher.publish(annotated_msg)
        
        self.publish_pose_landmarks(detection_result, msg.header)
        self.publish_tf(detection_result, msg.header, cv_image)
    
    def draw_landmarks(self, image, detection_result):
        annotated_image = image.copy()
        if detection_result.pose_landmarks:
            for landmark in detection_result.pose_landmarks[0]:
                x, y = int(landmark.x * image.shape[1]), int(landmark.y * image.shape[0])
                cv2.circle(annotated_image, (x, y), 5, (0, 255, 0), -1)
        return annotated_image
    
    def publish_pose_landmarks(self, detection_result, header):
        if not detection_result.pose_landmarks:
            return
        
        pose_array = PoseArray()
        pose_array.header = header
        
        for landmark in detection_result.pose_landmarks[0]:
            pose = Pose()
            pose.position.x = landmark.x
            pose.position.y = landmark.y
            pose.position.z = landmark.z
            pose_array.poses.append(pose)
        
        self.pose_publisher.publish(pose_array)
    
    def publish_tf(self, detection_result, header, image):
        if not detection_result.pose_landmarks or self.latest_depth_image is None:
            return
        
        landmarks = detection_result.pose_landmarks[0]
        hip_left = landmarks[23]
        hip_right = landmarks[24]
        
        x_l, y_l = int(hip_left.x * image.shape[1]), int(hip_left.y * image.shape[0])
        x_r, y_r = int(hip_right.x * image.shape[1]), int(hip_right.y * image.shape[0])
        
        depth_left = self.latest_depth_image[y_l, x_l] if 0 <= y_l < self.latest_depth_image.shape[0] and 0 <= x_l < self.latest_depth_image.shape[1] else 0
        depth_right = self.latest_depth_image[y_r, x_r] if 0 <= y_r < self.latest_depth_image.shape[0] and 0 <= x_r < self.latest_depth_image.shape[1] else 0
        
        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = 'person'
        transform.transform.translation.x = hip_left.x * depth_left
        transform.transform.translation.y = hip_left.y * depth_left
        transform.transform.translation.z = depth_left
        
        dx = hip_right.x - hip_left.x
        dy = hip_right.y - hip_left.y
        dz = hip_right.z - hip_left.z
        
        norm = np.sqrt(dx**2 + dy**2 + dz**2)
        if norm > 0:
            dx /= norm
            dy /= norm
            dz /= norm
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = dz
        transform.transform.rotation.w = np.sqrt(1 - dz**2)
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
