#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Parámetros
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('calib_file', '')

        dev      = self.get_parameter('device_id').value
        w        = self.get_parameter('width').value
        h        = self.get_parameter('height').value
        fps      = self.get_parameter('fps').value
        frame_id = self.get_parameter('camera_frame').value
        calib    = self.get_parameter('calib_file').value
        calib    = os.path.expanduser(calib)  # Expande “~”

        # Publicadores
        self.img_pub  = self.create_publisher(Image,      'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.br       = CvBridge()

        # Configura VideoCapture
        self.cap = cv2.VideoCapture(dev, cv2.CAP_ANY)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        # Carga CameraInfo con valores por defecto
        self.cam_info = CameraInfo()
        self.cam_info.width  = w
        self.cam_info.height = h
        if calib and os.path.isfile(calib):
            with open(calib, 'r') as f:
                data = yaml.safe_load(f)
            self.cam_info.k = data['camera_matrix']['data']
            self.cam_info.d = data['distortion_coefficients']['data']
            self.cam_info.r = data['rectification_matrix']['data']
            self.cam_info.p = data['projection_matrix']['data']
        self.cam_info.header.frame_id = frame_id

        # Timer de publicación
        period = 1.0 / fps
        self.create_timer(period, self.timer_callback)
        self.get_logger().info(f'CameraNode listo: publicando camera/image_raw @ {fps} FPS')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Fallo al capturar frame')
            return

        now = self.get_clock().now().to_msg()
        # Mensaje Image
        img_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.cam_info.header.frame_id
        # Mensaje CameraInfo
        self.cam_info.header.stamp = now

        # Publica ambos
        self.img_pub.publish(img_msg)
        self.info_pub.publish(self.cam_info)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    node.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

