#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.br = CvBridge()
        self.model = YOLO('yolov8n.pt')  # o tu modelo entrenado
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 1)
        self.get_logger().info('YOLO detector listo y suscrito a /camera/image_raw')

    def callback(self, msg: Image):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = f'{self.model.names[cls]} {conf:.2f}'
            cv2.rectangle(frame, (x1,y1),(x2,y2),(0,255,0),2)
            cv2.putText(frame, label, (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        cv2.imshow('YOLO detections', frame)
        key = cv2.waitKey(1)
        if key == 27:  # ESC cierra
            rclpy.shutdown()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

