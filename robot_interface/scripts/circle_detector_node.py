#!/usr/bin/env python3

import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class AutoContourDetectionNode(Node):
    def __init__(self):
        super().__init__('auto_contour_detection_node')

        # สร้าง CvBridge สำหรับแปลง ROS Image <-> OpenCV Image
        self.bridge = CvBridge()

        # สมัครสมาชิก topic กล้องดิบ
        self.image_subscription = self.create_subscription(
            Image,
            '/v4l/camera/image_raw',
            self.image_callback,
            10
        )

        # สมัครสมาชิกเพื่อรับคำสั่ง Auto Find Contour
        self.auto_find_subscription = self.create_subscription(
            Bool,
            '/circle_detection/auto_find',
            self.auto_find_callback,
            10
        )

        # ประกาศหัวข้อใหม่เพื่อพ่นภาพหลังจากตรวจจับวงรี
        self.publisher = self.create_publisher(Image, '/circle_detection/output_image', 10)

        # ตัวแปรเก็บสถานะว่าได้รับคำสั่ง Auto Find หรือยัง
        self.auto_find_enabled = False

        # สำเนาของภาพต้นฉบับที่ไม่มีวงรีวาดไว้
        self.original_image = None

    def auto_find_callback(self, msg):
        """ฟังก์ชันที่ถูกเรียกเมื่อได้รับคำสั่งจาก topic auto_find"""
        if msg.data:
            self.auto_find_enabled = True
            self.get_logger().info("Auto find contour command received!")

    def image_callback(self, msg):
        # ถ้าไม่ได้เปิดใช้งาน Auto Find ให้ข้ามขั้นตอนการตรวจจับวงรี
        if not self.auto_find_enabled:
            return

        # แปลง ROS Image เป็น OpenCV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # เก็บสำเนาของภาพต้นฉบับไว้เพื่อนำไปใช้ลบวงรีในลูปถัดไป
        self.original_image = cv_image.copy()

        # ใช้ HSV กรองเฉพาะสีเขียว
        mask = self.filter_green(cv_image)

        # หาขอบเขตของภาพและวาดขอบเขตที่ใหญ่ที่สุด
        cv_image = self.auto_find_largest_contour(cv_image, mask)

        # แปลงภาพ OpenCV กลับเป็น ROS Image และพ่นไปที่ topic
        output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher.publish(output_msg)

        # Reset auto find flag หลังจากตรวจจับวงรีเสร็จ
        self.auto_find_enabled = False

    def filter_green(self, image):
        """ฟังก์ชันกรองเฉพาะส่วนของภาพที่เป็นสีเขียว"""
        # แปลงภาพเป็น HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # กำหนดช่วงของสีเขียวใน HSV
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])

        # สร้างหน้ากาก (mask) สำหรับสีเขียว
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # ทำการขยายและย่อพื้นที่เพื่อกำจัด noise ที่เล็ก ๆ
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        return mask  # Return the binary mask

    def auto_find_largest_contour(self, image, mask):
        """ฟังก์ชันค้นหาและวาดคอนทัวร์ที่ใหญ่ที่สุด"""
        # หาคอนทัวร์จากหน้ากาก
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # หาคอนทัวร์ที่มีพื้นที่ใหญ่ที่สุด
            largest_contour = max(contours, key=cv2.contourArea)

            # วาดคอนทัวร์ที่ใหญ่ที่สุดด้วยสีแดง
            cv2.drawContours(image, [largest_contour], -1, (0, 0, 255), 4)

        return image

def main(args=None):
    rclpy.init(args=args)
    node = AutoContourDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
