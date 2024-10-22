#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time

class HandDetectionNode(Node):
    def __init__(self):
        super().__init__('hand_detection_node')

        # Subscriber สำหรับรับภาพจาก topic
        self.subscription = self.create_subscription(
            Image,
            '/v4l2/camera/image_raw',
            self.image_callback,
            10)

        # Publisher สำหรับภาพที่ประมวลผลแล้ว
        self.image_pub = self.create_publisher(Image, '/processed_image', 10)

        # Publisher สำหรับ action ที่ตรวจพบ
        self.action_pub = self.create_publisher(String, '/robot/action', 10)

        # แปลงภาพจาก ROS 2 Image message เป็น OpenCV
        self.bridge = CvBridge()

        # MediaPipe Hands model
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

        # MediaPipe drawing utility
        self.mp_drawing = mp.solutions.drawing_utils

        # เก็บสถานะปัจจุบันและสถานะก่อนหน้า พร้อมตัวจับเวลา
        self.previous_gesture = None
        self.current_gesture = None
        self.gesture_change_time = None  # เวลาเริ่มต้นที่ gesture เปลี่ยน
        self.gesture_time_threshold = 0.5  # ตั้งเวลาเป็น 0.5 วินาทีสำหรับยืนยันการเปลี่ยน gesture

    def image_callback(self, msg):
        # รับภาพจาก topic และแปลงเป็นรูปแบบ OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # แปลงภาพเป็น RGB เพื่อให้ใช้กับ MediaPipe
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_image)

        if results.multi_hand_landmarks:
            # ใช้แค่มือข้างแรกที่ตรวจจับได้
            hand_landmarks = results.multi_hand_landmarks[0]

            # วาด landmark ลงบนภาพ
            self.mp_drawing.draw_landmarks(cv_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

            # ตรวจจับ gesture จากตำแหน่งของนิ้ว
            gesture = self.detect_gesture(hand_landmarks)

            # หาก gesture เปลี่ยน
            if gesture != self.current_gesture:
                self.gesture_change_time = time.time()  # จับเวลาการเปลี่ยน gesture
                self.current_gesture = gesture

            # หาก gesture ยังคงเหมือนเดิมและเวลาเกินกว่า threshold ที่กำหนด
            if self.gesture_change_time is not None and (time.time() - self.gesture_change_time) > self.gesture_time_threshold:
                if self.current_gesture != self.previous_gesture:
                    self.previous_gesture = self.current_gesture
                    print(f"Gesture confirmed: {self.current_gesture}")
                    self.gesture_change_time = None  # รีเซ็ตเวลาเมื่อ gesture ได้รับการยืนยัน

                    # ส่ง action ที่ตรวจพบ
                    self.action_pub.publish(String(data=self.current_gesture))

        if self.previous_gesture:
            cv2.putText(cv_image, self.previous_gesture, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # ส่งภาพที่ประมวลผลแล้ว
        processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(processed_image_msg)

    def detect_gesture(self, hand_landmarks):
        # Get relevant landmarks
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP]  # knuckle
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]  # knuckle
        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        middle_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]  # knuckle
        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        ring_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP]  # knuckle
        pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
        pinky_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP]  # knuckle

        landmarks = hand_landmarks.landmark
        # Function to calculate Euclidean distance between two points
        def distance(point1, point2):
            return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2) ** 0.5

        # ตรวจจับ gesture ต่างๆ
        if all([landmarks[i].y < landmarks[i+3].y for i in range(9, 20, 4)]) and landmarks[4].x > landmarks[2].x:
            return "attack"

        if (landmarks[8].y < landmarks[6].y and
                landmarks[12].y < landmarks[10].y and
                landmarks[16].y < landmarks[14].y and
                landmarks[20].y < landmarks[18].y and
                landmarks[4].x < landmarks[2].x):            
                return "stop"

        # เพิ่ม gesture อื่นๆ ที่นี่

        # turn right
        if thumb_tip.y < thumb_mcp.y and index_tip.x < index_mcp.x and ring_tip.y > middle_tip.y:
            return "right"

        # walk
        if thumb_tip.x < thumb_mcp.x and index_tip.y < index_mcp.y and ring_tip.y > index_tip.y and thumb_tip.y > index_tip.y:
            return "walk"

        # turn left
        if thumb_tip.y < thumb_mcp.y and index_tip.x > index_mcp.x and ring_tip.y > middle_tip.y:
            return "left"

        # turn back
        if index_tip.y > index_mcp.y and thumb_tip.x > thumb_mcp.x:
            return "back"

        return None


def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionNode()
    rclpy.spin(node)

    # ปิดการทำงานของ node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
