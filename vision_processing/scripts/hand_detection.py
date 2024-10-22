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
        super().__init__('hand_detection_node2')

        # Subscriber สำหรับรับภาพจาก topic
        self.subscription = self.create_subscription(
            Image,
            '/v4l3/camera/image_raw',
            self.image_callback,
            1)

        # Publisher สำหรับภาพที่ประมวลผลแล้ว
        self.image_pub = self.create_publisher(Image, '/processed_image_2', 1)
        self.ready_player1=False
        self.ready_player2=False
        # Publisher สำหรับ action ที่ตรวจพบ
        self.action_pub = self.create_publisher(String, '/robot/action2', 10)
        # Subscriber สำหรับตรวจสอบ ready state ของผู้เล่น 1 และ 2
        self.ready_player1_sub = self.create_subscription(
            Bool, 
            '/state/ready_player1', 
            self.ready_player1_callback, 
            10
        )

        self.ready_player2_sub = self.create_subscription(
            Bool, 
            '/state/ready_player2', 
            self.ready_player2_callback, 
            10
        )

        # แปลงภาพจาก ROS 2 Image message เป็น OpenCV
        self.bridge = CvBridge()

        # MediaPipe Hands model
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
       # MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.7, min_tracking_confidence=0.7)

        # MediaPipe drawing utility
        self.mp_drawing = mp.solutions.drawing_utils

        # เก็บสถานะปัจจุบันและสถานะก่อนหน้า พร้อมตัวจับเวลา
        self.previous_gesture = None
        self.current_gesture = None
        self.gesture_change_time = None  # เวลาเริ่มต้นที่ gesture เปลี่ยน
        self.gesture_time_threshold = 0.5  # ตั้งเวลาเป็น 0.5 วินาทีสำหรับยืนยันการเปลี่ยน gesture
    def ready_player1_callback(self, msg):
        """Callback สำหรับสถานะ ready ของผู้เล่น 1"""
        self.ready_player1 = msg.data
        #self.get_logger().info(f"Player 1 ready: {self.ready_player1}")

    def ready_player2_callback(self, msg):
        """Callback สำหรับสถานะ ready ของผู้เล่น 2"""
        self.ready_player2 = msg.data
        #self.get_logger().info(f"Player 2 ready: {self.ready_player2}")

    def image_callback(self, msg):
        # รับภาพจาก topic และแปลงเป็นรูปแบบ OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # แปลงภาพเป็น RGB เพื่อให้ใช้กับ MediaPipe
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pose_results = self.pose.process(rgb_image)
        
        if pose_results.pose_landmarks:
            # ปรับขนาดและสีของเส้นที่วาดโครงร่าง
            self.mp_drawing.draw_landmarks(
                cv_image, 
                pose_results.pose_landmarks, 
                self.mp_pose.POSE_CONNECTIONS,
                self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=4, circle_radius=5),  # สีเขียวหนา 4 px สำหรับ landmark
                self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=3)  # สีฟ้าสำหรับการเชื่อมต่อระหว่างจุด
            )

        results = self.hands.process(rgb_image)
        # ตรวจจับมือ
        if results.multi_hand_landmarks:
            # ใช้แค่มือข้างแรกที่ตรวจจับได้
            hand_landmarks = results.multi_hand_landmarks[0]

           # วาด landmark ลงบนภาพ และปรับขนาดเส้น
            self.mp_drawing.draw_landmarks(
                cv_image,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=4, circle_radius=5),  # ขนาดเส้นและจุด
                self.mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2))  # ขนาดเส้นของ connection

            # ตรวจจับ gesture จากตำแหน่งของนิ้ว
            gesture = self.detect_gesture(hand_landmarks)

            # หาก gesture เปลี่ยน
            if gesture != self.current_gesture:
                self.gesture_change_time = time.time()  # จับเวลาการเปลี่ยน gesture
                self.current_gesture = gesture

            # หาก gesture ยังคงเหมือนเดิมและเวลาเกินกว่า threshold ที่กำหนด
            if self.gesture_change_time is not None and (time.time() - self.gesture_change_time) > self.gesture_time_threshold:
                if self.current_gesture != self.previous_gesture and self.current_gesture is not None:
                    self.previous_gesture = self.current_gesture
                    print(f"Gesture confirmed: {self.current_gesture}")
                    self.gesture_change_time = None  # รีเซ็ตเวลาเมื่อ gesture ได้รับการยืนยัน

                    # ส่ง action ที่ตรวจพบ เฉพาะเมื่อผู้เล่นทั้งสอง ready
                    if self.ready_player1 and self.ready_player2:
                        self.action_pub.publish(String(data=self.current_gesture))
                    else:
                        print("Players not ready, action not published.")

        if self.previous_gesture:
                    # วาดกรอบและตัวหนังสือซ้ายบน
                    text_size = cv2.getTextSize(self.previous_gesture, cv2.FONT_HERSHEY_SIMPLEX, 2, 4)[0]  # ขนาดของข้อความ
                    text_x, text_y = 50, 50
                    cv2.rectangle(cv_image, (text_x, text_y - text_size[1] - 10), (text_x + text_size[0] + 10, text_y + 10), (0, 0, 0), -1)  # กรอบสีดำ
                    cv2.putText(cv_image, self.previous_gesture, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)  # ข้อความขนาดใหญ่

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
