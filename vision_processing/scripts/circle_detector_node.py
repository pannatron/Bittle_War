#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import pygame
from threading import Thread

class VisionProcessingNode(Node):
    def __init__(self):
        super().__init__('vision_processing_node')

        self.bridge = CvBridge()
        self.is_ready_player1 = False
        self.is_ready_player2 = False
        self.was_go_displayed = False
        self.countdown_active = False
        self.countdown_start_time = None
        self.go_display_time = None
        self.players_ready = False

        # Initialize pygame mixer for sound
        pygame.mixer.init()
        
        # Load sounds
        self.countdown_sound = pygame.mixer.Sound("/home/pure/Desktop/Bittle_War/src/vision_processing/Music/3.wav")  # Path to countdown sound file

        # Subscriber for main camera image
        self.raw_image_subscription = self.create_subscription(
            Image,
            '/v4l1/camera/image_raw',
            self.image_callback,
            10
        )

        # Subscriber for player1 ready state
        self.ready_player1_sub = self.create_subscription(
            Bool,
            '/state/ready_player1',
            self.player1_ready_callback,
            10
        )

        # Subscriber for player2 ready state
        self.ready_player2_sub = self.create_subscription(
            Bool,
            '/state/ready_player2',
            self.player2_ready_callback,
            10
        )

        # Publisher for processed image (with countdown or GO text)
        self.image_publisher = self.create_publisher(Image, '/main_image', 10)

        # Publisher for game start state
        self.game_start_pub = self.create_publisher(Bool, '/game_start', 10)
        self.game_started = False

    def player1_ready_callback(self, msg):
        self.is_ready_player1 = msg.data
        self.get_logger().info(f"Player 1 ready: {self.is_ready_player1}")
        self.check_ready_state()

    def player2_ready_callback(self, msg):
        self.is_ready_player2 = msg.data
        self.get_logger().info(f"Player 2 ready: {self.is_ready_player2}")
        self.check_ready_state()

    def check_ready_state(self):
        if self.is_ready_player1 and self.is_ready_player2:
            if not self.players_ready:
                self.players_ready = True
                self.countdown_active = True
                self.countdown_start_time = time.time()
                self.publish_game_start(False)
                
                # Start playing countdown sound in a separate thread
                Thread(target=self.play_countdown_sound).start()
        else:
            self.countdown_active = False
            self.was_go_displayed = False
            self.go_display_time = None
            self.players_ready = False
            self.publish_game_start(False)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.countdown_active:
            elapsed_time = time.time() - self.countdown_start_time
            remaining_time = 3 - int(elapsed_time)

            if remaining_time > 0:
                frame = self.add_countdown_to_frame(frame, remaining_time)
            else:
                frame = self.add_go_text_to_frame(frame)
                self.countdown_active = False
                self.was_go_displayed = True
                self.go_display_time = time.time()
                self.publish_game_start(True)


        if self.was_go_displayed and (time.time() - self.go_display_time) < 1:
            frame = self.add_go_text_to_frame(frame)
        elif self.was_go_displayed and (time.time() - self.go_display_time) >= 1:
            self.was_go_displayed = False

        processed_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_publisher.publish(processed_image_msg)

    def add_countdown_to_frame(self, frame, count):
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = str(count)
        text_size = cv2.getTextSize(text, font, 3, 5)[0]
        text_x = (frame.shape[1] - text_size[0]) // 2
        text_y = (frame.shape[0] + text_size[1]) // 2
        cv2.putText(frame, text, (text_x, text_y), font, 3, (0, 0, 255), 5, cv2.LINE_AA)
        return frame

    def add_go_text_to_frame(self, frame):
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = "GO!"
        text_size = cv2.getTextSize(text, font, 3, 5)[0]
        text_x = (frame.shape[1] - text_size[0]) // 2
        text_y = (frame.shape[0] + text_size[1]) // 2
        cv2.putText(frame, text, (text_x, text_y), font, 3, (0, 255, 0), 5, cv2.LINE_AA)
        return frame

    def publish_game_start(self, state):
        game_start_msg = Bool()
        game_start_msg.data = state
        self.get_logger().info(f"Publishing game_start: {state}")
        self.game_start_pub.publish(game_start_msg)

    def play_countdown_sound(self):
        """Play countdown sound."""
        self.countdown_sound.play()

    def play_go_sound(self):
        """Play GO! sound."""
        self.go_sound.play()

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
