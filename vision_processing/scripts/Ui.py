#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from robot_communication_msgs.msg import CircleDetectionParams
from cv_bridge import CvBridge
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from PIL import Image as PILImage, ImageTk
from threading import Thread
import cv2

class VisionProcessingUI(Node):
    def __init__(self):
        super().__init__('vision_processing_ui')

        self.bridge = CvBridge()

        # Subscriber for raw camera image
        self.raw_image_subscription = self.create_subscription(
            Image,
            '/v4l/camera/image_raw',
            self.image_callback,
            10
        )

        # Subscriber for detected image
        self.detected_image_subscription = self.create_subscription(
            Image,
            '/circle_detection/output_image',
            self.detected_image_callback,
            10
        )

        # Publisher for auto find circle command
        self.auto_find_publisher = self.create_publisher(Bool, '/circle_detection/auto_find', 10)

        # Publisher for lock field, topview projection, and dangerous zone
        self.lock_field_publisher = self.create_publisher(Bool, '/circle_detection/lock_field', 10)
        self.topview_projection_publisher = self.create_publisher(Bool, '/circle_detection/topview_projection', 10)
        self.dangerous_zone_publisher = self.create_publisher(Bool, '/circle_detection/dangerous_zone', 10)

        # Subscriber for robot state
        self.robot_state_subscription = self.create_subscription(
            String,
            '/robot/state',
            self.robot_state_callback,
            10
        )

        # Publisher for detection parameters
        self.param_publisher = self.create_publisher(CircleDetectionParams, '/circle_detection/params', 10)

        # Image status flag
        self.showing_detected_image = False

        # Robot state string
        self.robot_state = "Idle"

        # Initialize sliders for HSV and blur parameters
        self.init_sliders()

    def publish_auto_find(self):
        auto_find_msg = Bool()
        auto_find_msg.data = True  # Start auto find circle
        self.auto_find_publisher.publish(auto_find_msg)
        self.showing_detected_image = True

    def publish_lock_field(self):
        lock_field_msg = Bool()
        lock_field_msg.data = True  # Lock the field
        self.lock_field_publisher.publish(lock_field_msg)

    def publish_topview_projection(self):
        topview_msg = Bool()
        topview_msg.data = True  # Enable topview projection
        self.topview_projection_publisher.publish(topview_msg)

    def publish_dangerous_zone(self):
        dangerous_zone_msg = Bool()
        dangerous_zone_msg.data = True  # Mark dangerous zone
        self.dangerous_zone_publisher.publish(dangerous_zone_msg)

    def robot_state_callback(self, msg):
        self.robot_state = msg.data
        app.update_robot_state(self.robot_state)

    def init_sliders(self):
        """Initialize the sliders for the various parameters"""
        self.slider_hue_low = 35
        self.slider_hue_high = 85
        self.slider_saturation_low = 50
        self.slider_saturation_high = 255
        self.slider_value_low = 50
        self.slider_value_high = 255
        self.slider_blur = 5

    def update_params(self):
        param_msg = CircleDetectionParams()
        param_msg.hue_low = float(self.slider_hue_low)
        param_msg.hue_high = float(self.slider_hue_high)
        param_msg.saturation_low = float(self.slider_saturation_low)
        param_msg.saturation_high = float(self.slider_saturation_high)
        param_msg.value_low = float(self.slider_value_low)
        param_msg.value_high = float(self.slider_value_high)
        param_msg.blur_size = float(self.slider_blur)

        self.param_publisher.publish(param_msg)
        print(f'Published parameters: {param_msg}')

    def image_callback(self, msg):
        if not self.showing_detected_image:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_pil = PILImage.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)
            app.update_image(img_tk)

    def detected_image_callback(self, msg):
        if self.showing_detected_image:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_pil = PILImage.fromarray(frame_rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)
            app.update_image(img_tk)

    def update_slider_values(self, param, value):
        """Update the parameter value and publish changes"""
        setattr(self, param, value)
        self.update_params()


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Enhanced Vision Processing UI")
        self.root.geometry("1200x800")

        self.main_frame = ttk.Frame(self.root, bootstyle="dark")
        self.main_frame.pack(fill=ttk.BOTH, expand=True, padx=10, pady=10)

        # Insert GIF at the top
        self.insert_gif()

        # Left control panel
        self.control_frame = ttk.Labelframe(self.main_frame, text="Control Panel", padding=10, bootstyle="primary")
        self.control_frame.pack(side=ttk.LEFT, fill=ttk.Y, padx=10, pady=10)

        # Section 1: Robot Actions
        self.robot_action_frame = ttk.Labelframe(self.control_frame, text="Robot Actions", bootstyle="info")
        self.robot_action_frame.pack(fill=ttk.X, pady=10)

        # Auto Find Button
        self.auto_find_button = ttk.Button(
            self.robot_action_frame, text="Auto Find Circle", bootstyle="success-outline",
            command=self.auto_find_circle
        )
        self.auto_find_button.pack(fill=ttk.X, pady=5)

        # Lock Field Button
        self.lock_field_button = ttk.Button(
            self.robot_action_frame, text="Lock Field", bootstyle="warning-outline",
            command=self.lock_field
        )
        self.lock_field_button.pack(fill=ttk.X, pady=5)

        # Topview Projection Button
        self.topview_projection_button = ttk.Button(
            self.robot_action_frame, text="Topview Projection", bootstyle="info-outline",
            command=self.topview_projection
        )
        self.topview_projection_button.pack(fill=ttk.X, pady=5)

        # Dangerous Zone Button
        self.dangerous_zone_button = ttk.Button(
            self.robot_action_frame, text="Dangerous Zone", bootstyle="danger-outline",
            command=self.dangerous_zone
        )
        self.dangerous_zone_button.pack(fill=ttk.X, pady=5)

        # Section 2: Circle Detection Parameters
        self.param_frame = ttk.Labelframe(self.control_frame, text="Detection Parameters", bootstyle="info")
        self.param_frame.pack(fill=ttk.X, pady=10)

        # HSV Low-High Sliders and blur
        self.create_slider_with_buttons(self.param_frame, "Hue Low", 0, 179, 35, "slider_hue_low")
        self.create_slider_with_buttons(self.param_frame, "Hue High", 0, 179, 85, "slider_hue_high")
        self.create_slider_with_buttons(self.param_frame, "Saturation Low", 0, 255, 50, "slider_saturation_low")
        self.create_slider_with_buttons(self.param_frame, "Saturation High", 0, 255, 255, "slider_saturation_high")
        self.create_slider_with_buttons(self.param_frame, "Value Low", 0, 255, 50, "slider_value_low")
        self.create_slider_with_buttons(self.param_frame, "Value High", 0, 255, 255, "slider_value_high")
        self.create_slider_with_buttons(self.param_frame, "Blur (Kernel Size)", 1, 25, 5, "slider_blur")

        # Section 3: Robot State Display (Move to the bottom and color adjustments)
        self.robot_state_label = ttk.Label(
            self.control_frame, 
            text="Robot State: Idle", 
            foreground="red", 
            font=('Ubuntu', 16, 'bold')  # Red color for robot state and bold font
        )
        self.robot_state_label.pack(anchor='w', padx=5, pady=10)

        # Camera Feed
        self.image_frame = ttk.Labelframe(self.main_frame, text="Camera Feed", padding=10, bootstyle="primary")
        self.image_frame.pack(side=ttk.RIGHT, fill=ttk.BOTH, expand=True, padx=10, pady=10)

        camera_label = ttk.Label(self.image_frame, text="Camera Feed", font=('Ubuntu', 16, 'bold'), foreground="white")
        camera_label.pack(anchor='w', padx=5, pady=5)

        self.image_label = ttk.Label(self.image_frame, background="black", foreground="white")
        self.image_label.pack(fill=ttk.BOTH, expand=True)

    def create_slider_with_buttons(self, parent, text, from_, to, default, param):
        frame = ttk.Frame(parent)
        frame.pack(fill=ttk.X, pady=5)

        label = ttk.Label(frame, text=f"{text}: {default}", foreground="white")
        label.pack(side=ttk.LEFT, padx=5)

        slider = ttk.Scale(frame, from_=from_, to=to, bootstyle="info", command=lambda value, lbl=label, txt=text: self.update_slider(lbl, txt, value, param))
        slider.set(default)
        slider.pack(side=ttk.LEFT, fill=ttk.X, expand=True, padx=5)

        minus_button = ttk.Button(frame, text="-", width=3, bootstyle="danger", command=lambda: self.adjust_slider(slider, -1, param))
        minus_button.pack(side=ttk.LEFT)

        plus_button = ttk.Button(frame, text="+", width=3, bootstyle="success", command=lambda: self.adjust_slider(slider, 1, param))
        plus_button.pack(side=ttk.LEFT)

    def adjust_slider(self, slider, step, param):
        current_value = slider.get()
        new_value = current_value + step
        slider.set(new_value)
        vision_processing_node.update_slider_values(param, new_value)

    def update_slider(self, label, text, value, param):
        label.config(text=f"{text}: {int(float(value))}")
        vision_processing_node.update_slider_values(param, value)

    def update_robot_state(self, state):
        self.robot_state_label.config(text=f"Robot State: {state}")

    def insert_gif(self):
        gif = PILImage.open("/home/borot/Desktop/bittle_ws/src/vision_processing/scripts/AA.gif")  # Replace with your GIF path
        self.gif_label = ttk.Label(self.main_frame)
        self.gif_label.pack(side=ttk.TOP, fill=ttk.X, pady=10)

        self.update_gif(gif, 0)

    def update_gif(self, gif, frame_idx):
        try:
            gif.seek(frame_idx)
            resized_gif = gif.resize((self.root.winfo_width(), 150))
            img_tk = ImageTk.PhotoImage(resized_gif)

            self.gif_label.config(image=img_tk)
            self.gif_label.image = img_tk
            self.root.after(100, self.update_gif, gif, frame_idx + 1)
        except EOFError:
            self.update_gif(gif, 0)

    def auto_find_circle(self):
        vision_processing_node.publish_auto_find()

    def lock_field(self):
        vision_processing_node.publish_lock_field()

    def topview_projection(self):
        vision_processing_node.publish_topview_projection()

    def dangerous_zone(self):
        vision_processing_node.publish_dangerous_zone()

    def update_image(self, img_tk):
        self.image_label.config(image=img_tk)
        self.image_label.image = img_tk

def ros_spin():
    rclpy.spin(vision_processing_node)


def main(args=None):
    global vision_processing_node, app

    rclpy.init(args=args)
    vision_processing_node = VisionProcessingUI()

    root = ttk.Window(themename="darkly")
    app = App(root)

    ros_thread = Thread(target=ros_spin)
    ros_thread.start()

    root.mainloop()

    vision_processing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
