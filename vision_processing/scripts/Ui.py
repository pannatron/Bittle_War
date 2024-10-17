#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
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

        # Subscriber for main camera image
        self.raw_image_subscription = self.create_subscription(
            Image,
            '/v4l1/camera/image_raw',
            self.image_callback,
            10
        )

        # Subscriber for secondary camera image
        self.second_image_subscription = self.create_subscription(
            Image,
            '/v4l2/camera/image_raw',
            self.second_image_callback,
            10
        )

        # Publisher for player1 ready state
        self.ready_publisher = self.create_publisher(Bool, '/state/ready_player1', 10)

        # Player 1 ready state (False = Not Ready, True = Ready)
        self.is_ready = False

        # Publisher for robot command state
        self.command_publisher = self.create_publisher(String, '/robot/command', 10)

        # Subscriber for robot action to highlight corresponding icon
        self.action_subscription = self.create_subscription(
            String,
            '/robot/action',
            self.action_callback,
            10
        )

    def send_command(self, command):
        """Send command to the robot"""
        command_msg = String()
        command_msg.data = command
        self.command_publisher.publish(command_msg)
        print(f"Sent command: {command}")

    def action_callback(self, msg):
        """Callback to highlight the icon based on received action"""
        app.control_widget.highlight_button(msg.data)

    def image_callback(self, msg):
        """Callback to update the main camera feed"""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_pil = PILImage.fromarray(frame_rgb)
        img_tk = ImageTk.PhotoImage(image=img_pil)
        app.update_main_image(img_tk)

    def second_image_callback(self, msg):
        """Callback to update the secondary camera feed"""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_pil = PILImage.fromarray(frame_rgb)

        # Resize the secondary image to 2/8 (or 1/4) of the main image size
        img_pil = img_pil.resize((int(app.root.winfo_width() / 5), int(app.root.winfo_height() / 5)))
        img_tk = ImageTk.PhotoImage(image=img_pil)
        app.update_secondary_image(img_tk)

    def toggle_ready_state(self):
        """Toggle the ready state and publish the new state"""
        self.is_ready = not self.is_ready
        ready_msg = Bool()
        ready_msg.data = self.is_ready
        self.ready_publisher.publish(ready_msg)

        # Update the button color and text based on the state
        app.update_ready_button(self.is_ready)


class ControlWidget:
    def __init__(self, parent_frame, control_callback):
        """Widget to handle control buttons"""
        self.control_callback = control_callback

        # Frame for the icons below the secondary camera
        self.icon_frame = ttk.Labelframe(parent_frame, text="Control Buttons", padding=10, bootstyle="secondary")
        self.icon_frame.pack(fill=ttk.X, pady=10)

        # Define custom styles for icon buttons
        self.default_style = {"bootstyle": "secondary", "padding": 5}
        self.highlighted_style = {"bootstyle": "warning", "padding": 5}

        # Add icons (e.g., walk, left, right, stop, back) below the secondary camera feed
        self.walk_icon = self.load_icon("/home/borot/Desktop/Bittle_War/src/vision_processing/handpic/walk.png", (40, 40))
        self.left_icon = self.load_icon("/home/borot/Desktop/Bittle_War/src/vision_processing/handpic/left.png", (40, 40))
        self.right_icon = self.load_icon("/home/borot/Desktop/Bittle_War/src/vision_processing/handpic/right.png", (40, 40))
        self.stop_icon = self.load_icon("/home/borot/Desktop/Bittle_War/src/vision_processing/handpic/stop.png", (40, 40))
        self.back_icon = self.load_icon("/home/borot/Desktop/Bittle_War/src/vision_processing/handpic/sit.png", (40, 40))
        self.attack_icon = self.load_icon("/home/borot/Desktop/Bittle_War/src/vision_processing/handpic/attack.png", (100, 100))

        # Row 1: Walk
        self.walk_button = ttk.Button(self.icon_frame, image=self.walk_icon, text="Walk", compound=TOP, **self.default_style, command=lambda: self.send_command("walk"))
        self.walk_button.grid(row=0, column=1, padx=10, pady=5)

        # Row 2: Left, Stop, Right
        self.left_button = ttk.Button(self.icon_frame, image=self.left_icon, text="Left", compound=TOP, **self.default_style, command=lambda: self.send_command("left"))
        self.left_button.grid(row=1, column=0, padx=10, pady=5)

        self.stop_button = ttk.Button(self.icon_frame, image=self.stop_icon, text="Stop", compound=TOP, **self.default_style, command=lambda: self.send_command("stop"))
        self.stop_button.grid(row=1, column=1, padx=10, pady=5)

        self.right_button = ttk.Button(self.icon_frame, image=self.right_icon, text="Right", compound=TOP, **self.default_style, command=lambda: self.send_command("right"))
        self.right_button.grid(row=1, column=2, padx=10, pady=5)

        # Row 3: Back
        self.back_button = ttk.Button(self.icon_frame, image=self.back_icon, text="Back", compound=TOP, **self.default_style, command=lambda: self.send_command("back"))
        self.back_button.grid(row=2, column=1, padx=10, pady=5)

        # Large Attack button to the right
        self.attack_button = ttk.Button(self.icon_frame, image=self.attack_icon, text="Attack", compound=TOP, bootstyle="danger", padding=10, command=lambda: self.send_command("attack"))
        self.attack_button.grid(row=0, column=3, rowspan=3, padx=10, pady=5)

    def load_icon(self, path, size):
        """Load and resize an icon from file"""
        img = PILImage.open(path)
        img = img.resize(size)
        return ImageTk.PhotoImage(img)

    def send_command(self, command):
        """Send the selected command"""
        self.control_callback(command)

    def highlight_button(self, action):
        """Highlight the button based on action"""
        buttons = {
            "walk": self.walk_button,
            "left": self.left_button,
            "right": self.right_button,
            "stop": self.stop_button,
            "back": self.back_button,
            "attack": self.attack_button,
        }

        # Reset all buttons to default
        for button in buttons.values():
            button.config(bootstyle="secondary")

        # Highlight the active button
        if action in buttons:
            if buttons[action] == "attack":
                buttons[action].config(bootstyle="danger")
            else:
                buttons[action].config(bootstyle="danger")



class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Enhanced Vision Processing UI")
        self.root.geometry("1200x800")

        self.main_frame = ttk.Frame(self.root, bootstyle="dark")
        self.main_frame.pack(fill=ttk.BOTH, expand=True, padx=10, pady=10)

        # GIF at the top
        self.insert_gif()

        # Left panel for the small camera feed (secondary camera)
        self.left_panel = ttk.Labelframe(self.main_frame, text="Controls & Secondary Camera", padding=10, bootstyle="primary")
        self.left_panel.pack(side=ttk.LEFT, fill=ttk.Y, padx=10, pady=10)

        # Create a frame for secondary camera feed
        self.secondary_image_frame = ttk.Labelframe(self.left_panel, text="Secondary Camera Feed", padding=10, bootstyle="secondary")
        self.secondary_image_frame.pack(fill=ttk.BOTH, expand=True, padx=10, pady=10)

        self.secondary_image_label = ttk.Label(self.secondary_image_frame, background="black", foreground="white", relief="solid", borderwidth=2)
        self.secondary_image_label.pack(fill=ttk.BOTH, expand=True)

        # Create control widget for the buttons
        self.control_widget = ControlWidget(self.left_panel, vision_processing_node.send_command)

        # Create modern styles for the ready button with centered text and modern font
        style = ttk.Style()
        style.configure('ReadyDanger.TButton', font=('Arial', 18, 'bold'), padding=20, anchor='center', relief='solid', borderwidth=2, bordercolor='red', foreground='white', background='red')
        style.configure('ReadySuccess.TButton', font=('Arial', 18, 'bold'), padding=20, anchor='center', relief='solid', borderwidth=2, bordercolor='green', foreground='white', background='green')

        # Ready button at the bottom of left panel (under secondary camera)
        self.ready_button = ttk.Button(self.left_panel, text="Not Ready", style='ReadyDanger.TButton', command=self.toggle_ready)
        self.ready_button.pack(side=ttk.BOTTOM, pady=20, padx=20, fill=ttk.X)  # ปุ่มขยายตามแนวนอน

        # Right panel for the large camera feed (main camera)
        self.image_frame = ttk.Labelframe(self.main_frame, text="Main Camera Feed", padding=10, bootstyle="primary")
        self.image_frame.pack(side=ttk.RIGHT, fill=ttk.BOTH, expand=True, padx=10, pady=10)

        self.main_image_label = ttk.Label(self.image_frame, background="black", foreground="white")
        self.main_image_label.pack(fill=ttk.BOTH, expand=True)

    def update_ready_button(self, is_ready):
        """Update the Ready button color and text based on the state"""
        if is_ready:
            # เปลี่ยนสีเป็นเขียวเมื่อ Ready พร้อมจัดตัวอักษรให้อยู่ตรงกลาง
            self.ready_button.config(text="Ready", style='ReadySuccess.TButton')
        else:
            # เปลี่ยนกลับเป็นสีแดงเมื่อ Not Ready
            self.ready_button.config(text="Not Ready", style='ReadyDanger.TButton')

    def toggle_ready(self):
        """Toggle the ready state in the node"""
        vision_processing_node.toggle_ready_state()
        self.update_ready_button(vision_processing_node.is_ready)

    def update_main_image(self, img_tk):
        """Update the main camera feed"""
        self.main_image_label.config(image=img_tk)
        self.main_image_label.image = img_tk

    def update_secondary_image(self, img_tk):
        """Update the secondary camera feed"""
        self.secondary_image_label.config(image=img_tk)
        self.secondary_image_label.image = img_tk

    def insert_gif(self):
        """Insert a GIF at the top"""
        gif = PILImage.open("/home/borot/Desktop/Bittle_War/src/vision_processing/scripts/AA.gif")
        self.gif_label = ttk.Label(self.main_frame)
        self.gif_label.pack(side=ttk.TOP, fill=ttk.X, pady=10)

        self.update_gif(gif, 0)

    def update_gif(self, gif, frame_idx):
        """Update GIF frames"""
        try:
            gif.seek(frame_idx)
            resized_gif = gif.resize((self.root.winfo_width(), 150))
            img_tk = ImageTk.PhotoImage(resized_gif)

            self.gif_label.config(image=img_tk)
            self.gif_label.image = img_tk
            self.root.after(100, self.update_gif, gif, frame_idx + 1)
        except EOFError:
            self.update_gif(gif, 0)


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
