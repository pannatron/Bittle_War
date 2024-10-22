#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String, Int32,Bool
import asyncio
import threading
from bleak import BleakClient, BleakError

# UUID ของ Bluetooth
uuid = "FC:B4:67:20:F2:D2"
characteristic_uuid_notify = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
characteristic_uuid_write = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

# Global ตัวนับสำหรับการเช็คข้อมูล rc
rc_count = 0

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller2')

        # ใช้ Callback group เพื่อแยกการทำงานของ ROS 2 callbacks
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # สร้าง subscriber สำหรับหัวข้อ /robot/action
        self.subscription = self.create_subscription(
            String,
            '/robot/action2',
            self.action_callback,
            10,
            callback_group=self.callback_group
        )
        self.subscription2 = self.create_subscription(
            Bool,
            '/game_start',
            self.game_start_callback,
            10,
            callback_group=self.callback_group
        )
        # สร้าง publisher สำหรับหัวข้อ /robot/rc_count
        self.rc_count_publisher = self.create_publisher(Int32, '/robot/rc_count_2', 10)

        # สร้าง client ของ BleakClient
        self.client = BleakClient(uuid)

        # เก็บคำสั่งล่าสุดที่ส่ง
        self.last_command = None
        self.game_start=False
        # สร้าง event loop และ thread สำหรับ asyncio
        self.loop = asyncio.new_event_loop()
        self.bluetooth_thread = threading.Thread(target=self.run_asyncio_loop)
        self.bluetooth_thread.start()

    def run_asyncio_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect_and_listen())

    async def connect_and_listen(self):
        try:
            # เชื่อมต่อ Bluetooth
            await self.client.connect()

            if self.client.is_connected:
                self.get_logger().info("เชื่อมต่อ Bluetooth สำเร็จ")
                await asyncio.sleep(3)
                await self.send_bluetooth_command(b'kbalance')  # ส่งคำสั่ง kbalance

                # สมัครรับการแจ้งเตือน
                await self.client.start_notify(characteristic_uuid_notify, self.notification_handler)

                # รอรับการแจ้งเตือนแบบต่อเนื่อง
                while True:
                    await asyncio.sleep(1)
        except BleakError as e:
            self.get_logger().error(f"เกิดข้อผิดพลาดขณะเชื่อมต่อกับ Bluetooth: {e}")

    async def send_bluetooth_command(self, command):
        await self.client.write_gatt_char(characteristic_uuid_write, command)
        self.get_logger().info(f"ส่งคำสั่ง {command.decode()} ไปยังหุ่นยนต์")
    def game_start_callback(self, msg):
        self.game_start = msg.data
    def action_callback(self, msg):
        action = msg.data.lower()
        self.get_logger().info(f"ได้รับคำสั่ง: {action}")

        # เช็คว่าคำสั่งที่ได้รับเป็นคำสั่งใหม่หรือไม่
        if action != self.last_command:
            self.last_command = action  # อัปเดตคำสั่งล่าสุด
            asyncio.run_coroutine_threadsafe(self.handle_command(action), self.loop)
        else:
            self.get_logger().info("คำสั่งยังคงเหมือนเดิม ไม่ส่งซ้ำ")

    async def handle_command(self, action):
        if self.game_start:
            if action == "walk":
                await self.send_bluetooth_command(b'kwkF')
            elif action == "left":
                await self.send_bluetooth_command(b'kvtL')
            elif action == "right":
                await self.send_bluetooth_command(b'kvtR')
            elif action == "stop":
                await self.send_bluetooth_command(b'kbalance')
            elif action == "back":
                await self.send_bluetooth_command(b'kbkF')
            elif action == "attack":
                await self.send_bluetooth_command(b'kat')


    def notification_handler(self, sender, data):
        global rc_count
        self.get_logger().info(f"ข้อมูลที่ได้รับจาก {sender}: {data}")

        # เช็คว่าข้อมูลเป็น 'rc'
        if data == bytearray(b'rc'):
            rc_count += 1
            self.get_logger().info(f"นับข้อมูล 'rc' ได้ {rc_count} ครั้ง")

            # ส่งจำนวน count ไปยังหัวข้อ /robot/rc_count
            msg = Int32()
            msg.data = rc_count
            self.rc_count_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # สร้าง MultiThreadedExecutor สำหรับ ROS 2
    executor = rclpy.executors.MultiThreadedExecutor()

    robot_controller = RobotController()

    # เพิ่ม node เข้าไปใน ROS 2 executor
    executor.add_node(robot_controller)

    try:
        executor.spin()  # รัน ROS 2 executor
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
