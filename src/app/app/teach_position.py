#!/usr/bin/env python3
# coding=utf8
"""
手动示教工具 — 松开机械臂所有舵机扭矩，手动掰到目标位置，按回车记录坐标。
用法: python3 teach_position.py
需要先启动底层驱动 (sdk_launch)
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.srv import GetArmCoords, BusServoCtrl

SERVO_IDS = [1, 2, 3, 4, 5, 6]


class TeachPosition(Node):
    def __init__(self):
        super().__init__('teach_position')
        self.coords_client = self.create_client(GetArmCoords, '/ros_robot_controller/arm/get_coords')
        self.servo_client = self.create_client(BusServoCtrl, '/ros_robot_controller/bus_servo/ctrl')

        self.get_logger().info('等待底层服务...')
        self.coords_client.wait_for_service()
        self.servo_client.wait_for_service()
        self.get_logger().info('服务就绪')

    def call_servo(self, servo_id, set_torque=False, torque_enable=False, set_position=False, position=0, acc=0, speed=0):
        req = BusServoCtrl.Request()
        req.id = servo_id
        req.set_torque = set_torque
        req.torque_enable = torque_enable
        req.set_position = set_position
        req.position = int(position)
        req.acc = acc
        req.speed = speed
        req.set_mode = False
        future = self.servo_client.call_async(req)
        # 等待结果
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def set_torque_all(self, enable):
        for sid in SERVO_IDS:
            result = self.call_servo(sid, set_torque=True, torque_enable=enable)
            if result:
                self.get_logger().info(f'舵机{sid} 扭矩{"开" if enable else "关"}')
            time.sleep(0.05)

    def get_coords(self):
        future = self.coords_client.call_async(GetArmCoords.Request())
        while not future.done():
            time.sleep(0.01)
        res = future.result()
        if res and res.success:
            return {
                'x': res.x, 'y': res.y, 'z': res.z,
                'pitch': res.pitch, 'roll': res.roll,
                'servos': list(res.servos),
            }
        return None


def main():
    rclpy.init()
    node = TeachPosition()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 在后台线程跑 executor，这样服务调用才能正常回调
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    positions = []

    print('\n' + '='*60)
    print('  机械臂示教工具')
    print('  松开扭矩后，手动掰机械臂到目标位置')
    print('  按 回车 记录当前位置')
    print('  输入 q 退出并打印所有记录')
    print('='*60)

    print('\n>>> 松开所有舵机扭矩...')
    node.set_torque_all(False)
    print('>>> 舵机已松开，可以手动移动机械臂了\n')

    idx = 0
    while True:
        cmd = input(f'[位置 {idx}] 掰好后按回车记录 (q=退出, l=锁定, u=松开): ').strip().lower()

        if cmd == 'q':
            break
        elif cmd == 'l':
            node.set_torque_all(True)
            print('>>> 舵机已锁定')
            continue
        elif cmd == 'u':
            node.set_torque_all(False)
            print('>>> 舵机已松开')
            continue

        coords = node.get_coords()
        if coords is None:
            print('  !! 读取失败，重试')
            continue

        x_m = coords['x'] / 1000.0
        y_m = coords['y'] / 1000.0
        z_m = coords['z'] / 1000.0

        positions.append({
            'idx': idx,
            'x_m': x_m, 'y_m': y_m, 'z_m': z_m,
            'x_mm': coords['x'], 'y_mm': coords['y'], 'z_mm': coords['z'],
            'pitch': coords['pitch'], 'roll': coords['roll'],
            'servos': coords['servos'],
        })

        print(f'  记录位置 {idx}:')
        print(f'    坐标(米):  [{x_m:.6f}, {y_m:.6f}, {z_m:.6f}]')
        print(f'    坐标(mm):  [{coords["x"]:.1f}, {coords["y"]:.1f}, {coords["z"]:.1f}]')
        print(f'    pitch={coords["pitch"]:.1f}  roll={coords["roll"]:.1f}')
        print(f'    舵机脉冲:  {coords["servos"]}')
        print()
        idx += 1

    node.set_torque_all(True)
    print('\n>>> 舵机已锁定')

    if positions:
        print('\n' + '='*60)
        print('  所有记录的位置 (可直接复制到代码中)')
        print('='*60)

        print('\n# 米为单位 (place_position):')
        for p in positions:
            print(f"  位置{p['idx']}: [{p['x_m']:.6f}, {p['y_m']:.6f}, {p['z_m']:.6f}]")

        print('\n# mm为单位 (ArmCoords):')
        for p in positions:
            print(f"  位置{p['idx']}: [{p['x_mm']:.1f}, {p['y_mm']:.1f}, {p['z_mm']:.1f}]  pitch={p['pitch']:.1f} roll={p['roll']:.1f}")

        print('\n# 舵机脉冲:')
        for p in positions:
            print(f"  位置{p['idx']}: {p['servos']}")

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
