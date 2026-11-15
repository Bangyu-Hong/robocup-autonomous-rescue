"""
Jibot1 拾取放置节点

提供一个简单的拾取-放置动作序列节点，
通过话题 /jibot1/pick_place 接收拾取/放置坐标。

使用方法:
  # 从 (100, 200, 20) 拾取，放置到 (-100, 200, 20)
  ros2 topic pub --once /jibot1/pick_place std_msgs/msg/String \
    "{data: '100,200,20,-100,200,20'}"

  # 格式: "pick_x, pick_y, pick_z, place_x, place_y, place_z"
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__('jibot1_pick_place')

        # 参数
        self.declare_parameter('approach_height', 80.0)   # 接近高度 (mm)
        self.declare_parameter('move_delay', 2.0)         # 运动等待时间 (s)
        self.declare_parameter('gripper_delay', 1.0)      # 夹爪动作等待 (s)

        # 发布器 —— 向驱动节点发指令
        self._pub_target = self.create_publisher(
            Point, '/jibot1/target_pose', 10)
        self._pub_gripper = self.create_publisher(
            Bool, '/jibot1/gripper', 10)
        self._pub_status = self.create_publisher(
            String, '/jibot1/status', 10)

        # 订阅器 —— 接收拾取放置指令
        self._sub_cmd = self.create_subscription(
            String, '/jibot1/pick_place', self._on_pick_place, 10)

        self.get_logger().info('拾取放置节点已启动')

    def _publish_target(self, x: float, y: float, z: float):
        msg = Point()
        msg.x, msg.y, msg.z = x, y, z
        self._pub_target.publish(msg)
        self.get_logger().info(f'目标: ({x:.1f}, {y:.1f}, {z:.1f})')

    def _publish_gripper(self, close: bool):
        msg = Bool()
        msg.data = close
        self._pub_gripper.publish(msg)
        self.get_logger().info(f'夹爪: {"闭合" if close else "张开"}')

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)

    def _on_pick_place(self, msg: String):
        """
        接收拾取放置指令
        格式: "pick_x,pick_y,pick_z,place_x,place_y,place_z"
        """
        try:
            parts = [float(v.strip()) for v in msg.data.split(',')]
            if len(parts) != 6:
                self.get_logger().error(
                    '格式错误，需要: pick_x,pick_y,pick_z,'
                    'place_x,place_y,place_z')
                return

            pick_x, pick_y, pick_z = parts[0], parts[1], parts[2]
            place_x, place_y, place_z = parts[3], parts[4], parts[5]
            self._execute_pick_place(
                pick_x, pick_y, pick_z,
                place_x, place_y, place_z)

        except ValueError as e:
            self.get_logger().error(f'解析失败: {e}')

    def _execute_pick_place(self, px, py, pz, tx, ty, tz):
        """执行完整的拾取-放置序列"""
        approach_h = self.get_parameter('approach_height').value
        move_delay = self.get_parameter('move_delay').value
        grip_delay = self.get_parameter('gripper_delay').value

        self._publish_status('PICK_PLACE:START')

        # 1. 张开夹爪
        self._publish_gripper(False)
        time.sleep(grip_delay)

        # 2. 移动到拾取点上方
        self._publish_status('PICK_PLACE:APPROACH_PICK')
        self._publish_target(px, py, approach_h)
        time.sleep(move_delay)

        # 3. 下降到拾取位置
        self._publish_status('PICK_PLACE:DESCEND_PICK')
        self._publish_target(px, py, pz)
        time.sleep(move_delay)

        # 4. 闭合夹爪 (夹取)
        self._publish_status('PICK_PLACE:GRIP')
        self._publish_gripper(True)
        time.sleep(grip_delay)

        # 5. 提起
        self._publish_status('PICK_PLACE:LIFT')
        self._publish_target(px, py, approach_h)
        time.sleep(move_delay)

        # 6. 移动到放置点上方
        self._publish_status('PICK_PLACE:APPROACH_PLACE')
        self._publish_target(tx, ty, approach_h)
        time.sleep(move_delay)

        # 7. 下降到放置位置
        self._publish_status('PICK_PLACE:DESCEND_PLACE')
        self._publish_target(tx, ty, tz)
        time.sleep(move_delay)

        # 8. 张开夹爪 (释放)
        self._publish_status('PICK_PLACE:RELEASE')
        self._publish_gripper(False)
        time.sleep(grip_delay)

        # 9. 抬起
        self._publish_status('PICK_PLACE:RETREAT')
        self._publish_target(tx, ty, approach_h)
        time.sleep(move_delay)

        self._publish_status('PICK_PLACE:DONE')
        self.get_logger().info('拾取放置完成')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
