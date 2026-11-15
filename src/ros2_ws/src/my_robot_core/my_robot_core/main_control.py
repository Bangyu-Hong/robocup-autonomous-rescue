import rclpy
from rclpy.node import Node

class MainControlNode(Node):
    def __init__(self):
        # 1. 初始化节点名称
        super().__init__('main_control_node')
        
        # 2. 打印启动成功信息
        self.get_logger().info('🚀 ELF2 上位机主控节点已启动！准备接管底盘与机械臂。')
        
        # 3. 创建一个定时器（相当于单片机的 Timer 中断），每 1 秒执行一次
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        # 4. 定时执行的逻辑（未来这里会用来监控视觉识别状态或发布控制指令）
        self.get_logger().info(f'正在监听传感器数据... (心跳包: {self.count})')
        self.count += 1

def main(args=None):
    # 初始化 ROS 2 接口
    rclpy.init(args=args)
    
    # 实例化我们的节点类
    node = MainControlNode()
    
    # 让节点保持运行状态，直到按下 Ctrl+C
    rclpy.spin(node)
    
    # 销毁节点并关闭 ROS 2 接口
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()