#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import math
import threading
import queue
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray

class BaseDriver:
    def __init__(self):
        rospy.init_node('base_driver')
        
        # ======================== 参数读取 ========================
        self.serial_port = rospy.get_param('~serial_port', '/dev/stm32_base')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.max_linear = rospy.get_param('~max_linear_speed', 0.8)
        self.max_angular = rospy.get_param('~max_angular_speed', 1.5)
        
        rospy.loginfo(f"Initializing Base Driver with Port: {self.serial_port}, Baud: {self.baud_rate}")
        
        # ======================== 状态变量 ========================
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_x, self.last_y = 0.0, 0.0
        self.last_time = rospy.Time.now()
        self.last_odom_time = rospy.Time.now()
        
        # ======================== 串口管理 ========================
        self.serial_connected = False
        self.running = True
        self.ser = None
        self.serial_lock = threading.Lock()  # ✅ 新增：线程锁
        self.rx_buffer = ""  # ✅ 新增：接收缓冲区
        self.tx_queue = queue.Queue(maxsize=50)  # ✅ 新增：发送队列
        self.last_cmd_time = rospy.Time.now()  # ✅ 新增：限速用
        self.cmd_interval = rospy.Duration(0.05)  # ✅ 新增：最快20Hz
        
        self.connect_serial()
        
        # ======================== ROS通信 ========================
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        self.tf_broadcaster = TransformBroadcaster()
        
        # ======================== 定时器 ========================
        rospy.Timer(rospy.Duration(0.5), self.check_heartbeat)
        rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        
        # ======================== 线程 ========================
        self.read_thread = threading.Thread(target=self.read_serial_data)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        self.write_thread = threading.Thread(target=self.write_serial_data)  # ✅ 新增：发送线程
        self.write_thread.daemon = True
        self.write_thread.start()
        
        rospy.loginfo("✓ Base driver initialized")
    
    def connect_serial(self):
        """✅ 改进的串口连接"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.01,  # ✅ 改为非阻塞
                write_timeout=0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # ✅ 增大缓冲区
            try:
                self.ser.set_buffer_size(rx_size=16384, tx_size=4096)
            except AttributeError:
                rospy.logwarn("set_buffer_size not supported")
            
            # 唤醒STM32
            self.ser.dtr = False
            self.ser.rts = False
            rospy.sleep(0.1)
            self.ser.dtr = True
            self.ser.rts = True
            
            # 清空缓冲区
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.rx_buffer = ""
            
            self.serial_connected = True
            rospy.loginfo(f"✓ Serial connected: {self.serial_port}")
            return True
            
        except Exception as e:
            self.serial_connected = False
            rospy.logerr(f"✗ Serial connection failed: {e}")
            return False
    
    def cmd_vel_callback(self, msg):
        """✅ 速度指令放入队列（非阻塞）"""
        if not self.serial_connected:
            return
        
        # ✅ 速率限制（避免淹没STM32）
        now = rospy.Time.now()
        if (now - self.last_cmd_time) < self.cmd_interval:
            return
        self.last_cmd_time = now
        
        vx = self.clamp(msg.linear.x, -self.max_linear, self.max_linear)
        vy = self.clamp(msg.linear.y, -self.max_linear, self.max_linear)
        wz = self.clamp(msg.angular.z, -self.max_angular, self.max_angular)
        cmd_str = f"${vx:.2f},{vy:.2f},{wz:.2f}\r\n"
        
        try:
            self.tx_queue.put_nowait(cmd_str.encode())
        except queue.Full:
            rospy.logwarn_throttle(1.0, "⚠ TX queue full")
    
    def write_serial_data(self):
        """✅ 独立的发送线程"""
        while self.running and not rospy.is_shutdown():
            try:
                cmd_bytes = self.tx_queue.get(timeout=0.1)
                
                if self.serial_connected and self.ser and self.ser.is_open:
                    with self.serial_lock:
                        self.ser.write(cmd_bytes)
                        self.ser.flush()
                    rospy.logdebug(f"Sent: {cmd_bytes.decode().strip()}")
                    
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logwarn(f"Write error: {e}")
                self.serial_connected = False
    
    def read_serial_data(self):
        """✅ 非阻塞读取"""
        while self.running and not rospy.is_shutdown():
            if not self.serial_connected:
                rospy.sleep(0.5)
                self.connect_serial()
                continue
            
            try:
                if self.ser and self.ser.is_open:
                    with self.serial_lock:
                        chunk = self.ser.read(128)
                    
                    if chunk:
                        self.rx_buffer += chunk.decode('utf-8', errors='ignore')
                        
                        while '\n' in self.rx_buffer:
                            line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                self.parse_odom_data(line)
                    else:
                        rospy.sleep(0.01)
                    
            except Exception as e:
                rospy.logwarn(f"Read error: {e}")
                self.serial_connected = False
                rospy.sleep(0.5)
    
    def parse_odom_data(self, line):
        """数据解析（保持不变）"""
        try:
            if not line.startswith('$'):
                return
            
            data_str = line[1:]
            data = data_str.split(',')
            if len(data) != 6:
                return
            
            x, y, th, vx, vy, vth = map(float, data)
            
            if abs(x) > 1000 or abs(y) > 1000:
                return
            if abs(vx) > 10 or abs(vy) > 10 or abs(vth) > 10:
                return
            
            if self.last_x != 0.0 or self.last_y != 0.0:
                jump = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
                if jump > 0.5:
                    rospy.logwarn(f"⚠ Position jump: {jump:.2f}m")
                    return
            
            self.last_x, self.last_y = x, y
            self.x, self.y, self.th = x, y, th
            self.publish_odom(vx, vy, vth)
            self.last_odom_time = rospy.Time.now()
            
        except Exception as e:
            rospy.logdebug(f"Parse error: {e}")
    
    def publish_odom(self, vx, vy, vth):
        """发布里程计（保持不变）"""
        current_time = rospy.Time.now()
        
        quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            quat,
            current_time,
            "base_footprint",
            "odom"
        )
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)
        
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.02
        
        self.odom_pub.publish(odom)
        self.last_time = current_time
    
    def check_heartbeat(self, event):
        """心跳检测"""
        elapsed = (rospy.Time.now() - self.last_odom_time).to_sec()
        if elapsed > 1.0:
            rospy.logwarn(f"⚠ No odom for {elapsed:.1f}s")
    
    def publish_diagnostics(self, event):
        """诊断信息"""
        diag = DiagnosticArray()
        diag.header.stamp = rospy.Time.now()
        
        status = DiagnosticStatus()
        status.name = "Base Driver"
        status.hardware_id = self.serial_port
        status.level = DiagnosticStatus.OK if self.serial_connected else DiagnosticStatus.ERROR
        status.message = "Connected" if self.serial_connected else "Disconnected"
        
        diag.status.append(status)
        self.diag_pub.publish(diag)
    
    def shutdown(self):
        """关闭"""
        rospy.loginfo("Shutting down...")
        self.running = False
        
        try:
            self.ser.write(b"$0.00,0.00,0.00\r\n")
            rospy.sleep(0.1)
            self.ser.close()
        except:
            pass
    
    @staticmethod
    def clamp(value, min_val, max_val):
        return max(min_val, min(max_val, value))

if __name__ == '__main__':
    try:
        driver = BaseDriver()
        rospy.on_shutdown(driver.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
