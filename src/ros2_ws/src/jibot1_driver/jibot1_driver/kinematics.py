"""
Jibot1 逆运动学求解器
从 ESP32 MicroPython 源码 (z_kinematics.py) 移植

坐标系定义：
  x: 左右方向 (正值=右, 负值=左), 单位 mm
  y: 前后方向 (正值=前方, 不可为负), 单位 mm
  z: 高度 (距离地面), 单位 mm

舵机编号：
  0: 底盘旋转 (云台)
  1: 大臂 (肩关节)
  2: 小臂 (肘关节)
  3: 腕关节
  4: (预留)
  5: 夹爪
"""

from math import sqrt, atan, acos, sin, cos, pi
from typing import List, Optional


class Kinematics:
    """4自由度机械臂逆运动学求解器"""

    def __init__(self, l0: int = 170, l1: int = 105,
                 l2: int = 75, l3: int = 185):
        """
        Args:
            l0: 底盘到第二舵机中心轴的距离 (mm)
            l1: 第二舵机到第三舵机的距离 (mm)
            l2: 第三舵机到第四舵机的距离 (mm)
            l3: 第四舵机到末端(爪闭合)的距离 (mm)
        """
        # 内部放大10倍以提高精度 (单位: 0.1mm)
        self._l0 = l0 * 10
        self._l1 = l1 * 10
        self._l2 = l2 * 10
        self._l3 = l3 * 10

    def solve(self, x: float, y: float, z: float,
              alpha: Optional[float] = None) -> Optional[List[int]]:
        """
        逆运动学求解

        Args:
            x: x坐标 (mm)
            y: y坐标 (mm), 必须 >= 0
            z: z坐标 (mm)
            alpha: 末端与水平面夹角 (度), None 则自动搜索最佳角度

        Returns:
            [pwm0, pwm1, pwm2, pwm3] 四个舵机PWM值 (500~2500),
            无解则返回 None
        """
        if y < 0:
            return None

        if alpha is not None:
            return self._ik(x, y, z, alpha)

        # 自动搜索最佳 alpha (-25° ~ -65°)
        candidates = []
        for a in range(-25, -65, -1):
            result = self._ik(x, y, z, a)
            if result is not None:
                candidates.append((a, result))

        if not candidates:
            return None

        # y 较远时选最大 alpha，近处选最小 alpha (与原始固件逻辑一致)
        if y > 215:
            best = max(candidates, key=lambda c: c[0])
        else:
            best = min(candidates, key=lambda c: c[0])
        return best[1]

    def _ik(self, x: float, y: float, z: float,
            alpha: float) -> Optional[List[int]]:
        """单次逆运动学计算"""
        x10 = x * 10
        y10 = y * 10
        z10 = z * 10

        # 底盘旋转角 theta6
        if x10 == 0 and y10 != 0:
            theta6 = 0.0
        elif x10 > 0 and y10 == 0:
            theta6 = 90.0
        elif x10 < 0 and y10 == 0:
            theta6 = -90.0
        elif x10 == 0 and y10 == 0:
            return None
        else:
            theta6 = atan(x10 / y10) * 180.0 / pi

        # 投影到竖直平面
        yy = sqrt(x10 * x10 + y10 * y10)
        yy -= self._l3 * cos(alpha * pi / 180.0)
        zz = z10 - self._l0 - self._l3 * sin(alpha * pi / 180.0)

        # 边界检查
        if zz < -self._l0:
            return None
        dist = sqrt(yy * yy + zz * zz)
        if dist > (self._l1 + self._l2):
            return None
        if dist == 0:
            return None

        ccc = acos(yy / dist)
        bbb = (yy * yy + zz * zz + self._l1 * self._l1
               - self._l2 * self._l2) / (2 * self._l1 * dist)
        if bbb > 1 or bbb < -1:
            return None

        zf_flag = -1 if zz < 0 else 1

        theta5 = (ccc * zf_flag + acos(bbb)) * 180.0 / pi
        if theta5 > 180.0 or theta5 < 0.0:
            return None

        aaa = -(yy * yy + zz * zz - self._l1 * self._l1
                - self._l2 * self._l2) / (2 * self._l1 * self._l2)
        if aaa > 1 or aaa < -1:
            return None

        theta4 = 180.0 - acos(aaa) * 180.0 / pi
        if theta4 > 135.0 or theta4 < -135.0:
            return None

        theta3 = alpha - theta5 + theta4
        if theta3 > 90.0 or theta3 < -90.0:
            return None

        # 角度 → PWM (与 ESP32 固件公式完全一致)
        pwm = [
            int(1500 - 2000.0 * theta6 / 270.0),
            int(1500 + 2000.0 * (theta5 - 90) / 270.0),
            int(1500 + 2000.0 * theta4 / 270.0),
            int(1500 + 2000.0 * theta3 / 270.0),
        ]
        return pwm

    def pwm_to_cmd(self, pwm_list: List[int], move_time: int) -> str:
        """
        将 PWM 列表转换为 ESP32 协议指令字符串

        Args:
            pwm_list: [pwm0, pwm1, pwm2, pwm3]
            move_time: 运动时间 (ms)

        Returns:
            协议字符串, 如 '{#000P1500T1000!#001P1500T1000!...}'
        """
        cmd = '{'
        for i, pwm in enumerate(pwm_list):
            pwm = max(500, min(2500, pwm))
            cmd += '#%03dP%04dT%04d!' % (i, pwm, move_time)
        cmd += '}'
        return cmd
