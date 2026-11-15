# Rasc_Core 学习说明

这份文档不是功能手册，而是给后续自己看代码、查问题、做修改时用的学习路线。

## 1. 项目现在在做什么

这个工程的核心目标是：

- 接收上位机发送的速度指令 `$vx,vy,wz`
- 通过运动学计算出四个轮子的目标速度
- 读取四路编码器作为反馈
- 用 PID 计算四个电机 PWM
- 用 IMU 提供机器人航向角 `yaw`
- 最后把里程计信息按协议发回上位机

当前发送协议为：

```text
$x,y,yaw,vx,vy,wz\r\n
```

含义：

- `x, y`：里程计位置
- `yaw`：当前航向角，单位 rad
- `vx, vy, wz`：当前线速度和角速度

## 2. 推荐阅读顺序

不要一上来把所有文件都看一遍。按下面顺序读，效率最高。

### 第一层：主流程

先看：

- `src/hal_entry.c`

重点理解：

- 上电后初始化了哪些外设
- 主循环多久执行一次
- 每个周期做了哪些事
- 最终发了什么给上位机

如果只看一个文件，优先看这个。

### 第二层：IMU 链路

再看：

- `src/imu.c`
- `src/imu.h`

重点理解：

- IMU 串口怎么初始化
- 串口字节怎么拼成 11 字节数据帧
- 为什么要做校验和
- `0x53` 角度帧是怎么解析出 `yaw`
- `yaw_offset` 是怎么做软件归零的

### 第三层：底盘控制链路

再看：

- `src/kinematics.c`
- `src/kinematics.h`
- `src/pid.c`
- `src/pid.h`
- `src/motor.c`
- `src/motor.h`

重点理解：

- 上位机给的 `vx, vy, wz` 怎么变成四个轮子的目标速度
- 编码器测出来的速度怎么变成反馈值
- PID 怎么算出 PWM
- PWM 和方向脚怎么驱动电机

### 第四层：硬件配置

最后看：

- `ra_gen/hal_data.c`
- `ra_gen/pin_data.c`
- `configuration.xml`

重点理解：

- UART/GPT/IO 是在哪里配置的
- 哪些引脚默认高低电平是什么
- 哪些文件是 FSP 自动生成的

注意：

- `ra_gen/*` 和 `configuration.xml` 主要用来“查配置”
- 不建议手改自动生成文件，除非明确知道后果

## 3. 代码主链路

把这个数据流看懂，整个项目就通了。

### 3.1 上位机命令输入

上位机发送：

```text
$vx,vy,wz
```

在 `src/hal_entry.c` 里解析后，得到：

- `cmd_vx`
- `cmd_vy`
- `cmd_wz`

然后换算成四个电机目标速度：

- `target_speed_A`
- `target_speed_B`
- `target_speed_C`
- `target_speed_D`

### 3.2 编码器反馈

在 `src/hal_entry.c` 里每 10ms 读取四路编码器计数差值，得到：

- `speed_A`
- `speed_B`
- `speed_C`
- `speed_D`

再经过一阶低通滤波，得到：

- `filt_speed_A`
- `filt_speed_B`
- `filt_speed_C`
- `filt_speed_D`

### 3.3 PID 输出

在 `src/pid.c` 中，把目标速度和反馈速度做比较，得到：

- `pwm_A`
- `pwm_B`
- `pwm_C`
- `pwm_D`

然后调用：

```c
Motor_SetAllSpeed(pwm_A, pwm_B, pwm_C, pwm_D);
```

### 3.4 IMU 姿态输入

IMU 通过串口不断发 11 字节帧：

- `0x51` 加速度
- `0x52` 角速度
- `0x53` 欧拉角

在 `src/imu.c` 中，重点解析 `0x53`，更新：

- `robot_imu.roll`
- `robot_imu.pitch`
- `robot_imu.yaw`

然后在 `src/hal_entry.c` 中使用：

```c
robot_odom.odom_yaw = robot_imu.yaw * (3.14159265f / 180.0f);
```

### 3.5 里程计输出

在 `src/kinematics.c` 中，通过四轮速度计算：

- `robot_odom.vx`
- `robot_odom.vy`
- `robot_odom.wz`

并更新：

- `robot_odom.odom_x`
- `robot_odom.odom_y`
- `robot_odom.odom_yaw`

最后在 `src/hal_entry.c` 中发送：

```text
$x,y,yaw,vx,vy,wz\r\n
```

## 4. 本项目几个最重要的全局变量

看代码时，优先追这些变量，不要一上来追所有变量。

### IMU 相关

- `robot_imu.yaw`
- `robot_imu.gyro_z`
- `yaw_offset`
- `imu_rx_count`
- `imu_frame_count`
- `imu_checksum_fail`

### 运动控制相关

- `target_speed_A/B/C/D`
- `filt_speed_A/B/C/D`
- `pwm_A/B/C/D`
- `run_flag`

### 里程计相关

- `robot_odom.odom_x`
- `robot_odom.odom_y`
- `robot_odom.odom_yaw`
- `robot_odom.vx`
- `robot_odom.vy`
- `robot_odom.wz`

## 5. 最近已经解决过的两个问题

后面再遇到类似问题，可以优先从这些经验开始排查。

### 5.1 IMU yaw 不更新 / 一直为 0

现象：

- `yaw` 一直是 0
- 或者几秒才更新一次

根因：

- 串口接收状态机不稳
- RA UART 回调取字节方式有问题
- 调试时发现 `rx_count` 增长，但帧并没有正确进入解析逻辑

修复点：

- 重写了 `src/imu.c` 中的接收状态机
- 加入 11 字节完整帧解析
- 加入校验和校验
- 加入 `0x50~0x5F` 类型码过滤
- 修正 RA 回调中 `RX_COMPLETE` 的取字节方式

以后快速判断：

- 如果 `rx` 在涨，说明有字节进 UART
- 如果 `frm` 不涨，说明帧没解析成功
- 如果 `h != 0x55` 或 `t` 不在 `0x50~0x5F`，说明收的数据不对

### 5.2 复位时 A 轮会自己转一圈

现象：

- 每次复位 A 轮先转一下

根因：

- A 轮某个方向脚默认上电为高电平
- PWM 定时器启动顺序过早
- 在方向脚和 PWM 都没完全进入安全状态前，电机桥臂短暂导通

修复点：

- 修改 `src/motor.c` 中 `Motor_Init()`
- 初始化时先把所有方向脚拉低
- 再打开 GPT
- 再把 PWM 设到停机值
- 最后再启动定时器

额外注意：

- FSP 引脚默认电平如果和软件预期不一致，重新生成代码后问题可能回来

## 6. 如何自己学会这份代码

不要试图一次看懂全部。推荐下面这种方法。

### 方法 1：一次只追一个变量

例如只追 `robot_imu.yaw`：

- 在哪里更新
- 在哪里被转换成 `robot_odom.odom_yaw`
- 在哪里被发给上位机

能把一个变量从输入追到输出，就是真的理解了一段代码。

### 方法 2：一次只讲清一个函数

例如看 `Motor_SetSpeed()`，只回答：

- 输入是什么
- 中间做了什么转换
- 最终控制了哪些引脚和 PWM

### 方法 3：做问题复盘

每解决一个问题，写 4 行：

- 现象
- 根因
- 改了哪里
- 怎么验证修好了

这样时间久了，你会形成自己的调试方法。

## 7. 建议自己先回答的几个问题

后面有空时，建议自己不看 AI 解释，直接读代码回答下面这些问题。

1. `robot_imu.yaw` 是在哪里更新的？
2. `robot_odom.odom_yaw` 是在哪里赋值的？
3. 为什么停机时 PWM 不是写 `0`，而是写 `10000`？
4. `target_speed_A/B/C/D` 是怎么从 `$vx,vy,wz` 算出来的？
5. 为什么 IMU 要做校验和，而不是直接收满 11 字节就用？

如果这些问题你能自己回答出来，这个项目的主体你就已经掌握得差不多了。

## 8. 后面继续学习时的建议

建议每次只花 20 到 30 分钟看一个模块，不要硬撑。

推荐节奏：

- 第一天：只看 `src/hal_entry.c`
- 第二天：只看 `src/imu.c`
- 第三天：只看 `src/kinematics.c`
- 第四天：只看 `src/pid.c`
- 第五天：只看 `src/motor.c`

每次看完后，自己写一个简短总结，比连续看 3 小时有效得多。

## 9. 一句话总结

这个项目最值得先学会的，不是“每一行代码”，而是这三件事：

- 数据是怎么流动的
- 问题是怎么定位的
- 配置和代码之间是怎么对应的

把这三件事学会了，以后就不只是“让 AI 帮你改”，而是你自己也能判断 AI 改得对不对。
