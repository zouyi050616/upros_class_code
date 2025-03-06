# 帧头
HEAD1 = 0xAA
HEAD2 = 0xAA

# 帧尾
END1 = 0x55
END2 = 0x55

# 读 RAM
CMD_READ_RAM = 0x03
# 写 RAM
CMD_WRITE_RAM = 0x06
# 写舵机位置
CMD_WRITE_SERVO_POS = 0x04
# 读舵机位置
CMD_READ_SERVO_POS = 0x05
# 读取数据返回
CMD_READ_RETURN = 0x30
#读取舵机位置返回
CMD_SERVO_READ_RETURN = 0x50

#初始化底盘地址
Init_Chassis = 3007
# 设置速度地址
Set_Vel_Addr = 3001
# 读取传感器地址
Read_Addr = 4000
#舵机地址
Servo_Addr = 0

Motor_Encoder = 64.0
Motor_Ratio = 90.0

Diff_Wheel_Radius = 0.99
Omni_Wheel_Radius = 0.075
Mec_wheel_Radius = 0.075

