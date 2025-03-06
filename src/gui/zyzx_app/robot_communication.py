from PyQt5.QtSerialPort import QSerialPort
import data_frame
import time
from PyQt5.QtCore import QTimer

# CRC 高位字节值表
s_crchi = [0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
    0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,0x00, 0xC1,
     0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
     0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
     0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 
     0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 
     0x81, 0x40,0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,0x00, 0xC1, 
     0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 
     0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
     0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 
     0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40]


# CRC 低位字节值表
s_crclo = [0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 
     0xCD,0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,0x1E, 0xDE, 0xDF, 
     0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31,
      0xF1, 0x33, 0xF3,0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,0x3B, 
      0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,0xEC, 0x2C, 0xE4, 0x24, 0x25, 
      0xE5, 0x27, 0xE7, 0xE6, 0x26,0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 
      0x67,0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,0x78, 0xB8, 0xB9, 
      0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 
      0x73, 0xB1, 0x71,0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,0x5D, 
      0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 
      0x4F, 0x8D, 0x4D, 0x4C, 0x8C,0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,0x43, 0x83, 0x41, 0x81, 0x80, 0x40]

M_PI = 3.1415926

class RobotCommunication:

    def __init__(self, qt):
        self.com = QSerialPort()
        self.com.readyRead.connect(self.com_receive_data)

        # 串口发送消息队列
        self.msg_list = []

        self.type = 1 # 1-mec,2-omni,3-diff

        # 开启定时发送消息定时器线程
        self.send_timer = QTimer(qt)
        self.send_timer.timeout.connect(self.send_timer_run)

        # 开启定时发送消息获取机器人状态线程
        self.get_data_timer = QTimer(qt)
        self.get_data_timer.timeout.connect(self.get_data_run)

    # 打开串口
    def open_com(self, com_port, com_baud):
        # 设定串口号
        self.com.setPortName(com_port)
        # 设定波特率
        self.com.setBaudRate(com_baud)
        try:
            if self.com.open(QSerialPort.ReadWrite) == False:
                print("串口打开失败")
                return False
            self.send_timer.start(30)
            self.get_data_timer.start(600)
            return True
        except:
            print("串口打开失败")
            return False

    # 关闭串口
    def close(self):
        self.com.close()

    # 串口数据接收
    def com_receive_data(self):
        try:
            rxData = bytes(self.com.readAll()) #读取串口缓存区数据
            data_size = len(rxData) 
            data = list(rxData)
            self.parse_data(data_size, data) #数据解析
        except:
            print("串口数据错误")

    # 定时发送消息列表
    def send_timer_run(self):
        if self.com.isOpen():
            if len(self.msg_list) > 0 :
                self.com.write(self.msg_list[0])
                time.sleep(0.01)
                self.msg_list.remove(self.msg_list[0])

    # 数据解析
    def parse_data(self, data_size, data):
        # 校验帧头
        if data[0] == data_frame.HEAD1 and data[1] == data_frame.HEAD2:

            # 底盘数据返回
            if data[4] == data_frame.CMD_READ_RETURN and data_size == 68:
                tof1 = int.from_bytes(data[7:9], byteorder='little', signed=False)
                ul1 = int.from_bytes(data[9:11], byteorder='little', signed=False)
                tof2 = int.from_bytes(data[11:13], byteorder='little', signed=False)
                ul2 = int.from_bytes(data[13:15], byteorder='little', signed=False)
                tof3 = int.from_bytes(data[15:17], byteorder='little', signed=False)
                ul3 = int.from_bytes(data[17:19], byteorder='little', signed=False)
                tof4 = int.from_bytes(data[19:21], byteorder='little', signed=False)
                ul4 = int.from_bytes(data[21:23], byteorder='little', signed=False)

                collision1 = (data[47] >> 0) & 1
                collision2 = (data[47] >> 1) & 1
                collision3 = (data[47] >> 2) & 1
                collision4 = (data[47] >> 3) & 1

                motor_1_encoder = int.from_bytes(data[48:52], byteorder='little', signed=True)
                motor_2_encoder = int.from_bytes(data[52:56], byteorder='little', signed=True)
                motor_3_encoder = int.from_bytes(data[56:60], byteorder='little', signed=True)
                motor_4_encoder = int.from_bytes(data[60:64], byteorder='little', signed=True)

                if self.base_callback is not None:
                    self.base_callback(tof1, ul1, tof2, ul2, tof3, ul3, tof4, ul4, collision1, collision2, collision3, collision4, motor_1_encoder, motor_2_encoder, motor_3_encoder, motor_4_encoder)

            # 舵机数据返回
            if data[4] == data_frame.CMD_SERVO_READ_RETURN and data_size == 35:
                # 生成伺服位置数组（使用列表推导式优化）
                servo_pos = [
                    int.from_bytes(data[i:i+4], byteorder='little', signed=True)
                    for i in range(7, 31, 4)  # 7,11,15,19,23,27 共6个位置
                ]
            if self.servo_callback is not None:
                self.servo_callback(servo_pos)  # 传递数组参数



    # 设置底盘信息数据回调
    def set_base_callback(self, data_callback):
        self.base_callback = data_callback

    # # 设置超声TOF数据回调
    # def set_ul_tof_callback(self, data_callback):
    #     self.ul_tof_callback = data_callback

    # # 设置碰撞数据回调
    # def set_collision_callback(self, data_callback):
    #     self.collision_callback = data_callback

    # 设置舵机信息回调
    def set_servo_callback(self, data_callback):
        self.servo_callback = data_callback

    # 定时发送消息获取机器人信息
    def get_data_run(self):
        self.get_base_data()
        time.sleep(0.1)

    # 获取底盘信息
    def get_base_data(self):
        data = [0] * 1
        data[0] = 0x39
        # 基于 READ_RAM 的 CMD，在 READ_ADDR 内存表地址查询 57 字节的数据
        data_pack = self.GenerateCmd(0x01, data_frame.CMD_READ_RAM, data_frame.Read_Addr, data, 1)
        # 发送队列加入查询指令
        self.msg_list.append(data_pack)

    #  获取机械臂各关节信息（舵机id为1~6）
    def get_servo_info(self):
        data = [i+1 for i in range(6)]
        data_pack = self.GenerateCmd(0x01, data_frame.CMD_READ_SERVO_POS, data_frame.Servo_Addr, data, 6)
        self.msg_list.append(data_pack)

    # 修改底盘运动模型
    def set_chassiss_type(self, type):
        data = [0] * 1
        if type == 1:
            data[0] = 0x03
        if type == 2:
            data[0] = 0x05
        if type == 3:
            data[0] = 0x07
        data_pack = self.GenerateCmd(0x01, data_frame.CMD_WRITE_RAM, data_frame.Init_Chassis, data, 1)
        self.msg_list.append(data_pack)
        self.type = type

    # # 清空里程计
    # def clear_odom(self):
    #     data = [0] * 1
    #     data[0] = 0x01
    #     data_pack = self.GenerateCmd(0x01, data_frame.CMD_WRITE_RAM, data_frame.Init_Chassis, data, 1)
    #     self.msg_list.append(data_pack)

    # 控制单个舵机运动
    def single_servo_control(self, servo_id, servo_pos, servo_speed):
        id = servo_id.to_bytes(1, byteorder='little', signed=False)
        angle = servo_pos.to_bytes(2, byteorder='little', signed=True)
        speed = servo_speed.to_bytes(2, byteorder='little', signed=False)
        data = id + angle + speed
        data_pack = self.GenerateCmd(0x01, data_frame.CMD_WRITE_SERVO_POS, data_frame.Servo_Addr, data, 5)
        self.msg_list.append(data_pack)

    # 控制机器人底盘移动速度
    def robot_move(self, speed_x, speed_y, speed_yaw):
        pos_x = speed_x.to_bytes(2, byteorder='little', signed=True)
        pos_y = speed_y.to_bytes(2, byteorder='little', signed=True)
        pos_yaw = speed_yaw.to_bytes(2, byteorder='little', signed=True)
        pos_data = pos_x + pos_y + pos_yaw
        data_pack = self.GenerateCmd(0x01, data_frame.CMD_WRITE_RAM, data_frame.Set_Vel_Addr, pos_data, 6)
        self.msg_list.append(data_pack)

    # # 修改舵机ID
    # def update_servo_id(self, old_id, new_id):
    #     data = [0] * 2
    #     data[0] = old_id
    #     data[1] = new_id
    #     data_pack = self.generateCmd(data_frame.ID_EDIT_SERVO_ID, data)
    #     self.msg_list.append(data_pack)

    #组合指令函数，输入指令参数，增加帧头，id，cmd，帧尾，校验等等，返回全部指令数组
    def GenerateCmd(self, id, cmd, address, params, params_len):
        #整体数组长度等于参数数组长度多了11个字节
        buffer = [0]*(params_len+11)
        #帧头1字节
        buffer[0] = 0xAA
        #帧头2字节
        buffer[1] = 0xAA
        #帧id字节
        buffer[2] = id
        #数据长度字节，参数数组长度+4
        buffer[3] = params_len + 4    
        #指令1字节
        buffer[4] = cmd
        #地址低8位字节
        buffer[5] = address & 0xFF
        #地址高8位字节(右移八位)
        buffer[6] = (address>>8) & 0xFF
        # 将params数组里的每一个元素值赋值给buffer[7]以后的元素
        for i in range(params_len):
            buffer[7 + i] = params[i]
        # 临时数组，用于计算crc校验
        temp_msgs =[0]*(params_len + 5)
        # 给临时数组赋值
        temp_msgs[0] = id
        temp_msgs[1] = params_len + 4
        temp_msgs[2] = cmd
        temp_msgs[3] = address & 0xFF
        temp_msgs[4] = (address>>8) & 0xFF
        # 将params数组里的每一个元素值赋值给buffer[7]以后的元素
        for i in range(params_len):
            temp_msgs[5 + i] = params[i]
        # 计算crc校验
        check = self.crc16_calc(temp_msgs, params_len + 5)
        # 校验低八位字节
        buffer[params_len+7] = check & 0xFF
        # 校验高八位字节
        buffer[params_len+8] = (check>>8) & 0xFF
        # 帧尾1字节
        buffer[params_len+9] = 0x55
        # 帧尾2字节
        buffer[params_len+10] = 0x55
        # 返回数组
        return buffer
    
    #CRC校验函数
    def crc16_calc(self, _pBuf, _usLen):
        ucCRCHi = 0xFF
        ucCRCLo = 0xFF
        for i in range(_usLen):
            usIndex = ucCRCLo ^ _pBuf[i]
            ucCRCLo = ucCRCHi ^ s_crchi[usIndex]
            ucCRCHi = s_crclo[usIndex]
        result = (ucCRCHi << 8) | ucCRCLo
        return result
