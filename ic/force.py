import socket
import struct
import time


PORT = 49152  # Ethernet DAQ使用的端口号
SAMPLE_COUNT = 100000  # 10个输入样本
SPEED = 10  # 1000 / SPEED = 频率（单位：赫兹）;  //0-255，频率通过 1000/num 计算
FILTER = 4  # 0 = 无滤波; 1 = 500赫兹; 2 = 150赫兹; 3 = 50赫兹; 4 = 15赫兹; 5 = 5赫兹; 6 = 1.5赫兹
BIASING_ON = 0xFF  # 开启偏置
BIASING_OFF = 0x00  # 关闭偏置
COMMAND_START = 0x0002  # Command for start streaming
COMMAND_STOP = 0x0000  # Command for stop streaming
COMMAND_BIAS = 0x0042  # Command for toggle biasing
COMMAND_FILTER = 0x0081  # Command for setting filter
COMMAND_SPEED = 0x0082  # Command for setting speed
# force = []

# def connect_force():
#     s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
#     s.connect(('192.168.2.3',49152))
#     send_command(s, COMMAND_SPEED, SPEED)
#     send_command(s, COMMAND_FILTER, FILTER)
#     send_command(s, COMMAND_BIAS, BIASING_ON)
#     send_command(s, COMMAND_START, SAMPLE_COUNT)
#     return s

# def send_command(sock, command, data):
#     request = bytearray(8)
#     struct.pack_into('!H', request, 0, 0x1234)
#     struct.pack_into('!H', request, 2, command)
#     struct.pack_into('!L', request, 4, data)
#     sock.send(request)
#     time.sleep(0.005)  # Wait a little, assuming the command has been processed by the Ethernet DAQ

# #处理数据
# def parse_data(inBuffer):
#     FORCE_DIV = 10000.0 
#     cur = list(range(6))
#     uItems = 0
#     cur[0] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
#     uItems += 1
#     cur[1] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
#     uItems += 1
#     cur[2] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
#     uItems += 1
#     cur[3] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
#     uItems += 1
#     cur[4] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
#     uItems += 1
#     cur[5] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
#     return cur


# def filt():
#     global force
#     for i in range(0,len(force)):
#         if force[i]<1 and force[i]>-1:
#             force[i] = 0

# def get_force(s):
#     global force
#     while True:
#         inBuffer = s.recv(36)
#         force = parse_data(inBuffer)
#         filt()

class Force():
    # PORT = 49152  # Ethernet DAQ使用的端口号
    # SAMPLE_COUNT = 100000  # 10个输入样本
    # SPEED = 10  # 1000 / SPEED = 频率（单位：赫兹）;  //0-255，频率通过 1000/num 计算
    # FILTER = 4  # 0 = 无滤波; 1 = 500赫兹; 2 = 150赫兹; 3 = 50赫兹; 4 = 15赫兹; 5 = 5赫兹; 6 = 1.5赫兹
    # BIASING_ON = 0xFF  # 开启偏置
    # BIASING_OFF = 0x00  # 关闭偏置
    # COMMAND_START = 0x0002  # Command for start streaming
    # COMMAND_STOP = 0x0000  # Command for stop streaming
    # COMMAND_BIAS = 0x0042  # Command for toggle biasing
    # COMMAND_FILTER = 0x0081  # Command for setting filter
    # COMMAND_SPEED = 0x0082  # Command for setting speed
    def __init__(self,ip='192.168.2.3',port=49152):
        self.ip=ip
        self.port=port
        self.socket=self.__connect_force()
        self.force=[0,0,0,0,0,0]
        self.__init()


    
    def get_force(self):
        while True:
            inBuffer = self.socket.recv(36)
            force = self.__parse_data(inBuffer)
            self.force = self.__filt(force)

    def __connect_force(self):
        s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        s.connect((self.ip,self.port))
        return s
    
    def __init(self):
        self.__send_command(COMMAND_SPEED, SPEED)
        self.__send_command(COMMAND_FILTER, FILTER)
        self.__send_command(COMMAND_BIAS, BIASING_ON)
        self.__send_command(COMMAND_START, SAMPLE_COUNT)

    def __send_command(self, command, data):
        request = bytearray(8)
        struct.pack_into('!H', request, 0, 0x1234)
        struct.pack_into('!H', request, 2, command)
        struct.pack_into('!L', request, 4, data)
        self.socket.send(request)
        time.sleep(0.005)  # Wait a little, assuming the command has been processed by the Ethernet DAQ
    
    def __parse_data(self, inBuffer):
        FORCE_DIV = 10000.0 
        cur = list(range(6))
        uItems = 0
        cur[0] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
        uItems += 1
        cur[1] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
        uItems += 1
        cur[2] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
        uItems += 1
        cur[3] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
        uItems += 1
        cur[4] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
        uItems += 1
        cur[5] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
        return cur
    
    def __filt(self,force):
        for i in range(0,len(force)):
            if force[i]<1 and force[i]>-1:
                force[i] = 0
        return force
    
# def initforce():
#     force = Force()