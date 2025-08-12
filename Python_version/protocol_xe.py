import serial
import struct
import time
import threading

"""
协议说明：
    1.无论是发送还是接受，都需要先定义DataType类中的数据类型
    2.无论是发送还是接受都要先初始化串口
协议发送使用方法（不用单开线程）：
    1.自定义对应消息发送函数（如send_drone_position）
    2.调用此函数即可
协议接收使用方法（需要单开线程）：
    1.自定义对应消息处理函数（如handle_drone_position）
    2.message_handle 函数中加入你刚刚定义的处理函数
    3.创建ProtocolReceiver类的实例，并传入已初始化的串口对象
    4.调用start()方法启动接收线程
"""

# =============================================协议常量定义================================
FRAME_HEADER = b'\xAB\xCD'
MAX_PAYLOAD_SIZE = 255

class DataType:
    """协议中定义的数据类型枚举"""
    DRONE_POSITION = 0x01

    # 在这里添加更多的数据类型...

# CRC 计算
def crc8_maxim(data: bytes) -> int:
    """
    计算给定字节串的 CRC-8/MAXIM 校验值。

    Args:
        data: 需要计算CRC的字节串 (bytes)。

    Returns:
        计算出的8位CRC校验值 (int)。
    """
    crc = 0x00  # 初始值
    poly = 0x31 # 多项式
    
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                # 反转后的多项式是 0x8C
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

# ==========================================================协议接收====================================================
# 接收类
class ProtocolReceiver(threading.Thread):
    def __init__(self, ser: serial.Serial, start_fly_trajectory_event:threading.Event, exit_flag:threading.Event):
        super().__init__()
        self.ser = ser
        self.start_fly_trajectory_event = start_fly_trajectory_event
        self.exit_flag = exit_flag

        # 状态机状态
        self.state = 'WAIT_HEADER_1'
        self.buffer = bytearray()
        self.data_type = 0
        self.data_len = 0

    def run(self):
        """线程的主执行体，不断读取和解析串口数据。"""
        print("Receiver thread started.")
        while not self.exit_flag.is_set():
            try:
                # read(1)会阻塞直到读取一个字节或超时
                if self.ser.in_waiting > 0:
                    # 超时返回空字节
                    byte = self.ser.read(1)
                    if byte:
                        self._parse_byte(ord(byte)) # ord()将bytes(长度1)转为int
 
            except serial.SerialException as e:
                print(f"Receiver thread error: {e}")

        print("Receiver thread finished.")

    def message_handle(self, data_type: int, payload: bytes):
        """
        处理接收到的完整数据帧。

        Args:
            data_type: 数据类型 (来自DataType类)。
            payload: 数据体 (bytes)。
        """
        # 在这里添加对不同数据类型的处理逻辑
        if data_type == DataType.DRONE_POSITION:
            if len(payload) == 12:
                handle_drone_position(payload)

    def _parse_byte(self, byte: int):
        """协议解析状态机，处理单个字节。"""
        if self.state == 'WAIT_HEADER_1':
            if byte == FRAME_HEADER[0]:
                self.state = 'WAIT_HEADER_2'
        
        elif self.state == 'WAIT_HEADER_2':
            if byte == FRAME_HEADER[1]:
                self.buffer.clear()
                self.state = 'READ_TYPE'
            else:
                if byte == FRAME_HEADER[0]:
                    self.state = 'WAIT_HEADER_2'
                else:
                    self.state = 'WAIT_HEADER_1'
        
        elif self.state == 'READ_TYPE':
            self.data_type = byte
            self.buffer.append(byte)
            self.state = 'READ_LENGTH'

        elif self.state == 'READ_LENGTH':
            self.data_len = byte
            self.buffer.append(byte)
            if self.data_len == 0:
                self.state = 'READ_CRC'
            else:
                self.state = 'READ_PAYLOAD'

        elif self.state == 'READ_PAYLOAD':
            self.buffer.append(byte)
            # 检查是否已接收完所有数据体字节 (类型+长度+数据体)
            if len(self.buffer) == self.data_len + 2:
                self.state = 'READ_CRC'

        elif self.state == 'READ_CRC':
            received_crc = byte
            calculated_crc = crc8_maxim(self.buffer)
            if calculated_crc == received_crc:
                # 校验成功，将完整帧放入队列
                # 帧内容: 数据体
                payload = self.buffer[2:]
                # 分发到不同函数
                self.message_handle(self.data_type, payload)

            else:
                # 丢弃数据包
                # print(f"CRC Error! Calc: {calculated_crc:02X}, Recv: {received_crc:02X}")
                pass
            
            # 无论成功失败，重置状态机
            self.state = 'WAIT_HEADER_1'

# 消息处理函数
def handle_drone_position(payload: bytes):
    """
    处理无人机位置数据。

    Args:
        payload: 数据体 (bytes)，应为12字节，包含3个float (X, Y, Z)。
    """
    if len(payload) != 12:
        return

    x, y, z = struct.unpack('<fff', payload)
    print(f"Drone Position - X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")


# ==========================================================协议发送====================================================

def protocol_send_frame(ser: serial.Serial, data_type: int, payload: bytes):
    """
    打包并发送一个完整的协议帧。

    此函数负责计算长度、计算CRC，并按协议格式将所有字节发送出去。

    Args:
        ser: 已初始化的 `serial.Serial` 对象。
        data_type: 消息的数据类型 (来自DataType类)。
        payload: 消息的数据体 (bytes)。
    """
    payload_len = len(payload)
    if payload_len > MAX_PAYLOAD_SIZE:
        print(f"Error: Payload size {payload_len} exceeds max {MAX_PAYLOAD_SIZE}. Frame not sent.")
        return

    # 1. 准备要计算CRC的数据部分: [类型, 长度, 数据体]
    # struct.pack('<B', val) 将一个整数打包成一个字节
    # '<' 表示小端序 (对于单字节无影响，但养成好习惯)
    crc_data = struct.pack('<B', data_type) + struct.pack('<B', payload_len) + payload

    # 2. 计算CRC
    crc_value = crc8_maxim(crc_data)

    # 3. 构造完整的帧: [帧头, 类型, 长度, 数据体, CRC]
    frame = FRAME_HEADER + crc_data + struct.pack('<B', crc_value)

    # 4. 发送
    try:
        ser.write(frame)
        # # 打印发送的内容以供调试 (可选)
        # print(f"Sent Frame -> Type: 0x{data_type:02X}, Len: {payload_len}, Payload: {payload.hex(' ').upper()}, CRC: 0x{crc_value:02X}")
        # print(f"Full Frame (hex): {frame.hex(' ').upper()}")
    except serial.SerialException as e:
        print(f"Error writing to serial port: {e}")

# 消息处理函数
def send_drone_position(ser: serial.Serial, x: float, y: float, z: float):
    """
    发送无人机坐标信息。

    Args:
        ser: 已初始化的 `serial.Serial` 对象。
        x: X轴坐标 (float)。
        y: Y轴坐标 (float)。
        z: Z轴坐标 (float)。
    """
    # 使用 struct.pack 将三个 float 打包成一个12字节的字节串
    # '<f' 表示小端序的单精度浮点数
    payload = struct.pack('<fff', x, y, z)
    protocol_send_frame(ser, DataType.DRONE_POSITION, payload)

# =============================================================================
#                               使用示例
# =============================================================================

# # 发送示例
# if __name__ == "__main__":
#     # --- 串口初始化 ---
#     # (COMx on Windows, /dev/ttyUSBx or /dev/ttyACMx on Linux)
#     SERIAL_PORT = 'COM1'  
#     BAUD_RATE = 115200
#     ser = None
#     try:
#         ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
#         print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} bps.")
#     except serial.SerialException as e:
#         print(f"Error opening serial port {SERIAL_PORT}: {e}")
#         exit()

#     # 发送命令演示 
#     try:
#         while True:
#             print("\n--- Sending DRONE_POSITION (1.0, 2.0, 3.0) ---")
#             send_drone_position(ser, 1.0, 2.0, 3.0)
#             time.sleep(2)  # 等待2秒

  
#             print("\n--- Sending DRONE_POSITION (-12.34, 56.78, -90.0) ---")
#             send_drone_position(ser, -12.34, 56.78, -90.0)
#             time.sleep(2)  # 等待2秒

#     except KeyboardInterrupt:
#         print("\nProgram interrupted by user. Closing serial port.")
#     finally:
#         if ser and ser.is_open:
#             ser.close()
#             print("Serial port closed.")

# # 接收示例
# if __name__ == "__main__":
#     # --- 串口初始化 ---
#     SERIAL_PORT = 'COM1' 
#     BAUD_RATE = 115200
#     ser = None
#     try:
#         ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) # 使用短超时，防止run循环卡死
#         print(f"Serial port {SERIAL_PORT} opened successfully.")
#     except serial.SerialException as e:
#         print(f"Error opening serial port: {e}")
#         exit()

#     # 创建并启动接收线程
#     receiver_thread = ProtocolReceiver(ser)
#     receiver_thread.start()

#     # 主循环
#     try:
#         while True:
#             # 让主循环稍微休息一下，避免CPU占用过高
#             time.sleep(0.01)

#     # Ctrl+C 退出
#     except KeyboardInterrupt:
#         print("\n[Main Thread] Program interrupted by user.")
#     finally:
#         # --- 4. 清理和退出 ---
#         print("[Main Thread] Stopping receiver thread...")
#         receiver_thread.stop()
#         receiver_thread.join() # 等待线程完全退出

#         if ser and ser.is_open:
#             ser.close()
#             print("[Main Thread] Serial port closed.")