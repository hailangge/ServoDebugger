# core/modbus_worker.py
import time
from PyQt6.QtCore import QObject, pyqtSignal, QThread
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

class ModbusWorker(QObject):
    connection_status = pyqtSignal(bool, str)
    log_message = pyqtSignal(str, str)
    read_result = pyqtSignal(int, object) # address, value or exception
    write_result = pyqtSignal(int, bool, object) # address, success, value or exception

    def __init__(self, port, baudrate, parity, stopbits, timeout):
        super().__init__()
        self.client = ModbusSerialClient(
            method='rtu',
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout
        )
        self.is_running = True

    def connect_device(self):
        try:
            if self.client.connect():
                self.connection_status.emit(True, f"成功连接到 {self.client.port}")
                self.log_message.emit("info", f"串口 {self.client.port} 已连接。")
            else:
                raise ConnectionError("连接失败")
        except Exception as e:
            self.connection_status.emit(False, f"连接失败: {e}")
            self.log_message.emit("error", f"连接串口 {self.client.port} 失败: {e}")

    def disconnect_device(self):
        if self.client.is_socket_open():
            self.client.close()
        self.is_running = False
        self.connection_status.emit(False, "已断开连接")
        self.log_message.emit("info", "连接已断开。")

    def read_register(self, address):
        if not self.client.is_socket_open():
            self.read_result.emit(address, ModbusException("客户端未连接"))
            return
        try:
            self.log_message.emit("info", f"读取寄存器: 地址={address}")
            # Modbus功能码 0x03 (Read Holding Registers)
            rr = self.client.read_holding_registers(address, 1, slave=1)
            if rr.isError():
                raise ModbusException(str(rr))
            value = rr.registers[0]
            self.log_message.emit("info", f"读取成功: 地址={address}, 值={value}")
            self.read_result.emit(address, value)
        except Exception as e:
            self.log_message.emit("error", f"读取地址 {address} 失败: {e}")
            self.read_result.emit(address, e)

    def write_register(self, address, value):
        if not self.client.is_socket_open():
            self.write_result.emit(address, False, ModbusException("客户端未连接"))
            return
        try:
            self.log_message.emit("info", f"写入寄存器: 地址={address}, 值={value}")
            # Modbus功能码 0x06 (Write Single Register)
            rq = self.client.write_register(address, value, slave=1)
            if rq.isError():
                raise ModbusException(str(rq))
            self.log_message.emit("info", f"写入成功: 地址={address}, 值={value}")
            self.write_result.emit(address, True, value)
        except Exception as e:
            self.log_message.emit("error", f"写入地址 {address} 失败: {e}")
            self.write_result.emit(address, False, e)