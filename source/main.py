import sys
import time
from datetime import datetime
from collections import defaultdict

# 尝试导入必要的库，如果失败则提示用户安装
try:
    from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                                 QLabel, QComboBox, QPushButton, QTabWidget,
                                 QSpinBox, QTextEdit, QMessageBox, QGroupBox, QScrollArea, QLayout, QGridLayout)
    from PyQt6.QtCore import Qt, pyqtSignal, QObject, QThread, QSize, QRect, QPoint
    from PyQt6.QtGui import QColor, QTextCharFormat, QFont, QPainter

    from pymodbus.client import ModbusSerialClient
    from pymodbus.exceptions import ModbusException
    from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder
    from pymodbus.constants import Endian
except ImportError as e:
    print(f"错误: 缺少必要的库 -> {e}")
    print("请使用以下命令安装所有依赖:")
    print("pip install PyQt6 pymodbus pyserial")
    sys.exit(1)

# ==============================================================================
# PART 1: COMPLETE AND FINAL REGISTER CONFIGURATION
# ==============================================================================
# ==============================================================================
# PART 1: ADVANCED REGISTER CONFIGURATION
# ==============================================================================
PA_BASE_ADDRESS = 4000  # Assumption for电机参数区

# ==============================================================================
# 7.1 监控参数 (SU-XX) - 只读 (V2 - 单位已整合入名称)
# 地址严格遵循手册 P93 MODBUS通讯地址表
# ==============================================================================
MONITORING_PARAMETERS = [
    # --- 驱动器状态 ---
    {"id": "SU-00", "name": "驱动器输出电流 (0.1A)", "group": "监控参数", "sub_group": "驱动器状态", "type": "s32", "address": 900,
     "word_order": "big", "read_only": True, "effect": "只读", "tooltip": "有效值"},
    {"id": "SU-01", "name": "驱动器母线电压 (V)", "group": "监控参数", "sub_group": "驱动器状态", "type": "u32", "address": 902,
     "word_order": "big", "read_only": True, "effect": "只读"},
    {"id": "SU-18", "name": "驱动器当前温度 (℃)", "group": "监控参数", "sub_group": "驱动器状态", "type": "s16", "address": 0,
     "read_only": True, "effect": "只读", "note": "P93未定义地址"},

    # --- 电机状态 ---
    {"id": "SU-02", "name": "伺服电机转速 (0.1r/min)", "group": "监控参数", "sub_group": "电机状态", "type": "s32", "address": 904,
     "word_order": "big", "read_only": True, "effect": "只读"},
    {"id": "SU-19", "name": "转动惯量显示 (0.01)", "group": "监控参数", "sub_group": "电机状态", "type": "u16", "address": 0,
     "read_only": True, "effect": "只读", "note": "P93未定义地址"},
    {"id": "SU-20", "name": "当前输出转矩 (%)", "group": "监控参数", "sub_group": "电机状态", "type": "s16", "address": 942,
     "read_only": True, "effect": "只读", "tooltip": "额定转矩百分比"},

    # --- 反馈位置 (相对) ---
    {"id": "SU-03_04", "name": "反馈相对位置-单圈", "group": "监控参数", "sub_group": "反馈位置 (相对)", "type": "u32", "address": 906,
     "word_order": "big", "read_only": True, "effect": "只读", "tooltip": "由SU-04(低)和SU-03(高)组成"},
    {"id": "SU-05_06", "name": "反馈相对位置-多圈", "group": "监控参数", "sub_group": "反馈位置 (相对)", "type": "s32", "address": 908,
     "word_order": "big", "read_only": True, "effect": "只读", "tooltip": "由SU-06(低)和SU-05(高)组成"},

    # --- 反馈位置 (绝对) ---
    {"id": "SU-23_24", "name": "绝对位置-单圈", "group": "监控参数", "sub_group": "反馈位置 (绝对)", "type": "u32", "address": 952,
     "word_order": "little", "read_only": True, "effect": "只读", "tooltip": "由SU-24(低)和SU-23(高)组成"},
    {"id": "SU-25_26", "name": "绝对位置-多圈", "group": "监控参数", "sub_group": "反馈位置 (绝对)", "type": "s32", "address": 954,
     "word_order": "little", "read_only": True, "effect": "只读", "tooltip": "由SU-26(低)和SU-25(高)组成"},

    # --- 指令状态 ---
    {"id": "SU-07_08", "name": "给定指令脉冲数", "group": "监控参数", "sub_group": "指令状态", "type": "u32", "address": 910,
     "word_order": "big", "read_only": True, "effect": "只读", "tooltip": "由SU-08(低)和SU-07(高)组成"},
    {"id": "SU-09", "name": "指令脉冲偏差计数", "group": "监控参数", "sub_group": "指令状态", "type": "s32", "address": 912,
     "word_order": "big", "read_only": True, "effect": "只读"},
    {"id": "SU-10", "name": "给定速度 (0.1r/min)", "group": "监控参数", "sub_group": "指令状态", "type": "s32", "address": 914,
     "word_order": "big", "read_only": True, "effect": "只读"},
    {"id": "SU-11", "name": "给定转矩 (%)", "group": "监控参数", "sub_group": "指令状态", "type": "s16", "address": 916,
     "read_only": True, "effect": "只读", "tooltip": "1% 额定转矩"},
    {"id": "SU-12", "name": "模拟量速度指令 (r/min)", "group": "监控参数", "sub_group": "指令状态", "type": "s16", "address": 919,
     "read_only": True, "effect": "只读"},
    {"id": "SU-13", "name": "模拟量转矩指令 (0.1%)", "group": "监控参数", "sub_group": "指令状态", "type": "s16", "address": 921,
     "read_only": True, "effect": "只读", "tooltip": "0.1% 额定转矩"},

    # --- I/O 状态 ---
    {"id": "SU-14_15", "name": "输入口(DI)状态", "group": "监控参数", "sub_group": "I/O 状态", "type": "bit_field", "address": 923,
     "read_only": True, "effect": "只读", "tooltip": "组合DI1-DI8的状态", "fields": [
        {"name": "DI1", "start_bit": 0, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DI2", "start_bit": 1, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DI3", "start_bit": 2, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DI4", "start_bit": 3, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DI5", "start_bit": 4, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DI6", "start_bit": 5, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DI7", "start_bit": 6, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DI8", "start_bit": 7, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
    ]},
    {"id": "SU-16_17", "name": "输出口(DO)状态", "group": "监控参数", "sub_group": "I/O 状态", "type": "bit_field", "address": 925,
     "read_only": True, "effect": "只读", "tooltip": "组合DO1-DO4和ALM的状态", "fields": [
        {"name": "DO1", "start_bit": 0, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DO2", "start_bit": 1, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DO3", "start_bit": 2, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "DO4", "start_bit": 3, "length": 1, "type": "enum", "options": {0: "断开", 1: "闭合"}},
        {"name": "ALM", "start_bit": 8, "length": 1, "type": "enum", "options": {0: "正常", 1: "报警"}},
    ]},

    # --- 高级状态 ---
    {"id": "SU-21", "name": "当前增益组", "group": "监控参数", "sub_group": "高级状态", "type": "u16", "address": 0,
     "read_only": True, "effect": "只读", "note": "P93未定义地址"},
    {"id": "SU-22", "name": "泄放时间 (10ms)", "group": "监控参数", "sub_group": "高级状态", "type": "u16", "address": 0,
     "read_only": True, "effect": "只读", "note": "P93未定义地址"},
    {"id": "SU-27", "name": "模拟量通道AV电压 (10mV)", "group": "监控参数", "sub_group": "高级状态", "type": "s16", "address": 940,
     "read_only": True, "effect": "只读"},
    {"id": "SU-28", "name": "模拟量通道AC电压 (10mV)", "group": "监控参数", "sub_group": "高级状态", "type": "s16", "address": 941,
     "read_only": True, "effect": "只读"},
    {"id": "SU-29", "name": "混合误差", "group": "监控参数", "sub_group": "高级状态", "type": "u16", "address": 0,
     "read_only": True, "effect": "只读", "note": "P93未定义地址"},
    {"id": "SU-30", "name": "全闭环反馈", "group": "监控参数", "sub_group": "高级状态", "type": "u16", "address": 0,
     "read_only": True, "effect": "只读", "note": "P93未定义地址"},
    {"id": "SU-31", "name": "龙门同步误差", "group": "监控参数", "sub_group": "高级状态", "type": "u16", "address": 0,
     "read_only": True, "effect": "只读", "note": "P93未定义地址"},
    {"id": "SU-32", "name": "保留", "group": "监控参数", "sub_group": "高级状态", "type": "u16", "address": 0, "read_only": True,
     "effect": "只读", "note": "P93未定义地址"},

    # --- 高速计数器 ---
    {"id": "SU-33", "name": "高速计数器1", "group": "监控参数", "sub_group": "高速计数器", "type": "u32", "address": 0,
     "read_only": True, "effect": "只读", "note": "P93未定义地址"},
    {"id": "SU-34", "name": "高速计数器2", "group": "监控参数", "sub_group": "高速计数器", "type": "u32", "address": 0,
     "read_only": True, "effect": "只读", "note": "P93未定义地址"},
    {"id": "SU-36_37", "name": "接收脉冲数", "group": "监控参数", "sub_group": "高速计数器", "type": "u32", "address": 0,
     "read_only": True, "effect": "只读", "tooltip": "由SU-37(低)和SU-36(高)组成", "note": "P93未定义地址"},
]

# ==============================================================================
# 7.2 辅助参数 (AU-XX)
# 地址规则: 参数号 + 800
# ==============================================================================
AUXILIARY_PARAMETERS = [
    # --- 基本信息与安全 ---
    {"id": "AU-00", "name": "软件版本", "group": "辅助参数", "sub_group": "基本信息与安全", "type": "u16", "address": 800,
     "read_only": True, "effect": "只读"},
    {"id": "AU-01", "name": "设定密码", "group": "辅助参数", "sub_group": "基本信息与安全", "type": "u16", "address": 801,
     "read_only": False, "range": "0-9999", "effect": "重新上电保存", "tooltip": "0表示密码无效, 可任意修改参数"},

    # --- 抱闸与制动 ---
    {"id": "AU-02", "name": "抱闸OFF延迟-使能OFF (10ms)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16",
     "address": 802, "read_only": False, "range": "0-50", "effect": "立即"},
    {"id": "AU-03", "name": "抱闸OFF延迟-电磁制动 (10ms)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16", "address": 803,
     "read_only": False, "range": "10-100", "effect": "立即"},
    {"id": "AU-04", "name": "外置制动电阻阻值 (Ω)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16", "address": 804,
     "read_only": False, "range": "8-1000", "effect": "立即"},
    {"id": "AU-05", "name": "泄放占空比 (%)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16", "address": 805,
     "read_only": False, "range": "0-100", "effect": "立即"},
    {"id": "AU-08", "name": "动态制动延时时间 (0.1ms)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16", "address": 808,
     "read_only": False, "range": "100-30000", "effect": "立即"},
    {"id": "AU-28", "name": "断电抱闸", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "enum16", "address": 828,
     "read_only": False, "options": {0: "关闭", 1: "开启"}, "effect": "立即"},
    {"id": "AU-29", "name": "断电抱闸延时时间 (0.5ms)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16", "address": 829,
     "read_only": False, "range": "500-3000", "effect": "立即"},
    {"id": "AU-16", "name": "电磁制动速度阈值 (0.1r/min)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16", "address": 816,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "AU-44", "name": "内置制动电阻阻值 (Ω)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16", "address": 844,
     "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "AU-45", "name": "内置制动电阻功率 (w)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16", "address": 845,
     "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "AU-46", "name": "电阻散热系数 (%)", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "u16", "address": 846,
     "read_only": False, "range": "1-50", "effect": "立即"},
    {"id": "AU-47", "name": "制动电阻设置", "group": "辅助参数", "sub_group": "抱闸与制动", "type": "enum16", "address": 847,
     "read_only": False, "options": {0: "使用内置", 1: "使用外接"}, "effect": "立即"},

    # --- 保护功能 ---
    {"id": "AU-06", "name": "输入电源缺相保护", "group": "辅助参数", "sub_group": "保护功能", "type": "enum16", "address": 806,
     "read_only": False, "options": {0: "屏蔽", 1: "开启"}, "effect": "立即"},
    {"id": "AU-15", "name": "编码器断线保护", "group": "辅助参数", "sub_group": "保护功能", "type": "enum16", "address": 815,
     "read_only": False, "options": {0: "关闭", 1: "开启"}, "effect": "立即"},
    {"id": "AU-17", "name": "正转禁止设置", "group": "辅助参数", "sub_group": "保护功能", "type": "enum16", "address": 817,
     "read_only": False, "options": {0: "无效", 1: "有效"}, "effect": "立即"},
    {"id": "AU-18", "name": "反转禁止设置", "group": "辅助参数", "sub_group": "保护功能", "type": "enum16", "address": 818,
     "read_only": False, "options": {0: "无效", 1: "有效"}, "effect": "立即"},
    {"id": "AU-31", "name": "驱动器过热报警温度 (℃)", "group": "辅助参数", "sub_group": "保护功能", "type": "u16", "address": 831,
     "read_only": False, "range": "60-100", "effect": "立即"},
    {"id": "AU-32", "name": "接地保护", "group": "辅助参数", "sub_group": "保护功能", "type": "enum16", "address": 832,
     "read_only": False, "options": {0: "屏蔽", 1: "开启"}, "effect": "立即"},
    {"id": "AU-34", "name": "电机堵转保护", "group": "辅助参数", "sub_group": "保护功能", "type": "enum16", "address": 834,
     "read_only": False, "options": {0: "屏蔽", 1: "开启"}, "effect": "立即"},
    {"id": "AU-35", "name": "过载预警信号输出电流 (%)", "group": "辅助参数", "sub_group": "保护功能", "type": "u16", "address": 835,
     "read_only": False, "range": "0-800", "effect": "立即"},
    {"id": "AU-36", "name": "过载预警滤波时间 (10ms)", "group": "辅助参数", "sub_group": "保护功能", "type": "u16", "address": 836,
     "read_only": False, "range": "0-1000", "effect": "立即"},
    {"id": "AU-37", "name": "电机过载系数设定 (%)", "group": "辅助参数", "sub_group": "保护功能", "type": "u16", "address": 837,
     "read_only": False, "range": "0-500", "effect": "立即"},
    {"id": "AU-38", "name": "锂电池欠压保护", "group": "辅助参数", "sub_group": "保护功能", "type": "enum16", "address": 838,
     "read_only": False, "options": {0: "屏蔽", 1: "开启"}, "effect": "立即"},
    {"id": "AU-39", "name": "软件超程保护", "group": "辅助参数", "sub_group": "保护功能", "type": "enum16", "address": 839,
     "read_only": False, "options": {0: "屏蔽", 1: "开启", 2: "停机但不报警"}, "effect": "立即"},
    {"id": "AU-40", "name": "堵转保护判断时间 (10ms)", "group": "辅助参数", "sub_group": "保护功能", "type": "u16", "address": 840,
     "read_only": False, "range": "10-1000", "effect": "立即"},

    # --- 系统行为与设置 ---
    {"id": "AU-07", "name": "电机停止模式", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "enum16", "address": 807,
     "read_only": False, "options": {0: "自由停车", 1: "动态制动", 2: "快速使能"}, "effect": "立即"},
    {"id": "AU-09", "name": "默认状态显示设置", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "enum16", "address": 809,
     "read_only": False, "range": "0-34", "effect": "立即",
     "options": {0: "输出电流", 1: "母线电压", 2: "电机转速", 3: "反馈脉冲(高)", 4: "反馈脉冲(低)", 5: "反馈转速(高)", 6: "反馈转速(低)", 7: "指令脉冲(高)",
                 8: "指令脉冲(低)", 9: "脉冲误差", 10: "给定速度", 11: "给定转矩", 12: "模拟速度指令", 13: "模拟转矩指令", 14: "DI8-5状态",
                 15: "DI4-1状态", 16: "其余输出口状态", 17: "DO4-D01状态", 18: "驱动器温度", 19: "转动惯量", 20: "输出转矩", 21: "增益组",
                 22: "泄放时间", 23: "绝对位置(高)", 24: "绝对位置(低)", 25: "绝对位置圈数(高)", 26: "绝对位置圈数(低)", 27: "AV电压", 28: "AC电压",
                 29: "混合误差", 30: "全闭环反馈", 31: "龙门同步误差", 32: "保留", 33: "高速计数器1", 34: "高速计数器2"}},
    {"id": "AU-13", "name": "JOG点动速度 (0.1r/min)", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "u16",
     "address": 813, "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "AU-14", "name": "JOG点动运行", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "u16", "address": 814,
     "read_only": True, "effect": "立即", "tooltip": "此为功能触发, 一般不直接写入"},
    {"id": "AU-23", "name": "转矩更新周期", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "enum16", "address": 823,
     "read_only": False, "options": {0: "100ms", 1: "1ms"}, "effect": "0->1"},
    {"id": "AU-25", "name": "电角度识别设定", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "enum16", "address": 825,
     "read_only": False, "options": {0: "不设定", 1: "设定"}, "effect": "立即"},
    {"id": "AU-30", "name": "绝对/相对位置设定", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "enum16", "address": 830,
     "read_only": False, "options": {0: "绝对位置", 1: "相对位置"}, "effect": "重新上电保存"},
    {"id": "AU-33", "name": "掉电去使能", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "enum16", "address": 833,
     "read_only": False, "options": {0: "屏蔽", 1: "开启"}, "effect": "立即"},
    {"id": "AU-41", "name": "设定机械原点", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "enum16", "address": 841,
     "read_only": False, "options": {0: "不设置", 1: "将当前位置设为原点"}, "effect": "立即"},
    {"id": "AU-42", "name": "报警输出占空比 (%)", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "u16", "address": 842,
     "read_only": False, "range": "0-100", "effect": "立即"},
    {"id": "AU-43", "name": "编码器复位", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "enum16", "address": 843,
     "read_only": False, "options": {0: "不复位", 1: "复位"}, "effect": "立即"},
    {"id": "AU-48", "name": "电机参数设置区密码", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "u16", "address": 848,
     "read_only": False, "effect": "立即", "tooltip": "设为1时可对电机参数(PA)进行设置"},
    {"id": "AU-49", "name": "恢复出厂设置", "group": "辅助参数", "sub_group": "系统行为与设置", "type": "enum16", "address": 849,
     "read_only": False, "options": {0: "不恢复", 1: "恢复出厂", 2: "Flash参数恢复"}, "effect": "重新上电不保存"},

    # --- 故障历史 ---
    {"id": "AU-10", "name": "最近一次故障代码", "group": "辅助参数", "sub_group": "故障历史", "type": "u16", "address": 810,
     "read_only": True, "effect": "只读"},
    {"id": "AU-11", "name": "最近第二次故障代码", "group": "辅助参数", "sub_group": "故障历史", "type": "u16", "address": 811,
     "read_only": True, "effect": "只读"},
    {"id": "AU-12", "name": "最近第三次故障代码", "group": "辅助参数", "sub_group": "故障历史", "type": "u16", "address": 812,
     "read_only": True, "effect": "只读"},

    # --- 保留项 ---
    {"id": "AU-19", "name": "保留", "group": "辅助参数", "sub_group": "保留项", "type": "u16", "address": 819,
     "read_only": True},
    {"id": "AU-20", "name": "保留", "group": "辅助参数", "sub_group": "保留项", "type": "u16", "address": 820,
     "read_only": True},
    {"id": "AU-21", "name": "保留", "group": "辅助参数", "sub_group": "保留项", "type": "u16", "address": 821,
     "read_only": True},
    {"id": "AU-22", "name": "保留", "group": "辅助参数", "sub_group": "保留项", "type": "u16", "address": 822,
     "read_only": True},
    {"id": "AU-24", "name": "保留", "group": "辅助参数", "sub_group": "保留项", "type": "u16", "address": 824,
     "read_only": True},
    {"id": "AU-26", "name": "电机编码器数据通讯格式", "group": "辅助参数", "sub_group": "保留项", "type": "enum16", "address": 826,
     "read_only": True, "options": {0: "多圈绝对值", 1: "单圈绝对值"}},
    {"id": "AU-27", "name": "保留", "group": "辅助参数", "sub_group": "保留项", "type": "u16", "address": 827,
     "read_only": True},
]

# ==============================================================================
# 7.3.1 系统参数 (FU000 ~ FU020)
# 地址规则: 参数号
# ==============================================================================
SYSTEM_PARAMETERS_0_20 = [
    # --- 基本配置 ---
    {"id": "FU000", "name": "电机代码", "group": "系统参数", "sub_group": "基本配置", "type": "u16", "address": 0,
     "read_only": True, "effect": "只读"},
    {"id": "FU001", "name": "控制模式与方向", "group": "系统参数", "sub_group": "基本配置", "type": "bit_field", "address": 1,
     "read_only": False, "effect": "重新上电保存", "fields": [
        {"name": "旋转方向", "start_bit": 15, "length": 1, "type": "enum", "options": {0: "顺时针(CW)", 1: "逆时针(CCW)"}},
        {"name": "控制模式", "start_bit": 0, "length": 5, "type": "enum", "options": {
            0: "内部寄存器速度模式", 1: "位置脉冲指令模式", 2: "内部寄存器转矩模式", 3: "外部模拟量速度模式", 4: "外部模拟量转矩模式", 5: "内部寄存器位置模式",
            6: "内部寄存器速度与位置脉冲指令混合模式", 7: "内部寄存器速度与内部寄存器转矩混合模式", 8: "内部寄存器速度与外部模拟量速度混合模式",
            9: "内部寄存器速度与外部模拟量转矩混合模式", 10: "内部寄存器速度与内部寄存器位置混合模式", 11: "内部寄存器转矩与位置脉冲指令混合模式",
            12: "外部模拟量速度与位置脉冲指令混合模式", 13: "外部模拟量转矩与位置脉冲指令混合模式", 14: "位置脉冲指令与内部寄存器位置混合模式",
            15: "外部模拟量速度与内部寄存器转矩混合模式", 16: "外部模拟量转矩与内部寄存器转矩混合模式", 17: "内部寄存器转矩与内部寄存器位置混合模式",
            18: "外部模拟量速度与外部模拟量转矩混合模式", 19: "外部模拟量速度与内部寄存器位置混合模式", 20: "外部模拟量转矩与内部寄存器位置混合模式"}}
    ]},
    {"id": "FU002", "name": "最高转速限制 (r/min)", "group": "系统参数", "sub_group": "基本配置", "type": "u16", "address": 2,
     "read_only": False, "range": "0-10000", "effect": "立即"},
    {"id": "FU004", "name": "伺服使能方式选择", "group": "系统参数", "sub_group": "基本配置", "type": "enum16", "address": 4,
     "read_only": False, "effect": "立即", "options": {0: "外部端子使能(SON-I)", 1: "内部参数使能(FU100)", 2: "强制使能"}},

    # --- 编码器分频 ---
    {"id": "FU003", "name": "编码器脉冲分频数(分子)", "group": "系统参数", "sub_group": "编码器分频", "type": "u16", "address": 3,
     "read_only": False, "range": "1-65535", "effect": "立即"},
    {"id": "FU005", "name": "编码器脉冲分频数(分母)", "group": "系统参数", "sub_group": "编码器分频", "type": "u32", "address": 5,
     "word_order": "big", "read_only": False, "range": "1-2147483647", "effect": "立即"},
    # FU005 and FU006 make a 32-bit value

    # --- 惯量与刚性 ---
    {"id": "FU007", "name": "负载惯量变化速度", "group": "系统参数", "sub_group": "惯量与刚性", "type": "u16", "address": 7,
     "read_only": False, "range": "1-100", "effect": "立即"},
    {"id": "FU008", "name": "转动惯量模式选择", "group": "系统参数", "sub_group": "惯量与刚性", "type": "enum16", "address": 8,
     "read_only": False, "effect": "立即", "options": {0: "不启用识别", 1: "离线正反转识别", 2: "离线单方向识别", 3: "保留"}},
    {"id": "FU009", "name": "离线惯量识别动作间隙 (Ms)", "group": "系统参数", "sub_group": "惯量与刚性", "type": "u16", "address": 9,
     "read_only": False, "range": "10-2000", "effect": "立即"},
    {"id": "FU010", "name": "刚性选择", "group": "系统参数", "sub_group": "惯量与刚性", "type": "u16", "address": 10,
     "read_only": False, "range": "1-30", "effect": "立即"},
    {"id": "FU013", "name": "转动惯量比", "group": "系统参数", "sub_group": "惯量与刚性", "type": "u16", "address": 13,
     "read_only": False, "range": "1-30000", "effect": "立即", "tooltip": "负载惯量与电机转子惯量之比, 乘以100"},
    {"id": "FU014", "name": "运动轨迹加减速时间 (Ms)", "group": "系统参数", "sub_group": "惯量与刚性", "type": "u16", "address": 14,
     "read_only": False, "range": "200-5000", "effect": "立即"},
    {"id": "FU015", "name": "离线惯量识别运动范围(脉冲)", "group": "系统参数", "sub_group": "惯量与刚性", "type": "u32", "address": 15,
     "word_order": "big", "read_only": False, "range": "20-2147483647", "effect": "立即"},

    # --- Z脉冲与分频输出 ---
    {"id": "FU017", "name": "Z脉冲分频输出宽度", "group": "系统参数", "sub_group": "Z脉冲与分频输出", "type": "u16", "address": 17,
     "read_only": False, "range": "30-30000", "effect": "立即"},
    {"id": "FU018", "name": "脉冲输出配置", "group": "系统参数", "sub_group": "Z脉冲与分频输出", "type": "bit_field", "address": 18,
     "read_only": False, "effect": "立即", "fields": [
        {"name": "分频指令来源", "start_bit": 8, "length": 3, "type": "enum",
         "options": {0: "电机轴", 1: "内部位置给定", 2: "集电极脉冲输入", 3: "高速计数器1", 4: "高速计数器2", 5: "位置指令"}},
        {"name": "Z脉冲指令来源", "start_bit": 4, "length": 1, "type": "enum", "options": {0: "电机轴", 1: "虚拟轴"}},
        {"name": "Z脉冲输出极性", "start_bit": 0, "length": 1, "type": "enum", "options": {0: "负极性", 1: "正极性"}}
    ]},
    {"id": "FU019", "name": "虚拟Z输出周期", "group": "系统参数", "sub_group": "Z脉冲与分频输出", "type": "u32", "address": 19,
     "word_order": "big", "read_only": False, "range": "1-2147483647", "effect": "立即"},
    {"id": "FU020", "name": "分频信号设置", "group": "系统参数", "sub_group": "Z脉冲与分频输出", "type": "enum16", "address": 20,
     "read_only": False, "effect": "重新上电保存", "tooltip": "配置特定引脚功能, 详见手册P79", "options": {
        0: "26/10脚=MODBUS, DO1可配",
        1: "26/10脚=MODBUS, DO1=Z相集电极",
        2: "26/10脚=A相差分, 30/15脚=B相差分"
    }},

    # --- 保留项 ---
    {"id": "FU011", "name": "保留", "group": "系统参数", "sub_group": "保留项", "type": "u16", "address": 11, "read_only": True},
    {"id": "FU012", "name": "保留", "group": "系统参数", "sub_group": "保留项", "type": "u16", "address": 12, "read_only": True},
    {"id": "FU016", "name": "保留", "group": "系统参数", "sub_group": "保留项", "type": "u16", "address": 16, "read_only": True}
]

# ==============================================================================
# 7.3.2 速度参数 (FU100 ~ FU145)
# 地址规则: 参数号
# ==============================================================================
VELOCITY_PARAMETERS = [
    # --- 增益与滤波 ---
    {"id": "FU101", "name": "第一速度环比例增益 (0.1Hz)", "group": "速度参数", "sub_group": "增益与滤波", "type": "u16", "address": 101,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU102", "name": "第一速度环积分增益 (0.1ms)", "group": "速度参数", "sub_group": "增益与滤波", "type": "u16", "address": 102,
     "read_only": False, "range": "0-10000", "effect": "立即"},
    {"id": "FU103", "name": "第二速度环比例增益 (0.1Hz)", "group": "速度参数", "sub_group": "增益与滤波", "type": "u16", "address": 103,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU104", "name": "第二速度环积分增益 (0.1ms)", "group": "速度参数", "sub_group": "增益与滤波", "type": "u16", "address": 104,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU105", "name": "第一速度环滤波时间 (0.01ms)", "group": "速度参数", "sub_group": "增益与滤波", "type": "u16", "address": 105,
     "read_only": False, "range": "1-20000", "effect": "立即"},
    {"id": "FU106", "name": "第二速度环滤波时间 (0.01ms)", "group": "速度参数", "sub_group": "增益与滤波", "type": "u16", "address": 106,
     "read_only": False, "range": "1-20000", "effect": "立即"},
    {"id": "FU107", "name": "转矩前馈增益", "group": "速度参数", "sub_group": "增益与滤波", "type": "u16", "address": 107,
     "read_only": False, "range": "0-1000", "effect": "立即"},
    {"id": "FU108", "name": "转矩前馈增益滤波 (0.01ms)", "group": "速度参数", "sub_group": "增益与滤波", "type": "u16", "address": 108,
     "read_only": False, "range": "1-30000", "effect": "立即"},

    # --- 速度与加减速 ---
    {"id": "FU100", "name": "内部使能设置", "group": "速度参数", "sub_group": "速度与加减速", "type": "enum16", "address": 100,
     "read_only": False, "effect": "立即,不记忆", "tooltip": "仅在FU004=1时有效", "options": {0: "外部端子使能", 1: "内部参数使能"}},
    {"id": "FU109", "name": "速度模式加速时间 (ms)", "group": "速度参数", "sub_group": "速度与加减速", "type": "u16", "address": 109,
     "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "FU110", "name": "速度模式减速时间 (ms)", "group": "速度参数", "sub_group": "速度与加减速", "type": "u16", "address": 110,
     "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "FU111", "name": "S曲线加减速时间 (ms)", "group": "速度参数", "sub_group": "速度与加减速", "type": "u16", "address": 111,
     "read_only": False, "range": "1-15000", "effect": "立即"},
    {"id": "FU112", "name": "S曲线启动标志", "group": "速度参数", "sub_group": "速度与加减速", "type": "enum16", "address": 112,
     "read_only": False, "effect": "立即", "options": {0: "不启用", 1: "启用"}},
    {"id": "FU113", "name": "内部速度给定 1 (0.1r/min)", "group": "速度参数", "sub_group": "速度与加减速", "type": "s16",
     "address": 113, "read_only": False, "range": "-32000~+32000", "effect": "立即"},
    {"id": "FU114", "name": "内部速度给定 2 (0.1r/min)", "group": "速度参数", "sub_group": "速度与加减速", "type": "s16",
     "address": 114, "read_only": False, "range": "-32000~+32000", "effect": "立即"},
    {"id": "FU115", "name": "内部速度给定 3 (0.1r/min)", "group": "速度参数", "sub_group": "速度与加减速", "type": "s16",
     "address": 115, "read_only": False, "range": "-32000~+32000", "effect": "立即"},
    {"id": "FU117", "name": "目标速度范围 (0.1r/min)", "group": "速度参数", "sub_group": "速度与加减速", "type": "u16", "address": 117,
     "read_only": False, "range": "0-30000", "effect": "立即", "tooltip": "速度到达信号(V-CMP)的触发范围"},
    {"id": "FU118", "name": "旋转检出值 (0.1r/min)", "group": "速度参数", "sub_group": "速度与加减速", "type": "u16", "address": 118,
     "read_only": False, "range": "0-30000", "effect": "立即", "tooltip": "旋转检出信号(TGON)的触发阈值"},

    # --- 原点回归 ---
    {"id": "FU119", "name": "原点检索", "group": "速度参数", "sub_group": "原点回归", "type": "bit_field", "address": 119,
     "read_only": False, "effect": "立即", "fields": [
        {"name": "[D]寻找方向", "start_bit": 0, "length": 1, "type": "enum", "options": {0: "反转寻找", 1: "正转寻找"}},
        {"name": "[C]原点类型", "start_bit": 4, "length": 3, "type": "enum",
         "options": {0: "左右限位", 1: "ORGP输入端", 2: "Z相脉冲", 3: "寻找机械原点", 4: "力矩到达(T-CMP)"}},
        {"name": "[B]找到后动作", "start_bit": 8, "length": 2, "type": "enum",
         "options": {0: "减速停止", 1: "反向寻Z", 2: "同向寻Z", 3: "寻ORGP上升沿"}},
        {"name": "[A]Z信号后动作", "start_bit": 12, "length": 1, "type": "enum", "options": {0: "减速停止", 1: "折返到Z信号"}}
    ]},
    {"id": "FU120", "name": "原点检索第一速度 (0.1r/min)", "group": "速度参数", "sub_group": "原点回归", "type": "u16", "address": 120,
     "read_only": False, "range": "0-20000", "effect": "立即"},
    {"id": "FU121", "name": "原点检索第二速度 (0.1r/min)", "group": "速度参数", "sub_group": "原点回归", "type": "u16", "address": 121,
     "read_only": False, "range": "0-10000", "effect": "立即"},
    {"id": "FU122", "name": "原点检索加减速时间 (Ms)", "group": "速度参数", "sub_group": "原点回归", "type": "u16", "address": 122,
     "read_only": False, "range": "0-1000", "effect": "立即"},
    {"id": "FU123", "name": "原点检索偏移脉冲数", "group": "速度参数", "sub_group": "原点回归", "type": "s32", "address": 123,
     "word_order": "big", "read_only": False, "range": "-2147483647~+2147483647", "effect": "立即"},
    {"id": "FU125", "name": "原点检索启动方式", "group": "速度参数", "sub_group": "原点回归", "type": "enum16", "address": 125,
     "read_only": False, "effect": "立即", "options": {0: "不寻找", 1: "开机自动寻找", 2: "I/O口触发", 3: "立即出发", 4: "开机和报警复位时自动寻找"}},
    {"id": "FU128", "name": "原点找到信号持续时间 (Ms)", "group": "速度参数", "sub_group": "原点回归", "type": "u16", "address": 128,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU129", "name": "原点检索超时时间 (Ms)", "group": "速度参数", "sub_group": "原点回归", "type": "u16", "address": 129,
     "read_only": False, "range": "100-65535", "effect": "立即"},

    # --- 增益切换 ---
    {"id": "FU130", "name": "增益切换方式", "group": "速度参数", "sub_group": "增益切换", "type": "enum16", "address": 130,
     "read_only": False, "effect": "立即", "options": {
        0: "不切换,用增益1", 1: "不切换,用增益2", 2: "速度>FU131切换", 3: "切换端子控制", 4: "位置误差>FU132切换", 5: "有脉冲输入时切换",
        6: "有脉冲且转速<FU131时切换"
    }},
    {"id": "FU131", "name": "增益切换速度 (0.1r/min)", "group": "速度参数", "sub_group": "增益切换", "type": "u16", "address": 131,
     "read_only": False, "range": "1-32000", "effect": "立即"},
    {"id": "FU132", "name": "增益切换脉冲", "group": "速度参数", "sub_group": "增益切换", "type": "u16", "address": 132,
     "read_only": False, "range": "1-32000", "effect": "立即"},
    {"id": "FU133", "name": "位置环增益切换时间 (0.1ms)", "group": "速度参数", "sub_group": "增益切换", "type": "u16", "address": 133,
     "read_only": False, "range": "1-32000", "effect": "立即"},
    {"id": "FU134", "name": "速度增益切换时间 (0.1ms)", "group": "速度参数", "sub_group": "增益切换", "type": "u16", "address": 134,
     "read_only": False, "range": "0-20000", "effect": "立即"},
    {"id": "FU135", "name": "增益2切至1延迟时间 (0.1ms)", "group": "速度参数", "sub_group": "增益切换", "type": "u16", "address": 135,
     "read_only": False, "range": "0-32000", "effect": "立即"},

    # --- 超程保护与机械原点 ---
    {"id": "FU136", "name": "机械原点单圈", "group": "速度参数", "sub_group": "超程保护与机械原点", "type": "u32", "address": 136,
     "word_order": "big", "read_only": False, "range": "0-2147483647", "effect": "立即"},
    {"id": "FU138", "name": "机械原点多圈", "group": "速度参数", "sub_group": "超程保护与机械原点", "type": "u32", "address": 138,
     "word_order": "big", "read_only": False, "range": "0-2147483647", "effect": "立即"},
    {"id": "FU140", "name": "超程保护正转范围脉冲数", "group": "速度参数", "sub_group": "超程保护与机械原点", "type": "u32", "address": 140,
     "word_order": "big", "read_only": False, "range": "0-2147483647", "effect": "立即"},
    {"id": "FU142", "name": "超程保护正转范围多圈数", "group": "速度参数", "sub_group": "超程保护与机械原点", "type": "u16", "address": 142,
     "read_only": False, "range": "0-32000", "effect": "立即"},
    {"id": "FU143", "name": "超程保护反转范围脉冲数", "group": "速度参数", "sub_group": "超程保护与机械原点", "type": "u32", "address": 143,
     "word_order": "big", "read_only": False, "range": "0-2147483647", "effect": "立即"},
    {"id": "FU145", "name": "超程保护反转范围多圈数", "group": "速度参数", "sub_group": "超程保护与机械原点", "type": "u16", "address": 145,
     "read_only": False, "range": "0-32000", "effect": "立即"},

    # --- 保留项 ---
    {"id": "FU116", "name": "保留", "group": "速度参数", "sub_group": "保留项", "type": "u16", "address": 116,
     "read_only": True},
    {"id": "FU124", "name": "保留", "group": "速度参数", "sub_group": "保留项", "type": "u16", "address": 124,
     "read_only": True},
    {"id": "FU137", "name": "保留", "group": "速度参数", "sub_group": "保留项", "type": "u16", "address": 137,
     "read_only": True},
    {"id": "FU139", "name": "保留", "group": "速度参数", "sub_group": "保留项", "type": "u16", "address": 139,
     "read_only": True},
]

# ==============================================================================
# 7.3.3 转矩参数 (FU200 ~ FU242)
# 地址规则: 参数号
# ==============================================================================
TORQUE_PARAMETERS = [
    # --- 电流环与转矩限制 ---
    {"id": "FU200", "name": "电流环第一带宽 (Hz)", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "u16", "address": 200,
     "read_only": False, "range": "10-3000", "effect": "立即"},
    {"id": "FU201", "name": "电流环第二带宽 (Hz)", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "u16", "address": 201,
     "read_only": False, "range": "10-3000", "effect": "立即"},
    {"id": "FU202", "name": "内部给定最大转矩限制 (%)", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "u16", "address": 202,
     "read_only": False, "range": "0-800", "effect": "立即", "tooltip": "1% 额定转矩"},
    {"id": "FU203", "name": "模拟量转矩限制", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "enum16", "address": 203,
     "read_only": False, "effect": "立即", "options": {0: "不使用", 1: "使用"}},
    {"id": "FU204", "name": "转矩内部给定 (%)", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "s16", "address": 204,
     "read_only": False, "range": "-800~+800", "effect": "立即", "tooltip": "1% 额定转矩"},
    {"id": "FU207", "name": "紧急停止时的转矩限制 (%)", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "u16", "address": 207,
     "read_only": False, "range": "1-300", "effect": "立即", "tooltip": "1% 额定转矩. 用于正/反转禁止或紧急停止"},
    {"id": "FU208", "name": "正转最大转矩限制 (%)", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "u16", "address": 208,
     "read_only": False, "range": "0-800", "effect": "立即", "tooltip": "1% 额定转矩"},
    {"id": "FU209", "name": "反转最大转矩限制 (%)", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "u16", "address": 209,
     "read_only": False, "range": "0-800", "effect": "立即", "tooltip": "1% 额定转矩"},
    {"id": "FU216", "name": "正反转禁止的转矩限制设定", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "enum16", "address": 216,
     "read_only": False, "effect": "立即", "options": {0: "实际限制为FU207设定值", 1: "限制值为0"}},
    {"id": "FU237", "name": "目标转矩范围 (%)", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "u16", "address": 237,
     "read_only": False, "range": "1-800", "effect": "立即", "tooltip": "1% 额定转矩. T-CMP信号触发范围"},
    {"id": "FU238", "name": "转矩滤波频率 (0.01Hz)", "group": "转矩参数", "sub_group": "电流环与转矩限制", "type": "u16", "address": 238,
     "read_only": False, "range": "1-1000", "effect": "立即"},

    # --- 速度限制与斜坡 ---
    {"id": "FU210", "name": "速度限制设置", "group": "转矩参数", "sub_group": "速度限制与斜坡", "type": "enum16", "address": 210,
     "read_only": False, "effect": "立即", "options": {0: "内部给定(FU211)", 1: "模拟量限制", 2: "取最高转速和电机最高转速的较小值"}},
    {"id": "FU211", "name": "速度限制内部给定 (0.1r/min)", "group": "转矩参数", "sub_group": "速度限制与斜坡", "type": "u16",
     "address": 211, "read_only": False, "range": "0-32000", "effect": "立即"},
    {"id": "FU212", "name": "转矩提升时间 (0.1ms)", "group": "转矩参数", "sub_group": "速度限制与斜坡", "type": "u16", "address": 212,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU213", "name": "转矩下降时间 (0.1ms)", "group": "转矩参数", "sub_group": "速度限制与斜坡", "type": "u16", "address": 213,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU214", "name": "第一转矩滤波时间 (0.01ms)", "group": "转矩参数", "sub_group": "速度限制与斜坡", "type": "u16", "address": 214,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU215", "name": "第二转矩滤波时间 (0.01ms)", "group": "转矩参数", "sub_group": "速度限制与斜坡", "type": "u16", "address": 215,
     "read_only": False, "range": "0-30000", "effect": "立即"},

    # --- 陷波滤波器 1 ---
    {"id": "FU217", "name": "第一陷波滤波器中心频率 (Hz)", "group": "转矩参数", "sub_group": "陷波滤波器 1", "type": "u16", "address": 217,
     "read_only": False, "range": "50-30000", "effect": "立即"},
    {"id": "FU218", "name": "第一陷波滤波器带宽 (Hz)", "group": "转矩参数", "sub_group": "陷波滤波器 1", "type": "u16", "address": 218,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU219", "name": "第一陷波滤波器深度", "group": "转矩参数", "sub_group": "陷波滤波器 1", "type": "u16", "address": 219,
     "read_only": False, "range": "0-100", "effect": "立即"},

    # --- 陷波滤波器 2 ---
    {"id": "FU220", "name": "第二陷波滤波器中心频率 (Hz)", "group": "转矩参数", "sub_group": "陷波滤波器 2", "type": "u16", "address": 220,
     "read_only": False, "range": "50-30000", "effect": "立即"},
    {"id": "FU221", "name": "第二陷波滤波器带宽 (Hz)", "group": "转矩参数", "sub_group": "陷波滤波器 2", "type": "u16", "address": 221,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU222", "name": "第二陷波滤波器深度", "group": "转矩参数", "sub_group": "陷波滤波器 2", "type": "u16", "address": 222,
     "read_only": False, "range": "0-100", "effect": "立即"},

    # --- 陷波滤波器 3 ---
    {"id": "FU223", "name": "第三陷波滤波器中心频率 (Hz)", "group": "转矩参数", "sub_group": "陷波滤波器 3", "type": "u16", "address": 223,
     "read_only": False, "range": "50-30000", "effect": "立即"},
    {"id": "FU224", "name": "第三陷波滤波器带宽 (Hz)", "group": "转矩参数", "sub_group": "陷波滤波器 3", "type": "u16", "address": 224,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU225", "name": "第三陷波滤波器深度", "group": "转矩参数", "sub_group": "陷波滤波器 3", "type": "u16", "address": 225,
     "read_only": False, "range": "0-100", "effect": "立即"},

    # --- 陷波滤波器 4 ---
    {"id": "FU226", "name": "第四陷波滤波器中心频率 (Hz)", "group": "转矩参数", "sub_group": "陷波滤波器 4", "type": "u16", "address": 226,
     "read_only": False, "range": "50-30000", "effect": "立即"},
    {"id": "FU227", "name": "第四陷波滤波器带宽 (Hz)", "group": "转矩参数", "sub_group": "陷波滤波器 4", "type": "u16", "address": 227,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU228", "name": "第四陷波滤波器深度", "group": "转矩参数", "sub_group": "陷波滤波器 4", "type": "u16", "address": 228,
     "read_only": False, "range": "0-100", "effect": "立即"},

    # --- 陷波滤波器通用设置 ---
    {"id": "FU229", "name": "陷波滤波器启动功能", "group": "转矩参数", "sub_group": "陷波滤波器通用设置", "type": "enum16", "address": 229,
     "read_only": False, "effect": "立即", "options": {0: "关闭自动配置", 1: "启动自动配置", 2: "正在自动配置"}},
    {"id": "FU230", "name": "陷波滤波器个数", "group": "转矩参数", "sub_group": "陷波滤波器通用设置", "type": "u16", "address": 230,
     "read_only": False, "range": "1-4", "effect": "立即"},

    # --- 补偿与抑制 ---
    {"id": "FU234", "name": "负载观测器增益", "group": "转矩参数", "sub_group": "补偿与抑制", "type": "u16", "address": 234,
     "read_only": False, "range": "0-1000", "effect": "立即", "tooltip": "补偿负载转矩, 增强刚性"},
    {"id": "FU235", "name": "负载观测器滤波时间 (0.01ms)", "group": "转矩参数", "sub_group": "补偿与抑制", "type": "u16", "address": 235,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU236", "name": "反电势补偿系数", "group": "转矩参数", "sub_group": "补偿与抑制", "type": "u16", "address": 236,
     "read_only": False, "range": "0-1000", "effect": "立即"},
    {"id": "FU239", "name": "重力力矩补偿 (0.1%)", "group": "转矩参数", "sub_group": "补偿与抑制", "type": "u16", "address": 239,
     "read_only": False, "range": "0-1000", "effect": "立即"},
    {"id": "FU240", "name": "抖动抑制中心频率 (0.1Hz)", "group": "转矩参数", "sub_group": "补偿与抑制", "type": "u16", "address": 240,
     "read_only": False, "range": "1-2000", "effect": "立即"},
    {"id": "FU241", "name": "抖动抑制宽度 (0.1Hz)", "group": "转矩参数", "sub_group": "补偿与抑制", "type": "u16", "address": 241,
     "read_only": False, "range": "1-100", "effect": "立即"},
    {"id": "FU242", "name": "抖动抑制强度", "group": "转矩参数", "sub_group": "补偿与抑制", "type": "u16", "address": 242,
     "read_only": False, "range": "0-100", "effect": "立即"},

    # --- 保留项 ---
    {"id": "FU205", "name": "保留", "group": "转矩参数", "sub_group": "保留项", "type": "u16", "address": 205,
     "read_only": True},
    {"id": "FU206", "name": "保留", "group": "转矩参数", "sub_group": "保留项", "type": "u16", "address": 206,
     "read_only": True},
]

# ==============================================================================
# 7.3.4 位置参数 (FU300 ~ FU389)
# 地址规则: 参数号
# ==============================================================================
POSITION_PARAMETERS = [
    # --- 位置环与脉冲指令 ---
    {"id": "FU301", "name": "第一位置环增益", "group": "位置参数", "sub_group": "位置环与脉冲指令", "type": "u16", "address": 301,
     "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "FU302", "name": "第二位置环增益", "group": "位置参数", "sub_group": "位置环与脉冲指令", "type": "u16", "address": 302,
     "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "FU303", "name": "位置环前馈增益", "group": "位置参数", "sub_group": "位置环与脉冲指令", "type": "u16", "address": 303,
     "read_only": False, "range": "0-1000", "effect": "立即"},
    {"id": "FU306", "name": "位置环滤波时间 (ms)", "group": "位置参数", "sub_group": "位置环与脉冲指令", "type": "u16", "address": 306,
     "read_only": False, "range": "1-10000", "effect": "立即"},
    {"id": "FU326", "name": "位置前馈滤波时间 (0.01ms)", "group": "位置参数", "sub_group": "位置环与脉冲指令", "type": "u16",
     "address": 326, "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "FU300", "name": "外部脉冲指令设置", "group": "位置参数", "sub_group": "位置环与脉冲指令", "type": "bit_field", "address": 300,
     "read_only": False, "effect": "立即", "fields": [
        {"name": "[A]分频输出相位", "start_bit": 12, "length": 1, "type": "enum", "options": {0: "反相位", 1: "正相位"}},
        {"name": "[B]脉冲输入逻辑", "start_bit": 8, "length": 2, "type": "enum",
         "options": {0: "PULS反,SIGN反", 1: "PULS正,SIGN正", 2: "PULS反,SIGN正", 3: "PULS正,SIGN反"}},
        {"name": "[C]脉冲输入滤波", "start_bit": 4, "length": 4, "type": "enum",
         "options": {0: "4MHz", 1: "2MHz", 2: "1MHz", 3: "500KHz", 4: "200KHz", 5: "150KHz", 6: "80KHz"}},
        {"name": "[D]脉冲模式", "start_bit": 0, "length": 2, "type": "enum",
         "options": {0: "脉冲+方向", 1: "脉冲+脉冲 (CW/CCW)", 2: "正交(A/B, 4倍频)"}}
    ]},

    # --- 电子齿轮比 ---
    {"id": "FU304", "name": "第一组电子齿轮比(分子)", "group": "位置参数", "sub_group": "电子齿轮比", "type": "u16", "address": 304,
     "read_only": False, "range": "0-65535", "effect": "立即"},
    {"id": "FU305", "name": "第一组电子齿轮比(分母)", "group": "位置参数", "sub_group": "电子齿轮比", "type": "u16", "address": 305,
     "read_only": False, "range": "1-65535", "effect": "立即"},
    {"id": "FU344", "name": "第二组电子齿轮比(分子)", "group": "位置参数", "sub_group": "电子齿轮比", "type": "s32", "address": 344,
     "word_order": "big", "read_only": False, "range": "1-2147483647", "effect": "立即"},
    {"id": "FU346", "name": "第二组电子齿轮比(分母)", "group": "位置参数", "sub_group": "电子齿轮比", "type": "s32", "address": 346,
     "word_order": "big", "read_only": False, "range": "1-2147483647", "effect": "立即"},
    {"id": "FU339", "name": "电子齿轮比选择", "group": "位置参数", "sub_group": "电子齿轮比", "type": "enum16", "address": 339,
     "read_only": False, "effect": "立即", "options": {0: "第一组", 1: "第二组", 2: "DI端子选择"}},

    # --- 位置误差与到达 ---
    {"id": "FU307", "name": "位置到达脉冲数范围", "group": "位置参数", "sub_group": "位置误差与到达", "type": "u16", "address": 307,
     "read_only": False, "range": "1-32000", "effect": "立即", "tooltip": "INP信号(定位完成)触发范围"},
    {"id": "FU309", "name": "位置误差报警脉冲数", "group": "位置参数", "sub_group": "位置误差与到达", "type": "u16", "address": 309,
     "read_only": False, "range": "1-32000", "effect": "立即"},
    {"id": "FU327", "name": "位置误差警告脉冲数", "group": "位置参数", "sub_group": "位置误差与到达", "type": "u16", "address": 327,
     "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "FU308", "name": "位置给定脉冲清零设置", "group": "位置参数", "sub_group": "位置误差与到达", "type": "bit_field", "address": 308,
     "read_only": False, "effect": "立即", "fields": [
        {"name": "[A]跟踪警告倍率", "start_bit": 12, "length": 1, "type": "enum", "options": {0: "1脉冲", 1: "100脉冲"}},
        {"name": "[B]误差报警倍率", "start_bit": 8, "length": 1, "type": "enum", "options": {0: "1脉冲", 1: "100脉冲"}},
        {"name": "[C]清零使能(CLR)", "start_bit": 4, "length": 1, "type": "enum", "options": {0: "禁止", 1: "允许外部IO"}},
        {"name": "[D]指令禁止(INH-P)", "start_bit": 0, "length": 1, "type": "enum", "options": {0: "禁止", 1: "允许外部IO"}}
    ]},

    # --- 内部位置模式 ---
    {"id": "FU340", "name": "位置模式FIR滤波器 (0.1ms)", "group": "位置参数", "sub_group": "内部位置模式", "type": "u16", "address": 340,
     "read_only": False, "range": "0-10000", "effect": "立即"},
    {"id": "FU341", "name": "内部位置模式选择", "group": "位置参数", "sub_group": "内部位置模式", "type": "enum16", "address": 341,
     "read_only": False, "effect": "立即", "options": {0: "相对模式", 1: "绝对模式"}},
    {"id": "FU342", "name": "内部位置触发", "group": "位置参数", "sub_group": "内部位置模式", "type": "enum16", "address": 342,
     "read_only": False, "effect": "立即", "options": {0: "不触发", 1: "触发"}},
    {"id": "FU343", "name": "位置模式加减速时间 (ms)", "group": "位置参数", "sub_group": "内部位置模式", "type": "u16", "address": 343,
     "read_only": False, "range": "0-10000", "effect": "立即"},
    {"id": "FU348", "name": "多段内部位置方式设定", "group": "位置参数", "sub_group": "内部位置模式", "type": "bit_field", "address": 348,
     "read_only": False, "effect": "立即", "fields": [
        {"name": "段数设定", "start_bit": 4, "length": 4, "type": "enum",
         "options": {2: "2段", 3: "3段", 4: "4段", 5: "5段", 6: "6段", 7: "7段", 8: "8段"}},
        {"name": "功能设定", "start_bit": 0, "length": 1, "type": "enum", "options": {0: "不启用", 1: "启用"}}
    ]},
    {"id": "FU349", "name": "多段内部位置循环次数", "group": "位置参数", "sub_group": "内部位置模式", "type": "u16", "address": 349,
     "read_only": False, "range": "0-30000", "effect": "立即", "tooltip": "0表示无限循环"},

    # --- 多段位置速度与位置(1-8) ---
    {"id": "FU330", "name": "位置1 给定速度 (0.1r/min)", "group": "位置参数", "sub_group": "多段位置速度(1-8)", "type": "u16",
     "address": 330, "read_only": False, "range": "1-65535", "effect": "立即"},
    {"id": "FU350", "name": "位置1 给定位置", "group": "位置参数", "sub_group": "多段位置(1-8)", "type": "s32", "address": 350,
     "word_order": "big", "read_only": False, "range": "-2147483647~+2147483647", "effect": "立即"},
    # ... (FU331-337 for speeds 2-8, FU352-364 for positions 2-8 would follow the same pattern)

    # --- 龙门同步与全闭环 ---
    {"id": "FU376", "name": "位置反馈来源", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "enum16", "address": 376,
     "read_only": False, "effect": "立即", "options": {0: "编码器反馈", 1: "高速计数器1", 2: "高速计数器2"}},
    {"id": "FU377", "name": "位置反馈脉冲比例(分子)", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "u16", "address": 377,
     "read_only": False, "range": "1-65535", "effect": "立即"},
    {"id": "FU378", "name": "位置反馈脉冲比例(分母)", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "u16", "address": 378,
     "read_only": False, "range": "1-65535", "effect": "立即"},
    {"id": "FU379", "name": "混合误差清除圈数", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "u16", "address": 379,
     "read_only": False, "range": "0-32000", "effect": "立即"},
    {"id": "FU380", "name": "混合误差报警脉冲", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "u16", "address": 380,
     "read_only": False, "range": "1-65535", "effect": "立即"},
    {"id": "FU381", "name": "龙门同步增益", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "u16", "address": 381,
     "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "FU382", "name": "龙门位置反馈来源", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "enum16", "address": 382,
     "read_only": False, "effect": "立即", "options": {0: "保留", 1: "保留"}},
    {"id": "FU383", "name": "龙门失同步报警脉冲数", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "u16", "address": 383,
     "read_only": False, "range": "10-32000", "effect": "立即"},
    {"id": "FU384", "name": "龙门同步反馈比例(分子)", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "s32", "address": 384,
     "word_order": "big", "read_only": False, "range": "1-2147483647", "effect": "立即"},
    {"id": "FU386", "name": "龙门同步反馈比例(分母)", "group": "位置参数", "sub_group": "龙门同步与全闭环", "type": "u16", "address": 386,
     "read_only": False, "range": "1-30000", "effect": "立即"},

    # --- 中断与方向滤波 ---
    {"id": "FU388", "name": "中断定长设置", "group": "位置参数", "sub_group": "中断与方向滤波", "type": "bit_field", "address": 388,
     "read_only": False, "effect": "立即", "fields": [
        {"name": "启用释放端子", "start_bit": 4, "length": 1, "type": "enum", "options": {0: "不启用", 1: "启用"}},
        {"name": "启用中断功能", "start_bit": 0, "length": 1, "type": "enum", "options": {0: "不启用", 1: "启用"}}
    ]},
    {"id": "FU389", "name": "位置指令方向滤波频率", "group": "位置参数", "sub_group": "中断与方向滤波", "type": "u16", "address": 389,
     "read_only": False, "range": "1-15", "effect": "立即"},
]

# Note: For brevity, repetitive multi-segment position (FU352-364) and speed (FU331-337)
# registers are omitted but follow the pattern of FU350 and FU330.

# ==============================================================================
# 7.3.5 输入输出参数 (FU400 ~ FU445)
# 地址规则: 参数号
# ==============================================================================
IO_PARAMETERS = [
    # --- 模拟量输入 ---
    {"id": "FU400", "name": "模拟量速度指令电压对应最大速度 (r/min)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "u16",
     "address": 400, "read_only": False, "range": "1-10000", "effect": "立即"},
    {"id": "FU401", "name": "模拟量转矩指令电压对应最大转矩 (%)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "u16",
     "address": 401, "read_only": False, "range": "0-800", "effect": "立即", "tooltip": "1% 额定转矩"},
    {"id": "FU402", "name": "AV零漂补偿 (mv)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16", "address": 402,
     "read_only": False, "range": "-5000~+5000", "effect": "立即"},
    {"id": "FU403", "name": "AC零漂补偿 (mv)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16", "address": 403,
     "read_only": False, "range": "-5000~+5000", "effect": "立即"},
    {"id": "FU404", "name": "模拟量速度指令滤波时间 (0.01ms)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "u16",
     "address": 404, "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "FU405", "name": "模拟量转矩指令滤波时间 (0.01ms)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "u16",
     "address": 405, "read_only": False, "range": "1-30000", "effect": "立即"},
    {"id": "FU406", "name": "AI自动调零", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "enum16", "address": 406,
     "read_only": False, "effect": "立即", "options": {0: "不动作", 1: "执行调零"}, "tooltip": "执行前请将模拟量输入接地"},
    {"id": "FU426", "name": "模拟量零漂报警范围 (mv)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "u16", "address": 426,
     "read_only": False, "range": "1-10000", "effect": "立即"},
    {"id": "FU427", "name": "模拟量端子控制", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "enum16", "address": 427,
     "read_only": False, "effect": "立即", "options": {0: "保留", 1: "保留"}},
    {"id": "FU428", "name": "模拟量速度指令来源", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "enum16", "address": 428,
     "read_only": False, "effect": "立即", "options": {0: "AV通道", 1: "AC通道"}},
    {"id": "FU429", "name": "模拟量转矩指令来源", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "enum16", "address": 429,
     "read_only": False, "effect": "立即", "options": {0: "AV通道", 1: "AC通道"}},
    {"id": "FU430", "name": "速度模拟量下限电压对应速度 (0.1%)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16",
     "address": 430, "read_only": False, "range": "-1000~+1000", "effect": "立即"},
    {"id": "FU431", "name": "速度模拟量下限电压 (0.01V)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16", "address": 431,
     "read_only": False, "range": "-1000~+1000", "effect": "立即"},
    {"id": "FU432", "name": "速度模拟量上限电压对应速度 (0.1%)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16",
     "address": 432, "read_only": False, "range": "-1000~+1000", "effect": "立即"},
    {"id": "FU433", "name": "速度模拟量上限电压 (0.01V)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16", "address": 433,
     "read_only": False, "range": "-1000~+1000", "effect": "立即"},
    {"id": "FU434", "name": "模拟量下限电压对应转矩 (0.1%)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16",
     "address": 434, "read_only": False, "range": "-1000~+1000", "effect": "立即"},
    {"id": "FU435", "name": "转矩模拟量下限电压 (0.01V)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16", "address": 435,
     "read_only": False, "range": "-1000~+1000", "effect": "立即"},
    {"id": "FU436", "name": "转矩模拟量上限电压对应转矩 (0.1%)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16",
     "address": 436, "read_only": False, "range": "-1000~+1000", "effect": "立即"},
    {"id": "FU437", "name": "转矩模拟量上限电压 (0.01V)", "group": "输入输出参数", "sub_group": "模拟量输入", "type": "s16", "address": 437,
     "read_only": False, "range": "-1000~+1000", "effect": "立即"},

    # --- DI端子功能 (手册中DI1-DI3对应FU407-419) ---
    {"id": "FU407_DI1", "name": "DI1端子功能", "group": "输入输出参数", "sub_group": "DI端子功能", "type": "bit_field",
     "address": 407, "read_only": False, "effect": "立即", "fields": [
        {"name": "节点类型", "start_bit": 8, "length": 1, "type": "enum", "options": {0: "常闭(NC)", 1: "常开(NO)"}},
        {"name": "功能选择", "start_bit": 0, "length": 6, "type": "enum",
         "options": {0: "伺服使能(SON-I)", 1: "报警复位(AL-RST)", 2: "正转转矩限制(F-CL)", 3: "反转转矩限制(R-CL)", 4: "内部速度选择1(SD-S1)",
                     5: "内部速度选择2(SD-S2)", 6: "内部速度方向(SD-DIR)", 7: "零速箝位(ZCLAMP)", 8: "增益切换(GAIN-SEL)",
                     9: "内部位置终止(STOP)", 10: "脉冲清除(CLR)", 11: "指令脉冲禁止(INH-P)", 12: "紧急停止(ESP)", 13: "反转禁止(R-INH)",
                     14: "正转禁止(F-INH)", 16: "内部位置选择1(SD0)", 17: "内部位置选择2(SD1)", 18: "内部位置选择3(SD2)", 19: "内部位置暂停(HOLD)",
                     20: "内部位置触发(CTRG)", 21: "原点检索触发(SHOM)", 22: "外部参考原点(ORGP)", 23: "模拟量速度正转(F-AS)",
                     24: "模拟量速度反转(R-AS)", 25: "模式切换(M-SEL)", 26: "正向点动(JOGU)", 27: "反向点动(JOGD)", 28: "电机过热(HOT)",
                     29: "中断定长释放(XintTrig)", 30: "中断定长启用(XintRest)", 31: "龙门同步启动(GAN-SYNC)", 33: "电子齿轮选择(GEAR_SEL)"}}
    ]},
    # ... Similarly, FU408 for DI2, FU409 for DI3, etc. up to FU419 ...

    # --- DO端子功能 ---
    {"id": "FU421_DO1", "name": "DO1端子功能", "group": "输入输出参数", "sub_group": "DO端子功能", "type": "bit_field",
     "address": 421, "read_only": False, "effect": "立即", "tooltip": "小功率型伺服此DO口输出由FU020参数控制", "fields": [
        {"name": "输出类型", "start_bit": 8, "length": 1, "type": "enum", "options": {0: "常闭(NC)", 1: "常开(NO)"}},
        {"name": "功能选择", "start_bit": 0, "length": 4, "type": "enum",
         "options": {0: "伺服准备好(S-RDY)", 1: "伺服使能(SON-O)", 2: "旋转检出(TGON)", 3: "速度到达(V-CMP)", 4: "位置到达(P-CMP)",
                     5: "转矩限制中(T-LT)", 6: "伺服报警(ALM)", 7: "电磁抱闸控制(BRAKE)", 8: "过载预警(OL-W)", 9: "速度限制中(S-LT)",
                     10: "内部位置触发中(CTRGING)", 11: "位置偏差过大警告(PER-W)", 12: "原点找到(HOME)", 13: "目标转矩到达(T-CMP)",
                     15: "Z信号输出(Z-OUT)"}}
    ]},
    # ... Similarly, FU422 for DO2, FU423 for DO3, FU425 for ALM ...

    # --- 端子滤波 ---
    {"id": "FU438", "name": "DI1端子滤波时间", "group": "输入输出参数", "sub_group": "端子滤波", "type": "u16", "address": 438,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU439", "name": "DI2端子滤波时间", "group": "输入输出参数", "sub_group": "端子滤波", "type": "u16", "address": 439,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU440", "name": "DI3端子滤波时间", "group": "输入输出参数", "sub_group": "端子滤波", "type": "u16", "address": 440,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU441", "name": "DI4端子滤波时间", "group": "输入输出参数", "sub_group": "端子滤波", "type": "u16", "address": 441,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU442", "name": "DI5端子滤波时间", "group": "输入输出参数", "sub_group": "端子滤波", "type": "u16", "address": 442,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU443", "name": "DI6端子滤波时间", "group": "输入输出参数", "sub_group": "端子滤波", "type": "u16", "address": 443,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU444", "name": "DI7端子滤波时间", "group": "输入输出参数", "sub_group": "端子滤波", "type": "u16", "address": 444,
     "read_only": False, "range": "0-30000", "effect": "立即"},
    {"id": "FU445", "name": "DI8端子滤波时间", "group": "输入输出参数", "sub_group": "端子滤波", "type": "u16", "address": 445,
     "read_only": False, "range": "0-30000", "effect": "立即"},
]


# Note: For brevity, repetitive DI/DO function selections (e.g., FU408, FU409, FU422, etc.)
# are omitted but would follow the same bit_field pattern as FU407 and FU421.


# ==============================================================================
# 7.3.6 通讯参数 (FU500 ~ FU505, FU020)
# 地址规则: 参数号
# ==============================================================================
COMMUNICATION_PARAMETERS = [
    # --- 通讯设置 ---
    {"id": "FU500", "name": "通讯地址", "group": "通讯参数", "sub_group": "通讯设置", "type": "u16", "address": 500, "read_only": False, "range": "1-254", "effect": "立即"},
    {"id": "FU501", "name": "通讯模式", "group": "通讯参数", "sub_group": "通讯设置", "type": "enum16", "address": 501, "read_only": False, "effect": "立即", "options": {0: "RTU", 1: "ASCII"}},
    {"id": "FU502", "name": "停止位", "group": "通讯参数", "sub_group": "通讯设置", "type": "enum16", "address": 502, "read_only": False, "effect": "立即", "options": {0: "1个停止位", 1: "2个停止位"}},
    {"id": "FU503", "name": "奇偶校验设置", "group": "通讯参数", "sub_group": "通讯设置", "type": "enum16", "address": 503, "read_only": False, "effect": "立即", "options": {0: "无校验", 1: "奇校验", 2: "偶校验"}},
    {"id": "FU504", "name": "通讯波特率 (bit/s)", "group": "通讯参数", "sub_group": "通讯设置", "type": "enum16", "address": 504, "read_only": False, "effect": "立即", "options": {
        0: "2400",
        1: "4800",
        2: "9600",
        3: "19200",
        4: "38400",
        5: "57600"
    }},
    {"id": "FU505", "name": "通讯读写允许", "group": "通讯参数", "sub_group": "通讯设置", "type": "enum16", "address": 505, "read_only": False, "effect": "立即", "options": {
        0: "读写允许",
        1: "读写不允许"
    }, "tooltip": "设为不允许时, 掉电后通讯写入的数据会丢失"},

    # --- 引脚功能复用 (与通讯相关) ---
    {"id": "FU020", "name": "分频信号/引脚功能设置", "group": "通讯参数", "sub_group": "引脚功能复用", "type": "enum16", "address": 20, "read_only": False, "effect": "重新上电保存", "tooltip": "警告: 此参数会改变硬件引脚功能！仅对小功率型号有效。详见手册P79, P91", "options": {
        0: "26/10脚=MODBUS, DO1可由FU421配置",
        1: "26/10脚=MODBUS, DO1固定为Z相集电极输出",
        2: "26/10脚=A相差分, 30/15脚=B相差分 (MODBUS不可用)"
    }},
]

# ==============================================================================
# 7.4 电机参数 (PA000 ~ PA121)
# 地址规则: 参数号 + PA_BASE_ADDRESS (假定为4000)
# 警告: 除非明确知道后果，否则不应修改这些参数。
# ==============================================================================
PA_BASE_ADDRESS = 4000

MOTOR_PARAMETERS = [
    # --- 电气特性 ---
    {"id": "PA000", "name": "伺服电机额定电压 (V)", "group": "电机参数", "sub_group": "电气特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 0, "read_only": False, "range": "1-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA001", "name": "伺服电机额定电流 (0.1A)", "group": "电机参数", "sub_group": "电气特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 1, "read_only": False, "range": "1-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA005", "name": "伺服电机相间电阻 (10⁻³Ω)", "group": "电机参数", "sub_group": "电气特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 5, "read_only": False, "range": "0-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA006", "name": "伺服电机D轴电感 (10⁻⁶H)", "group": "电机参数", "sub_group": "电气特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 6, "read_only": False, "range": "0-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA007", "name": "伺服电机Q轴电感 (10⁻⁶H)", "group": "电机参数", "sub_group": "电气特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 7, "read_only": False, "range": "0-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA008", "name": "反电动势线电压有效值 (0.1V/1000r/min)", "group": "电机参数", "sub_group": "电气特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 8, "read_only": False, "range": "0-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},

    # --- 机械特性 ---
    {"id": "PA002", "name": "伺服电机最高转速 (r/min)", "group": "电机参数", "sub_group": "机械特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 2, "read_only": False, "range": "0-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA003", "name": "伺服电机额定转速 (r/min)", "group": "电机参数", "sub_group": "机械特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 3, "read_only": False, "range": "1-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA004", "name": "伺服电机极对数 (对)", "group": "电机参数", "sub_group": "机械特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 4, "read_only": False, "range": "1-30", "effect": "立即", "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA011", "name": "伺服电机功率 (0.01KW)", "group": "电机参数", "sub_group": "机械特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 11, "read_only": False, "range": "0-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA012", "name": "伺服电机转动惯量 (10⁻⁶Kg·m²)", "group": "电机参数", "sub_group": "机械特性", "type": "u32",
     "address": PA_BASE_ADDRESS + 12, "word_order": "big", "read_only": False, "range": "0-2147483647", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA121", "name": "伺服电机过载敏感性", "group": "电机参数", "sub_group": "机械特性", "type": "u16",
     "address": PA_BASE_ADDRESS + 121, "read_only": False, "range": "1-30000", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},

    # --- 编码器特性 ---
    {"id": "PA016", "name": "伺服电机编码器线数 (线)", "group": "电机参数", "sub_group": "编码器特性", "type": "u32",
     "address": PA_BASE_ADDRESS + 16, "word_order": "big", "read_only": False, "range": "0-2147483647", "effect": "立即",
     "tooltip": "警告: 擅自修改可能导致损坏！"},
    {"id": "PA018", "name": "伺服电机编码器安装角度", "group": "电机参数", "sub_group": "编码器特性", "type": "s32",
     "address": PA_BASE_ADDRESS + 18, "word_order": "big", "read_only": False, "range": "-2147483647~+2147483647",
     "effect": "立即", "tooltip": "警告: 擅自修改可能导致损坏！"},
]

REGISTER_MAP = []

REGISTER_MAP.extend(MONITORING_PARAMETERS)
REGISTER_MAP.extend(AUXILIARY_PARAMETERS)
REGISTER_MAP.extend(SYSTEM_PARAMETERS_0_20)
REGISTER_MAP.extend(VELOCITY_PARAMETERS)
REGISTER_MAP.extend(TORQUE_PARAMETERS)
REGISTER_MAP.extend(POSITION_PARAMETERS)
REGISTER_MAP.extend(IO_PARAMETERS)
REGISTER_MAP.extend(COMMUNICATION_PARAMETERS)


# NOTE: This is a truncated REGISTER_MAP for demonstration.
# The full map from previous answers should be pasted here.


# ==============================================================================
# PART 2: UTILITY CLASSES (FlowLayout, StatusIndicator)
# ==============================================================================
class FlowLayout(QLayout):
    def __init__(self, parent=None, margin=0, spacing=-1):
        super(FlowLayout, self).__init__(parent)
        if parent is not None: self.setContentsMargins(margin, margin, margin, margin)
        self.setSpacing(spacing)
        self.itemList = []

    def __del__(self):
        item = self.takeAt(0)
        while item: item = self.takeAt(0)

    def addItem(self, item):
        self.itemList.append(item)

    def count(self):
        return len(self.itemList)

    def itemAt(self, index):
        return self.itemList[index] if 0 <= index < len(self.itemList) else None

    def takeAt(self, index):
        return self.itemList.pop(index) if 0 <= index < len(self.itemList) else None

    def expandingDirections(self):
        return Qt.Orientation(0)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return self._doLayout(QRect(0, 0, width, 0), True)

    def setGeometry(self, rect):
        super(FlowLayout, self).setGeometry(rect)
        self._doLayout(rect, False)

    def sizeHint(self):
        return self.minimumSize()

    def minimumSize(self):
        size = QSize()
        for item in self.itemList: size = size.expandedTo(item.minimumSize())
        margin, _, _, _ = self.getContentsMargins()
        size += QSize(2 * margin, 2 * margin)
        return size

    def _doLayout(self, rect, testOnly):
        x, y, lineHeight = rect.x(), rect.y(), 0
        spaceX, spaceY = self.spacing(), self.spacing()
        for item in self.itemList:
            wid = item.widget()
            nextX = x + item.sizeHint().width() + spaceX
            if nextX - spaceX > rect.right() and lineHeight > 0:
                x, y = rect.x(), y + lineHeight + spaceY
                nextX, lineHeight = x + item.sizeHint().width() + spaceX, 0
            if not testOnly: item.setGeometry(QRect(QPoint(x, y), item.sizeHint()))
            x = nextX
            lineHeight = max(lineHeight, item.sizeHint().height())
        return y + lineHeight - rect.y()


class StatusIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(20, 20)
        self._is_on = False

    def set_status(self, on):
        self._is_on = on
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = QColor("green") if self._is_on else QColor("red")
        painter.setBrush(color)
        painter.drawEllipse(0, 0, 19, 19)


# ==============================================================================
# PART 3: CORE LOGIC (ModbusWorker, RegisterWidget)
# ==============================================================================
class ModbusWorker(QObject):
    connection_status = pyqtSignal(bool, str)
    log_message = pyqtSignal(str, str)
    read_result = pyqtSignal(str, object)
    write_result = pyqtSignal(str, bool, object)

    def __init__(self, port, baudrate, parity, stopbits, timeout):
        super().__init__()
        self._port = port
        self._baudrate = baudrate

        self.client = ModbusSerialClient(
            port=self._port,
            baudrate=self._baudrate,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout
        )

        self.dtype_map = {
            'u16': self.client.DATATYPE.UINT16,
            's16': self.client.DATATYPE.INT16,
            'enum16': self.client.DATATYPE.UINT16,
            'bit_field': self.client.DATATYPE.UINT16,
            'u32': self.client.DATATYPE.UINT32,
            's32': self.client.DATATYPE.INT32
        }

    def read_single_register(self, config):
        """Wrapper to read a single register using the multiple-read logic."""
        self.read_multiple_registers([config])

    def read_multiple_registers(self, configs: list):
        """
        Reads a list of registers by grouping them into contiguous blocks
        and sending one read request per block.
        """
        if not self.client.is_socket_open():
            for cfg in configs:
                self.read_result.emit(cfg['id'], ModbusException("客户端未连接"))
            return
        if not configs:
            return

        # Sort configs by address to make grouping easier
        sorted_configs = sorted(configs, key=lambda x: x['address'])

        # --- Block Grouping Logic ---
        read_blocks = []
        if sorted_configs:
            current_block = {
                'start_address': sorted_configs[0]['address'],
                'word_count': 0,
                'configs': []
            }
            for cfg in sorted_configs:
                addr = cfg['address']
                words = 2 if cfg['type'] in ['u32', 's32'] else 1

                # If current register is not contiguous with the block, end the current block
                if addr != current_block['start_address'] + current_block['word_count']:
                    if current_block['configs']:
                        read_blocks.append(current_block)
                    # Start a new block
                    current_block = {'start_address': addr, 'word_count': 0, 'configs': []}

                # Add the current register to the current block
                current_block['word_count'] += words
                current_block['configs'].append(cfg)

            # Add the last block
            if current_block['configs']:
                read_blocks.append(current_block)

        # --- Execute Reads and Unpack Results ---
        for block in read_blocks:
            try:
                start = block['start_address']
                count = block['word_count']
                self.log_message.emit("info", f"批量读取: 地址={start}, 数量={count}")

                rr = self.client.read_holding_registers(address=start, count=count, slave=1)

                if rr.isError():
                    raise ModbusException(f"Modbus error on block read: {rr}")

                # --- Unpack the results for each register in the block ---
                offset = 0
                for cfg in block['configs']:
                    words = 2 if cfg['type'] in ['u32', 's32'] else 1
                    registers_slice = rr.registers[offset: offset + words]

                    word_order = Endian.BIG if cfg.get('word_order', 'big') == 'big' else Endian.LITTLE

                    value = self.client.convert_from_registers(
                        registers_slice,
                        self.dtype_map[cfg['type']],
                        word_order=word_order
                    )

                    self.read_result.emit(cfg['id'], value)
                    offset += words

            except Exception as e:
                self.log_message.emit("error", f"块读取失败: 地址={block['start_address']}, 错误: {e}")
                # Emit error for all registers in this failed block
                for cfg in block['configs']:
                    self.read_result.emit(cfg['id'], e)

    def connect_device(self):
        try:
            if self.client.connect():
                self.connection_status.emit(True, f"成功连接到 {self._port}")
                self.log_message.emit("info", f"串口 {self._port} 已连接。")
            else:
                raise ConnectionError(f"连接失败: 无法打开端口 {self._port}")
        except Exception as e:
            self.connection_status.emit(False, f"连接失败: {e}")
            self.log_message.emit("error", f"连接串口 {self._port} 失败: {e}")

    def disconnect_device(self):
        if self.client.is_socket_open():
            self.client.close()
        self.connection_status.emit(False, f"已断开连接 {self._port}")
        self.log_message.emit("info", f"连接已断开 {self._port}。")

    def read_logical_value(self, config):
        if not self.client.is_socket_open():
            self.read_result.emit(config['id'], ModbusException("客户端未连接"))
            return
        try:
            reg_type = config['type']
            address = config['address']
            self.log_message.emit("info", f"读取 {config['id']} (地址: {address})")

            word_count = 2 if reg_type in ['u32', 's32'] else 1

            rr = self.client.read_holding_registers(address, count=word_count, slave=1)

            if rr.isError():
                raise ModbusException(str(rr))

            word_order = Endian.BIG if config.get('word_order', 'big') == 'big' else Endian.LITTLE

            value = self.client.convert_from_registers(
                rr.registers,
                self.dtype_map[reg_type],
                word_order=word_order
            )

            self.log_message.emit("info", f"读取成功: {config['id']} = {value}")
            self.read_result.emit(config['id'], value)
        except Exception as e:
            self.log_message.emit("error", f"读取 {config['id']} 失败: {e}")
            self.read_result.emit(config['id'], e)

    def write_logical_value(self, config, value):
        if not self.client.is_socket_open():
            self.write_result.emit(config['id'], False, ModbusException("客户端未连接"))
            return
        try:
            reg_type = config['type']
            address = config['address']
            self.log_message.emit("info", f"写入 {config['id']} (地址: {address}) 值: {value}")

            word_order = Endian.BIG if config.get('word_order', 'big') == 'big' else Endian.LITTLE

            payload = self.client.convert_to_registers(
                value,
                self.dtype_map[reg_type],
                word_order=word_order
            )

            # write_registers is used for both single and multiple registers
            # The 'slave' argument is now a keyword argument as well.
            self.client.write_registers(address, payload, slave=1)

            self.log_message.emit("info", f"写入成功: {config['id']} = {value}")
            self.write_result.emit(config['id'], True, value)
        except Exception as e:
            self.log_message.emit("error", f"写入 {config['id']} 失败: {e}")
            self.write_result.emit(config['id'], False, e)


class RegisterWidget(QWidget):
    """
    Final, definitive version. Inherits from QWidget to avoid nested QGroupBox issues.
    Uses a StyleSheet to create a "card" visual effect.
    Internal layout is managed precisely with QVBoxLayout and QHBoxLayout.
    """
    read_requested = pyqtSignal(dict)
    write_requested = pyqtSignal(dict, int)

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.is_dirty = False
        self.has_been_read = False
        self.current_value = 0
        self.sub_widgets = []

        # Set the card-like style for the widget
        self.setStyleSheet("""
                    RegisterWidget {
                        background-color: #FAFAFA;
                        border: 1px solid #E0E0E0;
                        border-radius: 4px;
                    }
                    QWidget:disabled {
                        color: #A0A0A0;
                        background-color: #F0F0F0;
                    }
                    QComboBox:disabled, QSpinBox:disabled {
                        color: #555555;
                        background-color: #E8E8E8;
                    }
                """)

        # Main vertical layout for the entire widget
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # Title Label for the register
        self.title_label = QLabel(self.config['name'])
        self.title_label.setStyleSheet("font-weight: bold; color: #333;")
        main_layout.addWidget(self.title_label)

        # Create and add the value editing widgets
        self._create_widgets(main_layout)

        # Set tooltip
        tooltip_text = (f"ID: {config['id']}\n"
                        f"地址: {config['address']}\n"
                        f"类型: {config['type']}\n"
                        f"范围: {config.get('range', 'N/A')}\n"
                        f"生效: {config.get('effect', 'N/A')}\n"
                        f"{config.get('tooltip', '')}")
        self.setToolTip(tooltip_text)

    def _on_write(self):
        if not self.has_been_read and self.config['type'] == 'bit_field':
            QMessageBox.warning(self, "操作中断", "请先读取该寄存器的当前值，再进行写入操作。\n直接写入可能会覆盖未显示的配置位。")
            return
        self.write_requested.emit(self.config, self.get_value())

    def _create_widgets(self, layout):
        is_read_only = self.config.get('read_only', False)
        if self.config['type'] == 'bit_field':
            # Multi-line layout for bit_field using a grid for alignment
            grid = QGridLayout()
            grid.setColumnStretch(1, 1)  # Allow value control to expand
            for i, field in enumerate(self.config['fields']):
                field_label = QLabel(f"{field['name']}:")
                widget = self._create_value_control(field)
                widget.setMinimumWidth(150)  # Use minimum width for flexibility
                widget.setDisabled(is_read_only)
                grid.addWidget(field_label, i, 0)
                grid.addWidget(widget, i, 1)
                self.sub_widgets.append({'widget': widget, 'config': field})
            layout.addLayout(grid)
        else:
            # Single-line layout for simple types
            widget = self._create_value_control(self.config)
            widget.setMinimumWidth(150)  # Fixed width for value control
            widget.setDisabled(is_read_only)
            layout.addWidget(widget)
            self.sub_widgets.append({'widget': widget, 'config': self.config})

        # Add common Read/Write buttons
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        self.read_btn = QPushButton("读")
        self.write_btn = QPushButton("写")
        btn_layout.addWidget(self.read_btn)
        btn_layout.addWidget(self.write_btn)
        layout.addLayout(btn_layout)

        self.read_btn.clicked.connect(lambda: self.read_requested.emit(self.config))
        self.write_btn.clicked.connect(self._on_write)

        if self.config.get('read_only', False):
            self.write_btn.setEnabled(False)

    def _create_value_control(self, config):
        if config.get('type') in ['enum', 'enum16']:
            widget = QComboBox()
            if 'options' in config and config['options']:
                for val, desc in config['options'].items():
                    widget.addItem(f"({val}) {desc}", val)
            widget.currentIndexChanged.connect(self._mark_dirty)
        else:
            widget = QSpinBox()
            widget.setRange(-2147483648, 2147483647)
            widget.valueChanged.connect(self._mark_dirty)
        return widget

    def _mark_dirty(self):
        if not self.is_dirty:
            self.is_dirty = True
            # Update the internal title label's text
            self.title_label.setText(f"{self.config['name']} *")

    def _mark_clean(self):
        if self.is_dirty:
            self.is_dirty = False
            self.title_label.setText(self.config['name'])

    def set_value(self, value):
        self.has_been_read = True
        for item in self.sub_widgets:
            item['widget'].blockSignals(True)

        self.current_value = value
        if self.config['type'] == 'bit_field':
            for item in self.sub_widgets:
                field_cfg, field_widget = item['config'], item['widget']
                mask = (1 << field_cfg['length']) - 1
                field_val = (value >> field_cfg['start_bit']) & mask

                if field_cfg['type'] == 'enum':
                    index = field_widget.findData(field_val)
                    if index != -1: field_widget.setCurrentIndex(index)
                else:
                    field_widget.setValue(field_val)
        else:
            item = self.sub_widgets[0]
            widget = item['widget']
            if isinstance(widget, QComboBox):
                index = widget.findData(value)
                if index != -1:
                    widget.setCurrentIndex(index)
            elif isinstance(widget, QSpinBox):
                widget.setValue(int(value))

        for item in self.sub_widgets:
            item['widget'].blockSignals(False)

        self._mark_clean()

    def get_value(self):
        if self.config['type'] == 'bit_field':
            val = self.current_value
            for item in self.sub_widgets:
                field_cfg = item['config']
                field_widget = item['widget']
                field_val = field_widget.currentData() if field_cfg['type'] == 'enum' else field_widget.value()
                mask = (1 << field_cfg['length']) - 1
                val &= ~(mask << field_cfg['start_bit'])
                val |= (field_val & mask) << field_cfg['start_bit']
            return val
        else:
            item = self.sub_widgets[0]
            widget = item['widget']
            if isinstance(widget, QComboBox):
                return widget.currentData()
            elif isinstance(widget, QSpinBox):
                return widget.value()
            return 0  # Fallback


# ==============================================================================
# PART 4: MAIN UI (MainWindow)
# ==============================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("红森 HSX2M 伺服驱动器控制器 (v2.1)")
        self.setGeometry(100, 100, 1400, 900)
        self.setStyleSheet("""
            QMainWindow { background-color: #F0F0F0; }
            QGroupBox { 
                font-weight: bold;
                background-color: #FFFFFF; 
                border: 1px solid #CCCCCC; 
                border-radius: 5px; 
                margin-top: 1ex; 
            }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }
            QLabel { font-weight: normal; }
            QPushButton { padding: 5px 15px; border: 1px solid #B0B0B0; border-radius: 3px; background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #F6F6F6, stop:1 #E0E0E0); }
            QPushButton:hover { background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #E8E8E8, stop:1 #D0D0D0); }
            QPushButton:pressed { background-color: #D0D0D0; }
            QPushButton:disabled { color: #A0A0A0; background-color: #E8E8E8; }
            QComboBox, QSpinBox { padding: 3px; border: 1px solid #B0B0B0; border-radius: 3px; }
            QTabWidget::pane { border-top: 2px solid #C2C7CB; }
            QTabBar::tab { background: #E0E0E0; border: 1px solid #B0B0B0; border-bottom: none; border-top-left-radius: 4px; border-top-right-radius: 4px; padding: 8px 15px; }
            QTabBar::tab:selected { background: #FFFFFF; }
            QTextEdit { background-color: #2E2E2E; color: #F0F0F0; border-radius: 3px; font-family: Consolas, 'Courier New', monospace; }
            QScrollArea { border: none; background-color: transparent; }
        """)

        self.modbus_thread = None
        self.modbus_worker = None
        self.register_widgets = {}  # {id: widget}

        self._init_ui()

    def _init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        main_layout.addWidget(self._create_connection_panel())
        main_layout.addWidget(self._create_register_panel(), 1)
        main_layout.addWidget(self._create_log_panel())

    def _create_connection_panel(self):
        panel = QGroupBox("连接设置")
        layout = QHBoxLayout(panel)

        self.port_combo = QComboBox()
        # TODO: Populate with available serial ports dynamically
        for i in range(1, 21): self.port_combo.addItem(f"COM{i}")

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['2400', '4800', '9600', '19200', '38400', '57600'])
        self.baud_combo.setCurrentText('19200')

        self.connect_btn = QPushButton("连接")
        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.setEnabled(False)
        self.status_light = StatusIndicator()

        layout.addWidget(QLabel("串口:"))
        layout.addWidget(self.port_combo)
        layout.addWidget(QLabel("波特率:"))
        layout.addWidget(self.baud_combo)
        layout.addSpacing(20)
        layout.addWidget(self.connect_btn)
        layout.addWidget(self.disconnect_btn)
        layout.addWidget(self.status_light)
        layout.addStretch()

        self.connect_btn.clicked.connect(self.connect_device)
        self.disconnect_btn.clicked.connect(self.disconnect_device)

        return panel

    def _create_register_panel(self):
        self.tabs = QTabWidget()
        grouped_registers = defaultdict(lambda: defaultdict(list))
        group_order = []
        sub_group_order = defaultdict(list)

        for reg in REGISTER_MAP:
            if reg['name'] == "保留" or reg.get('sub_group') == "保留项" or reg.get('address') <= 0:
                continue
            group = reg['group']
            sub_group = reg.get('sub_group', '常规')

            if group not in group_order:
                group_order.append(group)
            if sub_group not in sub_group_order[group]:
                sub_group_order[group].append(sub_group)

            grouped_registers[reg['group']][reg.get('sub_group', '常规')].append(reg)

        for group_name in group_order:
            sub_groups = grouped_registers[group_name]

            # Start of tab creation logic (remains the same)
            tab_container_widget = QWidget()
            tab_main_layout = QVBoxLayout(tab_container_widget)

            btn_bar_layout = QHBoxLayout()
            btn_bar_layout.addStretch()
            read_all_btn = QPushButton(f"读取本页 ({group_name})")
            write_all_btn = QPushButton(f"写入本页修改")
            btn_bar_layout.addWidget(read_all_btn)
            btn_bar_layout.addWidget(write_all_btn)
            tab_main_layout.addLayout(btn_bar_layout)

            scroll_area = QScrollArea()
            scroll_area.setWidgetResizable(True)
            scroll_content_widget = QWidget()
            vertical_layout_for_subgroups = QVBoxLayout(scroll_content_widget)
            vertical_layout_for_subgroups.setAlignment(Qt.AlignmentFlag.AlignTop)
            # End of tab creation logic

            for sub_group_name in sub_group_order[group_name]:
                registers = sub_groups[sub_group_name]

                sub_group_box = QGroupBox(sub_group_name)
                flow_layout = FlowLayout(spacing=10)
                sub_group_box.setLayout(flow_layout)

                # The rest of the register creation loop remains the same
                for reg_config in registers:
                    widget = RegisterWidget(reg_config)
                    self.register_widgets[reg_config['id']] = widget
                    flow_layout.addWidget(widget)
                    widget.read_requested.connect(self.read_single_register)
                    widget.write_requested.connect(self.write_single_register)

                vertical_layout_for_subgroups.addWidget(sub_group_box)

            scroll_area.setWidget(scroll_content_widget)
            tab_main_layout.addWidget(scroll_area)
            self.tabs.addTab(tab_container_widget, group_name)

            read_all_btn.clicked.connect(lambda _, sc=scroll_content_widget: self.read_all_registers(sc))
            write_all_btn.clicked.connect(lambda _, sc=scroll_content_widget: self.write_all_registers(sc))

        return self.tabs

    def _create_log_panel(self):
        panel = QGroupBox("输出日志")
        layout = QVBoxLayout(panel)
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)

        clear_btn_layout = QHBoxLayout()
        clear_btn_layout.addStretch()
        clear_btn = QPushButton("清空")
        clear_btn_layout.addWidget(clear_btn)

        layout.addWidget(self.log_output)
        layout.addLayout(clear_btn_layout)

        clear_btn.clicked.connect(self.log_output.clear)
        return panel

    def log(self, level, message):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        log_line = f"[{timestamp}][{level.upper()}] {message}"

        color_map = {"info": QColor("#2ECC71"), "warn": QColor("#F39C12"), "error": QColor("#E74C3C")}

        self.log_output.moveCursor(self.log_output.textCursor().MoveOperation.End)

        char_format = QTextCharFormat()
        char_format.setForeground(color_map.get(level, QColor("white")))

        self.log_output.setCurrentCharFormat(char_format)
        self.log_output.append(log_line)  # Use append to automatically add a newline
        self.log_output.ensureCursorVisible()

    def connect_device(self):
        port = self.port_combo.currentText()
        baudrate = int(self.baud_combo.currentText())

        self.modbus_thread = QThread()
        self.modbus_worker = ModbusWorker(port, baudrate, parity='N', stopbits=1, timeout=1)
        self.modbus_worker.moveToThread(self.modbus_thread)

        self.modbus_thread.started.connect(self.modbus_worker.connect_device)
        self.modbus_worker.connection_status.connect(self.on_connection_status)
        self.modbus_worker.log_message.connect(self.log)
        self.modbus_worker.read_result.connect(self.on_read_result)
        self.modbus_worker.write_result.connect(self.on_write_result)

        self.modbus_thread.start()
        self.connect_btn.setEnabled(False)
        self.log("info", f"正在尝试连接 {port} @ {baudrate}...")

    def disconnect_device(self):
        if self.modbus_thread and self.modbus_thread.isRunning():
            # Direct call to worker method
            self.modbus_worker.disconnect_device()
            self.modbus_thread.quit()
            self.modbus_thread.wait(2000)
        self.on_connection_status(False, "手动断开")

    def on_connection_status(self, is_connected, message):
        self.status_light.set_status(is_connected)
        self.connect_btn.setEnabled(not is_connected)
        self.disconnect_btn.setEnabled(is_connected)

    def read_single_register(self, config):
        if self.modbus_worker: self.modbus_worker.read_logical_value(config)

    def write_single_register(self, config, value):
        if self.modbus_worker: self.modbus_worker.write_logical_value(config, value)

    def on_read_result(self, reg_id, result):
        if reg_id in self.register_widgets:
            if isinstance(result, Exception):
                self.log("warn", f"读取 {reg_id} 失败: {result}")
            else:
                self.register_widgets[reg_id].set_value(result)

    def on_write_result(self, reg_id, success, result):
        if success:
            if reg_id in self.register_widgets:
                self.register_widgets[reg_id].set_value(result)
        else:
            QMessageBox.critical(self, "写入失败", f"写入 {reg_id} 失败: {result}")
            self.log("error", f"写入 {reg_id} 失败: {result}")

    def read_all_registers(self, parent_widget):
        if not (self.modbus_worker and self.disconnect_btn.isEnabled()):
            self.log("warn", "请先连接设备")
            return

        # Collect all register configurations from the current tab
        configs_to_read = [w.config for w in parent_widget.findChildren(RegisterWidget)]

        if configs_to_read:
            self.log("info", f"开始批量读取 {len(configs_to_read)} 个寄存器...")
            # Pass the whole list to the new worker method
            self.modbus_worker.read_multiple_registers(configs_to_read)

    def write_all_registers(self, parent_widget):
        if not (self.modbus_worker and self.disconnect_btn.isEnabled()):
            self.log("warn", "请先连接设备")
            return

        dirty_widgets = [w for w in parent_widget.findChildren(RegisterWidget) if w.is_dirty]
        if not dirty_widgets:
            QMessageBox.information(self, "提示", "没有检测到已修改的参数。")
            return

        reply = QMessageBox.question(self, "确认写入",
                                     f"将要写入 {len(dirty_widgets)} 个已修改的参数，是否继续？",
                                     QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                     QMessageBox.StandardButton.No)

        if reply == QMessageBox.StandardButton.Yes:
            for widget in dirty_widgets:
                self.write_single_register(widget.config, widget.get_value())
                QApplication.processEvents()
                time.sleep(0.05)

    def closeEvent(self, event):
        self.disconnect_device()
        event.accept()

# ==============================================================================
# PART 5: APPLICATION LAUNCH
# ==============================================================================
if __name__ == '__main__':
    app = QApplication(sys.argv)

    # The classes need to be fully defined above, not just placeholders.
    # The following shows the intended logic assuming all classes are fully implemented in this file.

    window = MainWindow()
    window.show()

    sys.exit(app.exec())