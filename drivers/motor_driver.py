# pip install "pymodbus>=3.8"
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
# from pymodbus import pymodbus_apply_logging_config
import time, logging

GEARRATIO = 70
CNT2REV = 1.0 / (1E+4 * GEARRATIO)  # 카운트 → 회전수
CNT2RAD = 2.0 * 3.14159265358979323846 * CNT2REV  # 카운트 → radian
RAD2DEG = 180.0 / 3.14159265358979323846  # radian → degree
DEG2CNT = 1.0 / RAD2DEG / CNT2RAD  # degree → cnt
CNT2DEG = RAD2DEG * CNT2RAD  # cnt → degree

START = 2.87
END = -3.63
ZERO_POS = (START + END)/2 + 0.07
ZERO_POS_CNT = ZERO_POS * DEG2CNT  # 중앙 위치 기준점 (0.0 = 중앙 위치, -3.63 ~ +2.87 도 범위)

class Driver:
    def __init__(self,
                 port="COM3",
                 baudrate=38400,
                 parity="N",           # ← 기본을 'N' 으로
                 stopbits=1,
                 bytesize=8,
                 slave=1):
        self.client = ModbusSerialClient(
            port=port, baudrate=baudrate,
            parity=parity, stopbits=stopbits, bytesize=bytesize,
            timeout=1.0, retries=3)
        self.slave = slave
        self.stat  = {}
        self.qdeg = 0.0
        self.zoffset = ZERO_POS_CNT  # 절대 위치 기준점 (0.0 = 중앙 위치, -3.63 ~ +2.87 도 범위)

    # ---------- 통신 ----------
    def connect(self):
        logging.basicConfig(level=logging.INFO)
        # pymodbus_apply_logging_config("DEBUG")           # 파일 log
        if not self.client.connect():
            raise RuntimeError("Modbus 연결 실패")
        self.w16(0x1801, 0x1111) # clear alarm
        # self.w16(0x0403, 0x03)

        # current = self.rd16(0x0401)
        # print(f"DI3 current mode code = {current}")



    def _u16(self, reg):
        """INT16 → 부호있는 Python int"""
        return self.client.convert_from_registers([reg], data_type=self.client.DATATYPE.INT16)

    def _u32(self, regs):
        """INT32(2워드) → 부호있는 Python int"""
        return self.client.convert_from_registers(regs, data_type=self.client.DATATYPE.INT32)

    # ---------- 데이터 폴링 ----------
    def poll(self):
        # 상태·속도·토크 (0x0B05~0x0B07)
        rr = self.client.read_holding_registers(0x0B05, count=3, slave=self.slave)
        if rr.isError(): raise ModbusException(rr)

        w0, w1, w2 = rr.registers
        self.stat["ready"]   = bool(w0 & 0x0001)
        self.stat["run"]     = bool(w0 & 0x0002)
        self.stat["error"]   = bool(w0 & 0x0004)
        self.stat["homing"]   = bool(w0 & 0x0008)
        self.stat["qdot"]    = self._u16(w1)                  # cnt/s
        self.stat["torque"]  = self._u16(w2)                  # 단위: 매뉴얼 참조
        self.stat["vel"]     = self.stat["qdot"] * CNT2RAD

        # 절대 위치 (0x602C/0x602D)
        rr2 = self.client.read_holding_registers(0x602C, count=2, slave=self.slave)
        if rr2.isError(): raise ModbusException(rr2)
        self.stat["q"]   = self._u32(rr2.registers)           # cnt
        self.stat["pos"] = self.stat["q"] * CNT2RAD
        self.qdeg = self.stat["pos"] * RAD2DEG - self.zoffset * CNT2DEG  # 절대 위치 (degree)
        self.stat["qdeg"] = self.qdeg

        return self.stat
    
    def w16(self, addr, val):  
        self.client.write_register(addr, val & 0xFFFF, slave=self.slave)

    def w32(self, addr, val):
        hi, lo = (val >> 16) & 0xFFFF, val & 0xFFFF
        self.client.write_registers(addr, [hi, lo], slave=self.slave)

    def rd16(self, addr):
        """16-bit 단일 레지스터 읽기 (1-based 주소 → 0-based)"""
        rr = self.client.read_holding_registers(addr, count=1, slave=self.slave)
        if rr.isError():
            raise ModbusException(rr)
        return rr.registers[0]
    
    def homing(self):
        CMD_PR0 = 0x010 | 0  # PR1 실행 명령
        self.w16(0x6200, 0x0003)  # 제어워드 설정 0x0003 : Homing
        self.w16(0x6002, CMD_PR0)  # PR1 명령 전송

    def move(self, target_deg, vel, acc_dec, wait):    
        CMD_PR1 = 0x010 | 1  # PR1 실행 명령
        cmd_pos = int((target_deg)* DEG2CNT) + self.zoffset
        self.w16(0x6208, 0x0001)  # 제어워드 설정 0x0003 : Homing
        self.w32(0x6209, cmd_pos)  # 목표 위치 설정 (절대 위치)
        self.w16(0x620B, vel)      # 모터 위치 제어 모드
        self.w16(0x620C, acc_dec)
        self.w16(0x620D, acc_dec)      # 모터 속도 제어 모드
        self.w16(0x620E, wait)
        self.w16(0x6002, CMD_PR1)  # PR1 명령 전송

    def estop(self):
        """비상정지 명령"""
        CMD_ESTOP = 0x040
        self.w16(0x6002, CMD_ESTOP)

def decode_6002(word: int) -> str:
    """0x6002 상태코드 → 설명문"""
    path = word & 0xF           # 하위 nibble = Path 번호
    if   0x0000 <= word <= 0x000F:   # 0x000P
        return f"[PR{path}] positioning completed (idle)"
    elif 0x0010 <= word <= 0x001F:   # 0x01P
        return f"[PR{path}] command yet to be accepted"
    elif word in (0x020, 0x040):
        return "Reset / E-Stop yet to be accepted"
    elif 0x0100 <= word <= 0x010F:   # 0x10P
        return f"[PR{path}] path motion underway"
    elif word == 0x200:
        return "Command completed, waiting for next positioning"
    else:
        return f"Unknown status 0x{word:04X}"
