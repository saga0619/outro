# pip install "pymodbus>=3.8"
import time, logging
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from pymodbus import pymodbus_apply_logging_config
import msvcrt
from __future__ import annotations
import time, logging, msvcrt
from dataclasses import dataclass, replace
from typing import List


PORT      = "COM3"
BAUDRATE  = 38400
SLAVE_ID  = 1          # 드라이브 국번(RSC 다이얼 + Pr5.31)
GEARRATIO = 70
CNT2REV = 1.0 / (1E+4 * GEARRATIO)  # 카운트 → 회전수
CNT2RAD = 2.0 * 3.14159265358979323846 * CNT2REV  # 카운트 → radian
RAD2DEG = 180.0 / 3.14159265358979323846  # radian → degree
DEG2CNT = 1.0 / RAD2DEG / CNT2RAD  # degree → cnt

MOVE_ESTOP = -2
MOVE_HOMING = -1
MOVE_IDLE = 0
MOVE_M2 = 1
MOVE_M1 = 2
MOVE_E = 3
MOVE_P1 = 4
MOVE_P2 = 5

MOVE_PLUS = 10
MOVE_MINUS = 11

CMD_ESTOP = 0x040


START = 2.87
END = -3.63

TARGET_DEG = 2
TARGET_MIN_DEG = 0.3
ZERO_POS = (START + END)/2 + 0.07
VEL = 10 # RPM : VEL/GEARRATIO
ACC_DEC = 100 # ms/Krpm
DWELL = 0

WAIT_AT_BOTTOM = 2
WAIT_BEFORE_SLIDE = 10
@dataclass
class Command:
    t: float        # 상대 시간 [s]  (cycle 0 기준)
    deg: float      # 목표 각도 [deg]
    vel: int        # 0x620B 속도
    acc: int        # 가감속
    dwell: int      # dwell
    def shifted(self, delta: float) -> "Command":
        """cycle n용 절대시간으로 시프트한 사본"""
        return replace(self, t=self.t + delta)

def parse_schedule(text: str) -> List[Command]:
    """
    MOVE 전용 스케줄 파서
      예)  0.0,  +2.0,  10, 100, 0
           3.0,  -2.0,  10, 100, 0
    """
    cmds = []
    for ln in text.strip().splitlines():
        ln = ln.split("#", 1)[0].strip()          # 주석 제거
        if not ln:
            continue
        parts = [p.strip() for p in ln.split(",")]
        if len(parts) != 5:
            raise ValueError(f"잘못된 행: {ln}")
        t, deg, vel, acc, dwell = map(float, parts)
        cmds.append(Command(t, deg, int(vel), int(acc), int(dwell)))
    cmds.sort(key=lambda c: c.t)
    return cmds

def get_key_nonblocking():
    return msvcrt.getch().decode() if msvcrt.kbhit() else None


class Driver:
    

    # CNT2RAD = 1.0e-5    # 카운트 → rad

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

    # ---------- 통신 ----------
    def connect(self):
        logging.basicConfig(level=logging.INFO)
        # pymodbus_apply_logging_config("DEBUG")           # 파일 log
        if not self.client.connect():
            raise RuntimeError("Modbus 연결 실패")

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
        self.stat["qdot"]    = self._u16(w1)                  # cnt/s
        self.stat["torque"]  = self._u16(w2)                  # 단위: 매뉴얼 참조
        self.stat["vel"]     = self.stat["qdot"] * CNT2RAD

        # 절대 위치 (0x602C/0x602D)
        rr2 = self.client.read_holding_registers(0x602C, count=2, slave=self.slave)
        if rr2.isError(): raise ModbusException(rr2)
        self.stat["q"]   = self._u32(rr2.registers)           # cnt
        self.stat["pos"] = self.stat["q"] * CNT2RAD

        self.qdeg = st['pos'] * RAD2DEG - ZERO_POS

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
        drv.w16(0x6200, 0x0003)  # 제어워드 설정 0x0003 : Homing
        drv.w16(0x6002, CMD_PR0)  # PR1 명령 전송

    def move(self, target_deg, vel, acc_dec, wait):    
        CMD_PR1 = 0x010 | 1  # PR1 실행 명령
        cmd_pos = int((ZERO_POS + target_deg)* DEG2CNT)
        drv.w16(0x6208, 0x0001)  # 제어워드 설정 0x0003 : Homing
        drv.w32(0x6209, cmd_pos)  # 목표 위치 설정 (절대 위치)
        drv.w16(0x620B, vel)      # 모터 위치 제어 모드
        drv.w16(0x620C, acc_dec)
        drv.w16(0x620D, acc_dec)      # 모터 속도 제어 모드
        drv.w16(0x620E, wait)

        drv.w16(0x6002, CMD_PR1)  # PR1 명령 전송

    def estop(self):
        """비상정지 명령"""
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

# ---------------- 테스트 스크립트 ----------------
if __name__ == "__main__":
    schedule_text = """
      # t, deg, vel, acc, dwell
        0.0,  +2.0, 10, 100, 0
        5.0,  -2.0, 10, 100, 0
    """
    base_schedule = parse_schedule(schedule_text)
    cycle_period  = base_schedule[-1].t           # 마지막 명령 시각 = 한 사이클 길이


    drv = Driver();  drv.connect()


    cycle_idx    = 0
    queue: List[Command] = [cmd.shifted(0) for cmd in base_schedule]

    t0         = time.perf_counter()
    next_tick  = t0
    TICK       = 0.1                             # 10 Hz

    # get keyboard input

    move_mode = 0 # 0: stop, 1: move plus, 2: move minus, 3: move even, 4: move plus min, 5: move minus min
    last_move_mode = 0
    loop_start_time = 0.0
    wait_time = 0.0

    first_cmd = False
    cmd_once = False

    loop_mode = False


    try:
        while True:
            now = time.perf_counter()

            if loop_mode:
                while queue and (now - loop_start_time) >= queue[0].t:
                    cmd = queue.pop(0)
                    drv.move(cmd.deg, cmd.vel, cmd.acc, cmd.dwell)
                    logging.info(f"MOVE {cmd.deg:+}° (cycle {cycle_idx})")
                
                if not queue:  # 스케줄이 비었으면 다음 사이클로
                    cycle_idx += 1
                    shift = cycle_idx * cycle_period
                    queue = [cmd.shifted(shift) for cmd in base_schedule]

            key = get_key_nonblocking()
            if key == 'q':  # 종료
                drv.estop()
                break
            elif key == 'h': #homming
                move_mode = MOVE_HOMING
            elif key == 'e': #estop
                move_mode = MOVE_ESTOP
            elif key == 'a': #move positive 
                move_mode = MOVE_P2
            elif key == 'd':
                move_mode = MOVE_E
            elif key == 'g':
                move_mode = MOVE_M2
            elif key == 's': #move minus
                move_mode = MOVE_P1
            elif key == 'f': #move minus
                move_mode = MOVE_M1
            elif key == 'j': # 0.1 plus
                drv.move(drv.qdeg + 0.1, VEL, ACC_DEC, DWELL)
            elif key == 'k': # 0.1 minus
                drv.move(drv.qdeg - 0.1, VEL, ACC_DEC, DWELL)
            elif key == 'l': # Start Loop
                loop_mode = not loop_mode
                loop_start_time = time.perf_counter()

            if move_mode == MOVE_HOMING:
                drv.homing()
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                print("homing positive")

            elif move_mode == MOVE_E: 
                # if time.time() > triggering_time:
                drv.move(0, VEL, ACC_DEC, DWELL)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                print("move even")

            elif move_mode == MOVE_M2:
                # if time.time() > triggering_time:
                drv.move(-TARGET_DEG, VEL, ACC_DEC, DWELL)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                cmd_once = True
                print("move minus")

            elif move_mode == MOVE_P2:
                # if time.time() > triggering_time:
                drv.move(TARGET_DEG, VEL, ACC_DEC, DWELL)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                cmd_once = True
                print("move plus")

            elif move_mode == MOVE_P1:
                # if time.time() > triggering_time:
                drv.move(TARGET_MIN_DEG, VEL, ACC_DEC, DWELL)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                cmd_once = True
                print("move plus min")
            
            elif move_mode == MOVE_M1:
                # if time.time() > triggering_time:
                drv.move(-TARGET_MIN_DEG, VEL, ACC_DEC, DWELL)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                cmd_once = True
                print("move minus min")

            stat = drv.rd16(0x6002)
            st = drv.poll()

            print(f"[{time.time() - t0:5.2f}] p={drv.qdeg :+9.5f} deg "
                  f"v={st['vel']:+7.4f} rad/s "
                  f"T={st['torque']:+6d} "
                  f"RDY:{st['ready']} RUN:{st['run']} ERR:{st['error']} STAT:{hex(stat)} : {decode_6002(stat)}")

            next_tick += TICK
            sleep_dur = next_tick - time.perf_counter()
            if sleep_dur > 0:
                time.sleep(sleep_dur)
            else:
                next_tick = time.perf_counter()  # 지연 시 catch-up


    except KeyboardInterrupt:
        drv.estop()
        print("\n종료합니다")
    finally:
        drv.client.close()
