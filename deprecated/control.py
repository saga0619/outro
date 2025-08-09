# pip install "pymodbus>=3.8"
import time, logging
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from pymodbus import pymodbus_apply_logging_config
import msvcrt

PORT      = "COM3"
BAUDRATE  = 38400
SLAVE_ID  = 1          # 드라이브 국번(RSC 다이얼 + Pr5.31)

RAD2DEG = 180.0 / 3.14159265358979323846  # radian → degree

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


def get_key_nonblocking():
    return msvcrt.getch().decode() if msvcrt.kbhit() else None


class Driver:
    GEARRATIO = 70
    CNT2REV = 1.0 / (1E+4 * GEARRATIO)  # 카운트 → 회전수
    CNT2RAD = 2.0 * 3.14159265358979323846 * CNT2REV  # 카운트 → radian

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

    # ---------- 통신 ----------
    def connect(self):
        logging.basicConfig(level=logging.INFO)
        # pymodbus_apply_logging_config("DEBUG")           # 파일 log
        if not self.client.connect():
            raise RuntimeError("Modbus 연결 실패")

    def _u16(self, reg):
        """INT16 → 부호있는 Python int"""
        return self.client.convert_from_registers([reg],
                                                 data_type=self.client.DATATYPE.INT16)

    def _u32(self, regs):
        """INT32(2워드) → 부호있는 Python int"""
        return self.client.convert_from_registers(regs,
                                                 data_type=self.client.DATATYPE.INT32)

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
        self.stat["vel"]     = self.stat["qdot"] * self.CNT2RAD

        # 절대 위치 (0x602C/0x602D)
        rr2 = self.client.read_holding_registers(0x602C, count=2, slave=self.slave)
        if rr2.isError(): raise ModbusException(rr2)
        self.stat["q"]   = self._u32(rr2.registers)           # cnt
        self.stat["pos"] = self.stat["q"] * self.CNT2RAD
        return self.stat

    # ---------- 느린 연속 JOG ----------
    def slow_jog(self, direction="left", duration=3.0):
        """
        direction: 'left' or 'right'
        duration : sec
        드라이브 컨트롤워드(0x1801)에 100 ms JOG 트리거를
        90 ms 간격으로 반복해 모터를 계속 돌린다.
        """
        cmd = 0x4001 if direction == "left" else 0x8001
        stop_cmd = 0x0001
        end_t = time.time() + duration
        while time.time() < end_t:
            self.client.write_register(0x1801, cmd, slave=self.slave)
            time.sleep(0.09)                      # 90 ms < 100 ms 트리거 폭
        self.client.write_register(0x1801, stop_cmd, slave=self.slave)

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

    def trigger_pr(self, pr_index):
        self.w16(0x6011, pr_index)
        self.w16(0x6010, 0x0001)  # PR 실행 명령
        self.w16

    def move(self, target_deg, vel, acc_dec, wait):    
        CMD_PR6 = 0x010 | 6  # PR6 실행 명령
        self.w16(0x6230, 0x0001)  # 제어워드 설정 0x0003 : Homing
        self.w32(0x6231, target_deg)  # 목표 위치 설정 (절대 위치)
        self.w16(0x6233, vel)      # 모터 위치 제어 모드
        self.w16(0x6234, acc_dec)
        self.w16(0x6235, acc_dec)      # 모터 속도 제어 모드
        self.w16(0x6236, wait)    

        self.w16(0x6002, CMD_PR6)



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

def build_pr_ctrl(jump_en=0, jump_idx=0, coord=0, overlap=0,
                  int_mask=0, motion_type=1):
    """0x6200 제어워드 생성"""
    word = 0
    word |= (jump_en & 0x1) << 14
    word |= (jump_idx & 0x3F) << 8
    word |= (coord & 0x3) << 6
    word |= (overlap & 0x1) << 5
    word |= (int_mask & 0x1) << 4
    word |= (motion_type & 0xF)
    return word & 0xFFFF



# ---------------- 테스트 스크립트 ----------------
if __name__ == "__main__":
    drv = Driver();  drv.connect()

    pr0_ctrl = build_pr_ctrl(jump_en=0, motion_type=1, coord=0)


    DEG2CNT = 1.0 / RAD2DEG / drv.CNT2RAD  # degree → cnt
     # 목표 위치를 카운트로 변환
    
    PATH = 0
    CMD_EXEC = 0x010 | PATH
    CMD_ESTOP = 0x040
             
    # CMD_PR0 : homing
    CMD_PR0 = 0x010 | 0  # PR0 실행 명령
    drv.w16(0x6200, 0x0003)  # 제어워드 설정 0x0003 : Homing


    # CMD_PR1 : move even

    CMD_PR1 = 0x010 | 1  # PR1 실행 명령
    cmd_pos = int((ZERO_POS)* DEG2CNT)        
    drv.w16(0x6208, 0x0001)  # 제어워드 설정 0x0003 : Homing
    drv.w32(0x6209, cmd_pos)  # 목표 위치 설정 (절대 위치)
    drv.w16(0x620B, VEL)      # 모터 위치 제어 모드
    drv.w16(0x620C, ACC_DEC)
    drv.w16(0x620D, ACC_DEC)      # 모터 속도 제어 모드
    drv.w16(0x620E, 0x0000)

    # CMD_PR2 : move minus

    CMD_PR2 = 0x010 | 2  # PR2 실행 명령
    cmd_pos = int((ZERO_POS-TARGET_DEG)* DEG2CNT)        
    drv.w16(0x6210, 0x0001)  # 제어워드 설정 0x0003 : Homing
    drv.w32(0x6211, cmd_pos)  # 목표 위치 설정 (절대 위치)
    drv.w16(0x6213, VEL)      # 모터 위치 제어 모드
    drv.w16(0x6214, ACC_DEC)
    drv.w16(0x6215, ACC_DEC)      # 모터 속도 제어 모드
    drv.w16(0x6216, 0x0000)

    # CMD_PR3 : move plus
    
    CMD_PR3 = 0x010 | 3  # PR3 실행 명령
    cmd_pos = int((ZERO_POS+TARGET_DEG)* DEG2CNT)        
    drv.w16(0x6218, 0x0001)  # 제어워드 설정 0x0003 : Homing
    drv.w32(0x6219, cmd_pos)  # 목표 위치 설정 (절대 위치)
    drv.w16(0x621B, VEL)      # 모터 위치 제어 모드
    drv.w16(0x621C, ACC_DEC)
    drv.w16(0x621D, ACC_DEC)      # 모터 속도 제어 모드
    drv.w16(0x621E, 0x0000)

    CMD_PR4 = 0x010 | 4  # PR4 실행 명령
    drv.w16(0x6220, 0x0001)  # 제어워드 설정 0x0003 : Homing
    drv.w32(0x6221, int((ZERO_POS + TARGET_MIN_DEG)* DEG2CNT))  # 목표 위치 설정 (절대 위치)
    drv.w16(0x6223, VEL)      # 모터 위치 제어 모드
    drv.w16(0x6224, ACC_DEC)
    drv.w16(0x6225, ACC_DEC)      # 모터 속도 제어 모드
    drv.w16(0x6226, 0x0000)

    CMD_PR5 = 0x010 | 5  # PR5 실행 명령
    cmd_pos = int((ZERO_POS - TARGET_MIN_DEG )* DEG2CNT)
    drv.w16(0x6228, 0x0001)  # 제어워드 설정 0x0003 : Homing
    drv.w32(0x6229, cmd_pos)  # 목표 위치 설정 (절대 위치)
    drv.w16(0x622B, VEL)      # 모터 위치 제어 모드
    drv.w16(0x622C, ACC_DEC)
    drv.w16(0x622D, ACC_DEC)      # 모터 속도 제어 모드
    drv.w16(0x622E, 0x0000)



    t0, cnt = time.time(), 0

    t_jog = t0


    # get keyboard input


    move_mode = 0 # 0: stop, 1: move plus, 2: move minus, 3: move even, 4: move plus min, 5: move minus min
    last_move_mode = 0
    triggering_time = 0.0
    wait_time = 0.0

    first_cmd = False
    cmd_once = False

    loop_mode = False
    loop_seq = 0

    left = True
    try:
        while True:

            key = get_key_nonblocking()

            if key == 'q':  # 종료
                drv.w16(0x6002, CMD_ESTOP)
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
                move_mode = MOVE_MINUS
            elif key == 'k': # 0.1 minus
                move_mode = MOVE_PLUS
            elif key == 'l': # Start Loop
                loop_mode = not loop_mode
                loop_seq = 0
                triggering_time = time.time()

            # if move_mode == 1 or move_mode == 2 or move_mode == 4 or move_mode == 5:
            #     first_cmd = True
                # triggering_time = time.time()
                # print("trigger key")

            # else :
            #     print("키 입력: h(홈), e(비상정지), a(왼쪽 이동), d(오른쪽 이동), s(정지), q(종료)")


            if loop_mode:
                if triggering_time < time.time():
                    if loop_seq == 0:
                        move_mode = MOVE_P2
                        triggering_time = triggering_time + WAIT_AT_BOTTOM
                        loop_seq = 1
                    elif loop_seq == 1:
                        move_mode = MOVE_P1
                        triggering_time = triggering_time + WAIT_BEFORE_SLIDE
                        loop_seq = 2
                    elif loop_seq == 2:
                        move_mode = MOVE_M2
                        triggering_time = triggering_time + WAIT_AT_BOTTOM
                        loop_seq = 3
                    elif loop_seq == 3:
                        move_mode = MOVE_M1
                        triggering_time = triggering_time + WAIT_BEFORE_SLIDE
                        loop_seq = 0




            if move_mode == MOVE_HOMING:
                drv.w16(0x6002, CMD_PR0)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                print("homing positive")

            elif move_mode == MOVE_E: 
                # if time.time() > triggering_time:
                drv.w16(0x6002, CMD_PR1)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                print("move even")

            elif move_mode == MOVE_M2:
                # if time.time() > triggering_time:
                drv.w16(0x6002, CMD_PR2)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                cmd_once = True
                print("move minus")

            elif move_mode == MOVE_P2:
                # if time.time() > triggering_time:
                drv.w16(0x6002, CMD_PR3)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                cmd_once = True
                print("move plus")

            elif move_mode == MOVE_P1:
                # if time.time() > triggering_time:
                drv.w16(0x6002, CMD_PR4)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                cmd_once = True
                print("move plus min")
            
            elif move_mode == MOVE_M1:
                # if time.time() > triggering_time:
                drv.w16(0x6002, CMD_PR5)
                last_move_mode = move_mode
                move_mode = MOVE_IDLE
                cmd_once = True
                print("move minus min")

            stat = drv.rd16(0x6002)
            st = drv.poll();  cnt += 1
            q_deg = st['pos'] * RAD2DEG

            print(f"[{time.time() - t0:5.2f}] p={q_deg - ZERO_POS :+9.5f} deg "
                  f"v={st['vel']:+7.4f} rad/s "
                  f"T={st['torque']:+6d} "
                  f"RDY:{st['ready']} RUN:{st['run']} ERR:{st['error']} STAT:{hex(stat)} : {decode_6002(stat)}")


            # if first_cmd and cmd_once:
            #     if 0x0000 <= stat <= 0x000F:   # 0x000P
            #         if last_move_mode == MOVE_M2:
            #             move_mode = MOVE_P2
            #             cmd_once = False
            #             triggering_time_prev = time.time()
            #             triggering_time = triggering_time_prev + WAIT_AT_BOTTOM
            #         elif last_move_mode == MOVE_M1:
            #             move_mode = MOVE_P2
            #             cmd_once = False 
            #             triggering_time_prev = time.time()
            #             triggering_time = triggering_time_prev + WAIT_BEFORE_SLIDE
            #         elif last_move_mode == MOVE_P2:
            #             move_mode = MOVE_M2
            #             cmd_once = False
            #             triggering_time_prev = time.time()
            #             triggering_time = triggering_time_prev + WAIT_AT_BOTTOM
            #         elif last_move_mode == MOVE_P1:
            #             move_mode = MOVE_M2
            #             cmd_once = False
            #             triggering_time_prev = time.time()
            #             triggering_time = triggering_time_prev + WAIT_BEFORE_SLIDE

                    # print(f"다음 모드: {move_mode} at : {triggering_time + wait_time - t0:.2f} sec")

            
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        drv.w16(0x6002, CMD_ESTOP)
        print("\n종료합니다")
    finally:
        drv.client.close()
