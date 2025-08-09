import sys, time, logging, pathlib
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer
import json, pathlib
# ──────────────────────────────────────────────────────────────
# Driver 클래스는 별도 driver.py에 그대로 넣어 두었다고 가정
from driver import Driver, CNT2RAD, RAD2DEG, ZERO_POS
# ──────────────────────────────────────────────────────────────

from schedule_command import parse_schedule, Command
from worker import MotorWorker

def app_dir() -> pathlib.Path:
    """exe가 있는 폴더(개발 중에는 소스 폴더)"""
    if getattr(sys, 'frozen', False):
        return pathlib.Path(sys.executable).parent
    return pathlib.Path(__file__).parent

def resource_path(relative: str) -> pathlib.Path:
    """번들 내부 리소스(.ui, .json)에 접근 (sys._MEIPASS)"""
    base = pathlib.Path(getattr(sys, '_MEIPASS', app_dir()))
    return base / relative

OFFSET_FILE = app_dir() / "offset.json"
SCHEDULE_FILE = app_dir() / "schedule.txt"



VEL_DEF = 2      # RPM / 입력축 : 출력축 = 1rpm : 0.086deg/s 
ACC_DEF = 100    # 기본 가속도(도/초^2)



# ── 메인 윈도우 ────────────────────────────────────────────────
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # .ui 파일 로드 (동일 폴더)
        uic.loadUi(pathlib.Path(__file__).with_name("mainwindow.ui"), self)

        # 내부 상태
        self.drv: Driver | None = None
        self.worker: MotorWorker | None = None  # 워커 스레드

        self.connected = False

        self.schedule_text = self.scheduleload()
        self.base_schedule = parse_schedule(self.schedule_text)
        self.cycle_period = self.base_schedule[-1].t if self.base_schedule else 0

        self.saved_offset = self.offsetload()
        self.lineEdit.setText(str(self.saved_offset))  # 초기 오프셋 표시

        # 타이머: 10 Hz
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.on_tick)
        self.timer.start()

        # 버튼 연결
        self.pushButton_connect.clicked.connect(self.on_connect_clicked)

        self.pushButton_zoffset.clicked.connect(self.on_zoffset_clicked)
        self.pushButton_zoffset_save.clicked.connect(self.on_zoffset_save)



        self.pushButton_homing.setEnabled(False)  # 홈 버튼은 연결 후 활성화
        self.pushButton_estop.setEnabled(False)    # E-STOP 버튼은 연결 후 활성화
        
        self.pushButton_runloop.setEnabled(False)  # 루프 버튼은 연결 후 활성화
        self.pushButton_gozero.setEnabled(False)
        self.pushButton_m0.setEnabled(False)       # M0 버튼은 연결 후 활성화
        self.pushButton_m1.setEnabled(False)       # M1 버튼은 연결 후 활성화
        self.pushButton_m2.setEnabled(False)       # M2 버튼은 연결 후 활성화
        self.pushButton_m3.setEnabled(False)       # M3 버튼은 연결 후 활성화


        self.pushButton_homing.clicked.connect(self.on_homing_clicked)
        self.pushButton_runloop.clicked.connect(self.on_runloop_clicked)
        self.pushButton_estop.clicked.connect(self.on_estop_clicked)
        self.pushButton_gozero.clicked.connect(self.on_gozero_clicked)
        self.pushButton_m0.clicked.connect(self.on_m0_clicked)
        self.pushButton_m1.clicked.connect(self.on_m1_clicked)
        self.pushButton_m2.clicked.connect(self.on_m2_clicked)
        self.pushButton_m3.clicked.connect(self.on_m3_clicked)

    def scheduleload(self) -> str:
        """앱 시작 시 호출 schedule.txt 전체를 문자열로 반환"""
        if SCHEDULE_FILE.exists():
            try:
                return SCHEDULE_FILE.read_text(encoding="utf-8")
            except Exception as e:
                logging.warning(f"schedule load error: {e}")
        # 파일이 없거나 오류일 때 기본 샘플 제공
        return """
          # t, deg, vel, acc, dwell
            0.0,  +2.0, 10, 100, 0
            3.0,  -2.0, 10, 100, 0
            6.0,   0.0, 10, 100, 0
        """

    def offsetload(self) -> int:
        if OFFSET_FILE.exists():
            try:
                with OFFSET_FILE.open() as f:
                    data = json.load(f)
                    return int(data.get("zoffset", 0))
            except Exception as e:
                logging.warning(f"offset load error: {e}")
        return 0        # 기본값

    def start_worker(self):
        """워커 스레드 시작"""
        if self.worker is None:
            self.worker = MotorWorker(self.drv, self.base_schedule, self.cycle_period)
            self.worker.start()
            logging.info("Motor worker started")
        else:
            logging.warning("Motor worker already running")
    
    def stop_worker(self):
        """워커 스레드 중지"""
        if self.worker is not None:
            self.worker.stop()
            self.worker.join()
            self.worker = None
            logging.info("Motor worker stopped")

    
    # ─────────────────────────────────────────────────────────
    # 버튼 핸들러
    def on_connect_clicked(self):
        if not self.connected:
            try:
                self.drv = Driver()
                self.drv.connect()
                self.drv.zoffset = self.saved_offset  # 저장된 오프셋 적용
                self.connected = True
                self.t0 = time.perf_counter()
                self.label_connect.setText("YES")
                logging.info("Driver connected")
                self.pushButton_connect.setText("DISCONNECT")
                self.start_worker()
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Connect Error", str(e))
        else:
            # 이미 연결됨 → 해제
            self.stop_worker()

            self.drv.client.close()
            self.connected = False
            self.label_connect.setText("NO")
            logging.info("Driver disconnected")
            self.pushButton_connect.setText("CONNECT")

            self.pushButton_runloop.setEnabled(False)  # 루프 버튼 비활성화
            self.pushButton_gozero.setEnabled(False)
            self.pushButton_m0.setEnabled(False)       # M0 버튼 비활성화
            self.pushButton_m1.setEnabled(False)       # M1 버튼 비활성화
            self.pushButton_m2.setEnabled(False)       # M2 버튼 비활성화
            self.pushButton_m3.setEnabled(False)       # M3 버튼 비활성화
            self.pushButton_homing.setEnabled(False)  # 홈 버튼 비활성화
            self.pushButton_estop.setEnabled(False)    # E-STOP 버튼 비활성화

        
    def on_homing_clicked(self):
        if self.connected:
            with self.worker.lock:
                self.drv.homing()

    def on_runloop_clicked(self):
        if not self.connected:
            return

        self.worker.looping = not self.worker.looping

        self.loop_cmd_time = time.perf_counter()
        self.pushButton_runloop.setText("STOP LOOP" if self.worker.looping else "RUN LOOP")
        if self.worker.looping:

            if self.worker is not None:
                self.worker.start_loop()
        else:
            if self.worker is not None:
                self.worker.stop_loop()

    def on_estop_clicked(self):
        if self.connected:
            with self.worker.lock:
                self.drv.estop()
            
            self.worker.looping = False  # M2 버튼 클릭 시 루프 중지

    def on_gozero_clicked(self):
        if self.connected:
            with self.worker.lock:
                self.drv.move(0.0, VEL_DEF, ACC_DEF, 0)
            self.worker.looping = False  # M2 버튼 클릭 시 루프 중지

    def on_m0_clicked(self):
        if self.connected:
            with self.worker.lock:
                self.drv.move(self.drv.qdeg - 0.1, VEL_DEF, ACC_DEF, 0)
            self.worker.looping = False  # M2 버튼 클릭 시 루프 중지
    
    def on_m1_clicked(self):
        if self.connected:
            with self.worker.lock:
                self.drv.move(self.drv.qdeg - 0.05, VEL_DEF, ACC_DEF, 0)
            self.worker.looping = False  # M2 버튼 클릭 시 루프 중지
            

    def on_m2_clicked(self):
        if self.connected:
            with self.worker.lock:
                self.drv.move(self.drv.qdeg + 0.05, VEL_DEF, ACC_DEF, 0)
            self.worker.looping = False  # M2 버튼 클릭 시 루프 중지

    def on_m3_clicked(self):
        if self.connected:
            with self.worker.lock:
                self.drv.move(self.drv.qdeg + 0.1, VEL_DEF, ACC_DEF, 0)
            self.worker.looping = False  # M2 버튼 클릭 시 루프 중지


    def on_zoffset_clicked(self):
        if self.connected:
            try:
                z_offset = int(self.lineEdit.text())
                self.drv.zoffset = z_offset
                # self.drv.set_z_offset(z_offset)
                logging.info(f"Z Offset set to {z_offset} cnt")
            except ValueError:
                logging.error("Invalid Z Offset input")
                # QtWidgets.QMessageBox.warning(self, "Input Error", "Please enter a valid number for Z Offset.")

    def on_zoffset_save(self):
        try:
            with OFFSET_FILE.open("w") as f:
                json.dump({"zoffset": int(self.drv.zoffset)}, f)
            logging.info(f"Z Offset saved: {self.drv.zoffset} cnt")
            # QtWidgets.QMessageBox.information(self, "Save Offset", "저장 완료!")
        except Exception as e:
            print(f"Error saving offset: {e}")
            # QtWidgets.QMessageBox.critical(self, "Save Offset", str(e))

    # ─────────────────────────────────────────────────────────
    # 10 Hz 주기 함수
    def on_tick(self):
        # 1) 연결상태 업데이트
        if self.connected:
            try:
                self.label_time.setText(f"{time.perf_counter() - self.t0:6.2f}")

                with self.worker.lock:
                    st = self.worker.stat.copy()

                if st:
                # stat_word = self.drv.rd16(0x6002)
                    self.label_q.setText(f"{st['qdeg']:+7.3f}")
                    self.label_qdot.setText(f"{st['vel']*RAD2DEG:+7.3f}")
                    self.label_torque.setText(str(st['torque']))
                    self.label_rdy.setText("●" if st['ready'] else "")
                    self.label_run.setText("●" if st['run'] else "")
                    self.label_err.setText("●" if st['error'] else "")
                    self.label_hom.setText("●" if st['homing'] else "")
                    self.label_cnt.setText(f"{st['q']:+10d}") # cnt 단위

                    self.pushButton_homing.setEnabled(True)  # 홈 버튼은 연결 후 활성화
                    self.pushButton_estop.setEnabled(True)    # E-STOP 버튼은 연결 후 활성화

                    if st['homing']: # 호밍 완료 상태
                        self.pushButton_runloop.setEnabled(True) 
                        self.pushButton_gozero.setEnabled(True)
                        self.pushButton_m0.setEnabled(True)
                        self.pushButton_m1.setEnabled(True)
                        self.pushButton_m2.setEnabled(True)
                        self.pushButton_m3.setEnabled(True)
            except Exception as e:
                logging.error(e)
        
        else:
            self.label_time.setText("--")


# ── 진입점 ─────────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
