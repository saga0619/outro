import sys, time, logging, pathlib
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QTextCursor
import json, pathlib
# ──────────────────────────────────────────────────────────────
# Driver 클래스는 별도 driver.py에 그대로 넣어 두었다고 가정
from driver import Driver, CNT2RAD, RAD2DEG, ZERO_POS
# ──────────────────────────────────────────────────────────────

from schedule_command import parse_schedule, Command
from motor_worker import MotorWorker
from ardu_worker import ArduinoWorker

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


# ── QTextBrowser 로깅 핸들러 ────────────────────────────────────
class LogSignalEmitter(QObject):
    log_signal = pyqtSignal(str)

class QTextBrowserHandler(logging.Handler):
    def __init__(self, text_browser):
        super().__init__()
        self.text_browser = text_browser
        self.signal_emitter = LogSignalEmitter()
        self.signal_emitter.log_signal.connect(self.text_browser.append)
        
    def emit(self, record):
        msg = self.format(record)
        # 시그널을 통해 GUI 스레드에서 안전하게 텍스트 추가
        self.signal_emitter.log_signal.emit(msg)


# ── 메인 윈도우 ────────────────────────────────────────────────
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # .ui 파일 로드 (동일 폴더)
        uic.loadUi(pathlib.Path(__file__).with_name("mainwindow.ui"), self)

        # 내부 상태
        self.drv: Driver | None = None
        self.motor_worker: MotorWorker | None = None  # 워커 스레드
        self.arduino_worker: ArduinoWorker | None = None  # Arduino 워커 스레드

        self.connected = False
        self.arduino_connected = False

        self.schedule_text = self.scheduleload()
        self.base_schedule = parse_schedule(self.schedule_text)
        self.cycle_period = self.base_schedule[-1].t if self.base_schedule else 0

        self.saved_offset = self.offsetload()
        self.lineEdit.setText(str(self.saved_offset))  # 초기 오프셋 표시

        # 타이머: 60 Hz
        self.timer = QTimer(self)
        self.timer.setInterval(17)  # 1000ms / 60Hz ≈ 16.67ms
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

        # Arduino UI 요소 연결
        self.setup_arduino_ui()
        self.setup_logging()

        # Arduino 연결 버튼 연결
        self.pushButton_arduino_connect.clicked.connect(self.on_arduino_connect_clicked)
        
        # LED 제어 버튼들 연결
        if hasattr(self, 'pushButton_ledon'):
            self.pushButton_ledon.clicked.connect(self.on_ledon_clicked)
        if hasattr(self, 'pushButton_ledoff'):
            self.pushButton_ledoff.clicked.connect(self.on_ledoff_clicked)
        if hasattr(self, 'pushButton_ledcmd'):
            self.pushButton_ledcmd.clicked.connect(self.on_ledcmd_clicked)
        
        # Arduino 관련 버튼들 초기 비활성화
        self.pushButton_motoron.setEnabled(False)
        if hasattr(self, 'pushButton_ledon'):
            self.pushButton_ledon.setEnabled(False)
        if hasattr(self, 'pushButton_ledoff'):
            self.pushButton_ledoff.setEnabled(False)
        if hasattr(self, 'pushButton_ledcmd'):
            self.pushButton_ledcmd.setEnabled(False)

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

    def start_motor_worker(self):
        """워커 스레드 시작"""
        if self.motor_worker is None:
            self.motor_worker = MotorWorker(self.drv, self.base_schedule, self.cycle_period)
            self.motor_worker.start()
            logging.info("Motor worker started")
        else:
            logging.warning("Motor worker already running")
    
    def stop_motor_worker(self):
        """워커 스레드 중지"""
        if self.motor_worker is not None:
            self.motor_worker.stop()
            self.motor_worker.join()
            self.motor_worker = None
            logging.info("Motor worker stopped")

    def start_arduino_worker(self, port="COM4"):
        """Arduino 워커 스레드 시작"""
        if self.arduino_worker is None:
            self.arduino_worker = ArduinoWorker(port=port)
            if self.arduino_worker.connect():
                self.arduino_worker.start()
                self.arduino_connected = True
                logging.info(f"Arduino worker started on {port}")
                return True
            else:
                self.arduino_worker = None
                self.arduino_connected = False
                logging.error(f"Failed to connect Arduino on {port}")
                return False
        else:
            logging.warning("Arduino worker already running")
            return False
    
    def stop_arduino_worker(self):
        """Arduino 워커 스레드 중지"""
        if self.arduino_worker is not None:
            self.arduino_worker.stop()
            self.arduino_worker.join()
            self.arduino_worker = None
            self.arduino_connected = False
            logging.info("Arduino worker stopped")

    def setup_arduino_ui(self):
        """Arduino UI 요소 설정"""
        # Motor on/off 버튼 연결
        if hasattr(self, 'pushButton_motoron'):
            self.pushButton_motoron.clicked.connect(self.on_motoron_clicked)
            self.motor_on = False  # 모터 상태 추적
        
        # LED 라벨 초기화
        self.led_labels = []
        for i in range(1, 7):  # label_led1 ~ label_led6
            label_name = f'label_led{i}'
            if hasattr(self, label_name):
                label = getattr(self, label_name)
                self.led_labels.append(label)
                label.setText("OFF")
                # label.setStyleSheet("background-color: gray; color: white; padding: 2px;")
        
        # 스위치 라벨 초기화
        if hasattr(self, 'label_sw1'):
            self.label_sw1.setText("OFF")
            # self.label_sw1.setStyleSheet("background-color: gray; color: white; padding: 2px;")
        if hasattr(self, 'label_sw2'):
            self.label_sw2.setText("OFF")
            # self.label_sw2.setStyleSheet("background-color: gray; color: white; padding: 2px;")

    def setup_logging(self):
        """로깅 시스템 설정"""
        # QTextBrowser가 있는지 확인하고 로깅 핸들러 설정
        if hasattr(self, 'textBrowser'):
            # 기존 핸들러 제거
            logger = logging.getLogger()
            for handler in logger.handlers[:]:
                logger.removeHandler(handler)
            
            # QTextBrowser 핸들러 추가
            text_handler = QTextBrowserHandler(self.textBrowser)
            text_handler.setFormatter(logging.Formatter(
                '%(asctime)s - %(levelname)s - %(message)s',
                datefmt='%H:%M:%S'
            ))
            logger.addHandler(text_handler)
            logger.setLevel(logging.INFO)
            
            logging.info("Logging system initialized with QTextBrowser")
        else:
            logging.warning("textBrowser not found, using console logging")

    def on_motoron_clicked(self):
        """Motor On/Off 버튼 핸들러"""
        if self.arduino_worker is not None:
            self.motor_on = not self.motor_on
            signal = 1 if self.motor_on else 0
            self.arduino_worker.set_signal(signal)
            
            if hasattr(self, 'pushButton_motoron'):
                self.pushButton_motoron.setText("MOTOR OFF" if self.motor_on else "MOTOR ON")
                # self.pushButton_motoron.setStyleSheet(
                #     "background-color: red; color: white;" if self.motor_on 
                #     else "background-color: green; color: white;"
                # )
            
            logging.info(f"Motor {'ON' if self.motor_on else 'OFF'}")
        else:
            logging.warning("Arduino worker not available")

    def update_arduino_status_display(self, status):
        """Arduino 상태를 GUI에 업데이트"""

        # print(status)
        # LED 상태 업데이트
        if 'received_brightness' in status and len(self.led_labels) >= 6:
            brightness_values = status['received_brightness']
            for i, (label, brightness) in enumerate(zip(self.led_labels, brightness_values)):
                # if brightness > 128:  # 밝기가 50% 이상이면 ON
                #     label.setText(f"{brightness}")
                #     # label.setStyleSheet("background-color: green; color: white; padding: 2px;")
                # else:
                label.setText(f"{brightness}")
                    # label.setStyleSheet("background-color: gray; color: white; padding: 2px;")
        
        # 스위치 상태 업데이트
        if 'switch_states' in status:
            switch_states = status['switch_states']
            
            if hasattr(self, 'label_sw1'):
                sw1_state = bool(switch_states & 0x40)  # 6
                self.label_sw1.setText("ON" if sw1_state else "OFF")
                # self.label_sw1.setStyleSheet(
                #     "background-color: blue; color: white; padding: 2px;" if sw1_state
                #     else "background-color: gray; color: white; padding: 2px;"
                # )
            
            if hasattr(self, 'label_sw2'):
                sw2_state = bool(switch_states & 0x80)  # 7
                self.label_sw2.setText("ON" if sw2_state else "OFF")
                # self.label_sw2.setStyleSheet(
                #     "background-color: blue; color: white; padding: 2px;" if sw2_state
                #     else "background-color: gray; color: white; padding: 2px;"
                # )

    
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
                self.label_connect.setText("ON")
                logging.info("Driver connected")
                self.pushButton_connect.setText("DISCONNECT")
                self.start_motor_worker()
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Connect Error", str(e))
        else:
            # 이미 연결됨 → 해제
            self.stop_motor_worker()

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
            with self.motor_worker.lock:
                self.drv.homing()

    def on_runloop_clicked(self):
        if not self.connected:
            return

        self.motor_worker.looping = not self.motor_worker.looping

        self.loop_cmd_time = time.perf_counter()
        self.pushButton_runloop.setText("STOP LOOP" if self.motor_worker.looping else "RUN LOOP")
        if self.motor_worker.looping:

            if self.motor_worker is not None:
                self.motor_worker.start_loop()
        else:
            if self.motor_worker is not None:
                self.motor_worker.stop_loop()

    def on_estop_clicked(self):
        if self.connected:
            with self.motor_worker.lock:
                self.drv.estop()
            
            self.motor_worker.looping = False  # M2 버튼 클릭 시 루프 중지

    def on_gozero_clicked(self):
        if self.connected:
            with self.motor_worker.lock:
                self.drv.move(0.0, VEL_DEF, ACC_DEF, 0)
            self.motor_worker.looping = False  # M2 버튼 클릭 시 루프 중지

    def on_m0_clicked(self):
        if self.connected:
            with self.motor_worker.lock:
                self.drv.move(self.drv.qdeg - 0.1, VEL_DEF, ACC_DEF, 0)
            self.motor_worker.looping = False  # M2 버튼 클릭 시 루프 중지
    
    def on_m1_clicked(self):
        if self.connected:
            with self.motor_worker.lock:
                self.drv.move(self.drv.qdeg - 0.05, VEL_DEF, ACC_DEF, 0)
            self.motor_worker.looping = False  # M2 버튼 클릭 시 루프 중지
            

    def on_m2_clicked(self):
        if self.connected:
            with self.motor_worker.lock:
                self.drv.move(self.drv.qdeg + 0.05, VEL_DEF, ACC_DEF, 0)
            self.motor_worker.looping = False  # M2 버튼 클릭 시 루프 중지

    def on_m3_clicked(self):
        if self.connected:
            with self.motor_worker.lock:
                self.drv.move(self.drv.qdeg + 0.1, VEL_DEF, ACC_DEF, 0)
            self.motor_worker.looping = False  # M2 버튼 클릭 시 루프 중지


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
            logging.error(f"Error saving offset: {e}")
            # QtWidgets.QMessageBox.critical(self, "Save Offset", str(e))

    def on_arduino_connect_clicked(self):
        """Arduino 연결/해제 버튼 핸들러"""
        if not self.arduino_connected:
            # Arduino 연결 시도
            if self.start_arduino_worker():
                self.pushButton_arduino_connect.setText("SERIAL DISCONNECT")
                self.label_arduino_status.setText("ON")
                self.pushButton_motoron.setEnabled(True)
                # LED 제어 버튼들 활성화
                if hasattr(self, 'pushButton_ledon'):
                    self.pushButton_ledon.setEnabled(True)
                if hasattr(self, 'pushButton_ledoff'):
                    self.pushButton_ledoff.setEnabled(True)
                if hasattr(self, 'pushButton_ledcmd'):
                    self.pushButton_ledcmd.setEnabled(True)
                logging.info("Arduino connected successfully")
            else:
                logging.error("Failed to connect Arduino")
                QtWidgets.QMessageBox.critical(self, "Arduino Connect Error", "Failed to connect to Arduino on COM4")
        else:
            # Arduino 연결 해제
            self.stop_arduino_worker()
            self.pushButton_arduino_connect.setText("SERIAL CONNECT")
            self.label_arduino_status.setText("NO")
            self.pushButton_motoron.setEnabled(False)
            # LED 제어 버튼들 비활성화
            if hasattr(self, 'pushButton_ledon'):
                self.pushButton_ledon.setEnabled(False)
            if hasattr(self, 'pushButton_ledoff'):
                self.pushButton_ledoff.setEnabled(False)
            if hasattr(self, 'pushButton_ledcmd'):
                self.pushButton_ledcmd.setEnabled(False)
            # 모터 상태 초기화
            if hasattr(self, 'motor_on'):
                self.motor_on = False
                self.pushButton_motoron.setText("MOTOR ON")
            logging.info("Arduino disconnected")

    def on_ledon_clicked(self):
        """LED ON 버튼 핸들러 - 모든 LED를 255로 설정"""
        if self.arduino_worker is not None:
            self.arduino_worker.set_all_leds(255)
            logging.info("All LEDs turned ON (255)")
        else:
            logging.warning("Arduino worker not available")

    def on_ledoff_clicked(self):
        """LED OFF 버튼 핸들러 - 모든 LED를 0으로 설정"""
        if self.arduino_worker is not None:
            self.arduino_worker.set_all_leds(0)
            logging.info("All LEDs turned OFF (0)")
        else:
            logging.warning("Arduino worker not available")

    def on_ledcmd_clicked(self):
        """LED CMD 버튼 핸들러 - 특정 LED의 밝기 설정"""
        if self.arduino_worker is not None:
            try:
                # SpinBox에서 LED 번호 가져오기 (1-6을 0-5로 변환)
                if hasattr(self, 'spinBox_led'):
                    led_number = self.spinBox_led.value() - 1  # 1-6 -> 0-5
                else:
                    logging.error("spinBox_led not found")
                    return
                
                # LineEdit에서 밝기 값 가져오기
                if hasattr(self, 'lineEdit_led'):
                    brightness_text = self.lineEdit_led.text()
                    brightness = int(brightness_text)
                    
                    # 범위 체크
                    if not (0 <= brightness <= 255):
                        logging.error("Brightness value must be between 0 and 255")
                        QtWidgets.QMessageBox.warning(self, "Input Error", "Brightness value must be between 0 and 255")
                        return
                    
                    if not (0 <= led_number <= 5):
                        logging.error("LED number must be between 1 and 6")
                        QtWidgets.QMessageBox.warning(self, "Input Error", "LED number must be between 1 and 6")
                        return
                    
                    # LED 밝기 설정
                    self.arduino_worker.set_led_brightness(led_number, brightness)
                    logging.info(f"LED {led_number + 1} brightness set to {brightness}")
                    
                else:
                    logging.error("lineEdit_led not found")
                    return
                    
            except ValueError:
                logging.error("Invalid brightness value")
                QtWidgets.QMessageBox.warning(self, "Input Error", "Please enter a valid number (0-255)")
            except Exception as e:
                logging.error(f"Error setting LED brightness: {e}")
        else:
            logging.warning("Arduino worker not available")

    # ─────────────────────────────────────────────────────────
    # 60 Hz 주기 함수
    def on_tick(self):
        # 1) 연결상태 업데이트
        if self.connected:
            try:
                self.label_time.setText(f"{time.perf_counter() - self.t0:6.2f}")

                with self.motor_worker.lock:
                    st = self.motor_worker.stat.copy()

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

        # 2) Arduino 상태 업데이트
        if self.arduino_worker is not None:
            try:
                arduino_status = self.arduino_worker.get_status()
                # Arduino 상태를 GUI에 업데이트
                self.update_arduino_status_display(arduino_status)
                
                # 상세 로그는 debug 레벨로만 출력
                logging.debug(f"Arduino Status: Connected={arduino_status['connected']}, "
                            f"Digital Output={arduino_status['digital_output']}, "
                            f"Switch States={bin(arduino_status['switch_states'])}, "
                            f"Error Count={arduino_status['error_count']}")
            except Exception as e:
                logging.error(f"Arduino status update error: {e}")

    def closeEvent(self, event):
        """애플리케이션 종료 시 정리 작업"""
        logging.info("Application closing, stopping workers...")
        
        # Motor worker 정리
        if self.connected:
            self.stop_motor_worker()
            if self.drv:
                self.drv.client.close()
        
        # Arduino worker 정리
        self.stop_arduino_worker()
        
        event.accept()
        logging.info("Application closed")


# ── 진입점 ─────────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
