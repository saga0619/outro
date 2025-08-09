import sys, time, logging, pathlib
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QTextCursor
import json, pathlib
# ──────────────────────────────────────────────────────────────
# Driver 클래스는 별도 driver.py에 그대로 넣어 두었다고 가정
from drivers.motor_driver import Driver, CNT2RAD, RAD2DEG, ZERO_POS
from drivers.dynamixel.dynamixel_driver import DynamixelDriver, VELOCITY_CONTROL_MODE, POSITION_CONTROL_MODE, EXTENDED_POSITION_CONTROL_MODE
# ──────────────────────────────────────────────────────────────

from src.schedule_command import parse_schedule, Command
from src.motor_worker import MotorWorker
from src.ardu_worker import ArduinoWorker
from src.dynamixel_worker import DynamixelWorker

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
CONFIG_FILE = app_dir() / "config.json"



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
        # .ui 파일 로드 (exe에 포함된 리소스 파일 접근)
        ui_path = resource_path("resource/mainwindow.ui")
        uic.loadUi(ui_path, self)

        # 내부 상태
        self.drv: Driver | None = None
        self.motor_worker: MotorWorker | None = None  # 워커 스레드
        self.arduino_worker: ArduinoWorker | None = None  # Arduino 워커 스레드
        self.dynamixel_driver: DynamixelDriver | None = None  # Dynamixel 드라이버
        self.dynamixel_worker: DynamixelWorker | None = None  # Dynamixel 워커 스레드

        self.connected = False
        self.arduino_connected = False
        self.dynamixel_connected = False

        self.schedule_text = self.scheduleload()
        self.base_schedule = parse_schedule(self.schedule_text)
        self.cycle_period = self.base_schedule[-1].t if self.base_schedule else 0

        # Config 로드 및 초기화
        self.config = self.load_config()
        self.saved_offset = self.config.get("zoffset", 0)
        self.lineEdit.setText(str(self.saved_offset))  # 초기 오프셋 표시
        
        # Ring position 초기값 설정 - config에서 불러온 값으로 GUI 업데이트
        ring_positions = self.config.get("ring_positions", {})
        if hasattr(self, 'lineEdit_ringpos1'):
            ring1_value = ring_positions.get("ring1", 0.0)
            self.lineEdit_ringpos1.setText(str(ring1_value))
            logging.info(f"Ring Position 1 loaded from config: {ring1_value}°")
        if hasattr(self, 'lineEidit_ringpos2'):  # UI에서 오타가 있다면 그대로 사용
            ring2_value = ring_positions.get("ring2", 0.0)
            self.lineEidit_ringpos2.setText(str(ring2_value))
            logging.info(f"Ring Position 2 loaded from config: {ring2_value}°")

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

        # Dynamixel 관련 UI 설정
        self.setup_dynamixel_ui()
        
        # Ring Connect 버튼 연결
        if hasattr(self, 'pushButton_ringc'):
            self.pushButton_ringc.clicked.connect(self.on_ringc_clicked)

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

    def load_config(self) -> dict:
        """config.json 파일 로드"""
        if not CONFIG_FILE.exists():
            raise FileNotFoundError(f"Configuration file not found: {CONFIG_FILE}")
        
        try:
            with CONFIG_FILE.open() as f:
                return json.load(f)
        except Exception as e:
            raise RuntimeError(f"Failed to load config file: {e}")

    def save_config(self, config: dict):
        """config.json 파일 저장"""
        try:
            with CONFIG_FILE.open("w") as f:
                json.dump(config, f, indent=2)
            logging.info("Configuration saved to config.json")
        except Exception as e:
            logging.error(f"Error saving config: {e}")

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

    def start_arduino_worker(self):
        """Arduino 워커 스레드 시작"""
        if self.arduino_worker is None:
            # config에서 Arduino 포트 가져오기
            ports = self.config.get("ports", {})
            port = ports.get("arduino")
            
            if not port:
                logging.error("Arduino port not configured in config.json")
                return False
            
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

    def start_dynamixel_worker(self):
        """Dynamixel 워커 스레드 시작"""
        if self.dynamixel_worker is None and self.dynamixel_driver is not None:
            # config에서 모터 ID 가져오기
            motor_id = self.config.get("dynamixel", {}).get("motor_id", 1)
            
            self.dynamixel_worker = DynamixelWorker(self.dynamixel_driver, motor_id, update_rate=20.0)
            self.dynamixel_worker.start()
            logging.info(f"Dynamixel worker started for motor {motor_id}")
            return True
        else:
            if self.dynamixel_worker is not None:
                logging.warning("Dynamixel worker already running")
            else:
                logging.error("Dynamixel driver not available")
            return False
    
    def stop_dynamixel_worker(self):
        """Dynamixel 워커 스레드 중지"""
        if self.dynamixel_worker is not None:
            self.dynamixel_worker.stop()
            self.dynamixel_worker.join()
            self.dynamixel_worker = None
            logging.info("Dynamixel worker stopped")

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

    def setup_dynamixel_ui(self):
        """Dynamixel UI 요소 설정"""
        # Ring position 버튼들 연결
        if hasattr(self, 'pushButton_ringp1'):
            self.pushButton_ringp1.clicked.connect(self.on_ringp1_clicked)
            self.pushButton_ringp1.setEnabled(False)  # 초기 비활성화
        
        if hasattr(self, 'pushButton_ringp2'):
            self.pushButton_ringp2.clicked.connect(self.on_ringp2_clicked)
            self.pushButton_ringp2.setEnabled(False)  # 초기 비활성화
        
        if hasattr(self, 'pushButton_ringpos_save'):
            self.pushButton_ringpos_save.clicked.connect(self.on_ringpos_save_clicked)

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

    def connect_dynamixel(self):
        """Dynamixel 연결"""
        try:
            # config에서 포트와 통신속도 가져오기
            ports = self.config.get("ports", {})
            baudrates = self.config.get("baudrates", {})
            
            port = ports.get("dynamixel")
            baudrate = baudrates.get("dynamixel")
            
            if not port or not baudrate:
                raise ValueError("Dynamixel port or baudrate not configured in config.json")
            
            self.dynamixel_driver = DynamixelDriver(device_name=port, baudrate=baudrate)
            self.dynamixel_driver.connect()
            
            # 모터 스캔
            found_motors = self.dynamixel_driver.scan_motors()
            if found_motors:
                # 토크 활성화
                for motor in found_motors:
                    self.dynamixel_driver.enable_torque(motor['id'])
                
                self.dynamixel_connected = True
                logging.info(f"Dynamixel connected on {port} at {baudrate} baud: {len(found_motors)} motors found")
                
                # Dynamixel 워커 시작
                self.start_dynamixel_worker()
                
                # Ring position 버튼들 활성화
                if hasattr(self, 'pushButton_ringp1'):
                    self.pushButton_ringp1.setEnabled(True)
                if hasattr(self, 'pushButton_ringp2'):
                    self.pushButton_ringp2.setEnabled(True)
                
                return True
            else:
                logging.error("No Dynamixel motors found")
                return False
                
        except Exception as e:
            logging.error(f"Dynamixel connection failed: {e}")
            self.dynamixel_driver = None
            return False

    def disconnect_dynamixel(self):
        """Dynamixel 연결 해제"""
        # Dynamixel 워커 중지
        self.stop_dynamixel_worker()
        
        if self.dynamixel_driver is not None:
            try:
                # 모든 모터의 토크 비활성화
                for motor_id in self.dynamixel_driver.connected_motors.keys():
                    self.dynamixel_driver.disable_torque(motor_id)
                
                self.dynamixel_driver.disconnect()
                self.dynamixel_driver = None
                self.dynamixel_connected = False
                
                # Ring position 버튼들 비활성화
                if hasattr(self, 'pushButton_ringp1'):
                    self.pushButton_ringp1.setEnabled(False)
                if hasattr(self, 'pushButton_ringp2'):
                    self.pushButton_ringp2.setEnabled(False)
                
                logging.info("Dynamixel disconnected")
            except Exception as e:
                logging.error(f"Error disconnecting Dynamixel: {e}")

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
                sw1_state = bool(switch_states & 0x80)  # 7
                self.label_sw1.setText("ON" if sw1_state else "OFF")

                if hasattr(self, 'prev_sw1_state'):
                    if sw1_state and self.prev_sw1_state is not sw1_state:
                        self.on_ringp1_clicked()

                self.prev_sw1_state = sw1_state

                # self.label_sw1.setStyleSheet(
                #     "background-color: blue; color: white; padding: 2px;" if sw1_state
                #     else "background-color: gray; color: white; padding: 2px;"
                # )
            
            if hasattr(self, 'label_sw2'):
                sw2_state = bool(switch_states & 0xF0)  # 7
                self.label_sw2.setText("ON" if sw2_state else "OFF")
                # self.label_sw2.setStyleSheet(
                #     "background-color: blue; color: white; padding: 2px;" if sw2_state
                #     else "background-color: gray; color: white; padding: 2px;"
                # )

    def update_dynamixel_status_display(self, status):
        """Dynamixel 상태를 GUI에 업데이트"""
        # Ring 연결 상태 표시
        if hasattr(self, 'label_ring_status'):
            if status.get('connected', False):
                self.label_ring_status.setText("ON")
            else:
                self.label_ring_status.setText("NO")
        
        # 현재 각도 표시
        if hasattr(self, 'label_ring_angle'):
            angle = status.get('angle', 0.0)
            self.label_ring_angle.setText(f"{angle:+7.1f}°")
        
        # 이동 상태 표시
        if hasattr(self, 'label_ring_moving'):
            moving = status.get('moving', False)
            self.label_ring_moving.setText("●" if moving else "")
        
        # 온도 표시
        if hasattr(self, 'label_ring_temp'):
            temperature = status.get('temperature', 0)
            self.label_ring_temp.setText(f"{temperature}°C")
        
        # 전압 표시
        if hasattr(self, 'label_ring_voltage'):
            voltage = status.get('voltage', 0.0)
            self.label_ring_voltage.setText(f"{voltage:.1f}V")
        
        # Revolution 표시 (position / 4096)
        if hasattr(self, 'label_ring_position'):
            position = status.get('position', 0)
            revolution = position / 4096.0
            self.label_ring_position.setText(f"{revolution:+7.3f}")

    
    # ─────────────────────────────────────────────────────────
    # 버튼 핸들러
    def on_connect_clicked(self):
        if not self.connected:
            try:
                # config에서 포트와 통신속도 가져오기
                ports = self.config.get("ports", {})
                baudrates = self.config.get("baudrates", {})
                
                motor_port = ports.get("motor_driver")
                motor_baudrate = baudrates.get("motor_driver")
                
                if not motor_port or not motor_baudrate:
                    raise ValueError("Motor driver port or baudrate not configured in config.json")
                
                self.drv = Driver(port=motor_port, baudrate=motor_baudrate)
                self.drv.connect()
                self.drv.zoffset = self.saved_offset  # 저장된 오프셋 적용
                self.connected = True
                self.t0 = time.perf_counter()
                self.label_connect.setText("ON")
                logging.info(f"Driver connected on {motor_port} at {motor_baudrate} baud")
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
            # config에 zoffset 업데이트
            self.config["zoffset"] = int(self.drv.zoffset)
            self.save_config(self.config)
            
            # 기존 offset.json도 호환성을 위해 유지 (선택사항)
            with OFFSET_FILE.open("w") as f:
                json.dump({"zoffset": int(self.drv.zoffset)}, f)
                
            logging.info(f"Z Offset saved to config.json: {self.drv.zoffset} cnt")
        except Exception as e:
            logging.error(f"Error saving offset: {e}")

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
    # Dynamixel 관련 핸들러
    def on_ringp1_clicked(self):
        """Ring Position 1 버튼 핸들러 - DynamixelWorker를 통한 이동"""
        if not self.dynamixel_connected or self.dynamixel_worker is None:
            # Dynamixel 연결 시도
            if not self.connect_dynamixel():
                QtWidgets.QMessageBox.critical(self, "Dynamixel Error", "Failed to connect to Dynamixel motors")
                return
        
        try:
            # lineEdit_ringpos1에서 목표 각도 값 가져오기
            if hasattr(self, 'lineEdit_ringpos1'):
                angle_text = self.lineEdit_ringpos1.text()
                target_angle = float(angle_text) % 360  # 0-360 범위로 정규화
                
                # config에서 속도 가져오기
                velocity = self.config.get("dynamixel", {}).get("velocity", 100)
                
                # Extended Position Control 모드 설정
                self.dynamixel_worker.set_operating_mode(EXTENDED_POSITION_CONTROL_MODE)
                
                # 워커를 통해 이동 명령 전송
                self.dynamixel_worker.move_to_angle(target_angle, velocity)
                
                logging.info(f"Ring Position 1 command sent: {target_angle}° at velocity {velocity}")
                
            else:
                logging.error("lineEdit_ringpos1 not found")
                
        except ValueError:
            logging.error("Invalid angle value for Ring Position 1")
            QtWidgets.QMessageBox.warning(self, "Input Error", "Please enter a valid angle (0-360)")
        except Exception as e:
            logging.error(f"Error moving Ring Position 1: {e}")

    def on_ringp2_clicked(self):
        """Ring Position 2 버튼 핸들러 - DynamixelWorker를 통한 이동"""
        if not self.dynamixel_connected or self.dynamixel_worker is None:
            # Dynamixel 연결 시도
            if not self.connect_dynamixel():
                QtWidgets.QMessageBox.critical(self, "Dynamixel Error", "Failed to connect to Dynamixel motors")
                return
        
        try:
            # lineEidit_ringpos2에서 목표 각도 값 가져오기
            if hasattr(self, 'lineEidit_ringpos2'):
                angle_text = self.lineEidit_ringpos2.text()
                target_angle = float(angle_text) % 360  # 0-360 범위로 정규화
                
                # config에서 속도 가져오기
                velocity = self.config.get("dynamixel", {}).get("velocity", 100)
                
                # Extended Position Control 모드 설정
                self.dynamixel_worker.set_operating_mode(EXTENDED_POSITION_CONTROL_MODE)
                
                # 워커를 통해 이동 명령 전송
                self.dynamixel_worker.move_to_angle(target_angle, velocity)
                
                logging.info(f"Ring Position 2 command sent: {target_angle}° at velocity {velocity}")
                
            else:
                logging.error("lineEidit_ringpos2 not found")
                
        except ValueError:
            logging.error("Invalid angle value for Ring Position 2")
            QtWidgets.QMessageBox.warning(self, "Input Error", "Please enter a valid angle (0-360)")
        except Exception as e:
            logging.error(f"Error moving Ring Position 2: {e}")

    def on_ringpos_save_clicked(self):
        """Ring Position Save 버튼 핸들러"""
        try:
            # 현재 설정값들을 config에 업데이트
            ring1_angle = 0.0
            ring2_angle = 0.0
            
            if hasattr(self, 'lineEdit_ringpos1'):
                ring1_angle = float(self.lineEdit_ringpos1.text())
            
            if hasattr(self, 'lineEidit_ringpos2'):
                ring2_angle = float(self.lineEidit_ringpos2.text())
            
            # Z offset도 함께 업데이트
            current_zoffset = self.saved_offset
            if self.drv is not None:
                current_zoffset = int(self.drv.zoffset)
            
            # config 업데이트
            self.config["zoffset"] = current_zoffset
            self.config["ring_positions"]["ring1"] = ring1_angle
            self.config["ring_positions"]["ring2"] = ring2_angle
            
            # config.json에 저장
            self.save_config(self.config)
            
            logging.info(f"Configuration saved - Z Offset: {current_zoffset}, Ring1: {ring1_angle}°, Ring2: {ring2_angle}°")
            
        except ValueError:
            logging.error("Invalid values for ring positions")
            QtWidgets.QMessageBox.warning(self, "Input Error", "Please enter valid angle values")
        except Exception as e:
            logging.error(f"Error saving ring positions: {e}")

    def on_ringc_clicked(self):
        """Ring Connect 버튼 핸들러 - Dynamixel 연결/해제"""
        if not self.dynamixel_connected:
            # Dynamixel 연결 시도
            if self.connect_dynamixel():
                self.pushButton_ringc.setText("RING DISCON")
                logging.info("Dynamixel connected successfully via Ring Connect button")
                
                
            else:
                QtWidgets.QMessageBox.critical(self, "Dynamixel Error", "Failed to connect to Dynamixel motors")
        else:
            # Dynamixel 연결 해제
            self.disconnect_dynamixel()
            self.pushButton_ringc.setText("RING CON")
            logging.info("Dynamixel disconnected via Ring Connect button")


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

        # 3) Dynamixel 상태 업데이트
        if self.dynamixel_worker is not None:
            try:
                dynamixel_status = self.dynamixel_worker.get_status()
                self.update_dynamixel_status_display(dynamixel_status)
                
                # 상세 로그는 debug 레벨로만 출력
                logging.debug(f"Dynamixel Status: Connected={dynamixel_status['connected']}, "
                            f"Angle={dynamixel_status['angle']:.1f}°, "
                            f"Moving={dynamixel_status['moving']}, "
                            f"Temperature={dynamixel_status['temperature']}°C")
            except Exception as e:
                logging.error(f"Dynamixel status update error: {e}")

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
        
        # Dynamixel 정리
        self.disconnect_dynamixel()
        
        event.accept()
        logging.info("Application closed")


# ── 진입점 ─────────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
