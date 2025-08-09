"""
Dynamixel Worker Thread
지속적으로 Dynamixel 모터의 상태와 위치를 모니터링하는 워커 스레드
"""

import threading
import time
import logging
from typing import Dict, Any, Optional
from drivers.dynamixel.dynamixel_driver import DynamixelDriver


class DynamixelWorker(threading.Thread):
    def __init__(self, driver: DynamixelDriver, motor_id: int = 1, update_rate: float = 20.0):
        """
        Dynamixel 워커 스레드 초기화
        
        Args:
            driver: DynamixelDriver 인스턴스
            motor_id: 모니터링할 모터 ID
            update_rate: 업데이트 주기 (Hz)
        """
        super().__init__(daemon=True)
        self.driver = driver
        self.motor_id = motor_id
        self.update_interval = 1.0 / update_rate  # 초 단위
        
        # 스레드 제어
        self._stop_event = threading.Event()
        self._running = False
        
        # 상태 데이터 (스레드 안전)
        self.lock = threading.Lock()
        self.status = {
            'connected': False,
            'position': 0,
            'angle': 0.0,
            'velocity': 0,
            'current': 0,
            'temperature': 0,
            'voltage': 0.0,
            'moving': False,
            'error_count': 0,
            'last_update': 0.0
        }
        
        # 명령 큐
        self.command_queue = []
        self.command_lock = threading.Lock()
        
        logging.info(f"DynamixelWorker initialized for motor {motor_id} at {update_rate} Hz")
    
    def run(self):
        """워커 스레드 메인 루프"""
        self._running = True
        logging.info(f"DynamixelWorker started for motor {self.motor_id}")
        
        while not self._stop_event.is_set():
            try:
                # 명령 처리
                self._process_commands()
                
                # 상태 업데이트
                self._update_status()
                
                # 대기
                time.sleep(self.update_interval)
                
            except Exception as e:
                logging.error(f"DynamixelWorker error: {e}")
                with self.lock:
                    self.status['error_count'] += 1
                time.sleep(0.1)  # 에러 시 짧은 대기
        
        self._running = False
        logging.info(f"DynamixelWorker stopped for motor {self.motor_id}")
    
    def stop(self):
        """워커 스레드 중지"""
        logging.info(f"Stopping DynamixelWorker for motor {self.motor_id}")
        self._stop_event.set()
    
    def is_running(self) -> bool:
        """워커 스레드 실행 상태 확인"""
        return self._running
    
    def _update_status(self):
        """모터 상태 업데이트"""
        try:
            if self.driver is None:
                with self.lock:
                    self.status['connected'] = False
                return
            
            # 현재 위치 읽기
            position = self.driver.get_present_position(self.motor_id)
            angle = self.driver.position_to_angle(position)
            
            # 현재 속도 읽기
            velocity = self.driver.get_present_velocity(self.motor_id)
            
            # 이동 상태 확인
            moving = self.driver.is_moving(self.motor_id)
            
            # 추가 상태 정보 (에러 처리 포함)
            current = 0
            temperature = 0
            voltage = 0.0
            
            try:
                status_info = self.driver.get_status(self.motor_id)
                current = status_info.get('current', 0)
                temperature = status_info.get('temperature', 0)
                voltage = status_info.get('voltage', 0.0)
            except Exception as e:
                logging.debug(f"Error reading extended status: {e}")
            
            # 스레드 안전하게 상태 업데이트
            with self.lock:
                self.status.update({
                    'connected': True,
                    'position': position,
                    'angle': angle % 360,  # 0-360도 범위로 정규화
                    'velocity': velocity,
                    'current': current,
                    'temperature': temperature,
                    'voltage': voltage,
                    'moving': moving,
                    'last_update': time.time()
                })
                
        except Exception as e:
            logging.debug(f"Status update error for motor {self.motor_id}: {e}")
            with self.lock:
                self.status['connected'] = False
                self.status['error_count'] += 1
    
    def _process_commands(self):
        """명령 큐 처리"""
        with self.command_lock:
            while self.command_queue:
                command = self.command_queue.pop(0)
                try:
                    self._execute_command(command)
                except Exception as e:
                    logging.error(f"Command execution error: {e}")
    
    def _execute_command(self, command: Dict[str, Any]):
        """개별 명령 실행"""
        cmd_type = command.get('type')
        
        if cmd_type == 'move_to_angle':
            target_angle = command.get('angle', 0)
            velocity = command.get('velocity', 100)
            logging.info(f"Executing move to angle: {target_angle}° at velocity {velocity}")
            self.driver.move_to_angle_counterclockwise(self.motor_id, target_angle, velocity)
            
        elif cmd_type == 'set_velocity':
            velocity = command.get('velocity', 0)
            logging.info(f"Setting velocity: {velocity}")
            self.driver.set_goal_velocity(self.motor_id, velocity)
            
        elif cmd_type == 'stop':
            logging.info("Stopping motor")
            self.driver.stop_motor(self.motor_id)
            
        elif cmd_type == 'enable_torque':
            logging.info("Enabling torque")
            self.driver.enable_torque(self.motor_id)
            
        elif cmd_type == 'disable_torque':
            logging.info("Disabling torque")
            self.driver.disable_torque(self.motor_id)
            
        elif cmd_type == 'set_operating_mode':
            mode = command.get('mode', 3)
            logging.info(f"Setting operating mode: {mode}")
            self.driver.disable_torque(self.motor_id)
            self.driver.set_operating_mode(self.motor_id, mode)
            self.driver.enable_torque(self.motor_id)
            
        else:
            logging.warning(f"Unknown command type: {cmd_type}")
    
    def get_status(self) -> Dict[str, Any]:
        """현재 상태 반환 (스레드 안전)"""
        with self.lock:
            return self.status.copy()
    
    def move_to_angle(self, angle: float, velocity: int = 100):
        """각도로 이동 명령 추가"""
        command = {
            'type': 'move_to_angle',
            'angle': angle % 360,
            'velocity': velocity
        }
        with self.command_lock:
            self.command_queue.append(command)
        logging.info(f"Move command queued: {angle}° at velocity {velocity}")
    
    def set_velocity(self, velocity: int):
        """속도 설정 명령 추가"""
        command = {
            'type': 'set_velocity',
            'velocity': velocity
        }
        with self.command_lock:
            self.command_queue.append(command)
    
    def stop_motor(self):
        """모터 정지 명령 추가"""
        command = {'type': 'stop'}
        with self.command_lock:
            self.command_queue.append(command)
    
    def enable_torque(self):
        """토크 활성화 명령 추가"""
        command = {'type': 'enable_torque'}
        with self.command_lock:
            self.command_queue.append(command)
    
    def disable_torque(self):
        """토크 비활성화 명령 추가"""
        command = {'type': 'disable_torque'}
        with self.command_lock:
            self.command_queue.append(command)
    
    def set_operating_mode(self, mode: int):
        """동작 모드 설정 명령 추가"""
        command = {
            'type': 'set_operating_mode',
            'mode': mode
        }
        with self.command_lock:
            self.command_queue.append(command)
    
    def wait_for_completion(self, timeout: float = 10.0) -> bool:
        """이동 완료까지 대기"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self.get_status()
            if not status.get('moving', False):
                return True
            time.sleep(0.1)
        
        logging.warning(f"Movement timeout after {timeout} seconds")
        return False
    
    def get_current_angle(self) -> float:
        """현재 각도 반환"""
        status = self.get_status()
        return status.get('angle', 0.0)
    
    def is_connected(self) -> bool:
        """연결 상태 확인"""
        status = self.get_status()
        return status.get('connected', False)
    
    def is_moving(self) -> bool:
        """이동 상태 확인"""
        status = self.get_status()
        return status.get('moving', False)
