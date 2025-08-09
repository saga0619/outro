"""
Dynamixel 모터 제어를 위한 기본 드라이버 클래스
"""

import os
import time
import logging
from dynamixel_sdk import *

# Control table address for different Dynamixel models
# XM430-W350 기준 (다른 모델은 매뉴얼 참조)
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_GOAL_VELOCITY      = 104
ADDR_PRESENT_VELOCITY   = 128
ADDR_PROFILE_VELOCITY   = 112
ADDR_PROFILE_ACCELERATION = 108
ADDR_MOVING             = 122
ADDR_PRESENT_TEMPERATURE = 146
ADDR_PRESENT_VOLTAGE    = 144
ADDR_PRESENT_CURRENT    = 126
ADDR_OPERATING_MODE     = 11

# Operating modes
POSITION_CONTROL_MODE   = 3
VELOCITY_CONTROL_MODE   = 1
EXTENDED_POSITION_CONTROL_MODE = 4

# Protocol version
PROTOCOL_VERSION        = 2.0

# Default setting
BAUDRATE                = 57600
DEVICENAME              = 'COM3'  # Windows의 경우, Linux는 '/dev/ttyUSB0' 등

TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0
DXL_MINIMUM_POSITION_VALUE  = 0
DXL_MAXIMUM_POSITION_VALUE  = 4095
DXL_MOVING_STATUS_THRESHOLD = 20

class DynamixelDriver:
    def __init__(self, device_name=DEVICENAME, baudrate=BAUDRATE, protocol_version=PROTOCOL_VERSION):
        """
        Dynamixel 드라이버 초기화
        
        Args:
            device_name (str): 시리얼 포트 이름
            baudrate (int): 통신 속도
            protocol_version (float): 프로토콜 버전
        """
        self.device_name = device_name
        self.baudrate = baudrate
        self.protocol_version = protocol_version
        self.target_angle_prev = 0
        
        # Initialize PortHandler instance
        self.portHandler = PortHandler(self.device_name)
        
        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(self.protocol_version)
        
        self.connected_motors = {}  # {motor_id: motor_info}
        
    def connect(self):
        """포트 연결"""
        if self.portHandler.openPort():
            logging.info(f"포트 {self.device_name} 연결 성공")
        else:
            raise RuntimeError(f"포트 {self.device_name} 연결 실패")
            
        if self.portHandler.setBaudRate(self.baudrate):
            logging.info(f"통신속도 {self.baudrate} 설정 성공")
        else:
            raise RuntimeError(f"통신속도 {self.baudrate} 설정 실패")
    
    def disconnect(self):
        """포트 연결 해제"""
        self.portHandler.closePort()
        logging.info("포트 연결 해제")
    
    def scan_motors(self, id_range=(1, 10)):
        """연결된 모터 스캔"""
        found_motors = []
        for motor_id in range(id_range[0], id_range[1] + 1):
            # Ping the Dynamixel
            dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, motor_id)
            if dxl_comm_result == COMM_SUCCESS:
                found_motors.append({
                    'id': motor_id,
                    'model_number': dxl_model_number
                })
                logging.info(f"모터 ID {motor_id} 발견 (모델: {dxl_model_number})")
        
        self.connected_motors = {motor['id']: motor for motor in found_motors}
        return found_motors
    
    def enable_torque(self, motor_id):
        """토크 활성화"""
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"토크 활성화 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"토크 활성화 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
        
        logging.info(f"모터 ID {motor_id} 토크 활성화")
    
    def disable_torque(self, motor_id):
        """토크 비활성화"""
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"토크 비활성화 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"토크 비활성화 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
        
        logging.info(f"모터 ID {motor_id} 토크 비활성화")
    
    def set_goal_position(self, motor_id, position):
        """목표 위치 설정 (0-4095)"""
        # 위치 범위 체크
        position = max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, position))
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor_id, ADDR_GOAL_POSITION, position)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"위치 설정 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"위치 설정 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
    
    def get_present_position(self, motor_id):
        """현재 위치 읽기"""
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, motor_id, ADDR_PRESENT_POSITION)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"위치 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"위치 읽기 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
        
        return dxl_present_position
    
    def set_profile_velocity(self, motor_id, velocity):
        """프로파일 속도 설정"""
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor_id, ADDR_PROFILE_VELOCITY, velocity)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"속도 설정 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"속도 설정 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
    
    def set_profile_acceleration(self, motor_id, acceleration):
        """프로파일 가속도 설정"""
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor_id, ADDR_PROFILE_ACCELERATION, acceleration)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"가속도 설정 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"가속도 설정 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
    
    def is_moving(self, motor_id):
        """모터 이동 상태 확인"""
        dxl_moving, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
            self.portHandler, motor_id, ADDR_MOVING)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"이동상태 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"이동상태 읽기 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
        
        return bool(dxl_moving)
    
    def get_status(self, motor_id):
        """모터 상태 정보 읽기"""
        status = {}
        
        try:
            # 현재 위치
            status['position'] = self.get_present_position(motor_id)
            
            # 현재 속도
            dxl_present_velocity, _, _ = self.packetHandler.read4ByteTxRx(
                self.portHandler, motor_id, ADDR_PRESENT_VELOCITY)
            status['velocity'] = dxl_present_velocity
            
            # 현재 전류
            dxl_present_current, _, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, motor_id, ADDR_PRESENT_CURRENT)
            status['current'] = dxl_present_current
            
            # 온도
            dxl_present_temperature, _, _ = self.packetHandler.read1ByteTxRx(
                self.portHandler, motor_id, ADDR_PRESENT_TEMPERATURE)
            status['temperature'] = dxl_present_temperature
            
            # 전압
            dxl_present_voltage, _, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, motor_id, ADDR_PRESENT_VOLTAGE)
            status['voltage'] = dxl_present_voltage / 10.0  # 0.1V 단위
            
            # 이동 상태
            status['moving'] = self.is_moving(motor_id)
            
        except Exception as e:
            logging.error(f"상태 읽기 실패: {e}")
            
        return status
    
    def move_to_position(self, motor_id, position, velocity=None, acceleration=None, wait=False):
        """위치로 이동 (고급 함수)"""
        if velocity is not None:
            self.set_profile_velocity(motor_id, velocity)
        
        if acceleration is not None:
            self.set_profile_acceleration(motor_id, acceleration)
        
        self.set_goal_position(motor_id, position)
        
        if wait:
            while self.is_moving(motor_id):
                time.sleep(0.01)
    
    def position_to_angle(self, position, resolution=4096):
        """위치 값을 각도로 변환 (0-4095 -> 0-360도)"""
        return (position / resolution) * 360.0
    
    def angle_to_position(self, angle, resolution=4096):
        """각도를 위치 값으로 변환 (0-360도 -> 0-4095)"""
        return int((angle / 360.0) * resolution)
    
    def set_operating_mode(self, motor_id, mode):
        """동작 모드 설정 (토크 비활성화 상태에서만 가능)"""
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, motor_id, ADDR_OPERATING_MODE, mode)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"동작 모드 설정 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"동작 모드 설정 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
        
        logging.info(f"모터 ID {motor_id} 동작 모드 설정: {mode}")
    
    def set_goal_velocity(self, motor_id, velocity):
        """목표 속도 설정 (velocity control 모드에서 사용)"""
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor_id, ADDR_GOAL_VELOCITY, velocity)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"목표 속도 설정 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"목표 속도 설정 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
    
    def get_present_velocity(self, motor_id):
        """현재 속도 읽기"""
        dxl_present_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, motor_id, ADDR_PRESENT_VELOCITY)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"속도 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"속도 읽기 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
        
        return dxl_present_velocity
    
    def stop_motor(self, motor_id):
        """모터 정지 (velocity control 모드에서 속도를 0으로 설정)"""
        self.set_goal_velocity(motor_id, 0)
    
    def set_extended_goal_position(self, motor_id, position):
        """Extended Position Control 모드에서 목표 위치 설정 (다중 회전 가능)"""
        # Extended Position Control 모드에서는 위치 범위 제한이 없음
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, motor_id, ADDR_GOAL_POSITION, position)
        
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(f"Extended 위치 설정 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise RuntimeError(f"Extended 위치 설정 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
    
    def move_to_angle_counterclockwise(self, motor_id, target_angle, velocity=100):
        """
        Extended Position Control 모드로 반시계방향으로만 특정 각도까지 이동
        
        Args:
            motor_id: 모터 ID
            target_angle: 목표 각도 (0-360도)
            velocity: 이동 속도
        """
        # 현재 위치 읽기 (Extended 모드에서는 다중 회전 위치)
        current_position = self.get_present_position(motor_id)
        
        # 현재 각도 계산 (다중 회전 고려)
        current_angle_in_rotation = (current_position % 4096) / 4096 * 360
        target_angle = target_angle % 360

        # 반시계방향으로만 이동하도록 목표 위치 계산
        if target_angle >= current_angle_in_rotation:
            # 목표 각도가 현재 각도보다 크면 직접 이동
            angle_diff = target_angle - current_angle_in_rotation
        else:
            # 목표 각도가 현재 각도보다 작으면 한 바퀴 돌아서 이동
            angle_diff = (360 - current_angle_in_rotation) + target_angle

        if target_angle == self.target_angle_prev:
            angle_diff = 360
        
        # 새로운 목표 위치 계산 (현재 위치 + 각도 차이)
        position_diff = int((angle_diff / 360) * 4096)
        new_target_position = current_position + position_diff
        
        logging.info(f"Extended Position Control: Motor {motor_id} from {current_angle_in_rotation:.1f}° to {target_angle:.1f}° "
                    f"(position: {current_position} -> {new_target_position}, diff: +{angle_diff:.1f}°)")
        
        # 프로파일 속도 설정
        self.set_profile_velocity(motor_id, velocity)
        
        # 목표 위치로 이동
        self.set_extended_goal_position(motor_id, new_target_position)
        
        self.target_angle_prev = target_angle
        
        return new_target_position, current_position
