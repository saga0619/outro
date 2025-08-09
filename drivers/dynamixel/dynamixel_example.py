"""
Dynamixel 모터 제어 예제 코드
"""

import time
import logging
from dynamixel_driver import DynamixelDriver

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def basic_control_example():
    """기본 제어 예제"""
    print("=== Dynamixel 기본 제어 예제 ===")
    
    # 드라이버 초기화 (포트명과 통신속도는 환경에 맞게 수정)
    driver = DynamixelDriver(device_name='COM3', baudrate=57600)
    
    try:
        # 연결
        driver.connect()
        
        # 모터 스캔
        motors = driver.scan_motors(id_range=(1, 5))
        if not motors:
            print("연결된 모터가 없습니다. 연결을 확인하세요.")
            return
        
        motor_id = motors[0]['id']  # 첫 번째 모터 사용
        print(f"모터 ID {motor_id} 사용")
        
        # 토크 활성화
        driver.enable_torque(motor_id)
        
        # 속도와 가속도 설정
        driver.set_profile_velocity(motor_id, 100)  # 속도 설정
        driver.set_profile_acceleration(motor_id, 50)  # 가속도 설정
        
        # 위치 이동 예제
        positions = [1024, 2048, 3072, 2048]  # 90도, 180도, 270도, 180도
        
        for i, pos in enumerate(positions):
            print(f"위치 {i+1}: {pos} (약 {driver.position_to_angle(pos):.1f}도)")
            driver.set_goal_position(motor_id, pos)
            
            # 이동 완료까지 대기
            while driver.is_moving(motor_id):
                current_pos = driver.get_present_position(motor_id)
                print(f"  현재 위치: {current_pos} (약 {driver.position_to_angle(current_pos):.1f}도)")
                time.sleep(0.1)
            
            print(f"  위치 {pos} 도달 완료")
            time.sleep(1)
        
        # 토크 비활성화
        driver.disable_torque(motor_id)
        
    except Exception as e:
        print(f"에러 발생: {e}")
    finally:
        driver.disconnect()

def status_monitoring_example():
    """상태 모니터링 예제"""
    print("\n=== Dynamixel 상태 모니터링 예제 ===")
    
    driver = DynamixelDriver(device_name='COM3', baudrate=57600)
    
    try:
        driver.connect()
        motors = driver.scan_motors(id_range=(1, 5))
        
        if not motors:
            print("연결된 모터가 없습니다.")
            return
        
        motor_id = motors[0]['id']
        driver.enable_torque(motor_id)
        
        # 10초간 상태 모니터링
        print("10초간 모터 상태를 모니터링합니다...")
        start_time = time.time()
        
        while time.time() - start_time < 10:
            status = driver.get_status(motor_id)
            
            print(f"위치: {status.get('position', 'N/A')} "
                  f"속도: {status.get('velocity', 'N/A')} "
                  f"전류: {status.get('current', 'N/A')} "
                  f"온도: {status.get('temperature', 'N/A')}°C "
                  f"전압: {status.get('voltage', 'N/A')}V "
                  f"이동중: {status.get('moving', 'N/A')}")
            
            time.sleep(0.5)
        
        driver.disable_torque(motor_id)
        
    except Exception as e:
        print(f"에러 발생: {e}")
    finally:
        driver.disconnect()

def angle_control_example():
    """각도 기반 제어 예제"""
    print("\n=== 각도 기반 제어 예제 ===")
    
    driver = DynamixelDriver(device_name='COM3', baudrate=57600)
    
    try:
        driver.connect()
        motors = driver.scan_motors(id_range=(1, 5))
        
        if not motors:
            print("연결된 모터가 없습니다.")
            return
        
        motor_id = motors[0]['id']
        driver.enable_torque(motor_id)
        
        # 각도로 제어
        angles = [0, 90, 180, 270, 0]  # 도 단위
        
        for angle in angles:
            position = driver.angle_to_position(angle)
            print(f"목표 각도: {angle}도 (위치값: {position})")
            
            driver.move_to_position(motor_id, position, velocity=200, wait=True)
            
            # 실제 도달한 위치 확인
            actual_pos = driver.get_present_position(motor_id)
            actual_angle = driver.position_to_angle(actual_pos)
            print(f"실제 각도: {actual_angle:.1f}도")
            
            time.sleep(1)
        
        driver.disable_torque(motor_id)
        
    except Exception as e:
        print(f"에러 발생: {e}")
    finally:
        driver.disconnect()

def multi_motor_example():
    """다중 모터 제어 예제"""
    print("\n=== 다중 모터 제어 예제 ===")
    
    driver = DynamixelDriver(device_name='COM3', baudrate=57600)
    
    try:
        driver.connect()
        motors = driver.scan_motors(id_range=(1, 10))
        
        if len(motors) < 2:
            print("최소 2개의 모터가 필요합니다.")
            return
        
        # 모든 모터 토크 활성화
        for motor in motors:
            motor_id = motor['id']
            driver.enable_torque(motor_id)
            driver.set_profile_velocity(motor_id, 150)
        
        # 동시에 다른 위치로 이동
        target_positions = [1024, 2048, 3072]  # 각각 다른 위치
        
        for i, motor in enumerate(motors[:3]):  # 최대 3개 모터
            motor_id = motor['id']
            pos = target_positions[i % len(target_positions)]
            driver.set_goal_position(motor_id, pos)
            print(f"모터 ID {motor_id}: 위치 {pos}로 이동 시작")
        
        # 모든 모터가 이동 완료될 때까지 대기
        while True:
            all_stopped = True
            for motor in motors[:3]:
                if driver.is_moving(motor['id']):
                    all_stopped = False
                    break
            
            if all_stopped:
                break
            
            time.sleep(0.1)
        
        print("모든 모터 이동 완료")
        
        # 모든 모터 토크 비활성화
        for motor in motors:
            driver.disable_torque(motor['id'])
        
    except Exception as e:
        print(f"에러 발생: {e}")
    finally:
        driver.disconnect()

if __name__ == "__main__":
    print("Dynamixel 제어 예제를 실행합니다.")
    print("실행하기 전에 다음을 확인하세요:")
    print("1. Dynamixel SDK가 설치되어 있는지")
    print("2. 모터가 올바르게 연결되어 있는지")
    print("3. 포트명(COM3)과 통신속도(57600)가 올바른지")
    print("4. 모터 ID가 1-10 범위에 있는지")
    print()
    
    try:
        # 기본 제어 예제 실행
        basic_control_example()
        
        # 상태 모니터링 예제 실행
        # status_monitoring_example()
        
        # 각도 기반 제어 예제 실행
        # angle_control_example()
        
        # 다중 모터 제어 예제 실행
        # multi_motor_example()
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"예상치 못한 에러: {e}")
