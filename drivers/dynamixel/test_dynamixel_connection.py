"""
Dynamixel 연결 테스트 스크립트
"""

import sys
import time
import logging

def test_import():
    """SDK 임포트 테스트"""
    print("1. Dynamixel SDK 임포트 테스트...")
    try:
        import dynamixel_sdk
        print("   ✓ Dynamixel SDK 임포트 성공")
        return True
    except ImportError as e:
        print(f"   ✗ Dynamixel SDK 임포트 실패: {e}")
        print("   해결방법: pip install dynamixel-sdk")
        return False

def test_serial():
    """시리얼 통신 라이브러리 테스트"""
    print("2. 시리얼 통신 라이브러리 테스트...")
    try:
        import serial
        print("   ✓ pyserial 임포트 성공")
        return True
    except ImportError as e:
        print(f"   ✗ pyserial 임포트 실패: {e}")
        print("   해결방법: pip install pyserial")
        return False

def test_driver():
    """드라이버 클래스 테스트"""
    print("3. Dynamixel 드라이버 클래스 테스트...")
    try:
        from dynamixel_driver import DynamixelDriver
        driver = DynamixelDriver()
        print("   ✓ DynamixelDriver 클래스 생성 성공")
        return True
    except Exception as e:
        print(f"   ✗ DynamixelDriver 클래스 생성 실패: {e}")
        return False

def test_connection(port='COM5', baudrate=57600):
    """실제 연결 테스트"""
    print(f"4. 실제 연결 테스트 (포트: {port}, 속도: {baudrate})...")
    
    try:
        from dynamixel_driver import DynamixelDriver
        driver = DynamixelDriver(device_name=port, baudrate=baudrate)
        
        # 연결 시도
        driver.connect()
        print("   ✓ 포트 연결 성공")
        
        # 모터 스캔
        print("   모터 스캔 중...")
        motors = driver.scan_motors(id_range=(1, 5))
        
        if motors:
            print(f"   ✓ {len(motors)}개 모터 발견:")
            for motor in motors:
                print(f"     - ID: {motor['id']}, 모델: {motor['model_number']}")
        else:
            print("   ⚠ 연결된 모터가 없습니다.")
            print("   확인사항:")
            print("     - 모터 전원이 켜져 있는지")
            print("     - 케이블 연결이 올바른지")
            print("     - 모터 ID가 1-5 범위에 있는지")
            print("     - 통신속도가 올바른지")
        
        driver.disconnect()
        return len(motors) > 0
        
    except Exception as e:
        print(f"   ✗ 연결 테스트 실패: {e}")
        print("   확인사항:")
        print(f"     - 포트명 '{port}'이 올바른지")
        print("     - 다른 프로그램에서 포트를 사용하고 있지 않은지")
        print("     - USB 케이블이 제대로 연결되어 있는지")
        return False

def get_available_ports():
    """사용 가능한 시리얼 포트 목록"""
    print("사용 가능한 시리얼 포트:")
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        
        if ports:
            for port in ports:
                print(f"  - {port.device}: {port.description}")
        else:
            print("  사용 가능한 포트가 없습니다.")
            
    except ImportError:
        print("  포트 목록을 가져올 수 없습니다. (pyserial 필요)")

def main():
    print("=== Dynamixel 연결 테스트 ===")
    print()
    
    # 기본 테스트들
    tests = [
        test_import,
        test_serial,
        test_driver
    ]
    
    passed = 0
    for test in tests:
        if test():
            passed += 1
        print()
    
    if passed < len(tests):
        print(f"기본 테스트 실패 ({passed}/{len(tests)})")
        print("먼저 필요한 패키지를 설치하세요:")
        print("  python install_dynamixel.py")
        return
    
    print("기본 테스트 모두 통과!")
    print()
    
    # 포트 목록 표시
    get_available_ports()
    print()
    
    # 연결 테스트
    port = input("테스트할 포트명을 입력하세요 (기본값: COM3): ").strip()
    if not port:
        port = 'COM3'
    
    baudrate_input = input("통신속도를 입력하세요 (기본값: 57600): ").strip()
    try:
        baudrate = int(baudrate_input) if baudrate_input else 57600
    except ValueError:
        baudrate = 57600
    
    print()
    success = test_connection(port, baudrate)
    
    print()
    print("=== 테스트 완료 ===")
    if success:
        print("✓ 모든 테스트 통과! Dynamixel 사용 준비 완료")
        print("이제 dynamixel_example.py를 실행해보세요.")
    else:
        print("⚠ 일부 테스트 실패")
        print("문제를 해결한 후 다시 테스트해보세요.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n테스트가 중단되었습니다.")
    except Exception as e:
        print(f"\n예상치 못한 에러: {e}")
