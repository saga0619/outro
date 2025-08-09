# Dynamixel 제어 가이드

이 프로젝트는 Dynamixel 서보모터를 Python으로 제어하기 위한 완전한 솔루션을 제공합니다.

## 📁 파일 구조

```
├── dynamixel_setup.md              # 설치 가이드
├── dynamixel_driver.py             # Dynamixel 드라이버 클래스
├── dynamixel_example.py            # 사용 예제 코드
├── install_dynamixel.py            # 자동 설치 스크립트
├── test_dynamixel_connection.py    # 연결 테스트 스크립트
└── README_Dynamixel.md            # 이 파일
```

## 🚀 빠른 시작

### 1단계: SDK 설치

자동 설치 스크립트 실행:
```bash
python install_dynamixel.py
```

또는 수동 설치:
```bash
pip install dynamixel-sdk
pip install pyserial
```

### 2단계: 하드웨어 연결

1. Dynamixel 모터를 U2D2, USB2Dynamixel 등의 어댑터를 통해 컴퓨터에 연결
2. 모터 전원 공급 (보통 12V)
3. 모터 ID 설정 (기본값: 1)

### 3단계: 연결 테스트

```bash
python test_dynamixel_connection.py
```

### 4단계: 예제 실행

```bash
python dynamixel_example.py
```

## 🔧 기본 사용법

### 드라이버 초기화

```python
from dynamixel_driver import DynamixelDriver

# 드라이버 생성
driver = DynamixelDriver(device_name='COM3', baudrate=57600)

# 연결
driver.connect()

# 모터 스캔
motors = driver.scan_motors(id_range=(1, 10))
```

### 기본 제어

```python
motor_id = 1

# 토크 활성화
driver.enable_torque(motor_id)

# 위치 제어 (0-4095)
driver.set_goal_position(motor_id, 2048)  # 중앙 위치

# 현재 위치 읽기
current_pos = driver.get_present_position(motor_id)

# 토크 비활성화
driver.disable_torque(motor_id)

# 연결 해제
driver.disconnect()
```

### 각도 기반 제어

```python
# 각도를 위치값으로 변환
position = driver.angle_to_position(90)  # 90도

# 위치로 이동 (속도, 가속도 설정 포함)
driver.move_to_position(motor_id, position, velocity=200, wait=True)

# 위치값을 각도로 변환
angle = driver.position_to_angle(2048)  # 약 180도
```

### 상태 모니터링

```python
status = driver.get_status(motor_id)
print(f"위치: {status['position']}")
print(f"속도: {status['velocity']}")
print(f"전류: {status['current']}")
print(f"온도: {status['temperature']}°C")
print(f"전압: {status['voltage']}V")
print(f"이동중: {status['moving']}")
```

## 📋 지원 모델

- **AX 시리즈**: AX-12A, AX-18A 등
- **MX 시리즈**: MX-28, MX-64 등  
- **XL 시리즈**: XL320, XL430 등
- **XM 시리즈**: XM430, XM540 등
- **XH 시리즈**: XH430, XH540 등
- **PRO 시리즈**: H42, H54 등

## ⚙️ 설정 가능한 매개변수

### 연결 설정
- `device_name`: 시리얼 포트 (Windows: 'COM3', Linux: '/dev/ttyUSB0')
- `baudrate`: 통신속도 (기본값: 57600)
- `protocol_version`: 프로토콜 버전 (기본값: 2.0)

### 제어 매개변수
- `position`: 목표 위치 (0-4095)
- `velocity`: 프로파일 속도
- `acceleration`: 프로파일 가속도

## 🔍 문제 해결

### 연결 문제
1. **포트 에러**: 올바른 COM 포트 확인
2. **통신 실패**: 통신속도(baudrate) 확인
3. **모터 미발견**: 모터 ID, 전원, 케이블 연결 확인

### 일반적인 해결책
```bash
# 사용 가능한 포트 확인
python -c "import serial.tools.list_ports; [print(p.device, p.description) for p in serial.tools.list_ports.comports()]"

# 패키지 재설치
pip uninstall dynamixel-sdk
pip install dynamixel-sdk
```

## 📚 예제 코드

### 기본 제어
```python
# dynamixel_example.py의 basic_control_example() 참조
```

### 상태 모니터링
```python
# dynamixel_example.py의 status_monitoring_example() 참조
```

### 다중 모터 제어
```python
# dynamixel_example.py의 multi_motor_example() 참조
```

## 🔗 유용한 링크

- [Dynamixel SDK GitHub](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [ROBOTIS e-Manual](https://emanual.robotis.com/)
- [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

## ⚠️ 주의사항

1. **전원 공급**: 모터에 적절한 전압 공급 필요
2. **토크 제어**: 사용 후 반드시 토크 비활성화
3. **온도 모니터링**: 과열 방지를 위한 온도 확인
4. **ID 충돌**: 같은 네트워크에서 중복 ID 사용 금지

## 📞 지원

문제가 발생하면 다음을 확인하세요:
1. `test_dynamixel_connection.py` 실행
2. 하드웨어 연결 상태 확인
3. 모터 매뉴얼 참조
4. ROBOTIS 공식 문서 확인

---

**작성일**: 2025년 1월  
**버전**: 1.0  
**호환성**: Python 3.6+, Dynamixel SDK 3.7+
