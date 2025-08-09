# Dynamixel SDK 설치 가이드

## 1. Python용 Dynamixel SDK 설치

### pip를 통한 설치 (권장)
```bash
pip install dynamixel-sdk
```

### 또는 conda를 통한 설치
```bash
conda install -c conda-forge dynamixel-sdk
```

## 2. 수동 설치 (GitHub에서)

```bash
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/python
python setup.py install
```

## 3. 필요한 추가 패키지
```bash
pip install pyserial
```

## 4. 지원되는 Dynamixel 모델
- AX 시리즈 (AX-12A, AX-18A 등)
- MX 시리즈 (MX-28, MX-64 등)
- XL 시리즈 (XL320, XL430 등)
- XM 시리즈 (XM430, XM540 등)
- XH 시리즈 (XH430, XH540 등)
- PRO 시리즈 (H42, H54 등)

## 5. 연결 방법
- USB2Dynamixel 어댑터 사용
- OpenCM9.04 보드 사용
- OpenCR 보드 사용
- U2D2 어댑터 사용 (권장)
