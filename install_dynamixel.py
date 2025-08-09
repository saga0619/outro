"""
Dynamixel SDK 자동 설치 스크립트
"""

import subprocess
import sys
import os

def install_package(package_name):
    """패키지 설치"""
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])
        print(f"✓ {package_name} 설치 완료")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ {package_name} 설치 실패: {e}")
        return False

def check_package(package_name):
    """패키지 설치 여부 확인"""
    try:
        __import__(package_name.replace('-', '_'))
        print(f"✓ {package_name} 이미 설치됨")
        return True
    except ImportError:
        print(f"- {package_name} 설치 필요")
        return False

def main():
    print("=== Dynamixel SDK 설치 스크립트 ===")
    print()
    
    # 필요한 패키지 목록
    packages = [
        "dynamixel-sdk",
        "pyserial"
    ]
    
    # 설치 상태 확인
    print("1. 현재 설치 상태 확인:")
    need_install = []
    
    for package in packages:
        if not check_package(package):
            need_install.append(package)
    
    if not need_install:
        print("\n모든 패키지가 이미 설치되어 있습니다!")
        return
    
    # 설치 진행
    print(f"\n2. 필요한 패키지 설치 ({len(need_install)}개):")
    
    success_count = 0
    for package in need_install:
        print(f"\n{package} 설치 중...")
        if install_package(package):
            success_count += 1
    
    # 결과 출력
    print(f"\n=== 설치 완료 ===")
    print(f"성공: {success_count}/{len(need_install)}")
    
    if success_count == len(need_install):
        print("✓ 모든 패키지 설치 완료!")
        print("\n다음 단계:")
        print("1. Dynamixel 모터를 컴퓨터에 연결")
        print("2. 포트명 확인 (Windows: COM포트, Linux: /dev/ttyUSB* 등)")
        print("3. dynamixel_example.py 실행하여 테스트")
    else:
        print("✗ 일부 패키지 설치 실패")
        print("수동으로 설치를 시도해보세요:")
        for package in need_install:
            print(f"  pip install {package}")

if __name__ == "__main__":
    main()
