#!/bin/bash
set -e
cd /home/pi/lora_project

# LoRa: 자동 탐색 모드 (코드가 /dev/serial/by-id/*, /dev/ttyUSB*, /dev/ttyACM* 순으로 찾음)
export LORA_PORT=/dev/ttyUSB0
export LORA_BAUD=9600

# USB만 사용 → GPIO 제어 비활성
export LORA_USE_GPIO=0

# Firebase Admin
export SERVICE_ACCOUNT_JSON=/home/pi/lora_project/serviceAccount.json
export DB_URL="https://gps-mapping-397bc-default-rtdb.asia-southeast1.firebasedatabase.app"

# 게이트웨이 식별
export GATEWAY_ID=gw-rpi5
export GATEWAY_NICK="RPi5 GW"

# 게이트웨이 GPS
# 1) 아직 GPS 안 꽂았거나 테스트만 한다면 완전 비활성:
export GPS_USE_BB=0
export GPS_PORT="/dev/serial0"
export GPS_BAUD=9600
export GPS_UPLOAD_INTERVAL=1.0

# 2) USB-UART 동글로 GPS를 쓰려면 위의 빈 값 대신 예: export GPS_PORT=/dev/ttyACM0
# 3) GPIO 소프트UART라면:
# export GPS_USE_BB=1
# export GPS_SOFT_GPIO=18
# export GPS_BAUD=9600

source .venv/bin/activate
python3 lora_gateway.py
