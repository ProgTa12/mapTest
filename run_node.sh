#!/bin/bash
set -e
cd /home/pi/lora_project

# 장치별 식별
export LORA_NODE_ID=node1
export LORA_NICK=node1

# LoRa: HAT 점퍼 B → /dev/serial0
export LORA_PORT=/dev/serial0
export LORA_BAUD=9600

# GPS 기본: 소프트 UART(pigpio)로 수신 (NEO-6M TX → GPIO23)
export GPS_USE_BB=1
export GPS_SOFT_GPIO=23
export GPS_BAUD=9600

# 디버그(필요 시)
# export FAKE_LOC="37.5665,126.9780"
# export ALLOW_NOFIX=1

python3 lora_node.py
