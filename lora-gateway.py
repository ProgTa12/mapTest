#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raspberry Pi 5 게이트웨이:
- LoRa(HAT, USB 시리얼) JSON 수신 → Firebase RTDB /locations/<id>
- (옵션) 게이트웨이 자체 GPS → /gateways/<gw_id>
- 포트 자동 탐색 지원(LORA_PORT=auto), 열기 실패 시 재시도
- LORA_USE_GPIO=0 이면 GPIO 코드 완전 비활성화(RPi 5 호환)
"""

import os
import time
import json
import uuid
import glob
import serial
import threading

# =========================
# 환경 변수
# =========================
LORA_PORT = os.environ.get("LORA_PORT", "/dev/serial0")
LORA_BAUD = int(os.environ.get("LORA_BAUD", "9600"))

SERVICE_ACCOUNT_JSON = os.environ.get("SERVICE_ACCOUNT_JSON", "/home/pi/lora_project/serviceAccount.json")
DB_URL = os.environ.get("DB_URL", "https://gps-mapping-397bc-default-rtdb.asia-southeast1.firebasedatabase.app")

GW_ID = os.environ.get("GATEWAY_ID") or f"gw-{uuid.getnode():012x}"
GW_NICK = os.environ.get("GATEWAY_NICK", "LoRaGW")

# 게이트웨이 GPS 설정
GPS_USE_BB = os.environ.get("GPS_USE_BB", "0") == "1"   # 소프트 UART 사용 여부
GPS_SOFT_GPIO = int(os.environ.get("GPS_SOFT_GPIO", "23"))
GPS_PORT = os.environ.get("GPS_PORT", "")
GPS_BAUD = int(os.environ.get("GPS_BAUD", "9600"))
GPS_UPLOAD_INTERVAL = float(os.environ.get("GPS_UPLOAD_INTERVAL", "1.0"))

# LoRa 모드용 GPIO 사용 여부(USB만 쓰면 0)
LORA_USE_GPIO = os.environ.get("LORA_USE_GPIO", "0") == "1"

# GPIO는 필요할 때만 import
if LORA_USE_GPIO:
    import RPi.GPIO as GPIO
    M0_PIN, M1_PIN = 22, 27
else:
    GPIO = None


# =========================
# Firebase Admin
# =========================
import firebase_admin
from firebase_admin import credentials, db

def fb_init():
    if not firebase_admin._apps:
        cred = credentials.Certificate(SERVICE_ACCOUNT_JSON)
        firebase_admin.initialize_app(cred, {'databaseURL': DB_URL})


# =========================
# 직렬 포트 열기/탐색
# =========================
def try_open_serial(port, baud, timeout=1):
    """한 번만 시도해서 열기"""
    return serial.Serial(port, baudrate=baud, timeout=timeout)

def autodetect_port(candidates_order=None):
    """
    /dev/serial/by-id/* → /dev/ttyUSB* → /dev/ttyACM* 순으로 가장 먼저 보이는 포트 반환
    """
    if candidates_order is None:
        candidates_order = [
            "/dev/serial/by-id/*",
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
        ]
    for pattern in candidates_order:
        for path in sorted(glob.glob(pattern)):
            # 존재하고 접근 가능한지 가볍게 체크
            if os.path.exists(path):
                return path
    return None

def open_serial_with_retry(port, baud, label, retry_sec=2.0):
    """
    포트를 열 때까지 재시도. port="auto"면 자동 탐색.
    """
    while True:
        try:
            actual = port
            if port == "auto":
                cand = autodetect_port()
                if not cand:
                    raise FileNotFoundError("no serial candidates")
                actual = cand
            ser = try_open_serial(actual, baud, timeout=1)
            print(f"[Gateway] {label}: {actual}@{baud}")
            return ser
        except Exception as e:
            print(f"[Gateway] wait {label} ({port}) ... {e}")
            time.sleep(retry_sec)


# =========================
# LoRa 모드 GPIO
# =========================
def init_lora_mode_txrx():
    if not LORA_USE_GPIO:
        print("[Gateway] LORA_USE_GPIO=0 (skip M0/M1 GPIO)")
        return
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(M0_PIN, GPIO.OUT)
    GPIO.setup(M1_PIN, GPIO.OUT)
    GPIO.output(M0_PIN, GPIO.LOW)   # 통신 모드
    GPIO.output(M1_PIN, GPIO.LOW)
    time.sleep(0.05)


# =========================
# GPS 파싱 유틸
# =========================
def dm_to_deg(raw: str):
    if not raw: return None
    try:
        val = float(raw)
        deg = int(val/100)
        minutes = val - deg*100
        return deg + minutes/60.0
    except:
        return None

def parse_rmc(s: str):
    p = s.split(",")
    if len(p) < 12:
        return None
    fix = (p[2] == "A")
    la = dm_to_deg(p[3]); ns = p[4]
    lo = dm_to_deg(p[5]); ew = p[6]
    hdg = None
    try:
        hdg = float(p[8])
    except:
        pass
    if la and lo:
        if ns == "S": la = -la
        if ew == "W": lo = -lo
        return {"lat": la, "lon": lo, "heading": hdg or 0.0, "fix": fix}
    return None


# =========================
# Firebase 업로드
# =========================
def upload_node(doc):
    node_id = doc.get("id")
    if not node_id:
        print("[Gateway] drop: no id field in", doc)
        return False
    payload = {
        "latitude":  doc.get("lat"),
        "longitude": doc.get("lon"),
        "heading":   doc.get("heading", 0),
        "nickname":  doc.get("nickname", "LoRa"),
        "timestamp": int(doc.get("ts", time.time()*1000)),
    }
    try:
        db.reference(f"/locations/{node_id}").set(payload)
        print("[Gateway] upload ok:", node_id, payload)
        return True
    except Exception as e:
        print("[Gateway] upload fail:", e)
        return False

def upload_gateway(lat, lon, heading, fix):
    payload = {
        "latitude":  lat,
        "longitude": lon,
        "heading":   heading or 0.0,
        "nickname":  GW_NICK,
        "fix":       bool(fix),
        "timestamp": int(time.time()*1000),
    }
    try:
        db.reference(f"/gateways/{GW_ID}").set(payload)
        print("[Gateway] gw-pos ok:", GW_ID, payload)
        return True
    except Exception as e:
        print("[Gateway] gw-pos fail:", e)
        return False


# =========================
# 게이트웨이 GPS 리더
# =========================
class GPSReader:
    def __init__(self, port:str, baud:int, use_bb:bool, bb_gpio:int):
        self.use_bb = use_bb and (not port)
        self.buf = b""
        self.pi = None
        self.ser = None
        if self.use_bb:
            import pigpio
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon not running")
            self.gpio = bb_gpio
            self.pi.set_mode(self.gpio, pigpio.INPUT)
            self.pi.bb_serial_read_open(self.gpio, baud)
            print(f"[Gateway] GPS(bb)  : GPIO{self.gpio} @ {baud}")
        elif port:
            # 포트가 지정됐으면 그 포트를 열되, 실패 시 재시도
            while True:
                try:
                    self.ser = try_open_serial(port, baud, timeout=1)
                    print(f"[Gateway] GPS(uart): {port} @ {baud}")
                    break
                except Exception as e:
                    print(f"[Gateway] wait GPS({port}) ... {e}")
                    time.sleep(2.0)
        else:
            print("[Gateway] GPS: disabled")

    def read(self, max_bytes=256):
        if self.use_bb and self.pi:
            cnt, data = self.pi.bb_serial_read(self.gpio)
            return data if cnt > 0 else b""
        elif self.ser:
            return self.ser.read(max_bytes)
        return b""

    def close(self):
        if self.use_bb and self.pi:
            try:
                self.pi.bb_serial_read_close(self.gpio)
            finally:
                self.pi.stop()
        elif self.ser:
            try:
                self.ser.close()
            except:
                pass


def gps_worker():
    # GPS 비활성 조건
    if not (GPS_USE_BB or GPS_PORT):
        print("[Gateway] gps_worker: disabled")
        return

    reader = GPSReader(GPS_PORT, GPS_BAUD, GPS_USE_BB, GPS_SOFT_GPIO)
    buf = b""
    last_up = 0.0
    try:
        while True:
            chunk = reader.read(256)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors="ignore").strip()
                    if s.startswith("$GNRMC") or s.startswith("$GPRMC"):
                        r = parse_rmc(s)
                        if r:
                            now = time.time()
                            if now - last_up >= GPS_UPLOAD_INTERVAL:
                                upload_gateway(r["lat"], r["lon"], r.get("heading",0.0), r.get("fix",False))
                                last_up = now
            time.sleep(0.02)
    except Exception as e:
        print("[Gateway] gps_worker error:", e)
    finally:
        reader.close()


# =========================
# 메인 루프
# =========================
def main():
    print("[Gateway] start")
    print(f"[Gateway] LoRa: {LORA_PORT}@{LORA_BAUD}")
    print(f"[Gateway] DB  : {DB_URL}")

    fb_init()
    init_lora_mode_txrx()

    # 게이트웨이 GPS 업로더 스레드
    t = threading.Thread(target=gps_worker, daemon=True)
    t.start()

    # LoRa 수신 루프(자동 탐색 + 재시도)
    lora = open_serial_with_retry(LORA_PORT, LORA_BAUD, label="LoRa")
    buf = b""
    try:
        while True:
            chunk = lora.read(256)
            if not chunk:
                time.sleep(0.01)
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode(errors="ignore").strip()
                if not s:
                    continue
                try:
                    doc = json.loads(s)
                except json.JSONDecodeError:
                    print("[Gateway] non-JSON:", s[:160])
                    continue
                upload_node(doc)
    finally:
        try:
            lora.close()
        except:
            pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        if LORA_USE_GPIO and GPIO:
            try:
                GPIO.cleanup()
            except Exception:
                pass
