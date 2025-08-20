#!/usr/bin/env python3
import os, time, json, serial, uuid
import RPi.GPIO as GPIO

# === 선택: pigpio 소프트 UART로 GPS 읽기 ===
USE_BB = os.environ.get("GPS_USE_BB", "1") == "1"   # 기본 ON
BB_GPIO = int(os.environ.get("GPS_SOFT_GPIO", "23"))  # NEO-6M TX → GPIO23

# === 테스트/디버그 모드 ===
ALLOW_NOFIX = os.environ.get("ALLOW_NOFIX", "0") == "1"   # Fix 없어도 마지막 좌표 전송
FAKE_LOC    = os.environ.get("FAKE_LOC", "")              # "37.5665,126.9780" 형식이면 고정 좌표 송신

# === 장치 설정 ===
NODE_ID   = os.environ.get("LORA_NODE_ID") or f"node-{uuid.getnode():012x}"
NICKNAME  = os.environ.get("LORA_NICK", "LoRaNode")

GPS_PORT  = os.environ.get("GPS_PORT", "")     # 비워두면 소프트UART 사용
GPS_BAUD  = int(os.environ.get("GPS_BAUD", "9600"))

LORA_PORT = os.environ.get("LORA_PORT", "/dev/serial0")  # LoRa HAT (점퍼 B)
LORA_BAUD = int(os.environ.get("LORA_BAUD", "9600"))

# Waveshare SX1268 LoRa HAT 제어핀(BCM)
M0_PIN, M1_PIN = 22, 27

def init_lora_mode_txrx():
    GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM)
    GPIO.setup(M0_PIN, GPIO.OUT); GPIO.setup(M1_PIN, GPIO.OUT)
    GPIO.output(M0_PIN, GPIO.LOW)   # TX/RX 모드
    GPIO.output(M1_PIN, GPIO.LOW)
    time.sleep(0.05)

def open_serial(port, baud):
    return serial.Serial(port, baudrate=baud, timeout=1)

class GPSReader:
    def __init__(self, port:str, baud:int, use_bb:bool, bb_gpio:int):
        self.use_bb = use_bb and (not port)
        self.buf = b""
        self.fix_ok = False
        if self.use_bb:
            import pigpio
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon not running. Start with: sudo systemctl enable --now pigpiod")
            self.gpio = bb_gpio
            self.pi.set_mode(self.gpio, pigpio.INPUT)
            self.pi.bb_serial_read_open(self.gpio, baud)
            print(f"[Node] GPS(bb)  : GPIO{self.gpio} @ {baud}")
            self.ser = None
        else:
            self.ser = open_serial(port, baud)
            self.pi = None
            print(f"[Node] GPS(uart): {port} @ {baud}")

    def read(self, max_bytes=256):
        if self.use_bb:
            cnt, data = self.pi.bb_serial_read(self.gpio)
            return data if cnt > 0 else b""
        else:
            return self.ser.read(max_bytes)

    def close(self):
        if self.use_bb and self.pi:
            try:
                self.pi.bb_serial_read_close(self.gpio)
            finally:
                self.pi.stop()
        elif self.ser:
            self.ser.close()

def dm_to_deg(raw: str):
    if not raw: return None
    try:
        val = float(raw)
        d = int(val/100); m = val - d*100
        return d + m/60.0
    except:
        return None

def main():
    print("[Node] start")
    init_lora_mode_txrx()

    # 테스트용 고정 좌표 모드
    fake_lat, fake_lon = None, None
    if FAKE_LOC:
        try:
            parts = [p.strip() for p in FAKE_LOC.split(",")]
            if len(parts) == 2:
                fake_lat = float(parts[0]); fake_lon = float(parts[1])
                print(f"[Node] FAKE_LOC mode: {fake_lat}, {fake_lon}")
        except:
            print("[Node] FAKE_LOC parse error; ignoring")

    lora = open_serial(LORA_PORT, LORA_BAUD)
    print(f"[Node] LoRa : {LORA_PORT} @ {LORA_BAUD}")

    gps_reader = None
    if not FAKE_LOC:
        gps_reader = GPSReader(GPS_PORT, GPS_BAUD, USE_BB, BB_GPIO)

    lat, lon, heading = None, None, 0.0
    last_tx = 0
    interval = float(os.environ.get("TX_INTERVAL", "1.0"))

    while True:
        now = time.time()

        # FAKE 모드
        if fake_lat is not None and fake_lon is not None:
            if now - last_tx >= interval:
                doc = {
                    "id": NODE_ID, "nickname": NICKNAME,
                    "lat": round(fake_lat, 7), "lon": round(fake_lon, 7),
                    "heading": round(heading or 0.0, 1),
                    "ts": int(now * 1000)
                }
                lora.write((json.dumps(doc) + "\n").encode("utf-8"))
                print("[Node] TX(FAKE)", doc)
                last_tx = now
            time.sleep(0.05)
            continue

        # GPS 읽기
        if gps_reader:
            chunk = gps_reader.read(256)
            if chunk:
                gps_reader.buf += chunk
                while b"\n" in gps_reader.buf:
                    line, gps_reader.buf = gps_reader.buf.split(b"\n", 1)
                    s = line.decode(errors="ignore").strip()

                    if s.startswith("$GPRMC") or s.startswith("$GNRMC"):
                        p = s.split(",")
                        if len(p) >= 12:
                            gps_reader.fix_ok = (p[2] == "A")
                            la = dm_to_deg(p[3]); ns = p[4]
                            lo = dm_to_deg(p[5]); ew = p[6]
                            if la and lo:
                                if ns == "S": la = -la
                                if ew == "W": lo = -lo
                                lat, lon = la, lo
                            try:
                                heading = float(p[8])
                            except:
                                pass

        can_tx = (lat is not None and lon is not None) and (gps_reader is None or gps_reader.fix_ok or ALLOW_NOFIX)
        if can_tx and now - last_tx >= interval:
            doc = {
                "id": NODE_ID, "nickname": NICKNAME,
                "lat": round(lat, 7), "lon": round(lon, 7),
                "heading": round(heading or 0.0, 1),
                "ts": int(now * 1000)
            }
            lora.write((json.dumps(doc) + "\n").encode("utf-8"))
            print("[Node] TX", doc, "(fix_ok=" + str(gps_reader.fix_ok if gps_reader else True) + ")")
            last_tx = now
        elif (lat is None or lon is None) and now - last_tx >= 5:
            print("[Node] waiting GPS… (no lat/lon yet)")
            last_tx = now

        time.sleep(0.02)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
