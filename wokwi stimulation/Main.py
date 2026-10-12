# Smart Helmet - Raspberry Pi Pico W (MicroPython) - Wokwi simulation
# Sensors: DHT22, MPU6050, gas sensor, potentiometer (noise), HC-SR04 (worn), LCD I2C, buzzer, button
import network, time, math, ujson, gc
import dht
from machine import Pin, I2C, ADC, PWM, time_pulse_us

# ---------- Buzzer (PWM) - silent from the very first lines ----------
BUZZ_FREQ = 2000
buzzer = PWM(Pin(15))
buzzer.freq(BUZZ_FREQ)
buzzer.duty_u16(0)                 # duty 0 = silent

def buzz(on):
    buzzer.duty_u16(32768 if on else 0)

from umqtt_simple import MQTTClient

# ---------- WiFi (Wokwi) ----------
WIFI_SSID = "Wokwi-GUEST"
WIFI_PASS = ""

# ---------- HiveMQ Cloud (private cluster, TLS + auth) ----------
HM_HOST = ".s1.eu.hivemq.cloud"
HM_PORT = 8883
HM_USER = "user"      # same user you created in HiveMQ Cloud
HM_PASS = "password"

# Quick test switch: False = public broker, port 1883, no TLS/auth (for debugging only)
USE_TLS = True
BROKER_HOST = HM_HOST if USE_TLS else "broker.hivemq.com"
BROKER_PORT = HM_PORT if USE_TLS else 1883
TOPIC_TELEMETRY = b"so-m8k2/helmet/telemetry"
TOPIC_ALERTS = b"so-m8k2/helmet/alerts"

# ---------- Thresholds ----------
TEMP_LIMIT = 45.0          # C
O2_LIMIT = 88.0            # %
FALL_IMPACT_G = 1.5        # sudden spike (g)
STILL_BAND_G = 0.35        # after the spike: |a-1g| < this = still
CANCEL_WINDOW = 5000       # ms to press the button to cancel the fall alert
NOISE_LIMIT = 80.0         # % (potentiometer)
NOISE_TIME = 10000         # ms of continuous loud noise (demo value)
WORN_MAX_CM = 20           # distance below this = head inside the helmet

PUBLISH_EVERY = 2000       # ms
RETRY_EVERY = 5000         # ms
DHT_EVERY = 2000           # ms (DHT22 needs >= 2 s between reads)

# ---------- Pins ----------
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=100000)
dht_sensor = dht.DHT22(Pin(2))
adc_gas = ADC(Pin(27))
adc_noise = ADC(Pin(28))
trig = Pin(16, Pin.OUT)
echo = Pin(17, Pin.IN)
button = Pin(14, Pin.IN, Pin.PULL_UP)     # pressed = 0

# ---------- LCD 16x2 over PCF8574 ----------
class LCD:
    def __init__(self, i2c, addr=0x27):
        self.i2c, self.addr, self.bl = i2c, addr, 0x08
        # HD44780 init with the required delays (the simulated LCD ignores commands sent too fast)
        time.sleep_ms(100)
        for delay in (10, 5, 5):
            self._pulse(0x30)
            time.sleep_ms(delay)
        self._pulse(0x20)                 # switch to 4-bit mode
        time.sleep_ms(5)
        self.cmd(0x28)                    # 4-bit, 2 lines, 5x8 font
        self.cmd(0x0C)                    # display on, cursor off
        self.cmd(0x06)                    # entry mode
        self.cmd(0x01)                    # clear
        time.sleep_ms(10)
    def _w(self, d):
        self.i2c.writeto(self.addr, bytes([d | self.bl]))
    def _pulse(self, d):
        self._w(d | 0x04); self._w(d & ~0x04)
    def _send(self, v, mode):
        self._pulse((v & 0xF0) | mode)
        self._pulse(((v << 4) & 0xF0) | mode)
    def cmd(self, c):
        self._send(c, 0); time.sleep_ms(5 if c in (0x01, 0x02) else 2)
    def show(self, l1, l2=""):
        self.cmd(0x80)
        for ch in "{:<16}".format(l1)[:16]:
            self._send(ord(ch), 1)
        self.cmd(0xC0)
        for ch in "{:<16}".format(l2)[:16]:
            self._send(ord(ch), 1)

lcd = LCD(i2c)

# ---------- MPU6050 ----------
MPU = 0x68
mpu_ok = False
last_a = 1.0                  # safe value (1 g = standing still) if the MPU fails
last_mpu_try = 0

def mpu_init():
    global mpu_ok
    try:
        i2c.writeto_mem(MPU, 0x6B, b"\x00")       # wake up
        mpu_ok = True
        print("MPU6050 OK")
    except OSError as e:
        mpu_ok = False
        print("MPU6050 not found:", e, "| I2C devices:", [hex(x) for x in i2c.scan()])

mpu_init()

def read_accel_g():
    global mpu_ok, last_a, last_mpu_try
    if not mpu_ok:                                  # retry every 3 s, never crash
        if time.ticks_diff(time.ticks_ms(), last_mpu_try) >= 3000:
            last_mpu_try = time.ticks_ms()
            mpu_init()
        return last_a
    try:
        d = i2c.readfrom_mem(MPU, 0x3B, 6)
    except OSError as e:
        print("MPU read failed:", e)
        mpu_ok = False
        return last_a
    def s(h, l):
        v = (h << 8) | l
        return v - 65536 if v > 32767 else v
    ax, ay, az = s(d[0], d[1]) / 16384, s(d[2], d[3]) / 16384, s(d[4], d[5]) / 16384
    last_a = math.sqrt(ax * ax + ay * ay + az * az)
    return last_a

# ---------- Other sensors ----------
def read_distance_cm():
    trig.value(0); time.sleep_us(2)
    trig.value(1); time.sleep_us(10); trig.value(0)
    try:
        d = time_pulse_us(echo, 1, 30000)
    except OSError:
        return 999
    if d < 0:
        return 999
    return d * 0.0343 / 2

def calibrate_gas():
    # Clean-air baseline: average of 20 readings at boot
    total = 0
    for _ in range(20):
        total += adc_gas.read_u16()
        time.sleep_ms(20)
    return total // 20

gas_base = calibrate_gas()

def read_oxygen():
    # SIMULATION: gas above the clean-air baseline -> lower oxygen %.
    # Real hardware: use an O2 sensor (or MAX30102 for SpO2).
    raw = adc_gas.read_u16()
    extra = max(0, raw - gas_base) / max(1, 65535 - gas_base)
    return 98 - extra * 40

def read_noise():
    # SIMULATION: potentiometer = sound level 0-100 %
    return adc_noise.read_u16() / 65535 * 100

def read_gps():
    # SIMULATION: Wokwi has no GPS part. Real GPS: parse NMEA from UART.
    return 29.9511, 31.2345

def pressed():
    return button.value() == 0

# ---------- WiFi ----------
wlan = network.WLAN(network.STA_IF)

def connect_wifi():
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    print("Connecting WiFi", end="")
    while not wlan.isconnected():
        time.sleep(0.25)
        print(".", end="")
    print("\nWiFi connected:", wlan.ifconfig()[0])

# ---------- MQTT (HiveMQ Cloud, non-blocking reconnect) ----------
hm = None
last_hm_try = time.ticks_ms() - RETRY_EVERY
pending = []                       # alerts waiting for the connection

def maintain_hive():
    global hm, last_hm_try
    if hm is not None:
        return
    if time.ticks_diff(time.ticks_ms(), last_hm_try) < RETRY_EVERY:
        return
    last_hm_try = time.ticks_ms()
    if not wlan.isconnected():              # don't touch MQTT while WiFi is down
        print("WiFi lost, reconnecting...")
        wlan.connect(WIFI_SSID, WIFI_PASS)
        return
    gc.collect()                            # TLS handshake needs free RAM
    try:
        print("HiveMQ connect... ", end="")
        if USE_TLS:
            c = MQTTClient("pico-helmet-001", BROKER_HOST, port=BROKER_PORT,
                           user=HM_USER, password=HM_PASS, keepalive=60,
                           ssl=True, ssl_params={"server_hostname": BROKER_HOST})
        else:
            c = MQTTClient("pico-helmet-001", BROKER_HOST, port=BROKER_PORT, keepalive=60)
        c.connect()
        hm = c
        print("OK")
    except Exception as e:
        hm = None
        print("fail:", e)
        gc.collect()

def flush_pending():
    global hm
    while pending and hm is not None:
        try:
            hm.publish(TOPIC_ALERTS, pending[0], qos=1)
            pending.pop(0)
        except Exception as e:
            print("Alert publish failed:", e)
            hm = None

def send_alert(kind, msg, **extra):
    lat, lon = read_gps()
    data = {"type": kind, "message": msg, "lat": lat, "lon": lon}
    data.update(extra)
    pending.append(ujson.dumps(data).encode())   # bytes, so the MQTT length is correct with the emoji
    if len(pending) > 10:
        pending.pop(0)
    print("ALERT ->", data)
    flush_pending()

def publish_telemetry(payload):
    global hm
    if hm is None:
        return
    try:
        hm.publish(TOPIC_TELEMETRY, ujson.dumps(payload))
    except Exception as e:
        print("Telemetry publish failed:", e)
        hm = None

# ---------- State ----------
def reset_states():
    global fire_active, fire_silenced, gas_active, gas_silenced
    global noise_active, noise_silenced, noise_t0, fall_state, fall_t0, last_lcd, fall_armed
    fire_active = fire_silenced = gas_active = gas_silenced = False
    noise_active = noise_silenced = False
    noise_t0 = None
    fall_state, fall_t0, last_lcd = 0, 0, None
    fall_armed = True

fire_active = fire_silenced = gas_active = gas_silenced = False
noise_active = noise_silenced = False
noise_t0 = None
fall_state, fall_t0, last_lcd = 0, 0, None   # 0 none, 2 waiting cancel (5 s), 3 alert sent
fall_armed = True    # False after a fall until acceleration drops back below the limit

is_worn = False
worn_cnt = 0
temp = 25.0
last_dht = time.ticks_ms() - DHT_EVERY
last_ir = 0
last_tele = 0

connect_wifi()
lcd.show("Smart Helmet", "Wear the helmet")

while True:
    now = time.ticks_ms()
    maintain_hive()
    flush_pending()

    # ---- Worn detection (HC-SR04 inside the helmet, with debounce) ----
    if time.ticks_diff(now, last_ir) >= 200:
        last_ir = now
        if read_distance_cm() < WORN_MAX_CM:
            worn_cnt = min(worn_cnt + 1, 3)
        else:
            worn_cnt = max(worn_cnt - 1, 0)
        if worn_cnt >= 3 and not is_worn:
            is_worn = True
            reset_states()
            lcd.show("Helmet worn", "System started")
            send_alert("info", "Helmet worn - system started")
            time.sleep(2)
        elif worn_cnt == 0 and is_worn:
            is_worn = False
            reset_states()
            buzz(False)
            lcd.show("Helmet removed", "System paused")

    if not is_worn:
        if time.ticks_diff(now, last_tele) >= PUBLISH_EVERY:
            last_tele = now
            publish_telemetry({"worn": 0, "status": "NOT WORN"})
        time.sleep_ms(50)
        continue

    # ---- Read sensors ----
    if time.ticks_diff(now, last_dht) >= DHT_EVERY:
        last_dht = now
        try:
            dht_sensor.measure()
            temp = dht_sensor.temperature()
        except Exception as e:
            print("DHT read failed:", e)
    o2 = read_oxygen()
    a = read_accel_g()
    noise = read_noise()
    lat, lon = read_gps()

    # ---- Fire ----
    if temp > TEMP_LIMIT:
        if not fire_active:
            fire_active, fire_silenced = True, False
            send_alert("fire", "warning ⚠️ fire predicted", temp=round(temp, 1))
        if pressed():
            fire_silenced = True
    else:
        fire_active = False

    # ---- Gas / low oxygen ----
    if o2 < O2_LIMIT:
        if not gas_active:
            gas_active, gas_silenced = True, False
            send_alert("gas", "warning ⚠️ gas leakage", oxygen=round(o2, 1))
        if pressed():
            gas_silenced = True
    else:
        gas_active = False

    # ---- Noise (LCD + buzzer only, no email) ----
    if noise > NOISE_LIMIT:
        if noise_t0 is None:
            noise_t0 = now
        if time.ticks_diff(now, noise_t0) >= NOISE_TIME:
            if not noise_active:
                noise_active, noise_silenced = True, False
            if pressed():
                noise_silenced = True
    else:
        noise_t0, noise_active = None, False

    # ---- Fall detection ----
    if a <= FALL_IMPACT_G:
        fall_armed = True                     # re-arm only after acceleration is back to normal
    if fall_state == 0:
        if a > FALL_IMPACT_G and fall_armed:  # sudden jump -> fall predicted immediately
            fall_state, fall_t0 = 2, now
    elif fall_state == 2:
        if pressed():                         # cancelled: buzzer stops, no message
            fall_state = 0
            fall_armed = False                # don't re-trigger while the spike is still high
            buzz(False)
            time.sleep(1)
        elif time.ticks_diff(now, fall_t0) >= CANCEL_WINDOW:
            send_alert("fall", "warning ⚠️ fall predicted", accel=round(a, 2))
            fall_state, fall_t0 = 3, now
            fall_armed = False
    elif fall_state == 3:
        if pressed() or time.ticks_diff(now, fall_t0) > 30000:
            fall_state = 0

    # ---- Buzzer ----
    buz = (fire_active and not fire_silenced) or (gas_active and not gas_silenced) \
          or (noise_active and not noise_silenced) or fall_state == 2
    buzz(buz and (now // 300) % 2 == 0)      # beeps: 300 ms on / 300 ms off

    # ---- LCD (priority: fall > fire > gas > noise) ----
    if fall_state in (2, 3):
        l1, l2 = "FALL PREDICTED!", "Press btn <5s" if fall_state == 2 else "Alert sent"
    elif fire_active:
        l1, l2 = "FIRE PREDICTED!", "T={:.1f}C".format(temp)
    elif gas_active:
        l1, l2 = "GAS LEAKAGE!", "O2={:.1f}%".format(o2)
    elif noise_active:
        l1, l2 = "LOUD NOISE!", "{:.0f}% Ear plugs".format(noise)
    else:
        l1, l2 = "T:{:.1f}C O2:{:.0f}%".format(temp, o2), "Status: SAFE"
    if (l1, l2) != last_lcd:
        lcd.show(l1, l2)
        last_lcd = (l1, l2)

    # ---- Telemetry for the dashboard ----
    if time.ticks_diff(now, last_tele) >= PUBLISH_EVERY:
        last_tele = now
        status = "FALL" if fall_state >= 2 else "FIRE" if fire_active else \
                 "GAS" if gas_active else "NOISE" if noise_active else "SAFE"
        payload = {"temp": round(temp, 1), "oxygen": round(o2, 1), "accel": round(a, 2),
                   "noise": round(noise), "lat": lat, "lon": lon, "worn": 1, "status": status}
        print(payload)
        publish_telemetry(payload)

    time.sleep_ms(50)