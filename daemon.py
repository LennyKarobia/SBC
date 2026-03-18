import json
import math
import os
import time
import subprocess
import threading

import requests
from flask import Flask, jsonify
import paho.mqtt.client as mqtt

from fusion import Fusion
import imu


API_BASE = "https://getpassion.net"
MQTT_HOST = "44.254.150.42"
MQTT_PORT = 1883
MQTT_USER = "zahir"
MQTT_PASS = "Samaki123"

BINDING_FILE = os.path.expanduser("~/cart-daemon/binding.json")
HTTP_PORT = 6060

fusion = Fusion()

# Shared health/debug state
state = {
    "sbc_id": None,
    "tag_id": None,
    "mqtt_connected": False,
    "last_msg_ts": None,
    "last_error": None,
}


def get_sbc_id() -> str:
    out = subprocess.check_output("cat /proc/cpuinfo | grep Serial", shell=True).decode()
    return out.split(":")[1].strip()


def load_cached_tag_id() -> str | None:
    try:
        with open(BINDING_FILE, "r") as f:
            data = json.load(f)
        return data.get("tag_id")
    except Exception:
        return None


def save_cached_tag_id(sbc_id: str, tag_id: str) -> None:
    try:
        os.makedirs(os.path.dirname(BINDING_FILE), exist_ok=True)
        with open(BINDING_FILE, "w") as f:
            json.dump(
                {"sbc_id": sbc_id, "tag_id": tag_id, "saved_at": int(time.time())},
                f,
            )
    except Exception as e:
        state["last_error"] = f"cache_write_failed: {e}"


def fetch_tag_id_from_api(sbc_id: str) -> str | None:
    url = f"{API_BASE}/api/units/by-sbc/{sbc_id}"
    try:
        r = requests.get(url, timeout=5)
        if r.status_code != 200:
            state["last_error"] = f"api_status_{r.status_code}: {r.text[:200]}"
            return None
        data = r.json()
        return data.get("tag_id")
    except Exception as e:
        state["last_error"] = f"api_exception: {e}"
        return None


def resolve_tag_id(sbc_id: str) -> str:
    cached = load_cached_tag_id()
    if cached:
        return cached

    backoff = 2
    while True:
        tag_id = fetch_tag_id_from_api(sbc_id)
        if tag_id:
            save_cached_tag_id(sbc_id, tag_id)
            return tag_id
        time.sleep(backoff)
        backoff = min(backoff * 2, 30)


def mqtt_worker():
    client = mqtt.Client()

    def on_connect(c, userdata, flags, rc):
        state["mqtt_connected"] = True
        if state["tag_id"]:
            topic = f"unit/{state['tag_id']}/position"
            c.subscribe(topic)

    def on_disconnect(c, userdata, rc):
        state["mqtt_connected"] = False

    def on_message(c, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            x = payload.get("x")
            y = payload.get("y")
            fusion.uwb_update(x, y)
            state["last_msg_ts"] = int(time.time())
        except Exception as e:
            state["last_error"] = f"mqtt_payload_error: {e}"

    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    while True:
        try:
            client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
            client.loop_forever()
        except Exception as e:
            state["last_error"] = f"mqtt_exception: {e}"
            time.sleep(2)


def imu_loop():
    last = time.time()
    while True:
        now = time.time()
        dt = now - last
        last = now

        yaw_deg = imu.get_heading()
        yaw = math.radians(yaw_deg)

        ax, ay, _ = imu.get_linear_accel()
        _, _, gz = imu.get_gyro()

        fusion.imu_predict(yaw, ax, ay, gz, dt)
        time.sleep(0.01)


app = Flask(__name__)


@app.route("/position")
def position():
    return fusion.get_position()


@app.route("/health")
def health():
    return jsonify(
        {
            "sbc_id": state["sbc_id"],
            "tag_id": state["tag_id"],
            "mqtt_connected": state["mqtt_connected"],
            "last_msg_ts": state["last_msg_ts"],
            "last_error": state["last_error"],
        }
    )


def main():
    imu.init_imu()

    sbc_id = get_sbc_id()
    state["sbc_id"] = sbc_id

    tag_id = resolve_tag_id(sbc_id)
    state["tag_id"] = tag_id

    print(f"Bound sbc_id={sbc_id} to tag_id={tag_id}")

    threading.Thread(target=mqtt_worker, daemon=True).start()
    threading.Thread(target=imu_loop, daemon=True).start()

    app.run(host="0.0.0.0", port=HTTP_PORT)


if __name__ == "__main__":
    main()
