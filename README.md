# SBC Cart Daemon

Raspberry Pi cart daemon that fuses UWB + IMU into a `/position` API.

## Files

- `daemon.py` - Main service (MQTT + IMU loop + Flask API)
- `fusion.py` - EKF fusion logic for `x`, `y`, velocity, and heading context
- `imu.py` - BNO085 setup and sensor reads (heading, accel, gyro)
- `requirements.txt` - Python dependencies
- `cart-daemon.service` - Systemd unit file
- `cart-daemon.env.example` - MQTT credentials template (copy to `cart-daemon.env`)
- `kiosk.py` / `kiosk.desktop` - Fullscreen Passion app launcher
- `nginx-cart.conf` - nginx proxy for `/position` and `app.getpassion.net`
- `passion-logo.png` - Desktop icon
- `binding.json` - Cached `sbc_id` to `tag_id` mapping (runtime cache)

## 1) Setup (SBC)

```bash
cd /home/passion/cart-daemon
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
cp cart-daemon.env.example cart-daemon.env
# edit cart-daemon.env with MQTT credentials
```

## 2) Run manually (for testing)

```bash
cd /home/passion/cart-daemon
source venv/bin/activate
python daemon.py
```

API endpoints:

- `GET http://localhost:6060/position`
- `GET http://localhost:6060/health`

## 3) Install as systemd service

```bash
sudo cp /home/passion/cart-daemon/cart-daemon.service /etc/systemd/system/cart-daemon.service
sudo systemctl daemon-reload
sudo systemctl enable cart-daemon
sudo systemctl restart cart-daemon
sudo systemctl status cart-daemon --no-pager
```

## 4) Logs

```bash
journalctl -u cart-daemon -f
```

## 5) Notes

- `binding.json` is written automatically by the daemon.
- MQTT credentials are loaded from `cart-daemon.env` (not committed to git).
- If IMU is not found, `imu.py` tries addresses `0x4A` then `0x4B`.
- Heading is startup-zeroed in `imu.py`.
- `/position` clears `x`/`y` when UWB data is older than 3 seconds (`position_stale`).
